"""ROS connection of the chassis operator console.

The scope runtime's ``/rt/capabilities`` declaration,
``/control/set_enabled``, ``/rt/reset_fault`` (FAILED only), controller-manager and
controller parameter services, ``/diagnostics`` and the declared velocity topic;
plus the controller's RT-Control private mode service/state inside this domain. The
console never starts hardware, writes drive words or enables on its own.
"""

from __future__ import annotations

import asyncio
import json
import math
from pathlib import Path
import signal
import threading
import time

from controller_manager_msgs.srv import ListHardwareComponents
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from robot_rt_control_interfaces.srv import SetControlEnabled
import robot_interfaces_qos
from rt_control_interfaces.msg import ChassisState
from rt_control_interfaces.action import ChassisRelativeMove
from rt_control_interfaces.srv import ChassisSetMode, RtEnable
from std_msgs.msg import Bool, String

from .config import build_config, load_token
from .chassis_mode import ChassisModeView
from .chassis_operation import (
    ChassisOperation,
    LIMIT_FIELDS as OP_LIMIT_FIELDS,
    PARAMETERS as OP_PARAMETERS,
)
from .teleop import PUBLISH_RATE_HZ, DriveArbiter, TeleopError
from .tuning import STEERING_PARAMETERS, FieldTuning, TuningError

LIMIT_PARAMETERS = (
    "max_steering_slew_limit",
    "max_steering_acceleration_limit",
    "max_steering_jerk_limit",
)
# Startup default, used only as a fallback for controllers without the runtime mode topic.
DRIVE_MODE_PARAMETERS = ("drive_mode",)
STALE_S = 2.5


async def call_service(client, request, timeout):
    """Await a ROS service future from the aiohttp event loop."""
    loop = asyncio.get_running_loop()
    completion = loop.create_future()
    future = client.call_async(request)

    def completed(done):
        def resolve():
            if completion.done():
                return
            try:
                completion.set_result(done.result())
            except Exception as exc:  # noqa: BLE001 - surfaced to the operator
                completion.set_exception(exc)

        loop.call_soon_threadsafe(resolve)

    future.add_done_callback(completed)
    try:
        return await asyncio.wait_for(completion, timeout=timeout)
    except (asyncio.TimeoutError, asyncio.CancelledError):
        client.remove_pending_request(future)
        raise


class ChassisConnection:
    """Chassis state from the scope runtime and the operator operations on it."""

    def __init__(self, node, software_test_only, tuning):
        self.node = node
        self.software_test_only = software_test_only
        self.tuning = tuning
        self.lock = threading.RLock()
        self.hardware = False
        self.hardware_at = -math.inf
        self.kind = "disconnected"
        self.steering = []
        self.drive = []
        self.managed = []
        self.topic = ""
        self.parameters_at = -math.inf
        self.state = "DISCONNECTED"
        self.stage = ""
        self.enabled = False
        self.diag_at = -math.inf
        self.ignore_diag_before = 0
        self.busy = False
        self.mode_busy = False
        self.last_error = ""
        self.pending = {}
        self.capability = None
        self.steering_motion = {}
        self.steering_motion_at = -math.inf
        self.runtime_mode = None
        self.runtime_mode_at = -math.inf
        node.create_subscription(
            Bool,
            "/rt_control/chassis/align_first",
            self.received_mode,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        controller = node.config.controller_node
        self.chassis_mode = ChassisModeView()
        self.mode_client = node.create_client(ChassisSetMode, controller + "/set_mode")
        action_client = ActionClient(node, ChassisRelativeMove, controller + "/relative_move")
        self.operation = ChassisOperation(
            action_client,
            self.make_operation_goal,
            lambda: self.status()["operation"]["can_execute"],
            self.lock,
        )
        node.create_subscription(ChassisState, controller + "/state", self.received_chassis_mode, 1)
        node.create_subscription(
            String,
            "/rt/capabilities",
            self.capabilities,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.enable_client = node.create_client(SetControlEnabled, "/control/set_enabled")
        self.reset_client = node.create_client(RtEnable, "/rt/reset_fault")
        self.hardware_client = node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )
        self.swerve_client = node.create_client(GetParameters, controller + "/get_parameters")
        self.steering_set = node.create_client(SetParameters, controller + "/set_parameters")
        self.enable_params = node.create_client(
            GetParameters, "/ctrl_enable_manager/get_parameters"
        )
        node.create_subscription(
            DiagnosticArray, "/diagnostics", self.diagnostics, robot_interfaces_qos.diagnostic()
        )
        node.create_timer(0.5, self.refresh)

    # ------------------------------------------------------------------ inputs
    def capabilities(self, msg):
        """The scope runtime declares what it offers; the console needs enable and velocity."""
        try:
            document = json.loads(msg.data)
            names = {item["name"]: item for item in document["capabilities"]}
            usable = (
                document.get("schema_version") == 1
                and "enable" in names
                and "chassis_velocity" in names
            )
        except (ValueError, KeyError, TypeError):
            usable = False
        with self.lock:
            self.capability = (document, names) if usable else None

    def diagnostics(self, msg):
        for status in msg.status:
            if status.name != "/robot/rt_control/enable_manager":
                continue
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            values = {v.key: v.value for v in status.values}
            with self.lock:
                if stamp < self.ignore_diag_before:
                    return
                self.state = values.get("state", "UNKNOWN")
                self.stage = values.get("stage", "")
                self.enabled = self.state == "ENABLED"
                self.diag_at = time.monotonic()
            return

    def request(self, key, client, request, callback):
        prior = self.pending.get(key)
        if prior is not None and not prior[0].done():
            if time.monotonic() - prior[1] < STALE_S:
                return
            client.remove_pending_request(prior[0])
            prior[0].cancel()
        if not client.service_is_ready():
            return
        future = client.call_async(request)
        self.pending[key] = (future, time.monotonic())

        def finished(done):
            try:
                callback(done.result())
            except Exception as exc:  # noqa: BLE001 - the next refresh retries
                self.node.get_logger().debug(f"{key}: {exc}")

        future.add_done_callback(finished)

    def refresh(self):
        self.request(
            "chassis_mode_config",
            self.swerve_client,
            GetParameters.Request(names=["relative.enabled", *OP_PARAMETERS]),
            self.received_mode_config,
        )
        self.request(
            "hardware",
            self.hardware_client,
            ListHardwareComponents.Request(),
            self.received_hardware,
        )
        self.request(
            "swerve",
            self.swerve_client,
            GetParameters.Request(names=["steering_joints", "drive_joints"]),
            self.received_swerve,
        )
        self.request(
            "managed",
            self.enable_params,
            GetParameters.Request(names=["managed_joints"]),
            self.received_managed,
        )
        self.request(
            "steering_params",
            self.swerve_client,
            GetParameters.Request(
                names=[*STEERING_PARAMETERS, *LIMIT_PARAMETERS, *DRIVE_MODE_PARAMETERS]
            ),
            self.received_steering,
        )

    def received_hardware(self, response):
        components = response.component
        with self.lock:
            capability = self.capability
        if capability is None:
            return
        document, _ = capability
        active = all(c.state.id == 3 for c in components)
        declared = {(item["name"], item["plugin"]) for item in document["hardware"]}
        actual = {(c.name, c.class_type) for c in components}
        mock = bool(document["use_mock_hardware"])
        with self.lock:
            self.kind = "mock" if mock else "real"
            self.hardware = (
                active and actual == declared and (mock if self.software_test_only else not mock)
            )
            self.hardware_at = time.monotonic()

    def received_swerve(self, response):
        if len(response.values) != 2:
            return
        with self.lock:
            if self.capability is None:
                return
            self.steering = list(response.values[0].string_array_value)
            self.drive = list(response.values[1].string_array_value)
            self.topic = self.capability[1]["chassis_velocity"]["topic"]
            self.parameters_at = time.monotonic()

    def received_managed(self, response):
        if len(response.values) == 1:
            with self.lock:
                self.managed = list(response.values[0].string_array_value)

    def received_mode(self, message):
        with self.lock:
            self.runtime_mode = "align_first" if message.data else "steer_while_driving"
            self.runtime_mode_at = time.monotonic()

    def received_mode_config(self, response):
        with self.lock:
            enabled = None
            if response.values:
                value = response.values[0]
                if value.type == ParameterType.PARAMETER_BOOL:
                    enabled = value.bool_value
            self.chassis_mode.configure(enabled)
            limits = {}
            for name, value in zip(OP_LIMIT_FIELDS, response.values[1:]):
                if value.type == ParameterType.PARAMETER_DOUBLE:
                    limits[name] = value.double_value
                elif value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    limits[name] = list(value.double_array_value)
            self.operation.configure(limits)

    def received_chassis_mode(self, message):
        stamp = message.stamp.sec * 1_000_000_000 + message.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
        with self.lock:
            self.chassis_mode.receive(message, age)

    def received_steering(self, response):
        names = [*STEERING_PARAMETERS, *LIMIT_PARAMETERS]
        if len(response.values) != len(names) + len(DRIVE_MODE_PARAMETERS):
            return
        values = {name: math.degrees(v.double_value) for name, v in zip(names, response.values)}
        drive = {"drive_mode": response.values[len(names)].string_value or None}
        # An unlimited jerk (trapezoidal profile) is reported as None; JSON has no infinity.
        if values["max_steering_jerk"] == math.inf:
            values["max_steering_jerk"] = None
        if not all(v is None or (math.isfinite(v) and v > 0) for v in values.values()):
            return
        with self.lock:
            self.steering_motion = {**values, **drive}
            self.steering_motion_at = time.monotonic()

    # ------------------------------------------------------------------ status
    def scope_ready(self):
        now = time.monotonic()
        axes = self.steering + self.drive
        return (
            self.hardware
            and now - self.hardware_at < STALE_S
            and now - self.parameters_at < STALE_S
            and len(self.steering) == len(self.drive) == 4
            and len(set(axes)) == 8
            and set(self.managed) == set(axes)
            and self.topic == self.node.config.command_topic
        )

    def status(self):
        with self.lock:
            now = time.monotonic()
            scope = self.scope_ready()
            diagnostic_fresh = now - self.diag_at < STALE_S
            busy = self.busy or self.mode_busy or self.operation.running or self.operation.unknown
            connected = scope and diagnostic_fresh and self.enable_client.service_is_ready()
            if not connected:
                reason = (
                    "等待底盘运行栈启动"
                    if now - self.hardware_at >= STALE_S
                    else "正在连接底盘接口"
                )
            elif self.state not in ("IDLE", "ENABLED"):
                reason = f"底盘状态：{self.state} / {self.stage}"
            elif self.stage == "jtc_activate_failed":
                reason = "运动控制器激活失败"
            else:
                reason = "已使能" if self.enabled else "未使能"
            steering = dict(self.steering_motion) if now - self.steering_motion_at < 5.0 else {}
            if steering and self.runtime_mode_at != -math.inf:
                steering["drive_mode"] = (
                    self.runtime_mode if now - self.runtime_mode_at < STALE_S else None
                )
            mode = self.chassis_mode.status(
                connected=connected,
                enabled=self.enabled is True,
                busy=busy,
                service_ready=self.mode_client.service_is_ready(),
            )
            return {
                "backend_kind": self.kind if now - self.hardware_at < STALE_S else "disconnected",
                "software_test_only": self.software_test_only,
                "connected": connected,
                "enabled": self.enabled if connected else None,
                "busy": busy,
                "can_enable": connected and self.state in ("IDLE", "ENABLED") and not busy,
                "state": self.state if diagnostic_fresh else "DISCONNECTED",
                "stage": self.stage,
                "message": reason,
                "last_error": self.last_error,
                "command_topic": self.node.config.command_topic,
                "enable_service": "/control/set_enabled",
                "can_reset": diagnostic_fresh
                and self.state == "FAILED"
                and not busy
                and self.reset_client.service_is_ready(),
                "can_tune_steering": connected
                and self.state in ("IDLE", "FAILED")
                and not busy
                and bool(steering)
                and steering.get("drive_mode") in ("align_first", "steer_while_driving")
                and self.tuning is not None,
                "steering": steering,
                "chassis_mode": mode,
                "operation": self.operation.status(mode, connected, self.enabled is True, busy),
                "steering_tuning": self.tuning.source()
                if self.tuning is not None
                else {"source": "unavailable", "updated_at": "未配置部署产物目录"},
            }

    def allowed(self):
        value = self.status()
        return (
            value["connected"]
            and value["enabled"] is True
            and not value["busy"]
            and value["chassis_mode"]["velocity_allowed"]
            and self.node.publisher.get_subscription_count() > 0
        )

    # ------------------------------------------------------------------ operations
    @staticmethod
    def make_operation_goal(values):
        goal = ChassisRelativeMove.Goal()
        goal.dx, goal.dy, goal.dyaw = values["dx"], values["dy"], values["dyaw"]
        goal.max_duration = values["max_duration"]
        goal.corridor_token = "operator-web"
        for name in OP_LIMIT_FIELDS:
            if name != "max_duration":
                setattr(goal.limits, name, values[name])
        return goal

    async def run_operation(self, values):
        self.node.arbiter.reset_motion("operation_start")
        self.node.publish_zero()
        return await self.operation.execute(values)

    async def set_chassis_mode(self, mode):
        if mode not in ("navigation", "operation"):
            raise TeleopError("invalid_mode", "只支持导航或作业模式")
        with self.lock:
            status = self.status()["chassis_mode"]
            if not status["can_switch"]:
                raise TeleopError("mode_not_ready", status["message"])
            if status["mode"] == mode:
                return {"ok": True, "message": "已经处于目标模式"}
            self.mode_busy = True
            self.last_error = ""
        self.node.arbiter.reset_motion("mode_change")
        self.node.publish_zero()
        target = 1 if mode == "navigation" else 2
        try:
            result = await call_service(
                self.mode_client,
                ChassisSetMode.Request(mode=target, confirm=True),
                60,
            )
            ok = (
                result.code == ChassisSetMode.Response.OK
                and result.state.mode == target
                and result.state.fault_code == 0
            )
            if ok:
                self.received_chassis_mode(result.state)
                message = "已确认切换到" + ("导航模式" if target == 1 else "作业模式")
            else:
                reasons = {
                    1: "模式请求无效",
                    2: "尚未满足静止条件",
                    3: "控制器正在执行其他操作",
                    4: "控制器未就绪",
                    5: "故障已闭锁",
                    6: "四轮模式交接失败",
                }
                message = (
                    f"模式反馈故障：{result.state.fault_code}"
                    if result.state.fault_code
                    else reasons.get(result.code, "模式反馈与请求不一致")
                )
            if not ok:
                with self.lock:
                    self.chassis_mode.invalidate()
                    self.last_error = message
            return {"ok": ok, "message": message}
        except asyncio.TimeoutError:
            with self.lock:
                self.chassis_mode.invalidate()
                self.last_error = "模式切换响应超时，结果未确认；等待实际反馈，勿连续重试"
            return {"ok": False, "message": self.last_error}
        except Exception:
            with self.lock:
                self.chassis_mode.invalidate()
                self.last_error = "模式切换异常，等待控制器实际状态"
            raise
        finally:
            self.node.arbiter.reset_motion("mode_change_complete")
            with self.lock:
                self.mode_busy = False

    async def operate(self, op, modules):
        if modules != ("chassis",) or op not in ("enable", "disable", "reset"):
            raise TeleopError("unsupported_operation", "只支持底盘使能、失能和故障复位")
        if op == "reset":
            return await self.reset_fault()
        status = self.status()
        if not status["connected"]:
            raise TeleopError("chassis_unavailable", status["message"])
        if op == "enable" and not status["can_enable"]:
            raise TeleopError("chassis_not_ready", status["message"])
        with self.lock:
            self.busy = True
            self.last_error = ""
        self.node.arbiter.reset_motion("enable_change")
        self.node.publish_zero()
        request = SetControlEnabled.Request(
            enabled=op == "enable", reason="chassis operator console"
        )
        try:
            result = await call_service(self.enable_client, request, 45)
            message = result.error.message
            with self.lock:
                if result.accepted:
                    self.enabled = bool(result.enabled)
                    self.state = "ENABLED" if self.enabled else "IDLE"
                    self.diag_at = time.monotonic()
                    self.ignore_diag_before = self.node.get_clock().now().nanoseconds
                else:
                    self.last_error = message or "底盘操作被拒绝"
                    self.enabled = None
                    self.diag_at = -math.inf
            return {
                "ok": result.accepted,
                "enabled": result.enabled,
                "stage": message,
                "modules": ["chassis"],
            }
        except asyncio.TimeoutError:
            with self.lock:
                self.last_error = "底盘服务响应超时，请查看实际状态"
                self.enabled = None
                self.diag_at = -math.inf
            return {"ok": False, "stage": self.last_error, "modules": ["chassis"]}
        finally:
            self.node.arbiter.reset_motion("enable_change_complete")
            with self.lock:
                self.busy = False

    async def reset_fault(self):
        if not self.status()["can_reset"]:
            raise TeleopError("reset_not_allowed", "只有底盘处于故障（FAILED）时才能复位")
        with self.lock:
            self.busy = True
            self.last_error = ""
        self.node.arbiter.reset_motion("fault_reset")
        try:
            result = await call_service(self.reset_client, RtEnable.Request(), 30)
            with self.lock:
                self.diag_at = -math.inf  # wait for the manager's next report
            return {
                "ok": bool(result.ok),
                "enabled": False,
                "stage": "故障已复位，可重新使能" if result.ok else (result.stage or "复位被拒绝"),
                "modules": ["chassis"],
            }
        except asyncio.TimeoutError:
            return {
                "ok": False,
                "enabled": False,
                "stage": "复位服务响应超时",
                "modules": ["chassis"],
            }
        finally:
            with self.lock:
                self.busy = False

    async def set_steering(self, slew_deg, accel_deg, jerk_deg, alignment_deg, release_deg=0.0):
        status = self.status()
        if not status["can_tune_steering"]:
            raise TeleopError("steering_locked", "只能在底盘未使能时修改转向参数")
        limits = status["steering"]
        # alignment_deg/release_deg belong to the running drive mode; the other mode keeps its
        # current values so the saved file is always a complete snapshot.
        align_mode = limits.get("drive_mode") == "align_first"
        checks = [
            (slew_deg, limits["max_steering_slew_limit"], "转向速度", "°/s"),
            (accel_deg, limits["max_steering_acceleration_limit"], "转向加速度", "°/s²"),
            (jerk_deg, limits["max_steering_jerk_limit"], "转向加加速度", "°/s³"),
            (alignment_deg, 90.0, "对齐门限", "°"),
        ]
        if align_mode:
            checks.append((release_deg, alignment_deg, "恢复门限", "°（不大于对齐门限）"))
        for value, limit, label, unit in checks:
            if not 0 < value <= limit:
                raise TeleopError("steering_out_of_range", f"{label}须在 0–{limit:.0f}{unit}")
        values = {
            "max_steering_slew": math.radians(slew_deg),
            "max_steering_acceleration": math.radians(accel_deg),
            "max_steering_jerk": math.radians(jerk_deg),
            "alignment_threshold": math.radians(
                limits["alignment_threshold"] if align_mode else alignment_deg
            ),
            "align_first_threshold": math.radians(
                alignment_deg if align_mode else limits["align_first_threshold"]
            ),
            "align_first_release": math.radians(
                release_deg if align_mode else limits["align_first_release"]
            ),
        }
        gate = (
            f"对齐门限 {alignment_deg:.0f}°/恢复 {release_deg:.0f}°"
            if align_mode
            else f"对齐门限 {alignment_deg:.0f}°"
        )
        message = (
            f"转向参数已保存：{slew_deg:.1f}°/s，{accel_deg:.1f}°/s²，{jerk_deg:.0f}°/s³，"
            f"{gate}，下次使能生效（重启后保留）"
        )
        return await self.apply_steering(values, "set", message, persist=True)

    async def restore_steering_defaults(self):
        if not self.status()["can_tune_steering"]:
            raise TeleopError("steering_locked", "只能在底盘未使能时修改转向参数")
        try:
            values = self.tuning.defaults()
        except TuningError as exc:
            raise TeleopError("defaults_unavailable", str(exc)) from exc
        return await self.apply_steering(
            values, "restore_defaults", "已恢复默认转向参数，下次使能生效", persist=False
        )

    async def apply_steering(self, values_rad, action, message, persist):
        request = SetParameters.Request(
            parameters=[
                Parameter(
                    name=name,
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value),
                )
                for name, value in values_rad.items()
            ]
        )
        try:
            result = await call_service(self.steering_set, request, 10)
        except asyncio.TimeoutError:
            return {"ok": False, "message": "转向控制器参数服务响应超时"}
        reasons = [r.reason for r in result.results if not r.successful]
        self.refresh()
        if reasons:
            return {"ok": False, "message": "转向参数被拒绝：" + "; ".join(reasons)}
        try:
            self.tuning.store(values_rad if persist else None, action)
        except (OSError, TuningError) as exc:
            return {"ok": False, "message": f"参数已生效，但保存失败（重启后不保留）：{exc}"}
        return {"ok": True, "message": message}


class OperatorWebNode(Node):
    def __init__(self, *, software_test_only=False):
        super().__init__("integ_operator_web")
        defaults = {
            "bind_host": "127.0.0.1",
            "port": 18120,
            "command_topic": "/cmd_vel",
            "controller_node": "/swerve_controller",
            "gear_names": ["leisure", "sport"],
            "gear_linear_speeds": [0.05, 0.15],
            "gear_angular_speeds": [0.1, 0.3],
            "linear_limits": [0.3, 0.5, 1.0, 2.0],
            "angular_limits": [0.6, 1.0, 2.0, 4.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("static_dir", "")
        # Deployment artifact of the running scope: steering defaults and the unit name.
        self.declare_parameter("generated_dir", "")
        self.config = build_config(**{k: self.get_parameter(k).value for k in defaults})
        generated_dir = self.get_parameter("generated_dir").value
        tuning = None
        if generated_dir:
            tuning = FieldTuning(Path(generated_dir), self.config.controller_node.lstrip("/"))
        self.arbiter = DriveArbiter(self.config.gears, self.config.limits, time.monotonic)
        self.publisher = self.create_publisher(
            Twist, self.config.command_topic, robot_interfaces_qos.control()
        )
        self.backend = ChassisConnection(self, software_test_only, tuning)
        self.create_timer(1 / PUBLISH_RATE_HZ, self.publish_command)

    def publish_zero(self):
        self.publisher.publish(Twist())

    def publish_command(self):
        if not self.backend.allowed():
            self.arbiter.reset_motion("chassis_not_enabled")
            return
        command = self.arbiter.tick()
        if command is not None:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = command
            self.publisher.publish(msg)

    def robot_status(self):
        return {
            "chassis": self.backend.status(),
            "drive_available": self.backend.allowed(),
            "command_subscribers": self.publisher.get_subscription_count(),
        }


def arm_page(arm, sessions, operators, audit, static_dir):
    """The arm page and its ROS bridge (own executor thread), sharing login, lease rule and audit."""
    from ament_index_python.packages import get_package_share_directory
    import yaml

    from .arm.console import ArmConsole, config_from_mapping
    from .arm.page import ArmPage
    from .arm.ros_bridge import RosBridge, spin_in_background
    from .arm.vacuum_control import VacuumControl

    config = config_from_mapping(yaml.safe_load(arm["config"].read_text(encoding="utf-8")))
    bridge = RosBridge(arm["joint_state_topic"])
    console = ArmConsole(config, "", bridge, arm["poses"])
    bridge.console = console
    spin_in_background(bridge)
    page = ArmPage(
        console,
        sessions,
        operators,
        static_dir / "arm",
        get_package_share_directory,
        bridge.set_enabled,
        bridge.reset_fault,
        audit,
        view_model_share=arm["view_model_share"],
        vacuum=VacuumControl(arm["vacuum_script"]) if arm["vacuum_script"].is_file() else None,
    )
    return page, bridge


def head_page(head, sessions, operators, audit, static_dir):
    """The head page and its ROS bridge (own executor thread)."""
    import yaml

    from .arm.ros_bridge import spin_in_background
    from .head.console import HeadConsole
    from .head.page import HeadPage
    from .head.ros_bridge import HeadBridge

    bridge = HeadBridge(head["joint_state_topic"])
    console = HeadConsole(
        yaml.safe_load(head["config"].read_text(encoding="utf-8")), bridge.publish
    )
    bridge.console = console
    spin_in_background(bridge)
    page = HeadPage(
        console,
        sessions,
        operators,
        static_dir / "head",
        bridge.set_enabled,
        bridge.reset_fault,
        audit,
    )
    return page, bridge


def diag_page(sessions, static_dir):
    """The read-only diagnostics page and its subscriptions (own executor thread)."""
    from integ_diagnostics.report import state_directory

    from .diag.feed import DiagnosticsFeed, FeedNode
    from .diag.page import DiagPage

    feed = DiagnosticsFeed()
    node = FeedNode(feed)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin, name="operator-diag", daemon=True).start()
    page = DiagPage(feed, sessions, static_dir / "diag", lambda: state_directory() / "self_check")
    return page, node


def main(argv=None, *, software_test_only=False, audit_path=None, arm=None, head=None):
    from aiohttp import web

    from .audit import AuditLog
    from .operators import Operators
    from .server import OperatorServer
    from .session import Sessions

    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = OperatorWebNode(software_test_only=software_test_only)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    sessions, operators, audit = Sessions(load_token()), Operators(), AuditLog(audit_path)
    static_dir = Path(node.get_parameter("static_dir").value)
    diag, diag_node = diag_page(sessions, static_dir)
    pages, bridge = (diag,), None
    if arm is not None:
        page, bridge = arm_page(arm, sessions, operators, audit, static_dir)
        pages = (page, diag)
    head_bridge = None
    if head is not None:
        page, head_bridge = head_page(head, sessions, operators, audit, static_dir)
        pages = (*pages, page)
    server = OperatorServer(
        node.arbiter,
        sessions,
        node.robot_status,
        static_dir,
        module_operation=node.backend.operate,
        drive_allowed=node.backend.allowed,
        steering_operation=node.backend.set_steering,
        steering_defaults=node.backend.restore_steering_defaults,
        mode_operation=node.backend.set_chassis_mode,
        motion_operation=node.backend.run_operation,
        motion_cancel=node.backend.operation.cancel,
        audit=audit,
        operators=operators,
        pages=pages,
    )

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin, name="operator-ros", daemon=True)
    thread.start()
    try:
        web.run_app(
            server.build_app(),
            host=node.config.bind_host,
            port=node.config.port,
            access_log=None,
            print=None,
        )
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.arbiter.reset_motion("process_shutdown")
        for _ in range(3):
            node.publish_zero()
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        if bridge is not None:
            bridge.destroy_node()
        diag_node.destroy_node()
        if head_bridge is not None:
            head_bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
