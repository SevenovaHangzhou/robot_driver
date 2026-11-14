"""Chassis backend of the operator web: controller_manager state, steering parameters, enable."""

from __future__ import annotations

import asyncio
import json
import math
import threading
import time

from controller_manager_msgs.srv import ListHardwareComponents
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import robot_interfaces_qos
from robot_rt_control_interfaces.srv import RtEnable
from std_msgs.msg import String

from .teleop import TeleopError
from .tuning import STEERING_PARAMETERS, TuningError

LIMIT_PARAMETERS = (
    "max_steering_slew_limit",
    "max_steering_acceleration_limit",
    "max_steering_jerk_limit",
)
# Configure-time drive mode of the swerve controller (shown only, never set here).
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
    except asyncio.TimeoutError:
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
        self.last_error = ""
        self.pending = {}
        self.capability = None
        self.steering_motion = {}
        self.steering_motion_at = -math.inf
        controller = node.config.controller_node
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
        self.enable_clients = {
            "enable": node.create_client(RtEnable, "/rt/enable"),
            "disable": node.create_client(RtEnable, "/rt/disable"),
        }
        self.reset_client = node.create_client(RtEnable, "/rt/reset_fault")
        self.hardware_client = node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )
        self.swerve_client = node.create_client(GetParameters, controller + "/get_parameters")
        self.steering_set = node.create_client(SetParameters, controller + "/set_parameters")
        self.enable_params = node.create_client(GetParameters, "/enable_manager/get_parameters")
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
            return {
                "backend_kind": self.kind if now - self.hardware_at < STALE_S else "disconnected",
                "software_test_only": self.software_test_only,
                "connected": connected,
                "enabled": self.enabled if connected else None,
                "busy": self.busy,
                "can_enable": connected and self.state in ("IDLE", "ENABLED") and not self.busy,
                "state": self.state if diagnostic_fresh else "DISCONNECTED",
                "stage": self.stage,
                "message": reason,
                "last_error": self.last_error,
                "command_topic": self.node.config.command_topic,
                "enable_service": "/rt/enable",
                "can_reset": diagnostic_fresh
                and self.state == "FAILED"
                and not self.busy
                and self.reset_client.service_is_ready(),
                "can_tune_steering": connected
                and self.state in ("IDLE", "FAILED")
                and not self.busy
                and bool(steering)
                and self.tuning is not None,
                "steering": steering,
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
            and self.node.publisher.get_subscription_count() > 0
        )

    # ------------------------------------------------------------------ operations
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
        try:
            result = await call_service(self.enable_clients[op], RtEnable.Request(), 45)
            message = result.error.message
            with self.lock:
                if result.ok:
                    self.enabled = op == "enable"
                    self.state = "ENABLED" if self.enabled else "IDLE"
                    self.diag_at = time.monotonic()
                    self.ignore_diag_before = self.node.get_clock().now().nanoseconds
                else:
                    self.last_error = message or "底盘操作被拒绝"
                    self.enabled = None
                    self.diag_at = -math.inf
            return {
                "response_received": True,
                "ok": result.ok,
                "enabled": self.enabled,
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
                "response_received": True,
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
