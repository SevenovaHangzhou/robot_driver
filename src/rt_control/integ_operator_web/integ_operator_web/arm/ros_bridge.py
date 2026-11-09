"""ROS 2 side of the arm console: public topics/services/actions only."""

from __future__ import annotations

import json
from functools import partial
import threading
import time
from typing import Callable, Optional, Sequence

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RosDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import FluidPressure, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from ..enable_services import EnableServices
from .console import ArmConsole, ConsoleError
from .model import ModelError, load_arm_model
from .trajectory import TrajectoryPoint
from .readback import ArmReadback
from .mode_service import ModeService
from .modes import ArmModes
from .rolling_hold import RollingHold
from .rolling_rejection import RollingRejections

CAPABILITY = "arm_joint_trajectory"
ROLLING_PARAMETERS = ["joints", "joint_position_lower", "joint_position_upper", "joint_velocity_limits",
                      "joint_acceleration_limits", "position_margin",
                      "buffer_capacity", "max_horizon_ms", "update_timeout_ms", "replace_lead_ms"]
ENABLE_DIAGNOSTIC = "/robot/rt_control/enable_manager"


def _duration(seconds: float) -> Duration:
    nanoseconds = int(round(seconds * 1e9))
    return Duration(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


class RosBridge(Node):
    """Feeds ``ArmConsole`` and implements its ``Driver`` protocol."""

    def __init__(self, joint_state_topic: str) -> None:
        super().__init__("integ_operator_web_arm")
        self.console: Optional[ArmConsole] = None
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._goal = None
        self._action: Optional[ActionClient] = None
        self._action_topic = ""
        self._controller = ""
        self._pending = {}
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/rt/capabilities", self._capabilities, latched, callback_group=self._group
        )
        self.create_subscription(
            JointState, joint_state_topic, self._joint_states, 10, callback_group=self._group
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._diagnostics, 10, callback_group=self._group
        )
        # Latest display sample only; best effort also matches reliable publishers.
        pressure_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for side in ("left", "right"):
            self.create_subscription(
                FluidPressure, f"/rt/vacuum/{side}/pressure",
                partial(self._vacuum_pressure, side),
                pressure_qos, callback_group=self._group,
            )
        services = EnableServices(self, self._group, ConsoleError)
        self.set_enabled = services.set_enabled
        self.reset_fault = services.reset_fault
        self._controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers", callback_group=self._group
        )
        self._manager_parameters = self.create_client(
            GetParameters, "/controller_manager/get_parameters", callback_group=self._group
        )
        self._controller_parameters = None
        self._readback = None
        self._mode_parameters = None
        self._rolling_parameters = None
        self.rolling_hold = None
        self.rolling_rejections = None
        self.rolling_parameters = None
        self.rolling_parameters_at = -float("inf")
        self.create_timer(0.5, self._refresh, callback_group=self._group)

    def attach_console(self, console):
        self.console = console
        self._readback = ArmReadback(self, self._group, console)
        mode = ModeService(self)
        console.modes = ArmModes(console, mode.switch, mode.available, mode.matches)

    # ------------------------------------------------------------------ discovery
    def _capabilities(self, message: String) -> None:
        scope_has_arms = None
        try:
            document = json.loads(message.data)
            items = {item["name"]: item for item in document["capabilities"]}
            scope_has_arms = CAPABILITY in items
            arm = items[CAPABILITY]
            capability = {
                "topic": arm["topic"],
                "controller": arm["controller"],
                "mock": bool(document["use_mock_hardware"]),
                "scope": document.get("control_scope", ""),
            }
        except (ValueError, KeyError, TypeError):
            capability = None
        with self._lock:
            if capability and capability["topic"] != self._action_topic:
                if self._action_topic:
                    self.get_logger().error("arm capability changed; restart the console")
                    return
                self._action_topic = capability["topic"]
                self._controller = capability["controller"]
                self._action = ActionClient(
                    self, FollowJointTrajectory, self._action_topic, callback_group=self._group
                )
                self._controller_parameters = self.create_client(
                    GetParameters, f"/{self._controller}/get_parameters", callback_group=self._group
                )
                self._readback.bind(self._controller, self._action_topic)
        if self.console is not None:
            if capability is not None:
                self.console.controller = capability["controller"]
            self.console.update(
                capability=capability, mock=None if capability is None else capability["mock"],
                scope_has_arms=scope_has_arms
            )

    def _request(self, key: str, client, request, callback: Callable) -> None:
        prior = self._pending.get(key)
        if prior is not None and not prior[0].done():
            if time.monotonic() - prior[1] < 3.0:
                return
            client.remove_pending_request(prior[0])
        if client is None or not client.service_is_ready():
            return
        future = client.call_async(request)
        self._pending[key] = (future, time.monotonic())

        def finished(done) -> None:
            try:
                callback(done.result())
            except Exception as exc:  # noqa: BLE001 - logged; the next refresh retries
                self.get_logger().warning(f"{key}: {exc}")

        future.add_done_callback(finished)

    def _refresh(self) -> None:
        if self.console is None:
            return
        self._request(
            "controllers", self._controllers, ListControllers.Request(), self._received_controllers
        )
        if self._mode_parameters is not None:
            self._request("stationary_policy", self._mode_parameters,
                          GetParameters.Request(names=["motion_joints", "mode_switch_stable_velocity_thresholds",
                                                       "mode_switch_stable_interval_count"]), self._stationary_policy)
        if self._rolling_parameters is not None:
            self._request("rolling_parameters", self._rolling_parameters,
                          GetParameters.Request(names=ROLLING_PARAMETERS), self._received_rolling_parameters)
        if self._controller_parameters is not None:
            self._request(
                "joints",
                self._controller_parameters,
                GetParameters.Request(names=["joints", "constraints.stopped_velocity_tolerance"]),
                self._received_joints,
            )

    def _received_controllers(self, response) -> None:
        for item in response.controller:
            if item.type.endswith("/EnableManagerController") and self._mode_parameters is None:
                self._mode_parameters = self.create_client(
                    GetParameters, f"/{item.name}/get_parameters", callback_group=self._group)
            if item.type.endswith("/RollingTrajectoryController") and self._rolling_parameters is None:
                self._rolling_parameters = self.create_client(
                    GetParameters, f"/{item.name}/get_parameters", callback_group=self._group)
                self.rolling_hold = RollingHold(self, item.name, self._group)
                self.rolling_rejections = RollingRejections(self, item.name, self._group)
        self._readback.evidence.controllers(response.controller, self._controller)
        active = any(
            c.name == self._controller and c.state == "active" for c in response.controller
        )
        self.console.update(controller_active=active, controller_at=time.monotonic())

    def _stationary_policy(self, response):
        values = response.values
        if len(values) == 3 and [v.type for v in values] == [9, 8, 2]:
            self._readback.evidence.set_stationary_policy(
                values[0].string_array_value, values[1].double_array_value, values[2].integer_value)

    def _received_rolling_parameters(self, response):
        fields = {2: "integer_value", 3: "double_value", 8: "double_array_value", 9: "string_array_value"}
        if len(response.values) == len(ROLLING_PARAMETERS) and all(v.type in fields for v in response.values):
            self.rolling_parameters = {n: list(getattr(v, fields[v.type])) if v.type in (8, 9)
                                       else getattr(v, fields[v.type]) for n, v in zip(ROLLING_PARAMETERS, response.values)}
            self.rolling_parameters_at = time.monotonic()

    def _received_joints(self, response) -> None:
        joints = list(response.values[0].string_array_value) if response.values else []
        if not joints:
            return
        value = response.values[1] if len(response.values) > 1 else None
        self._readback.evidence.set_tolerance(value.double_value if value and value.type == 3 else None)
        if self.console.model is not None:
            if tuple(joints) != self.console.joint_names:
                self._readback.evidence.unknown = True
            return

        def received_model(model_response) -> None:
            urdf = model_response.values[0].string_value if model_response.values else ""
            try:
                self.console.set_model(load_arm_model(urdf, joints))
                self.get_logger().info(f"arm model loaded: {', '.join(joints)}")
            except (ModelError, ConsoleError) as exc:
                self.console.model_error = str(exc)

        self._request(
            "model",
            self._manager_parameters,
            GetParameters.Request(names=["robot_description"]),
            received_model,
        )

    # ------------------------------------------------------------------ feedback
    def _vacuum_pressure(self, side: str, message: FluidPressure) -> None:
        # R-OUT-05L/R: gauge Pa, negative for vacuum; invalid samples are never published.
        if self.console is not None:
            self.console.update_vacuum_pressure(side, message.fluid_pressure / 1000.0, True, "", "kPa")

    def _joint_states(self, message: JointState) -> None:
        if self.console is None or len(message.name) != len(message.position):
            return
        positions = dict(zip(message.name, message.position))
        self._readback.evidence.measured(message.name, message.position,
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)
        self.console.update(positions=positions, feedback_at=time.monotonic())

    def _diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == ENABLE_DIAGNOSTIC and self.console is not None:
                values = {item.key: item.value for item in status.values}
                self.console.update(
                    enable_state=values.get("state", "UNKNOWN"),
                    enable_stage=values.get("stage", ""),
                    enable_at=time.monotonic(),
                )

    # ------------------------------------------------------------------ Driver
    def send(
        self,
        joints: Sequence[str],
        points: Sequence[TrajectoryPoint],
        done: Callable[[bool, str], None],
        start: float,
        *,
        start_delay: float = 0.0,
    ) -> float:
        action = self._action
        if action is None or not action.server_is_ready():
            raise RuntimeError("轨迹 action 不可用")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        for point in points:
            goal.trajectory.points.append(
                JointTrajectoryPoint(
                    positions=list(point.positions),
                    velocities=list(point.velocities),
                    accelerations=list(point.accelerations),
                    time_from_start=_duration(point.time),
                )
            )
        # Allocate and populate points before reserving the startup lead. Stop
        # trajectories retain their existing absolute timeline, even if past.
        if start_delay:
            start = max(start, time.monotonic() + start_delay)
        stamp = self.get_clock().now() + RosDuration(seconds=start - time.monotonic())
        goal.trajectory.header.stamp = stamp.to_msg()

        def accepted(future) -> None:
            try:
                handle = future.result()
            except Exception as exc:
                self._readback.evidence.resolved(unknown=True)
                done(False, f'轨迹接收结果未确认：{exc}')
                return
            if not handle.accepted:
                self._readback.evidence.resolved()
                done(False, "控制器拒绝轨迹（请查JTC日志：点数上限、起点一致性或参数校验）")
                return
            with self._lock:
                self._goal = handle

            def finished(result_future) -> None:
                try:
                    wrapped = result_future.result()
                except Exception as exc:
                    self._readback.evidence.resolved(unknown=True)
                    done(False, f'轨迹结束结果未确认：{exc}')
                    return
                result = wrapped.result
                ok = wrapped.status == 4 and result.error_code == 0  # STATUS_SUCCEEDED
                self._readback.evidence.resolved()
                done(
                    ok,
                    result.error_string
                    or f"status={wrapped.status} error_code={result.error_code}",
                )

            try:
                handle.get_result_async().add_done_callback(finished)
            except Exception as exc:
                self._readback.evidence.resolved(unknown=True)
                done(False, f'轨迹结束请求结果未确认：{exc}')

        self._readback.evidence.submitted()
        try:
            action.send_goal_async(goal).add_done_callback(accepted)
        except Exception:
            self._readback.evidence.resolved(unknown=True)
            raise
        return start

    def cancel(self) -> None:
        with self._lock:
            handle = self._goal
        if handle is not None:
            handle.cancel_goal_async()


def spin_in_background(node: Node) -> threading.Thread:
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, name="ros", daemon=True)
    thread.start()
    return thread
