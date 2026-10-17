"""ROS 2 side of the arm console: public topics/services/actions only."""

from __future__ import annotations

import json
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
from robot_rt_control_interfaces.srv import SetControlEnabled
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from .console import ArmConsole, ConsoleError
from .model import ModelError, load_arm_model
from .trajectory import TrajectoryPoint

CAPABILITY = "arm_joint_trajectory"
ENABLE_DIAGNOSTIC = "/robot/rt_control/enable_manager"
ENABLE_TIMEOUT_S = 95.0


def _duration(seconds: float) -> Duration:
    nanoseconds = int(round(seconds * 1e9))
    return Duration(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


class RosBridge(Node):
    """Feeds ``ArmConsole`` and implements its ``Driver`` protocol."""

    def __init__(self, joint_state_topic: str) -> None:
        super().__init__("rt_control_arm_console")
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
        self._enable = self.create_client(
            SetControlEnabled, "/control/set_enabled", callback_group=self._group
        )
        self._controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers", callback_group=self._group
        )
        self._manager_parameters = self.create_client(
            GetParameters, "/controller_manager/get_parameters", callback_group=self._group
        )
        self._controller_parameters = None
        self.create_timer(0.5, self._refresh, callback_group=self._group)

    # ------------------------------------------------------------------ discovery
    def _capabilities(self, message: String) -> None:
        try:
            document = json.loads(message.data)
            items = {item["name"]: item for item in document["capabilities"]}
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
        if self.console is not None:
            if capability is not None:
                self.console.controller = capability["controller"]
            self.console.update(
                capability=capability, mock=None if capability is None else capability["mock"]
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
        if self.console.model is None and self._controller_parameters is not None:
            self._request(
                "joints",
                self._controller_parameters,
                GetParameters.Request(names=["joints"]),
                self._received_joints,
            )

    def _received_controllers(self, response) -> None:
        active = any(
            c.name == self._controller and c.state == "active" for c in response.controller
        )
        self.console.update(controller_active=active, controller_at=time.monotonic())

    def _received_joints(self, response) -> None:
        joints = list(response.values[0].string_array_value) if response.values else []
        if not joints:
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
    def _joint_states(self, message: JointState) -> None:
        if self.console is None or len(message.name) != len(message.position):
            return
        positions = dict(zip(message.name, message.position))
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
    ) -> None:
        action = self._action
        if action is None or not action.server_is_ready():
            raise RuntimeError("轨迹 action 不可用")
        goal = FollowJointTrajectory.Goal()
        # Explicit start time: the controller and the console share one timeline.
        stamp = self.get_clock().now() + RosDuration(seconds=start - time.monotonic())
        goal.trajectory.header.stamp = stamp.to_msg()
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

        def accepted(future) -> None:
            handle = future.result()
            if not handle.accepted:
                done(False, "控制器拒绝轨迹（起点不一致或参数不符）")
                return
            with self._lock:
                self._goal = handle

            def finished(result_future) -> None:
                wrapped = result_future.result()
                result = wrapped.result
                ok = wrapped.status == 4 and result.error_code == 0  # STATUS_SUCCEEDED
                done(
                    ok,
                    result.error_string
                    or f"status={wrapped.status} error_code={result.error_code}",
                )

            handle.get_result_async().add_done_callback(finished)

        action.send_goal_async(goal).add_done_callback(accepted)

    def cancel(self) -> None:
        with self._lock:
            handle = self._goal
        if handle is not None:
            handle.cancel_goal_async()

    # ------------------------------------------------------------------ enable
    def set_enabled(self, enabled: bool) -> dict:
        if not self._enable.service_is_ready():
            raise ConsoleError("enable_unavailable", "使能服务不可用")
        request = SetControlEnabled.Request(enabled=enabled, reason="ELECTRI-127 arm console")
        event = threading.Event()
        future = self._enable.call_async(request)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(ENABLE_TIMEOUT_S):
            self._enable.remove_pending_request(future)
            raise ConsoleError("enable_timeout", "使能服务响应超时，请查看实际状态")
        result = future.result()
        return {
            "accepted": bool(result.accepted),
            "enabled": bool(result.enabled),
            "message": result.error.message,
        }


def spin_in_background(node: Node) -> threading.Thread:
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, name="ros", daemon=True)
    thread.start()
    return thread
