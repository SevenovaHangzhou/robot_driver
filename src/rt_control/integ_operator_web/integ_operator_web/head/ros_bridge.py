"""ROS 2 side of the head page: public capability, joint states, diagnostics, enable services."""

from __future__ import annotations

import json
import threading
import time
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from ..arm.model import ModelError, load_arm_model
from ..enable_services import EnableServices
from .console import HeadConsole, HeadError

CAPABILITY = "head_position"
ENABLE_DIAGNOSTIC = "/robot/rt_control/enable_manager"


class HeadBridge(Node):
    """Feeds ``HeadConsole`` and publishes its targets on the declared command topic."""

    def __init__(self, joint_state_topic: str) -> None:
        super().__init__("integ_operator_web_head")
        self.console: Optional[HeadConsole] = None
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._publisher = None
        self._controller_parameters = None
        self._model_pending = False
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
        self._manager_parameters = self.create_client(
            GetParameters, "/controller_manager/get_parameters", callback_group=self._group
        )
        services = EnableServices(self, self._group, HeadError)
        self.set_enabled = services.set_enabled
        self.reset_fault = services.reset_fault
        self.create_timer(0.5, self._refresh, callback_group=self._group)

    def publish(self, positions: list[float]) -> None:
        with self._lock:
            publisher = self._publisher
        if publisher is None:
            raise HeadError("capability_missing", "运行栈未声明头部位置能力")
        publisher.publish(Float64MultiArray(data=list(positions)))

    def _capabilities(self, message: String) -> None:
        try:
            document = json.loads(message.data)
            item = {entry["name"]: entry for entry in document["capabilities"]}[CAPABILITY]
            capability = {
                "topic": item["topic"],
                "controller": item["controller"],
                "mock": bool(document["use_mock_hardware"]),
                "scope": document.get("control_scope", ""),
            }
        except (ValueError, KeyError, TypeError):
            capability = None
        with self._lock:
            if capability is not None and self._publisher is None:
                self._publisher = self.create_publisher(Float64MultiArray, capability["topic"], 10)
                self._controller_parameters = self.create_client(
                    GetParameters,
                    f"/{capability['controller']}/get_parameters",
                    callback_group=self._group,
                )
            elif capability is not None and capability["topic"] != self._publisher.topic_name:
                self.get_logger().error("head capability changed; restart the operator web")
                return
        if self.console is not None:
            self.console.update(capability=capability)

    def _refresh(self) -> None:
        console, client = self.console, self._controller_parameters
        if console is None or console.joints or client is None or self._model_pending:
            return
        if not client.service_is_ready() or not self._manager_parameters.service_is_ready():
            return
        self._model_pending = True

        def received_model(joints, response) -> None:
            self._model_pending = False
            urdf = response.result().values[0].string_value if response.result().values else ""
            try:
                model = load_arm_model(urdf, joints)
            except ModelError as exc:
                console.update(model_error=str(exc))
                return
            console.set_model(joints, {j.name: (j.lower, j.upper) for j in model.joints})

        def received_joints(response) -> None:
            values = response.result().values
            joints = list(values[0].string_array_value) if values else []
            if not joints:
                self._model_pending = False
                return
            future = self._manager_parameters.call_async(
                GetParameters.Request(names=["robot_description"])
            )
            future.add_done_callback(lambda done: received_model(joints, done))

        client.call_async(GetParameters.Request(names=["joints"])).add_done_callback(
            received_joints
        )

    def _joint_states(self, message: JointState) -> None:
        if self.console is None or len(message.name) != len(message.position):
            return
        positions = {**self.console.positions, **dict(zip(message.name, message.position))}
        self.console.update(positions=positions, feedback_at=time.monotonic())

    def _diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != ENABLE_DIAGNOSTIC or self.console is None:
                continue
            values = {item.key: item.value for item in status.values}
            allowed = [v for k, v in values.items() if k.endswith(".motion_allowed")]
            faults = [v for k, v in values.items() if k.endswith(".fault_latched")]
            self.console.update(
                enable_state=values.get("state", "UNKNOWN"),
                motion_allowed=bool(allowed) and all(v == "true" for v in allowed),
                device_fault=any(v == "true" for v in faults),
                enable_at=time.monotonic(),
            )
