from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Callable, Sequence


EXECUTION_UNKNOWN = 0
EXECUTION_IDLE = 1
EXECUTION_EXECUTING = 2
EXECUTION_STOPPING = 3
EXECUTION_HOLDING = 4
EXECUTION_FAULT = 5


def require_preview_only(configured: bool) -> None:
    """The draft has no commissioned actuator stop/hold or lifecycle backend."""
    if configured:
        raise ValueError(
            "position execution is not admitted: controller ownership, stop confirmation "
            "and lifecycle backend must be implemented before configured=true"
        )


@dataclass(frozen=True)
class PositionFeedback:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    valid: bool
    received_monotonic: float
    received_ros_ns: int


class PositionResourceCore:
    def __init__(
        self,
        name: str,
        joint_names: Sequence[str],
        *,
        configured: bool,
        tolerance: float,
        velocity_tolerance: float,
        feedback_timeout_s: float,
        settle_samples: int,
        command_timeout_s: float = math.nan,
        minimum_position: float = math.nan,
        maximum_position: float = math.nan,
    ) -> None:
        if not name or not joint_names or any(not item for item in joint_names):
            raise ValueError("resource and joint names must be explicit")
        numeric = (tolerance, velocity_tolerance, feedback_timeout_s, command_timeout_s)
        if configured and not all(
            math.isfinite(item) and item > 0.0 for item in numeric
        ):
            raise ValueError("configured resources require positive tolerances and timeouts")
        if configured and settle_samples < 1:
            raise ValueError("configured resources require positive settle_samples")
        if configured and (
            not math.isfinite(minimum_position)
            or not math.isfinite(maximum_position)
            or minimum_position >= maximum_position
        ):
            raise ValueError("configured resources require finite ordered position limits")
        self.name = name
        self.joint_names = tuple(joint_names)
        self.configured = bool(configured)
        self.tolerance = float(tolerance)
        self.velocity_tolerance = float(velocity_tolerance)
        self.feedback_timeout_s = float(feedback_timeout_s)
        self.settle_samples = int(settle_samples)
        self.command_timeout_s = float(command_timeout_s)
        self.minimum_position = float(minimum_position)
        self.maximum_position = float(maximum_position)
        self._lock = threading.Lock()
        self._active = False
        self._feedback = PositionFeedback((), (), False, -math.inf, 0)
        self._goal_id: bytes | None = None
        self._state = EXECUTION_IDLE if configured else EXECUTION_UNKNOWN

    def validate_target(self, target: Sequence[float]) -> tuple[float, ...]:
        values = tuple(float(value) for value in target)
        if len(values) != len(self.joint_names):
            raise ValueError("target dimension does not match resource joints")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("target positions must be finite")
        if not self.configured:
            raise ValueError(f"{self.name} is not configured")
        if any(
            value < self.minimum_position or value > self.maximum_position
            for value in values
        ):
            raise ValueError("target position is outside configured limits")
        return values

    def try_start(self, goal_id: bytes) -> bool:
        with self._lock:
            if self._active or not self.configured:
                return False
            self._active = True
            self._goal_id = bytes(goal_id)
            self._state = EXECUTION_EXECUTING
            return True

    def finish(self, state: int) -> None:
        with self._lock:
            self._state = state
            self._active = False
            self._goal_id = None

    def set_state(self, state: int) -> None:
        with self._lock:
            self._state = state

    def update_feedback(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        received: float,
        received_ros_ns: int,
    ) -> None:
        position_values = tuple(float(value) for value in positions)
        velocity_values = tuple(float(value) for value in velocities)
        valid = (
            len(position_values) == len(self.joint_names)
            and all(math.isfinite(value) for value in position_values)
            and (not velocity_values or len(velocity_values) == len(self.joint_names))
            and all(math.isfinite(value) for value in velocity_values)
        )
        with self._lock:
            self._feedback = PositionFeedback(
                position_values,
                velocity_values,
                valid,
                float(received),
                int(received_ros_ns),
            )

    def feedback(self) -> PositionFeedback:
        with self._lock:
            return self._feedback

    def state(self) -> tuple[int, bytes | None]:
        with self._lock:
            return self._state, self._goal_id

    def feedback_is_fresh(self, now_monotonic: float) -> bool:
        feedback = self.feedback()
        age = float(now_monotonic) - feedback.received_monotonic
        return feedback.valid and 0.0 <= age <= self.feedback_timeout_s

    def target_reached(self, target: Sequence[float], now_monotonic: float) -> bool:
        feedback = self.feedback()
        if not self.feedback_is_fresh(now_monotonic):
            return False
        if any(
            abs(actual - expected) > self.tolerance
            for actual, expected in zip(feedback.positions, target)
        ):
            return False
        return bool(feedback.velocities) and all(
            abs(value) <= self.velocity_tolerance for value in feedback.velocities
        )


def main(args=None) -> None:
    import rclpy
    from robot_interfaces_qos import state
    from robot_rt_control_interfaces.action import MoveAxis, MoveHead
    from robot_rt_control_interfaces.msg import MotionExecutionState, ObservationMeta
    from robot_system_interfaces.msg import ErrorInfo
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    def set_uuid(message, value: bytes | uuid.UUID) -> None:
        raw = value.bytes if isinstance(value, uuid.UUID) else bytes(value)
        message.uuid = list(raw[:16].ljust(16, b"\0"))

    def fill_error(message, code: int, text: str, retryable: bool) -> None:
        message.code = int(code)
        message.message = text
        message.retryable = retryable
        message.severity = ErrorInfo.OK if code == 0 else ErrorInfo.FAULT
        message.source = "rt_control"
        message.detail = ""

    class PositionActionAdapter(Node):
        def __init__(self) -> None:
            super().__init__("position_action_adapter")
            self.declare_parameter("joint_state_topic", "/joint_states")
            self.declare_parameter("state_topic", "/rt_control/motion/state")
            self.declare_parameter("feedback_publish_period_s", 0.05)
            self._callback_group = ReentrantCallbackGroup()
            self._instance_id = uuid.uuid4()
            self._resources = {
                "left_pp": self._axis_resource("left_pp", "left_moving_jaw_joint"),
                "right_pp": self._axis_resource("right_pp", "right_moving_jaw_joint"),
                "updown": self._axis_resource("updown", "updown"),
                "head": self._head_resource(),
            }
            for resource in self._resources.values():
                require_preview_only(resource.configured)
            self._commands = {}
            for name, resource in self._resources.items():
                if not resource.configured:
                    continue
                topic = str(self.get_parameter(name + ".command_topic").value)
                self._commands[name] = self.create_publisher(Float64MultiArray, topic, 1)
            self._state_publisher = self.create_publisher(
                MotionExecutionState,
                str(self.get_parameter("state_topic").value),
                state(),
            )
            self._joint_subscription = self.create_subscription(
                JointState,
                str(self.get_parameter("joint_state_topic").value),
                self._on_joint_state,
                state(),
                callback_group=self._callback_group,
            )
            self._servers = [
                self._axis_server("left_pp", "/pp/left/move"),
                self._axis_server("right_pp", "/pp/right/move"),
                self._axis_server("updown", "/updown/move"),
                self._head_server("head", "/head/move"),
            ]
            self._timer = self.create_timer(0.1, self._publish_resource_states)

        def _common_parameters(self, name: str) -> dict:
            self.declare_parameter(name + ".command_timeout_s", math.nan)
            return {
                "configured": self.declare_parameter(name + ".configured", False).value,
                "tolerance": self.declare_parameter(name + ".tolerance", math.nan).value,
                "velocity_tolerance": self.declare_parameter(
                    name + ".velocity_tolerance", math.nan
                ).value,
                "feedback_timeout_s": self.declare_parameter(
                    name + ".feedback_timeout_s", math.nan
                ).value,
                "settle_samples": self.declare_parameter(
                    name + ".settle_samples", 0
                ).value,
                "command_timeout_s": self.get_parameter(
                    name + ".command_timeout_s"
                ).value,
                "minimum_position": self.declare_parameter(
                    name + ".minimum_position", math.nan
                ).value,
                "maximum_position": self.declare_parameter(
                    name + ".maximum_position", math.nan
                ).value,
            }

        def _axis_resource(self, name: str, joint_name: str) -> PositionResourceCore:
            self.declare_parameter(name + ".command_topic", f"/{name}_controller/commands")
            return PositionResourceCore(name, (joint_name,), **self._common_parameters(name))

        def _head_resource(self) -> PositionResourceCore:
            name = "head"
            self.declare_parameter(
                name + ".command_topic", "/head_position_controller/commands"
            )
            return PositionResourceCore(
                name,
                ("head_joint", "head_pitch_joint"),
                **self._common_parameters(name),
            )

        def _axis_server(self, name: str, action_name: str):
            return ActionServer(
                self,
                MoveAxis,
                action_name,
                execute_callback=lambda handle: self._execute_axis(name, handle),
                goal_callback=lambda goal: self._goal_response(
                    name, (goal.target_position_m,)
                ),
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self._callback_group,
            )

        def _head_server(self, name: str, action_name: str):
            return ActionServer(
                self,
                MoveHead,
                action_name,
                execute_callback=lambda handle: self._execute_head(name, handle),
                goal_callback=lambda goal: self._goal_response(
                    name,
                    (goal.head_joint_position_rad, goal.head_pitch_joint_position_rad),
                ),
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self._callback_group,
            )

        def _goal_response(self, name: str, target: Sequence[float]):
            resource = self._resources[name]
            try:
                resource.validate_target(target)
            except ValueError as exc:
                self.get_logger().error(f"reject {name} goal: {exc}")
                return GoalResponse.REJECT
            state_value, goal_id = resource.state()
            if goal_id is not None or state_value in (EXECUTION_EXECUTING, EXECUTION_STOPPING):
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

        def _on_joint_state(self, message: JointState) -> None:
            index = {joint: position for position, joint in enumerate(message.name)}
            received = time.monotonic()
            received_ros_ns = self.get_clock().now().nanoseconds
            for resource in self._resources.values():
                if not all(joint in index for joint in resource.joint_names):
                    continue
                positions = tuple(message.position[index[joint]] for joint in resource.joint_names)
                velocities = ()
                if message.velocity and len(message.velocity) == len(message.name):
                    velocities = tuple(
                        message.velocity[index[joint]] for joint in resource.joint_names
                    )
                resource.update_feedback(
                    positions, velocities, received, received_ros_ns
                )

        def _execute_axis(self, name, goal_handle):
            target = (float(goal_handle.request.target_position_m),)
            result = MoveAxis.Result()
            self._execute_resource(
                name,
                goal_handle,
                target,
                result.final_state,
                lambda state_message: self._axis_feedback(goal_handle, state_message),
                result.error,
            )
            return result

        def _execute_head(self, name, goal_handle):
            target = (
                float(goal_handle.request.head_joint_position_rad),
                float(goal_handle.request.head_pitch_joint_position_rad),
            )
            result = MoveHead.Result()
            self._execute_resource(
                name,
                goal_handle,
                target,
                result.final_state,
                lambda state_message: self._head_feedback(goal_handle, state_message),
                result.error,
            )
            return result

        @staticmethod
        def _axis_feedback(goal_handle, state_message) -> None:
            feedback = MoveAxis.Feedback()
            feedback.state = state_message
            goal_handle.publish_feedback(feedback)

        @staticmethod
        def _head_feedback(goal_handle, state_message) -> None:
            feedback = MoveHead.Feedback()
            feedback.state = state_message
            goal_handle.publish_feedback(feedback)

        def _execute_resource(
            self,
            name,
            goal_handle,
            target,
            final_state,
            feedback_callback: Callable,
            result_error,
        ) -> None:
            resource = self._resources[name]
            goal_id = bytes(goal_handle.goal_id.uuid)
            if not resource.try_start(goal_id):
                fill_error(result_error, 1102, f"{name} is busy or unavailable", True)
                goal_handle.abort()
                return
            self._publish_command(name, target)
            deadline = time.monotonic() + resource.command_timeout_s
            settled = 0
            last_settled_sample = None
            canceling = False
            active_target = target
            terminal_state = EXECUTION_FAULT
            terminal_error = (2, "position action timed out", False)
            while rclpy.ok():
                now_monotonic = time.monotonic()
                if goal_handle.is_cancel_requested and not canceling:
                    resource.set_state(EXECUTION_STOPPING)
                    current = resource.feedback()
                    if not resource.feedback_is_fresh(now_monotonic):
                        terminal_error = (
                            1130,
                            "cannot confirm cancel stop: position feedback is unavailable",
                            True,
                        )
                        goal_handle.abort()
                        break
                    active_target = current.positions
                    self._publish_command(name, active_target)
                    canceling = True
                    settled = 0
                    last_settled_sample = current.received_monotonic
                if not resource.feedback_is_fresh(now_monotonic):
                    terminal_state = EXECUTION_FAULT
                    terminal_error = (1130, "position feedback is unavailable or stale", True)
                    goal_handle.abort()
                    break
                if resource.target_reached(active_target, now_monotonic):
                    sample_id = resource.feedback().received_monotonic
                    if sample_id != last_settled_sample:
                        settled += 1
                        last_settled_sample = sample_id
                    if settled >= resource.settle_samples:
                        terminal_state = EXECUTION_HOLDING
                        if canceling:
                            terminal_error = (
                                80,
                                "position action canceled after stop confirmation",
                                False,
                            )
                            goal_handle.canceled()
                        else:
                            terminal_error = (0, "target reached", False)
                            goal_handle.succeed()
                        break
                else:
                    settled = 0
                if now_monotonic >= deadline:
                    current = resource.feedback()
                    if current.valid:
                        self._publish_command(name, current.positions)
                    goal_handle.abort()
                    break
                state_message = self._state_message(resource)
                feedback_callback(state_message)
                time.sleep(float(self.get_parameter("feedback_publish_period_s").value))
            resource.finish(terminal_state)
            completed = self._state_message(resource)
            final_state.header = completed.header
            final_state.resource_id = completed.resource_id
            final_state.joint_names = completed.joint_names
            final_state.actual_positions = completed.actual_positions
            final_state.feedback_observation = completed.feedback_observation
            final_state.execution_state = completed.execution_state
            final_state.stop_confirmed = completed.stop_confirmed
            final_state.stop_confirmation_valid = completed.stop_confirmation_valid
            final_state.goal_known = completed.goal_known
            final_state.goal_id = completed.goal_id
            fill_error(final_state.error, *terminal_error)
            fill_error(result_error, *terminal_error)

        def _publish_command(self, name: str, target: Sequence[float]) -> None:
            command = Float64MultiArray()
            command.data = [float(value) for value in target]
            self._commands[name].publish(command)

        def _state_message(self, resource: PositionResourceCore):
            message = MotionExecutionState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.resource_id = resource.name
            message.joint_names = list(resource.joint_names)
            feedback = resource.feedback()
            message.actual_positions = list(feedback.positions) if feedback.valid else []
            message.feedback_observation.stamp.sec = int(
                feedback.received_ros_ns // 1_000_000_000
            )
            message.feedback_observation.stamp.nanosec = int(
                feedback.received_ros_ns % 1_000_000_000
            )
            message.feedback_observation.time_source = ObservationMeta.TIME_SOURCE_HOST_RECEIVE
            message.feedback_observation.valid = resource.feedback_is_fresh(time.monotonic())
            set_uuid(message.feedback_observation.source_instance_id, self._instance_id)
            state_value, goal_id = resource.state()
            message.execution_state = state_value
            message.stop_confirmation_valid = (
                message.feedback_observation.valid
                and len(feedback.velocities) == len(resource.joint_names)
            )
            message.stop_confirmed = message.stop_confirmation_valid and all(
                abs(value) <= resource.velocity_tolerance for value in feedback.velocities
            )
            message.goal_known = goal_id is not None
            if goal_id is not None:
                set_uuid(message.goal_id, goal_id)
            fill_error(
                message.error,
                0 if message.feedback_observation.valid else 1130,
                "" if message.feedback_observation.valid else "position feedback unavailable",
                not message.feedback_observation.valid,
            )
            return message

        def _publish_resource_states(self) -> None:
            for resource in self._resources.values():
                self._state_publisher.publish(self._state_message(resource))

    rclpy.init(args=args)
    node = PositionActionAdapter()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
