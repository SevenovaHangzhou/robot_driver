"""R-IN-16 /rt/head/move: one absolute head target per goal over the head position controller.

A busy head rejects a second goal. Cancel, timeout and lost readiness command the measured
position (hold). Speed is limited by the DaMiao hardware, not here.
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
import robot_interfaces_qos
from robot_rt_control_interfaces.action import MoveHead
from robot_rt_control_interfaces.msg import MotionExecutionState, ObservationMeta
from robot_system_interfaces.msg import DomainReadiness, ErrorCode, ErrorInfo
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .motion import arrived, load_head, target_error, time_limit

ACTION = "/rt/head/move"
READINESS_TOPIC = "/rt/readiness"
JOINT_STATES = "/joint_states"
# Same limit as enable_manager's device-offline check (user 2026-11-03: 500 ms).
FEEDBACK_STALE_S = 0.5
PERIOD_S = 0.05


class HeadMoveServer(Node):
    def __init__(self, monotonic=time.monotonic, **options):
        super().__init__("rt_head_move", **options)
        generated_dir = self.declare_parameter("generated_dir", "").value
        topic, self._joints = load_head(generated_dir)
        self._names = [joint.name for joint in self._joints]
        self._monotonic = monotonic
        self._lock = threading.Lock()
        self._positions, self._sample_at, self._stamp = None, -float("inf"), None
        self._ready, self._blockers = False, ["no_readiness"]
        self._busy = False
        group = ReentrantCallbackGroup()
        self._command = self.create_publisher(Float64MultiArray, topic, 10)
        self.create_subscription(
            JointState, JOINT_STATES, self._on_joints, 10, callback_group=group
        )
        self.create_subscription(
            DomainReadiness,
            READINESS_TOPIC,
            self._on_readiness,
            robot_interfaces_qos.latched(),
            callback_group=group,
        )
        ActionServer(
            self,
            MoveHead,
            ACTION,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )

    def _on_joints(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in self._names):
            with self._lock:
                self._positions = [values[name] for name in self._names]
                self._sample_at, self._stamp = self._monotonic(), message.header.stamp

    def _on_readiness(self, message):
        with self._lock:
            self._ready, self._blockers = message.ready, list(message.blockers)

    def _on_goal(self, _goal):
        with self._lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _sample(self):
        with self._lock:
            fresh = self._monotonic() - self._sample_at <= FEEDBACK_STALE_S
            return (list(self._positions) if fresh else None), self._stamp

    def _state(self, execution, positions, stamp):
        state = MotionExecutionState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.resource_id = "head"
        state.joint_names = self._names
        state.execution_state = execution
        state.feedback_observation.time_source = ObservationMeta.TIME_SOURCE_HOST_RECEIVE
        state.feedback_observation.valid = positions is not None
        if positions is not None:
            state.actual_positions = positions
            state.feedback_observation.stamp = stamp
        return state

    def _finish(self, handle, outcome, code, message, execution, positions, stamp):
        result = MoveHead.Result()
        result.final_state = self._state(execution, positions, stamp)
        result.error.code = code
        result.error.message = message
        result.error.source = "rt_control/head_move"
        result.error.retryable = code >= 100 and (code // 100) % 10 == 1  # DREE recovery digit
        if code in (ErrorCode.SUCCESS, ErrorCode.CANCELED):
            result.error.severity = ErrorInfo.OK
        else:
            result.error.severity = ErrorInfo.FAULT if code >= 1000 else ErrorInfo.WARN
        getattr(handle, outcome)()
        self.get_logger().info(f"{ACTION} {outcome}: {message}")
        return result

    def _hold(self, positions):
        if positions is not None:
            self._command.publish(Float64MultiArray(data=positions))

    def _execute(self, handle):
        try:
            return self._run(handle)
        finally:
            with self._lock:
                self._busy = False

    def _run(self, handle):
        idle, holding = MotionExecutionState.EXECUTION_IDLE, MotionExecutionState.EXECUTION_HOLDING
        target = [
            handle.request.head_joint_position_rad,
            handle.request.head_pitch_joint_position_rad,
        ]
        positions, stamp = self._sample()
        invalid = target_error(self._joints, target)
        if invalid is not None:
            return self._finish(
                handle, "abort", ErrorCode.INVALID_GOAL, invalid, idle, positions, stamp
            )
        with self._lock:
            ready, blockers = self._ready, self._blockers
        if not ready:
            message = f"RT-Control not ready: {','.join(blockers)}"
            return self._finish(
                handle, "abort", ErrorCode.NOT_READY, message, idle, positions, stamp
            )
        if positions is None:
            return self._finish(
                handle,
                "abort",
                ErrorCode.RT_DEVICE_FEEDBACK_STALE,
                "head joint states stale",
                idle,
                positions,
                stamp,
            )
        deadline = self._monotonic() + time_limit(self._joints, positions, target)
        self._command.publish(Float64MultiArray(data=target))
        feedback = MoveHead.Feedback()
        while True:
            time.sleep(PERIOD_S)
            positions, stamp = self._sample()
            with self._lock:
                ready, blockers = self._ready, self._blockers
            if positions is None:
                return self._finish(
                    handle,
                    "abort",
                    ErrorCode.RT_DEVICE_FEEDBACK_STALE,
                    "head joint states stale",
                    idle,
                    positions,
                    stamp,
                )
            if handle.is_cancel_requested:
                self._hold(positions)
                return self._finish(
                    handle,
                    "canceled",
                    ErrorCode.CANCELED,
                    "canceled, holding",
                    holding,
                    positions,
                    stamp,
                )
            if not ready:
                self._hold(positions)
                message = f"RT-Control not ready: {','.join(blockers)}"
                return self._finish(
                    handle, "abort", ErrorCode.NOT_READY, message, holding, positions, stamp
                )
            if arrived(target, positions):
                return self._finish(
                    handle, "succeed", ErrorCode.SUCCESS, "arrived", idle, positions, stamp
                )
            if self._monotonic() > deadline:
                self._hold(positions)
                return self._finish(
                    handle,
                    "abort",
                    ErrorCode.TIMEOUT,
                    "not arrived in time",
                    holding,
                    positions,
                    stamp,
                )
            feedback.state = self._state(MotionExecutionState.EXECUTION_EXECUTING, positions, stamp)
            handle.publish_feedback(feedback)


def main():
    rclpy.init()
    node = HeadMoveServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
