from __future__ import annotations

import enum
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Protocol, Sequence


LEFT = "left"
RIGHT = "right"
CHANNELS = (LEFT, RIGHT)
GRIP = 1
RELEASE = 2
UNVERIFIED = 0
ATTACHED_VERIFIED = 1


class PumpErrorCode(enum.IntEnum):
    OK = 0
    PLC_UNAVAILABLE = 1
    COMMAND_REJECTED = 2
    ACTIVE_VACUUM_COMMAND = 3
    POSSIBLE_LOAD_HELD = 4


@dataclass(frozen=True)
class PlcVacuumSnapshot:
    connected: bool
    data_fresh: bool
    left_attached: bool
    right_attached: bool
    left_valve_open: bool
    right_valve_open: bool
    pump_enabled: bool
    error: str = ""

    def channel_attached(self, channel: str) -> bool:
        if channel == LEFT:
            return self.left_attached
        if channel == RIGHT:
            return self.right_attached
        raise ValueError(f"unsupported vacuum channel: {channel}")

    def channel_valve_open(self, channel: str) -> bool:
        if channel == LEFT:
            return self.left_valve_open
        if channel == RIGHT:
            return self.right_valve_open
        raise ValueError(f"unsupported vacuum channel: {channel}")

    @property
    def any_possible_load_held(self) -> bool:
        return (
            self.left_attached
            or self.right_attached
            or self.left_valve_open
            or self.right_valve_open
        )


@dataclass(frozen=True)
class VacuumChannelStateData:
    channel: str
    attached: bool
    pump_enabled: bool
    valve_commanded_open: bool
    data_fresh: bool


@dataclass(frozen=True)
class VacuumChannelResultData:
    channel: str
    command_accepted: bool
    valve_actuation_completed: bool
    verification_level: int
    error: str = ""


@dataclass(frozen=True)
class VacuumExecutionResult:
    succeeded: bool
    accepted: bool
    overall_verification_level: int
    error: str
    channel_results: tuple[VacuumChannelResultData, ...]


@dataclass(frozen=True)
class PlcCommandResult:
    success: bool
    message: str = ""


@dataclass(frozen=True)
class PumpCommandResult:
    accepted: bool
    enabled: bool
    error_code: PumpErrorCode
    message: str


class VacuumIo(Protocol):
    def set_output(self, output_name: str, enabled: bool) -> PlcCommandResult:
        ...

    def read_snapshot(self) -> PlcVacuumSnapshot:
        ...

    def wait_for_attachment(
        self, channels: Sequence[str], timeout_s: float
    ) -> PlcVacuumSnapshot:
        ...


def normalize_channels(channels: Sequence[str]) -> tuple[str, ...]:
    normalized = tuple(str(channel).strip().lower() for channel in channels)
    if not 1 <= len(normalized) <= 2:
        raise ValueError("channels must contain one or two entries")
    if len(set(normalized)) != len(normalized):
        raise ValueError("channels must be unique")
    invalid = [channel for channel in normalized if channel not in CHANNELS]
    if invalid:
        raise ValueError(f"unsupported vacuum channel(s): {', '.join(invalid)}")
    return normalized


def build_vacuum_channel_states(
    snapshot: PlcVacuumSnapshot,
) -> tuple[VacuumChannelStateData, VacuumChannelStateData]:
    return (
        VacuumChannelStateData(
            channel=LEFT,
            attached=snapshot.left_attached,
            pump_enabled=snapshot.pump_enabled,
            valve_commanded_open=snapshot.left_valve_open,
            data_fresh=snapshot.connected and snapshot.data_fresh,
        ),
        VacuumChannelStateData(
            channel=RIGHT,
            attached=snapshot.right_attached,
            pump_enabled=snapshot.pump_enabled,
            valve_commanded_open=snapshot.right_valve_open,
            data_fresh=snapshot.connected and snapshot.data_fresh,
        ),
    )


class VacuumAdapterCore:
    def __init__(
        self,
        io: VacuumIo,
        *,
        grip_verify_timeout_s: float = 3.0,
        accepted_grip_profile_ids: Sequence[str] = ("default",),
    ) -> None:
        if not math.isfinite(grip_verify_timeout_s) or grip_verify_timeout_s <= 0.0:
            raise ValueError("grip_verify_timeout_s must be finite and greater than zero")
        profiles = tuple(str(item).strip() for item in accepted_grip_profile_ids)
        if not profiles or any(not item for item in profiles):
            raise ValueError("accepted_grip_profile_ids must contain non-empty strings")
        self._io = io
        self._grip_verify_timeout_s = float(grip_verify_timeout_s)
        self._accepted_grip_profile_ids = frozenset(profiles)
        self._operation_lock = threading.Lock()
        self._active_operation = False

    def set_pump_enabled(self, enabled: bool, reason: str = "") -> PumpCommandResult:
        del reason
        if not enabled:
            with self._operation_lock:
                if self._active_operation:
                    return PumpCommandResult(
                        False,
                        True,
                        PumpErrorCode.ACTIVE_VACUUM_COMMAND,
                        "reject pump disable: active vacuum command is running",
                    )
            snapshot = self._io.read_snapshot()
            if not snapshot.connected or not snapshot.data_fresh:
                return PumpCommandResult(
                    False,
                    snapshot.pump_enabled,
                    PumpErrorCode.PLC_UNAVAILABLE,
                    "reject pump disable: PLC vacuum state is unavailable or stale",
                )
            if snapshot.any_possible_load_held:
                return PumpCommandResult(
                    False,
                    True,
                    PumpErrorCode.POSSIBLE_LOAD_HELD,
                    "reject pump disable: possible load is held by vacuum",
                )

        command = self._io.set_output("pump", enabled)
        if not command.success:
            return PumpCommandResult(
                False,
                not enabled,
                PumpErrorCode.COMMAND_REJECTED,
                f"pump command rejected: {command.message}",
            )
        return PumpCommandResult(
            True,
            enabled,
            PumpErrorCode.OK,
            f"pump {'enabled' if enabled else 'disabled'}",
        )

    def execute_goal(
        self,
        command: int,
        channels: Sequence[str],
        grip_profile_id: str,
        context: str,
        feedback_callback: Callable[[tuple[VacuumChannelStateData, ...]], None]
        | None = None,
    ) -> VacuumExecutionResult:
        del context
        try:
            selected_channels = normalize_channels(channels)
        except ValueError as exc:
            return VacuumExecutionResult(False, False, UNVERIFIED, str(exc), ())

        profile_id = str(grip_profile_id).strip()
        if profile_id not in self._accepted_grip_profile_ids:
            return VacuumExecutionResult(
                False,
                False,
                UNVERIFIED,
                f"unsupported grip_profile_id: {profile_id or '<empty>'}",
                self._channel_results(
                    selected_channels,
                    command_accepted=False,
                    valve_actuation_completed=False,
                    verification_level=UNVERIFIED,
                    error="unsupported grip_profile_id",
                ),
            )

        if int(command) not in (GRIP, RELEASE):
            return VacuumExecutionResult(
                False,
                False,
                UNVERIFIED,
                f"unsupported vacuum command: {command}",
                self._channel_results(
                    selected_channels,
                    command_accepted=False,
                    valve_actuation_completed=False,
                    verification_level=UNVERIFIED,
                    error="unsupported command",
                ),
            )

        with self._operation_lock:
            if self._active_operation:
                return VacuumExecutionResult(
                    False,
                    False,
                    UNVERIFIED,
                    "another vacuum command is already running",
                    self._channel_results(
                        selected_channels,
                        command_accepted=False,
                        valve_actuation_completed=False,
                        verification_level=UNVERIFIED,
                        error="busy",
                    ),
                )
            self._active_operation = True

        try:
            if int(command) == GRIP:
                return self._execute_grip(selected_channels, feedback_callback)
            return self._execute_release(selected_channels, feedback_callback)
        finally:
            with self._operation_lock:
                self._active_operation = False

    def _execute_grip(
        self,
        selected_channels: tuple[str, ...],
        feedback_callback: Callable[[tuple[VacuumChannelStateData, ...]], None] | None,
    ) -> VacuumExecutionResult:
        pump_result = self._io.set_output("pump", True)
        if not pump_result.success:
            return VacuumExecutionResult(
                False,
                False,
                UNVERIFIED,
                f"pump enable failed: {pump_result.message}",
                self._channel_results(
                    selected_channels,
                    command_accepted=False,
                    valve_actuation_completed=False,
                    verification_level=UNVERIFIED,
                    error="pump enable failed",
                ),
            )

        for channel in selected_channels:
            command_result = self._io.set_output(channel, True)
            if not command_result.success:
                return VacuumExecutionResult(
                    False,
                    False,
                    UNVERIFIED,
                    f"{channel} valve command failed: {command_result.message}",
                    self._channel_results(
                        selected_channels,
                        command_accepted=False,
                        valve_actuation_completed=False,
                        verification_level=UNVERIFIED,
                        error="valve command failed",
                    ),
                )

        snapshot = self._io.read_snapshot()
        self._publish_feedback(snapshot, feedback_callback)
        snapshot = self._io.wait_for_attachment(
            selected_channels, self._grip_verify_timeout_s
        )
        self._publish_feedback(snapshot, feedback_callback)

        if not snapshot.connected or not snapshot.data_fresh:
            return self._attachment_failed(
                selected_channels,
                snapshot,
                "PLC vacuum observation is unavailable or stale",
            )
        missing = [
            channel
            for channel in selected_channels
            if not snapshot.channel_attached(channel)
        ]
        if missing:
            return self._attachment_failed(
                selected_channels,
                snapshot,
                f"attachment not verified: {', '.join(missing)}",
            )
        return VacuumExecutionResult(
            True,
            True,
            ATTACHED_VERIFIED,
            "",
            tuple(
                VacuumChannelResultData(
                    channel=channel,
                    command_accepted=True,
                    valve_actuation_completed=True,
                    verification_level=ATTACHED_VERIFIED,
                    error="",
                )
                for channel in selected_channels
            ),
        )

    def _execute_release(
        self,
        selected_channels: tuple[str, ...],
        feedback_callback: Callable[[tuple[VacuumChannelStateData, ...]], None] | None,
    ) -> VacuumExecutionResult:
        channel_results: list[VacuumChannelResultData] = []
        for channel in selected_channels:
            command_result = self._io.set_output(channel, False)
            channel_results.append(
                VacuumChannelResultData(
                    channel=channel,
                    command_accepted=command_result.success,
                    valve_actuation_completed=command_result.success,
                    verification_level=UNVERIFIED,
                    error="" if command_result.success else command_result.message,
                )
            )
            if not command_result.success:
                return VacuumExecutionResult(
                    False,
                    False,
                    UNVERIFIED,
                    f"{channel} valve release failed: {command_result.message}",
                    tuple(channel_results),
                )
        self._publish_feedback(self._io.read_snapshot(), feedback_callback)
        return VacuumExecutionResult(True, True, UNVERIFIED, "", tuple(channel_results))

    def _attachment_failed(
        self,
        selected_channels: tuple[str, ...],
        snapshot: PlcVacuumSnapshot,
        error: str,
    ) -> VacuumExecutionResult:
        return VacuumExecutionResult(
            False,
            True,
            UNVERIFIED,
            error,
            tuple(
                VacuumChannelResultData(
                    channel=channel,
                    command_accepted=True,
                    valve_actuation_completed=snapshot.channel_valve_open(channel),
                    verification_level=UNVERIFIED,
                    error=error,
                )
                for channel in selected_channels
            ),
        )

    @staticmethod
    def _channel_results(
        selected_channels: tuple[str, ...],
        *,
        command_accepted: bool,
        valve_actuation_completed: bool,
        verification_level: int,
        error: str,
    ) -> tuple[VacuumChannelResultData, ...]:
        return tuple(
            VacuumChannelResultData(
                channel=channel,
                command_accepted=command_accepted,
                valve_actuation_completed=valve_actuation_completed,
                verification_level=verification_level,
                error=error,
            )
            for channel in selected_channels
        )

    @staticmethod
    def _publish_feedback(
        snapshot: PlcVacuumSnapshot,
        feedback_callback: Callable[[tuple[VacuumChannelStateData, ...]], None]
        | None,
    ) -> None:
        if feedback_callback is not None:
            feedback_callback(build_vacuum_channel_states(snapshot))


def main(args=None) -> None:
    import rclpy
    from robot_control_interfaces.action import VacuumGrip
    from robot_control_interfaces.msg import VacuumChannelFeedback, VacuumChannelResult
    from robot_control_interfaces.msg import VacuumChannelState, VacuumState
    from robot_control_interfaces.srv import SetPumpEnabled
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from rt_control_interfaces.msg import PlcIoState
    from std_srvs.srv import SetBool

    class RosVacuumIo:
        def __init__(self, node: Node, callback_group: ReentrantCallbackGroup) -> None:
            self._node = node
            self._service_timeout_s = float(
                node.get_parameter("plc_service_timeout_s").value
            )
            self._plc_state_timeout_s = float(
                node.get_parameter("plc_state_timeout_s").value
            )
            self._condition = threading.Condition()
            self._latest_message = None
            self._latest_received_s: float | None = None
            self._clients = {
                "left": node.create_client(
                    SetBool,
                    str(node.get_parameter("left_solenoid_service").value),
                    callback_group=callback_group,
                ),
                "right": node.create_client(
                    SetBool,
                    str(node.get_parameter("right_solenoid_service").value),
                    callback_group=callback_group,
                ),
                "pump": node.create_client(
                    SetBool,
                    str(node.get_parameter("pump_service").value),
                    callback_group=callback_group,
                ),
            }
            self._subscription = node.create_subscription(
                PlcIoState,
                str(node.get_parameter("plc_state_topic").value),
                self._on_plc_state,
                QoSProfile(depth=10),
                callback_group=callback_group,
            )

        def _on_plc_state(self, message: PlcIoState) -> None:
            with self._condition:
                self._latest_message = message
                self._latest_received_s = time.monotonic()
                self._condition.notify_all()

        def set_output(self, output_name: str, enabled: bool) -> PlcCommandResult:
            client = self._clients[output_name]
            if not client.wait_for_service(timeout_sec=self._service_timeout_s):
                return PlcCommandResult(False, f"{output_name} PLC service unavailable")
            request = SetBool.Request()
            request.data = bool(enabled)
            future = client.call_async(request)
            deadline = time.monotonic() + self._service_timeout_s
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                return PlcCommandResult(False, f"{output_name} PLC service timed out")
            exception = future.exception()
            if exception is not None:
                return PlcCommandResult(False, str(exception))
            response = future.result()
            if response is None:
                return PlcCommandResult(False, "PLC service returned no response")
            return PlcCommandResult(bool(response.success), str(response.message))

        def read_snapshot(self) -> PlcVacuumSnapshot:
            with self._condition:
                return self._snapshot_locked()

        def wait_for_attachment(
            self, channels: Sequence[str], timeout_s: float
        ) -> PlcVacuumSnapshot:
            deadline = time.monotonic() + float(timeout_s)
            with self._condition:
                while rclpy.ok():
                    snapshot = self._snapshot_locked()
                    if (
                        snapshot.connected
                        and snapshot.data_fresh
                        and all(snapshot.channel_attached(channel) for channel in channels)
                    ):
                        return snapshot
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        return snapshot
                    self._condition.wait(timeout=min(0.1, remaining))
            return self._snapshot_locked()

        def _snapshot_locked(self) -> PlcVacuumSnapshot:
            message = self._latest_message
            received = self._latest_received_s
            if message is None or received is None:
                return PlcVacuumSnapshot(False, False, False, False, False, False, False, "")
            age_s = time.monotonic() - received
            fresh_by_age = 0.0 <= age_s <= self._plc_state_timeout_s
            return PlcVacuumSnapshot(
                connected=bool(message.connected),
                data_fresh=bool(message.data_fresh) and fresh_by_age,
                left_attached=bool(message.left_vacuum_established),
                right_attached=bool(message.right_vacuum_established),
                left_valve_open=bool(message.left_solenoid_on),
                right_valve_open=bool(message.right_solenoid_on),
                pump_enabled=bool(message.vacuum_pump_on),
                error=str(message.error),
            )

    class VacuumAdapterNode(Node):
        def __init__(self) -> None:
            super().__init__("vacuum_adapter")
            self.declare_parameter("plc_state_topic", "/plc/io_state")
            self.declare_parameter("vacuum_state_topic", "/vacuum/state")
            self.declare_parameter("pump_set_enabled_service", "/vacuum/pump/set_enabled")
            self.declare_parameter("grip_action_name", "/vacuum/grip")
            self.declare_parameter("left_solenoid_service", "/plc/left_solenoid")
            self.declare_parameter("right_solenoid_service", "/plc/right_solenoid")
            self.declare_parameter("pump_service", "/plc/vacuum_pump")
            self.declare_parameter("publish_period_s", 0.05)
            self.declare_parameter("plc_service_timeout_s", 2.0)
            self.declare_parameter("plc_state_timeout_s", 1.5)
            self.declare_parameter("grip_verify_timeout_s", 3.0)
            self.declare_parameter("accepted_grip_profile_ids", ["default"])

            self._callback_group = ReentrantCallbackGroup()
            self._io = RosVacuumIo(self, self._callback_group)
            self._core = VacuumAdapterCore(
                self._io,
                grip_verify_timeout_s=float(
                    self.get_parameter("grip_verify_timeout_s").value
                ),
                accepted_grip_profile_ids=list(
                    self.get_parameter("accepted_grip_profile_ids").value
                ),
            )
            self._publisher = self.create_publisher(
                VacuumState,
                str(self.get_parameter("vacuum_state_topic").value),
                QoSProfile(depth=10),
            )
            self._pump_service = self.create_service(
                SetPumpEnabled,
                str(self.get_parameter("pump_set_enabled_service").value),
                self._handle_set_pump_enabled,
                callback_group=self._callback_group,
            )
            self._action_server = ActionServer(
                self,
                VacuumGrip,
                str(self.get_parameter("grip_action_name").value),
                execute_callback=self._execute_goal,
                goal_callback=self._handle_goal,
                cancel_callback=self._handle_cancel,
                callback_group=self._callback_group,
            )
            self._timer = self.create_timer(
                float(self.get_parameter("publish_period_s").value),
                self._publish_state,
                callback_group=self._callback_group,
            )

        def _handle_goal(self, goal_request):
            try:
                normalize_channels(goal_request.channels)
            except ValueError as exc:
                self.get_logger().error(f"reject /vacuum/grip goal: {exc}")
                return GoalResponse.REJECT
            if int(goal_request.command) not in (GRIP, RELEASE):
                self.get_logger().error(
                    f"reject /vacuum/grip goal: unsupported command {goal_request.command}"
                )
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

        def _handle_cancel(self, _goal_handle):
            return CancelResponse.ACCEPT

        def _handle_set_pump_enabled(self, request, response):
            result = self._core.set_pump_enabled(bool(request.enabled), str(request.reason))
            response.accepted = result.accepted
            response.enabled = result.enabled
            response.error_code = int(result.error_code)
            response.message = result.message
            if result.accepted:
                self.get_logger().info(result.message)
            else:
                self.get_logger().error(result.message)
            return response

        def _execute_goal(self, goal_handle):
            goal = goal_handle.request
            result = self._core.execute_goal(
                int(goal.command),
                list(goal.channels),
                str(goal.grip_profile_id),
                str(goal.context),
                feedback_callback=lambda channels: self._publish_feedback(
                    goal_handle, channels
                ),
            )
            response = VacuumGrip.Result()
            response.accepted = result.accepted
            response.overall_verification_level = int(result.overall_verification_level)
            response.error = result.error
            response.channel_results = [
                self._to_channel_result(item) for item in result.channel_results
            ]
            if goal_handle.is_cancel_requested and not result.succeeded:
                goal_handle.canceled()
            elif result.succeeded:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return response

        def _publish_feedback(self, goal_handle, channels) -> None:
            feedback = VacuumGrip.Feedback()
            feedback.channel_feedback = [
                self._to_channel_feedback(item) for item in channels
            ]
            goal_handle.publish_feedback(feedback)

        def _publish_state(self) -> None:
            snapshot = self._io.read_snapshot()
            message = VacuumState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "vacuum"
            message.connected = snapshot.connected
            message.data_fresh = snapshot.connected and snapshot.data_fresh
            message.pump_enabled = snapshot.pump_enabled
            message.channels = [
                self._to_channel_state(item)
                for item in build_vacuum_channel_states(snapshot)
            ]
            message.error = snapshot.error
            self._publisher.publish(message)

        @staticmethod
        def _to_channel_state(item: VacuumChannelStateData):
            message = VacuumChannelState()
            message.channel = item.channel
            message.attached = item.attached
            message.valve_commanded_open = item.valve_commanded_open
            message.data_fresh = item.data_fresh
            return message

        @staticmethod
        def _to_channel_feedback(item: VacuumChannelStateData):
            message = VacuumChannelFeedback()
            message.channel = item.channel
            message.attached = item.attached
            message.pump_enabled = item.pump_enabled
            message.valve_commanded_open = item.valve_commanded_open
            message.data_fresh = item.data_fresh
            return message

        @staticmethod
        def _to_channel_result(item: VacuumChannelResultData):
            message = VacuumChannelResult()
            message.channel = item.channel
            message.command_accepted = item.command_accepted
            message.valve_actuation_completed = item.valve_actuation_completed
            message.verification_level = int(item.verification_level)
            message.error = item.error
            return message

    rclpy.init(args=args)
    node = VacuumAdapterNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
