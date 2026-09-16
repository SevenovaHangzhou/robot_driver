from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Protocol, Sequence

from .public_error import (
    ErrorInfoData,
    PublicErrorCode,
    assign_error_info,
    error_info,
)

LEFT = "left"
RIGHT = "right"
CHANNELS = (LEFT, RIGHT)
GRIP = 1
RELEASE = 2
UNVERIFIED = 0
ATTACHED_VERIFIED = 1


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
    vacuum_pressure_valid: bool = False
    vacuum_pressure_kpa: float = math.nan
    vacuum_released: bool = False

    def channel_attached(self, channel: str) -> bool:
        if channel == LEFT:
            return self.left_attached
        if channel == RIGHT:
            return self.right_attached
        raise ValueError(f"unsupported vacuum channel: {channel}")

    def channel_valve_open(self, channel: str) -> bool:
        if channel in CHANNELS:
            # 公共消息保留历史字段名；单继电器硬件中它表示泵继电器命令状态。
            return self.pump_enabled
        raise ValueError(f"unsupported vacuum channel: {channel}")

    @property
    def any_possible_load_held(self) -> bool:
        return (
            self.left_attached
            or self.right_attached
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
    error: ErrorInfoData
    channel_results: tuple[VacuumChannelResultData, ...]


@dataclass(frozen=True)
class PlcCommandResult:
    success: bool
    message: str
    error_code: PublicErrorCode


@dataclass(frozen=True)
class PumpCommandResult:
    accepted: bool
    enabled: bool
    error: ErrorInfoData


class VacuumIo(Protocol):
    def set_output(self, output_name: str, enabled: bool) -> PlcCommandResult:
        ...

    def read_snapshot(self) -> PlcVacuumSnapshot:
        ...

    def wait_for_attachment(
        self,
        channels: Sequence[str],
        timeout_s: float,
        cancel_requested: Callable[[], bool],
        state_callback: Callable[[PlcVacuumSnapshot], None],
    ) -> PlcVacuumSnapshot:
        ...

    def wait_for_release(
        self,
        timeout_s: float,
        cancel_requested: Callable[[], bool],
        state_callback: Callable[[PlcVacuumSnapshot], None],
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
            valve_commanded_open=snapshot.channel_valve_open(LEFT),
            data_fresh=snapshot.connected and snapshot.data_fresh,
        ),
        VacuumChannelStateData(
            channel=RIGHT,
            attached=snapshot.right_attached,
            pump_enabled=snapshot.pump_enabled,
            valve_commanded_open=snapshot.channel_valve_open(RIGHT),
            data_fresh=snapshot.connected and snapshot.data_fresh,
        ),
    )


class VacuumAdapterCore:
    def __init__(
        self,
        io: VacuumIo,
        *,
        grip_verify_timeout_s: float = 3.0,
        release_verify_timeout_s: float = 3.0,
        accepted_grip_profile_ids: Sequence[str] = ("default",),
    ) -> None:
        if not math.isfinite(grip_verify_timeout_s) or grip_verify_timeout_s <= 0.0:
            raise ValueError("grip_verify_timeout_s must be finite and greater than zero")
        if not math.isfinite(release_verify_timeout_s) or release_verify_timeout_s <= 0.0:
            raise ValueError("release_verify_timeout_s must be finite and greater than zero")
        profiles = tuple(str(item).strip() for item in accepted_grip_profile_ids)
        if not profiles or any(not item for item in profiles):
            raise ValueError("accepted_grip_profile_ids must contain non-empty strings")
        self._io = io
        self._grip_verify_timeout_s = float(grip_verify_timeout_s)
        self._release_verify_timeout_s = float(release_verify_timeout_s)
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
                        error_info(
                            PublicErrorCode.RT_OPERATION_IN_PROGRESS,
                            "reject pump disable: active vacuum command is running",
                        ),
                    )
            snapshot = self._io.read_snapshot()
            if not snapshot.connected or not snapshot.data_fresh:
                return PumpCommandResult(
                    False,
                    snapshot.pump_enabled,
                    error_info(
                        PublicErrorCode.RT_PLC_UNAVAILABLE,
                        "reject pump disable: PLC vacuum state is unavailable or stale",
                    ),
                )
            if snapshot.any_possible_load_held:
                return PumpCommandResult(
                    False,
                    True,
                    error_info(
                        PublicErrorCode.RT_POSSIBLE_LOAD_HELD,
                        "reject pump disable: possible load is held by vacuum",
                    ),
                )

        command = self._io.set_output("pump", enabled)
        if not command.success:
            return PumpCommandResult(
                False,
                not enabled,
                error_info(
                    command.error_code,
                    f"pump command rejected: {command.message}",
                ),
            )
        return PumpCommandResult(
            True,
            enabled,
            error_info(
                PublicErrorCode.SUCCESS,
                f"pump {'enabled' if enabled else 'disabled'}",
            ),
        )

    def execute_goal(
        self,
        command: int,
        channels: Sequence[str],
        grip_profile_id: str,
        context: str,
        feedback_callback: Callable[[tuple[VacuumChannelStateData, ...]], None]
        | None = None,
        cancel_requested: Callable[[], bool] | None = None,
    ) -> VacuumExecutionResult:
        del context
        try:
            selected_channels = normalize_channels(channels)
        except ValueError as exc:
            return VacuumExecutionResult(
                False,
                False,
                UNVERIFIED,
                error_info(PublicErrorCode.INVALID_GOAL, str(exc)),
                (),
            )

        profile_id = str(grip_profile_id).strip()
        if profile_id not in self._accepted_grip_profile_ids:
            return VacuumExecutionResult(
                False,
                False,
                UNVERIFIED,
                error_info(
                    PublicErrorCode.INVALID_GOAL,
                    f"unsupported grip_profile_id: {profile_id or '<empty>'}",
                ),
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
                error_info(
                    PublicErrorCode.INVALID_GOAL,
                    f"unsupported vacuum command: {command}",
                ),
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
                    error_info(
                        PublicErrorCode.RT_OPERATION_IN_PROGRESS,
                        "another vacuum command is already running",
                    ),
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
            is_cancel_requested = cancel_requested or (lambda: False)
            if is_cancel_requested():
                return self._canceled(selected_channels)
            if int(command) == GRIP:
                return self._execute_grip(
                    selected_channels, feedback_callback, is_cancel_requested
                )
            return self._execute_release(
                selected_channels, feedback_callback, is_cancel_requested
            )
        finally:
            with self._operation_lock:
                self._active_operation = False

    def _execute_grip(
        self,
        selected_channels: tuple[str, ...],
        feedback_callback: Callable[[tuple[VacuumChannelStateData, ...]], None] | None,
        cancel_requested: Callable[[], bool],
    ) -> VacuumExecutionResult:
        pump_result = self._io.set_output("pump", True)
        if not pump_result.success:
            return VacuumExecutionResult(
                False,
                False,
                UNVERIFIED,
                error_info(
                    pump_result.error_code,
                    f"pump enable failed: {pump_result.message}",
                ),
                self._channel_results(
                    selected_channels,
                    command_accepted=False,
                    valve_actuation_completed=False,
                    verification_level=UNVERIFIED,
                    error="pump enable failed",
                ),
            )

        if cancel_requested():
            return self._canceled(selected_channels)

        snapshot = self._io.read_snapshot()
        self._publish_feedback(snapshot, feedback_callback)
        snapshot = self._io.wait_for_attachment(
            selected_channels,
            self._grip_verify_timeout_s,
            cancel_requested,
            lambda current: self._publish_feedback(current, feedback_callback),
        )
        self._publish_feedback(snapshot, feedback_callback)

        if cancel_requested():
            return self._canceled(selected_channels, snapshot)

        if not snapshot.connected or not snapshot.data_fresh:
            return self._attachment_failed(
                selected_channels,
                snapshot,
                "PLC vacuum observation is unavailable or stale",
                PublicErrorCode.RT_PLC_UNAVAILABLE,
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
                PublicErrorCode.RT_VACUUM_NOT_ESTABLISHED,
            )
        return VacuumExecutionResult(
            True,
            True,
            ATTACHED_VERIFIED,
            error_info(PublicErrorCode.SUCCESS, ""),
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
        cancel_requested: Callable[[], bool],
    ) -> VacuumExecutionResult:
        pump_result = self._io.set_output("pump", False)
        if not pump_result.success:
            return VacuumExecutionResult(
                False,
                False,
                UNVERIFIED,
                error_info(
                    pump_result.error_code,
                    f"pump release command failed: {pump_result.message}",
                ),
                self._channel_results(
                    selected_channels,
                    command_accepted=False,
                    valve_actuation_completed=False,
                    verification_level=UNVERIFIED,
                    error="pump release command failed",
                ),
            )
        if cancel_requested():
            return self._canceled(selected_channels)

        snapshot = self._io.read_snapshot()
        self._publish_feedback(snapshot, feedback_callback)
        snapshot = self._io.wait_for_release(
            self._release_verify_timeout_s,
            cancel_requested,
            lambda current: self._publish_feedback(current, feedback_callback),
        )
        self._publish_feedback(snapshot, feedback_callback)
        if cancel_requested():
            return self._canceled(selected_channels, snapshot)
        if not snapshot.connected or not snapshot.data_fresh or not snapshot.vacuum_pressure_valid:
            return self._release_failed(
                selected_channels,
                snapshot,
                "vacuum pressure observation is unavailable or stale",
                PublicErrorCode.RT_PLC_UNAVAILABLE,
            )
        if not snapshot.vacuum_released:
            return self._release_failed(
                selected_channels,
                snapshot,
                f"release pressure not reached: {snapshot.vacuum_pressure_kpa:.3f} kPa",
                PublicErrorCode.TIMEOUT,
            )
        return VacuumExecutionResult(
            True,
            True,
            UNVERIFIED,
            error_info(PublicErrorCode.SUCCESS, ""),
            self._channel_results(
                selected_channels,
                command_accepted=True,
                valve_actuation_completed=True,
                verification_level=UNVERIFIED,
                error="",
            ),
        )

    def _release_failed(
        self,
        selected_channels: tuple[str, ...],
        snapshot: PlcVacuumSnapshot,
        error: str,
        error_code: PublicErrorCode,
    ) -> VacuumExecutionResult:
        return VacuumExecutionResult(
            False,
            True,
            UNVERIFIED,
            error_info(error_code, error),
            self._channel_results(
                selected_channels,
                command_accepted=True,
                valve_actuation_completed=not snapshot.pump_enabled,
                verification_level=UNVERIFIED,
                error=error,
            ),
        )

    def _canceled(
        self,
        selected_channels: tuple[str, ...],
        snapshot: PlcVacuumSnapshot | None = None,
    ) -> VacuumExecutionResult:
        # 取消只停止本次软件等待。这里不自动关阀或共用泵，因为另一侧可能仍在持载。
        current = snapshot if snapshot is not None else self._io.read_snapshot()
        return VacuumExecutionResult(
            False,
            True,
            UNVERIFIED,
            error_info(PublicErrorCode.CANCELED, "vacuum action canceled"),
            tuple(
                VacuumChannelResultData(
                    channel=channel,
                    command_accepted=current.channel_valve_open(channel),
                    valve_actuation_completed=current.channel_valve_open(channel),
                    verification_level=UNVERIFIED,
                    error="canceled; outputs left unchanged",
                )
                for channel in selected_channels
            ),
        )

    def _attachment_failed(
        self,
        selected_channels: tuple[str, ...],
        snapshot: PlcVacuumSnapshot,
        error: str,
        error_code: PublicErrorCode,
    ) -> VacuumExecutionResult:
        return VacuumExecutionResult(
            False,
            True,
            UNVERIFIED,
            error_info(error_code, error),
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
    from robot_rt_control_interfaces.action import VacuumGrip
    from robot_rt_control_interfaces.msg import VacuumChannelFeedback, VacuumChannelResult
    from robot_rt_control_interfaces.msg import VacuumChannelState, VacuumState
    from robot_rt_control_interfaces.srv import SetPumpEnabled
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from robot_interfaces_qos import state
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
            self._message_generation = 0
            self._clients = {
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
                self._message_generation += 1
                self._condition.notify_all()

        def set_output(self, output_name: str, enabled: bool) -> PlcCommandResult:
            client = self._clients[output_name]
            if not client.wait_for_service(timeout_sec=self._service_timeout_s):
                return PlcCommandResult(
                    False,
                    f"{output_name} PLC service unavailable",
                    PublicErrorCode.RT_PLC_UNAVAILABLE,
                )
            request = SetBool.Request()
            request.data = bool(enabled)
            future = client.call_async(request)
            deadline = time.monotonic() + self._service_timeout_s
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                return PlcCommandResult(
                    False,
                    f"{output_name} PLC service timed out",
                    PublicErrorCode.RT_PLC_UNAVAILABLE,
                )
            exception = future.exception()
            if exception is not None:
                return PlcCommandResult(
                    False,
                    str(exception),
                    PublicErrorCode.RT_PLC_UNAVAILABLE,
                )
            response = future.result()
            if response is None:
                return PlcCommandResult(
                    False,
                    "PLC service returned no response",
                    PublicErrorCode.RT_PLC_UNAVAILABLE,
                )
            success = bool(response.success)
            return PlcCommandResult(
                success,
                str(response.message),
                PublicErrorCode.SUCCESS
                if success
                else PublicErrorCode.RT_PUMP_COMMAND_REJECTED,
            )

        def read_snapshot(self) -> PlcVacuumSnapshot:
            with self._condition:
                return self._snapshot_locked()

        def wait_for_attachment(
            self,
            channels: Sequence[str],
            timeout_s: float,
            cancel_requested: Callable[[], bool],
            state_callback: Callable[[PlcVacuumSnapshot], None],
        ) -> PlcVacuumSnapshot:
            deadline = time.monotonic() + float(timeout_s)
            with self._condition:
                starting_generation = self._message_generation
                while rclpy.ok():
                    snapshot = self._snapshot_locked()
                    state_callback(snapshot)
                    if cancel_requested():
                        return snapshot
                    if (
                        snapshot.connected
                        and snapshot.data_fresh
                        and self._message_generation > starting_generation
                        and all(snapshot.channel_attached(channel) for channel in channels)
                    ):
                        return snapshot
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        return snapshot
                    self._condition.wait(timeout=min(0.1, remaining))
            return self._snapshot_locked()

        def wait_for_release(
            self,
            timeout_s: float,
            cancel_requested: Callable[[], bool],
            state_callback: Callable[[PlcVacuumSnapshot], None],
        ) -> PlcVacuumSnapshot:
            deadline = time.monotonic() + float(timeout_s)
            with self._condition:
                starting_generation = self._message_generation
                while rclpy.ok():
                    snapshot = self._snapshot_locked()
                    state_callback(snapshot)
                    if cancel_requested():
                        return snapshot
                    if (
                        snapshot.connected
                        and snapshot.data_fresh
                        and self._message_generation > starting_generation
                        and snapshot.vacuum_pressure_valid
                        and snapshot.vacuum_released
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
                vacuum_pressure_valid=bool(message.vacuum_pressure_valid),
                vacuum_pressure_kpa=float(message.vacuum_pressure_kpa),
                vacuum_released=bool(message.vacuum_released),
            )

    class VacuumAdapterNode(Node):
        def __init__(self) -> None:
            super().__init__("vacuum_adapter")
            self.declare_parameter("plc_state_topic", "/plc/io_state")
            self.declare_parameter("vacuum_state_topic", "/vacuum/state")
            self.declare_parameter("pump_set_enabled_service", "/vacuum/pump/set_enabled")
            self.declare_parameter("grip_action_name", "/vacuum/grip")
            self.declare_parameter("pump_service", "/plc/vacuum_pump")
            self.declare_parameter("publish_period_s", 0.05)
            self.declare_parameter("plc_service_timeout_s", 2.0)
            self.declare_parameter("plc_state_timeout_s", 1.5)
            self.declare_parameter("grip_verify_timeout_s", 3.0)
            self.declare_parameter("release_verify_timeout_s", 3.0)
            self.declare_parameter("accepted_grip_profile_ids", ["default"])

            self._callback_group = ReentrantCallbackGroup()
            self._io = RosVacuumIo(self, self._callback_group)
            self._core = VacuumAdapterCore(
                self._io,
                grip_verify_timeout_s=float(
                    self.get_parameter("grip_verify_timeout_s").value
                ),
                release_verify_timeout_s=float(
                    self.get_parameter("release_verify_timeout_s").value
                ),
                accepted_grip_profile_ids=list(
                    self.get_parameter("accepted_grip_profile_ids").value
                ),
            )
            self._publisher = self.create_publisher(
                VacuumState,
                str(self.get_parameter("vacuum_state_topic").value),
                state(),
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
            assign_error_info(response.error, result.error)
            if result.accepted:
                self.get_logger().info(result.error.message)
            else:
                self.get_logger().error(result.error.message)
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
                cancel_requested=lambda: bool(goal_handle.is_cancel_requested),
            )
            response = VacuumGrip.Result()
            response.accepted = result.accepted
            response.overall_verification_level = int(result.overall_verification_level)
            assign_error_info(response.error, result.error)
            response.channel_results = [
                self._to_channel_result(item) for item in result.channel_results
            ]
            if result.error.code == PublicErrorCode.CANCELED:
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
