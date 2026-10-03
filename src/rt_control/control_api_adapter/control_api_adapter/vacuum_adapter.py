from __future__ import annotations

import math
import threading
import time
import uuid
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
TARGET_SUCTION = 1
TARGET_RELEASE = 2
OUTCOME_UNSPECIFIED = 0
OUTCOME_CONFIRMED = 1
OUTCOME_NOT_EXECUTED = 2
OUTCOME_UNKNOWN = 3
VACUUM_UNKNOWN = 0
VACUUM_ATTACHED = 1
VACUUM_INTERMEDIATE = 2
VACUUM_RELEASED = 3


@dataclass(frozen=True)
class ChannelSnapshot:
    channel: str
    sensor_id: str = ""
    pressure_stamp_sec: int = 0
    pressure_stamp_nanosec: int = 0
    pressure_raw: int = 0
    pressure_valid: bool = False
    pressure_kpa: float = math.nan
    valve_valid: bool = False
    valve_on: bool = False


@dataclass(frozen=True)
class PlcVacuumSnapshot:
    hardware_configured: bool
    connected: bool
    data_fresh: bool
    pump_valid: bool
    pump_enabled: bool
    left: ChannelSnapshot
    right: ChannelSnapshot
    state_stamp_sec: int = 0
    state_stamp_nanosec: int = 0
    error: str = ""

    def channel(self, name: str) -> ChannelSnapshot:
        if name == LEFT:
            return self.left
        if name == RIGHT:
            return self.right
        raise ValueError(f"unsupported vacuum channel: {name}")


@dataclass(frozen=True)
class PlcCommandResult:
    accepted: bool
    outcome: int
    observed_value: bool
    observation_valid: bool
    message: str
    error_code: PublicErrorCode


@dataclass(frozen=True)
class OutputResultData:
    resource_id: str
    attempt_id: uuid.UUID
    outcome: int
    requested_value: bool
    observed_value: bool
    observation_valid: bool
    started_ns: int
    completed_ns: int
    error: ErrorInfoData


@dataclass(frozen=True)
class PumpExecutionResult:
    accepted: bool
    output: OutputResultData
    error: ErrorInfoData


@dataclass(frozen=True)
class ValveExecutionResult:
    accepted: bool
    succeeded: bool
    canceled: bool
    request_id: bytes
    channel_results: tuple[OutputResultData, ...]
    error: ErrorInfoData


@dataclass(frozen=True)
class ValveContext:
    request_id: bytes
    attempt_id: uuid.UUID
    target: int
    completed_ns: int
    confirmed: bool


class VacuumIo(Protocol):
    def set_output(self, output_name: str, enabled: bool) -> PlcCommandResult:
        ...

    def read_snapshot(self) -> PlcVacuumSnapshot:
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


def classify_vacuum(
    observation: ChannelSnapshot,
    *,
    data_fresh: bool,
    attached_threshold_kpa: float = -60.0,
    released_threshold_kpa: float = -1.0,
) -> int:
    if (
        not data_fresh
        or not observation.pressure_valid
        or not math.isfinite(observation.pressure_kpa)
    ):
        return VACUUM_UNKNOWN
    if observation.pressure_kpa <= attached_threshold_kpa:
        return VACUUM_ATTACHED
    if observation.pressure_kpa >= released_threshold_kpa:
        return VACUUM_RELEASED
    return VACUUM_INTERMEDIATE


class VacuumAdapterCore:
    def __init__(
        self,
        io: VacuumIo,
        *,
        attached_threshold_kpa: float = -60.0,
        released_threshold_kpa: float = -1.0,
        clock_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        if not math.isfinite(attached_threshold_kpa):
            raise ValueError("attached_threshold_kpa must be finite")
        if not math.isfinite(released_threshold_kpa):
            raise ValueError("released_threshold_kpa must be finite")
        if attached_threshold_kpa >= released_threshold_kpa:
            raise ValueError("attached threshold must be lower than released threshold")
        self._io = io
        self.attached_threshold_kpa = float(attached_threshold_kpa)
        self.released_threshold_kpa = float(released_threshold_kpa)
        self._clock_ns = clock_ns
        self._lock = threading.Lock()
        self._active_channels: set[str] = set()
        self._unknown_channels: set[str] = set()
        self._pump_transaction_active = False
        self._last_context: dict[str, ValveContext] = {}
        self._last_pump_requested: bool | None = None

    def channel_state(self, snapshot: PlcVacuumSnapshot, channel: str) -> int:
        return classify_vacuum(
            snapshot.channel(channel),
            data_fresh=snapshot.hardware_configured and snapshot.data_fresh,
            attached_threshold_kpa=self.attached_threshold_kpa,
            released_threshold_kpa=self.released_threshold_kpa,
        )

    def last_context(self, channel: str) -> ValveContext | None:
        with self._lock:
            return self._last_context.get(channel)

    def last_pump_requested(self) -> bool | None:
        with self._lock:
            return self._last_pump_requested

    def set_pump_enabled(self, enabled: bool, reason: str = "") -> PumpExecutionResult:
        del reason
        started_ns = self._clock_ns()
        attempt_id = uuid.uuid4()
        snapshot = self._io.read_snapshot()
        with self._lock:
            self._reconcile_unknown_locked(snapshot)
            if self._pump_transaction_active or self._active_channels:
                return self._pump_rejected(
                    enabled,
                    started_ns,
                    attempt_id,
                    PublicErrorCode.RT_OPERATION_IN_PROGRESS,
                    "reject pump request: a vacuum output operation is running",
                    snapshot,
                )
            if not snapshot.hardware_configured:
                return self._pump_rejected(
                    enabled,
                    started_ns,
                    attempt_id,
                    PublicErrorCode.RT_PLC_UNAVAILABLE,
                    "reject pump request: vacuum hardware is not configured",
                    snapshot,
                )
            if not enabled:
                reason_text = self._pump_disable_rejection(snapshot)
                if reason_text:
                    return self._pump_rejected(
                        enabled,
                        started_ns,
                        attempt_id,
                        PublicErrorCode.RT_POSSIBLE_LOAD_HELD,
                        reason_text,
                        snapshot,
                    )
            self._pump_transaction_active = True

        try:
            command = self._io.set_output("pump", enabled)
        finally:
            with self._lock:
                self._pump_transaction_active = False
        output = self._output_result("pump", enabled, started_ns, attempt_id, command)
        if command.accepted:
            with self._lock:
                self._last_pump_requested = bool(enabled)
        accepted = command.accepted
        overall = output.error
        return PumpExecutionResult(accepted, output, overall)

    def execute_valves(
        self,
        target: int,
        channels: Sequence[str],
        request_id: bytes,
        *,
        feedback_callback: Callable[[PlcVacuumSnapshot, int], None] | None = None,
        cancel_requested: Callable[[], bool] | None = None,
    ) -> ValveExecutionResult:
        try:
            selected = normalize_channels(channels)
        except ValueError as exc:
            return ValveExecutionResult(
                False,
                False,
                False,
                bytes(request_id),
                (),
                error_info(PublicErrorCode.INVALID_GOAL, str(exc)),
            )
        if target not in (TARGET_SUCTION, TARGET_RELEASE):
            return ValveExecutionResult(
                False,
                False,
                False,
                bytes(request_id),
                (),
                error_info(PublicErrorCode.INVALID_GOAL, f"unsupported valve target: {target}"),
            )

        snapshot = self._io.read_snapshot()
        with self._lock:
            self._reconcile_unknown_locked(snapshot)
            if self._pump_transaction_active or any(
                channel in self._active_channels for channel in selected
            ):
                return self._valve_rejected(
                    selected,
                    target,
                    request_id,
                    PublicErrorCode.RT_OPERATION_IN_PROGRESS,
                    "one or more requested valve channels are busy",
                    snapshot,
                )
            if not snapshot.hardware_configured or any(
                not snapshot.channel(channel).valve_valid for channel in selected
            ):
                return self._valve_rejected(
                    selected,
                    target,
                    request_id,
                    PublicErrorCode.RT_PLC_UNAVAILABLE,
                    "requested valve output state is unavailable or stale",
                    snapshot,
                )
            if target == TARGET_SUCTION and (
                not snapshot.pump_valid or not snapshot.pump_enabled
            ):
                return self._valve_rejected(
                    selected,
                    target,
                    request_id,
                    PublicErrorCode.RT_PUMP_UNAVAILABLE,
                    "pump output is not valid and enabled",
                    snapshot,
                )
            self._active_channels.update(selected)

        canceled = False
        results: list[OutputResultData] = []
        is_canceled = cancel_requested or (lambda: False)
        requested_value = target == TARGET_SUCTION
        try:
            for channel in selected:
                if is_canceled():
                    canceled = True
                    results.append(
                        self._not_executed(
                            channel,
                            requested_value,
                            PublicErrorCode.CANCELED,
                            "canceled before output write",
                            snapshot.channel(channel),
                        )
                    )
                    continue
                started_ns = self._clock_ns()
                attempt_id = uuid.uuid4()
                command = self._io.set_output(channel, requested_value)
                result = self._output_result(
                    channel, requested_value, started_ns, attempt_id, command
                )
                results.append(result)
                with self._lock:
                    if result.outcome == OUTCOME_UNKNOWN:
                        self._unknown_channels.add(channel)
                    self._last_context[channel] = ValveContext(
                        bytes(request_id),
                        attempt_id,
                        target,
                        result.completed_ns,
                        result.outcome == OUTCOME_CONFIRMED,
                    )
                snapshot = self._io.read_snapshot()
                if feedback_callback is not None:
                    feedback_callback(snapshot, 3)
        finally:
            with self._lock:
                self._active_channels.difference_update(selected)

        all_confirmed = bool(results) and all(
            result.outcome == OUTCOME_CONFIRMED for result in results
        )
        if canceled:
            overall_error = error_info(PublicErrorCode.CANCELED, "valve action canceled")
        elif all_confirmed:
            overall_error = error_info(PublicErrorCode.SUCCESS, "valve outputs confirmed")
        elif any(result.outcome == OUTCOME_UNKNOWN for result in results):
            overall_error = error_info(
                PublicErrorCode.RT_PLC_UNAVAILABLE,
                "one or more valve output results are unknown",
            )
        else:
            overall_error = error_info(
                PublicErrorCode.RT_PUMP_COMMAND_REJECTED,
                "one or more valve outputs were not executed",
            )
        return ValveExecutionResult(
            True,
            all_confirmed,
            canceled,
            bytes(request_id),
            tuple(results),
            overall_error,
        )

    def _pump_disable_rejection(self, snapshot: PlcVacuumSnapshot) -> str:
        if self._unknown_channels:
            return "reject pump disable: a valve output result remains unknown"
        if not snapshot.connected or not snapshot.data_fresh or not snapshot.pump_valid:
            return "reject pump disable: PLC vacuum state is unavailable or stale"
        for channel in CHANNELS:
            observation = snapshot.channel(channel)
            if not observation.valve_valid or observation.valve_on:
                return f"reject pump disable: {channel} valve is not confirmed released"
            if self.channel_state(snapshot, channel) != VACUUM_RELEASED:
                return f"reject pump disable: {channel} pressure is not confirmed released"
        return ""

    def _reconcile_unknown_locked(self, snapshot: PlcVacuumSnapshot) -> None:
        for channel in tuple(self._unknown_channels):
            if snapshot.data_fresh and snapshot.channel(channel).valve_valid:
                self._unknown_channels.remove(channel)

    def _pump_rejected(
        self,
        enabled: bool,
        started_ns: int,
        attempt_id: uuid.UUID,
        code: PublicErrorCode,
        message: str,
        snapshot: PlcVacuumSnapshot,
    ) -> PumpExecutionResult:
        error = error_info(code, message)
        output = OutputResultData(
            "pump",
            attempt_id,
            OUTCOME_NOT_EXECUTED,
            enabled,
            snapshot.pump_enabled,
            snapshot.pump_valid and snapshot.data_fresh,
            started_ns,
            self._clock_ns(),
            error,
        )
        return PumpExecutionResult(False, output, error)

    def _valve_rejected(
        self,
        channels: tuple[str, ...],
        target: int,
        request_id: bytes,
        code: PublicErrorCode,
        message: str,
        snapshot: PlcVacuumSnapshot,
    ) -> ValveExecutionResult:
        error = error_info(code, message)
        requested_value = target == TARGET_SUCTION
        results = tuple(
            OutputResultData(
                channel,
                uuid.uuid4(),
                OUTCOME_NOT_EXECUTED,
                requested_value,
                snapshot.channel(channel).valve_on,
                snapshot.channel(channel).valve_valid and snapshot.data_fresh,
                self._clock_ns(),
                self._clock_ns(),
                error,
            )
            for channel in channels
        )
        return ValveExecutionResult(False, False, False, bytes(request_id), results, error)

    def _not_executed(
        self,
        channel: str,
        requested_value: bool,
        code: PublicErrorCode,
        message: str,
        snapshot: ChannelSnapshot,
    ) -> OutputResultData:
        now_ns = self._clock_ns()
        return OutputResultData(
            channel,
            uuid.uuid4(),
            OUTCOME_NOT_EXECUTED,
            requested_value,
            snapshot.valve_on,
            snapshot.valve_valid,
            now_ns,
            now_ns,
            error_info(code, message),
        )

    def _output_result(
        self,
        resource_id: str,
        requested_value: bool,
        started_ns: int,
        attempt_id: uuid.UUID,
        command: PlcCommandResult,
    ) -> OutputResultData:
        error = error_info(command.error_code, command.message)
        return OutputResultData(
            resource_id,
            attempt_id,
            command.outcome,
            requested_value,
            command.observed_value,
            command.observation_valid,
            started_ns,
            self._clock_ns(),
            error,
        )


def main(args=None) -> None:
    import rclpy
    from robot_rt_control_interfaces.action import SetVacuumValves
    from robot_rt_control_interfaces.msg import (
        ObservationMeta,
        OutputCommandResult,
        OutputState,
        VacuumChannelState,
        VacuumState,
        VacuumStateEvent,
    )
    from robot_rt_control_interfaces.srv import SetPumpEnabled
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from robot_interfaces_qos import state
    from rt_control_interfaces.msg import PlcIoState
    from rt_control_interfaces.srv import SetDigitalOutput
    from unique_identifier_msgs.msg import UUID

    def set_uuid(message: UUID, value: bytes | uuid.UUID) -> None:
        raw = value.bytes if isinstance(value, uuid.UUID) else bytes(value)
        message.uuid = list(raw[:16].ljust(16, b"\0"))

    def set_time(message, nanoseconds: int) -> None:
        message.sec = int(nanoseconds // 1_000_000_000)
        message.nanosec = int(nanoseconds % 1_000_000_000)

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
                "pump": node.create_client(
                    SetDigitalOutput,
                    str(node.get_parameter("pump_service").value),
                    callback_group=callback_group,
                ),
                LEFT: node.create_client(
                    SetDigitalOutput,
                    str(node.get_parameter("left_valve_service").value),
                    callback_group=callback_group,
                ),
                RIGHT: node.create_client(
                    SetDigitalOutput,
                    str(node.get_parameter("right_valve_service").value),
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
                return PlcCommandResult(
                    False,
                    OUTCOME_NOT_EXECUTED,
                    False,
                    False,
                    f"{output_name} PLC service unavailable",
                    PublicErrorCode.RT_PLC_UNAVAILABLE,
                )
            request = SetDigitalOutput.Request()
            request.enabled = bool(enabled)
            future = client.call_async(request)
            deadline = time.monotonic() + self._service_timeout_s
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                return PlcCommandResult(
                    True,
                    OUTCOME_UNKNOWN,
                    False,
                    False,
                    f"{output_name} PLC service timed out",
                    PublicErrorCode.RT_PLC_UNAVAILABLE,
                )
            if future.exception() is not None or future.result() is None:
                return PlcCommandResult(
                    True,
                    OUTCOME_UNKNOWN,
                    False,
                    False,
                    str(future.exception() or "PLC service returned no response"),
                    PublicErrorCode.RT_PLC_UNAVAILABLE,
                )
            response = future.result()
            outcome = int(response.outcome)
            if outcome == OUTCOME_CONFIRMED:
                code = PublicErrorCode.SUCCESS
            elif outcome == OUTCOME_NOT_EXECUTED:
                code = PublicErrorCode.RT_PUMP_COMMAND_REJECTED
            else:
                code = PublicErrorCode.RT_PLC_UNAVAILABLE
            return PlcCommandResult(
                bool(response.accepted),
                outcome,
                bool(response.observed_value),
                bool(response.observation_valid),
                str(response.message),
                code,
            )

        def read_snapshot(self) -> PlcVacuumSnapshot:
            with self._condition:
                message = self._latest_message
                received = self._latest_received_s
                if message is None or received is None:
                    return PlcVacuumSnapshot(
                        False,
                        False,
                        False,
                        False,
                        False,
                        ChannelSnapshot(LEFT),
                        ChannelSnapshot(RIGHT),
                    )
                age_s = time.monotonic() - received
                fresh_by_age = 0.0 <= age_s <= self._plc_state_timeout_s
                return PlcVacuumSnapshot(
                    bool(message.hardware_configured),
                    bool(message.connected),
                    fresh_by_age,
                    bool(message.pump_output_valid) and fresh_by_age,
                    bool(message.vacuum_pump_on),
                    ChannelSnapshot(
                        LEFT,
                        str(message.left_pressure_sensor_id),
                        int(message.left_pressure_stamp.sec),
                        int(message.left_pressure_stamp.nanosec),
                        int(message.left_pressure_raw),
                        bool(message.left_pressure_valid) and fresh_by_age,
                        float(message.left_pressure_kpa),
                        bool(message.left_valve_output_valid) and fresh_by_age,
                        bool(message.left_solenoid_on),
                    ),
                    ChannelSnapshot(
                        RIGHT,
                        str(message.right_pressure_sensor_id),
                        int(message.right_pressure_stamp.sec),
                        int(message.right_pressure_stamp.nanosec),
                        int(message.right_pressure_raw),
                        bool(message.right_pressure_valid) and fresh_by_age,
                        float(message.right_pressure_kpa),
                        bool(message.right_valve_output_valid) and fresh_by_age,
                        bool(message.right_solenoid_on),
                    ),
                    int(message.header.stamp.sec),
                    int(message.header.stamp.nanosec),
                    str(message.error),
                )

    class VacuumAdapterNode(Node):
        def __init__(self) -> None:
            super().__init__("vacuum_adapter")
            self.declare_parameter("plc_state_topic", "/plc/io_state")
            self.declare_parameter("vacuum_state_topic", "/vacuum/state")
            self.declare_parameter("vacuum_event_topic", "/vacuum/events")
            self.declare_parameter("pump_set_enabled_service", "/vacuum/pump/set_enabled")
            self.declare_parameter("valve_action_name", "/vacuum/valves/set")
            self.declare_parameter("pump_service", "/plc/vacuum_pump")
            self.declare_parameter("left_valve_service", "/plc/vacuum_valve/left")
            self.declare_parameter("right_valve_service", "/plc/vacuum_valve/right")
            self.declare_parameter("publish_period_s", 0.05)
            self.declare_parameter("plc_service_timeout_s", 2.0)
            self.declare_parameter("plc_state_timeout_s", 1.5)
            self.declare_parameter("attached_threshold_kpa", -60.0)
            self.declare_parameter("released_threshold_kpa", -1.0)

            self._callback_group = ReentrantCallbackGroup()
            self._io = RosVacuumIo(self, self._callback_group)
            self._core = VacuumAdapterCore(
                self._io,
                attached_threshold_kpa=float(
                    self.get_parameter("attached_threshold_kpa").value
                ),
                released_threshold_kpa=float(
                    self.get_parameter("released_threshold_kpa").value
                ),
                clock_ns=lambda: self.get_clock().now().nanoseconds,
            )
            self._producer_instance = uuid.uuid4()
            self._event_sequence = 0
            self._last_vacuum_states = {LEFT: VACUUM_UNKNOWN, RIGHT: VACUUM_UNKNOWN}
            self._projection_lock = threading.Lock()
            self._pressure_sequences = {LEFT: 0, RIGHT: 0}
            self._last_pressure_stamps = {LEFT: (0, 0), RIGHT: (0, 0)}
            self._publisher = self.create_publisher(
                VacuumState,
                str(self.get_parameter("vacuum_state_topic").value),
                state(),
            )
            self._event_publisher = self.create_publisher(
                VacuumStateEvent,
                str(self.get_parameter("vacuum_event_topic").value),
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
                SetVacuumValves,
                str(self.get_parameter("valve_action_name").value),
                execute_callback=self._execute_goal,
                goal_callback=self._handle_goal,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
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
                self.get_logger().error(f"reject valve goal: {exc}")
                return GoalResponse.REJECT
            if int(goal_request.target) not in (TARGET_SUCTION, TARGET_RELEASE):
                self.get_logger().error(
                    f"reject valve goal: unsupported target {goal_request.target}"
                )
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

        def _handle_set_pump_enabled(self, request, response):
            result = self._core.set_pump_enabled(bool(request.enabled), str(request.reason))
            response.accepted = result.accepted
            self._fill_output_result(response.result, result.output)
            assign_error_info(response.error, result.error)
            return response

        def _execute_goal(self, goal_handle):
            request_id = bytes(goal_handle.request.request_id.uuid)
            result = self._core.execute_valves(
                int(goal_handle.request.target),
                list(goal_handle.request.channels),
                request_id,
                feedback_callback=lambda snapshot, stage: self._publish_feedback(
                    goal_handle, request_id, snapshot, stage
                ),
                cancel_requested=lambda: bool(goal_handle.is_cancel_requested),
            )
            response = SetVacuumValves.Result()
            response.request_id = goal_handle.request.request_id
            response.all_outputs_confirmed = result.succeeded
            response.channel_results = []
            for item in result.channel_results:
                message = OutputCommandResult()
                self._fill_output_result(message, item)
                response.channel_results.append(message)
            assign_error_info(response.error, result.error)
            if result.canceled:
                goal_handle.canceled()
            elif result.succeeded:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return response

        def _publish_feedback(self, goal_handle, request_id, snapshot, stage) -> None:
            feedback = SetVacuumValves.Feedback()
            set_uuid(feedback.request_id, request_id)
            feedback.execution_stage = int(stage)
            feedback.channels = [
                self._channel_message(snapshot, channel) for channel in goal_handle.request.channels
            ]
            goal_handle.publish_feedback(feedback)

        def _publish_state(self) -> None:
            snapshot = self._io.read_snapshot()
            message = VacuumState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "vacuum"
            set_uuid(message.producer_instance_id, self._producer_instance)
            pump_requested = self._core.last_pump_requested()
            self._fill_output_state(
                message.pump,
                snapshot.pump_enabled,
                snapshot.pump_valid and snapshot.data_fresh,
                snapshot.state_stamp_sec,
                snapshot.state_stamp_nanosec,
                pump_requested is not None,
                "pump",
                bool(pump_requested) if pump_requested is not None else False,
            )
            message.channels = [
                self._channel_message(snapshot, LEFT),
                self._channel_message(snapshot, RIGHT),
            ]
            self._publisher.publish(message)
            self._publish_events(snapshot, message.channels)

        def _publish_events(self, snapshot, channels) -> None:
            for message in channels:
                previous = self._last_vacuum_states[message.channel]
                current = int(message.vacuum_state)
                if previous != current:
                    event = VacuumStateEvent()
                    event.header.stamp = self.get_clock().now().to_msg()
                    event.header.frame_id = "vacuum"
                    set_uuid(event.producer_instance_id, self._producer_instance)
                    self._event_sequence += 1
                    event.event_sequence = self._event_sequence
                    event.channel = message.channel
                    event.previous_state = previous
                    event.current_state = current
                    event.detected_at = event.header.stamp
                    event.observation = message
                    self._event_publisher.publish(event)
                    self._last_vacuum_states[message.channel] = current

        def _channel_message(self, snapshot, channel):
            observation = snapshot.channel(channel)
            message = VacuumChannelState()
            message.channel = channel
            message.sensor_id = observation.sensor_id
            message.pressure_kpa = observation.pressure_kpa
            message.raw_value = float(observation.pressure_raw)
            message.raw_unit = "mV"
            message.pressure_observation.stamp.sec = observation.pressure_stamp_sec
            message.pressure_observation.stamp.nanosec = observation.pressure_stamp_nanosec
            message.pressure_observation.time_source = ObservationMeta.TIME_SOURCE_READ_COMPLETE
            message.pressure_observation.valid = (
                snapshot.hardware_configured
                and snapshot.data_fresh
                and observation.pressure_valid
            )
            set_uuid(
                message.pressure_observation.source_instance_id,
                self._producer_instance,
            )
            stamp = (observation.pressure_stamp_sec, observation.pressure_stamp_nanosec)
            with self._projection_lock:
                if stamp != (0, 0) and stamp != self._last_pressure_stamps[channel]:
                    self._last_pressure_stamps[channel] = stamp
                    self._pressure_sequences[channel] += 1
                sequence = self._pressure_sequences[channel]
            message.pressure_observation.sample_sequence = sequence
            if message.pressure_observation.valid:
                assign_error_info(
                    message.pressure_observation.error,
                    error_info(PublicErrorCode.SUCCESS, ""),
                )
            else:
                assign_error_info(
                    message.pressure_observation.error,
                    error_info(
                        PublicErrorCode.RT_PLC_UNAVAILABLE,
                        snapshot.error or "pressure observation unavailable",
                    ),
                )
            message.vacuum_state = self._core.channel_state(snapshot, channel)
            context = self._core.last_context(channel)
            self._fill_output_state(
                message.valve,
                observation.valve_on,
                observation.valve_valid and snapshot.data_fresh,
                snapshot.state_stamp_sec,
                snapshot.state_stamp_nanosec,
                context is not None,
                channel,
                context.target == TARGET_SUCTION if context is not None else False,
            )
            if context is not None:
                message.last_request_known = True
                set_uuid(message.last_request_id, context.request_id)
                set_uuid(message.last_attempt_id, context.attempt_id)
                message.last_valve_target = context.target
                set_time(message.last_output_confirmed_at, context.completed_ns)
                message.last_output_confirmation_valid = context.confirmed
            return message

        def _fill_output_state(
            self,
            message,
            observed_value,
            observation_valid,
            stamp_sec,
            stamp_nanosec,
            requested_known,
            resource_id,
            requested_value=False,
        ) -> None:
            del resource_id
            message.requested_value_known = bool(requested_known)
            message.requested_value = bool(requested_value)
            message.observed_value = bool(observed_value)
            message.observation.stamp.sec = int(stamp_sec)
            message.observation.stamp.nanosec = int(stamp_nanosec)
            message.observation.time_source = ObservationMeta.TIME_SOURCE_READ_COMPLETE
            message.observation.valid = bool(observation_valid)
            set_uuid(message.observation.source_instance_id, self._producer_instance)
            assign_error_info(
                message.observation.error,
                error_info(
                    PublicErrorCode.SUCCESS if observation_valid else PublicErrorCode.RT_PLC_UNAVAILABLE,
                    "" if observation_valid else "output observation unavailable",
                ),
            )

        def _fill_output_result(self, message, item: OutputResultData) -> None:
            message.resource_id = item.resource_id
            set_uuid(message.attempt_id, item.attempt_id)
            message.outcome = item.outcome
            message.requested_value = item.requested_value
            message.output.requested_value_known = True
            message.output.requested_value = item.requested_value
            message.output.observed_value = item.observed_value
            set_time(message.started_at, item.started_ns)
            set_time(message.completed_at, item.completed_ns)
            message.output.observation.stamp = message.completed_at
            message.output.observation.time_source = ObservationMeta.TIME_SOURCE_READ_COMPLETE
            message.output.observation.valid = item.observation_valid
            set_uuid(message.output.observation.source_instance_id, self._producer_instance)
            assign_error_info(message.output.observation.error, item.error)
            assign_error_info(message.error, item.error)

    rclpy.init(args=args)
    node = VacuumAdapterNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
