from __future__ import annotations

import math
import time
import threading
import uuid
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class ProtectiveStopSnapshot:
    installed: bool
    enabled: bool
    inputs_valid: bool
    latched: bool
    active_inputs: tuple[str, ...]
    reset_conditions_met: bool
    stop_required: bool
    reason: str


class ProtectiveStopCore:
    def __init__(self, *, installed: bool, enabled: bool) -> None:
        if enabled and not installed:
            raise ValueError("edge protection cannot be enabled when not installed")
        self.installed = bool(installed)
        self.enabled = bool(enabled)
        self._inputs_valid = False
        self._active_inputs: tuple[str, ...] = ()
        self._latched = False
        self._motion_stopped = False
        self._lock = threading.Lock()

    def update_inputs(self, states: Iterable[tuple[str, bool, bool]]) -> ProtectiveStopSnapshot:
        values = tuple(states)
        with self._lock:
            self._inputs_valid = bool(values) and all(valid for _, _, valid in values)
            self._active_inputs = tuple(
                input_id for input_id, active, valid in values if valid and active
            )
            if self.enabled and (not self._inputs_valid or self._active_inputs):
                self._latched = True
            return self._snapshot_locked()

    def set_motion_stopped(self, stopped: bool) -> ProtectiveStopSnapshot:
        with self._lock:
            self._motion_stopped = bool(stopped)
            return self._snapshot_locked()

    def reset(self, operator_confirmation_id: str) -> tuple[bool, ProtectiveStopSnapshot]:
        with self._lock:
            can_reset = (
                self.installed
                and self.enabled
                and bool(str(operator_confirmation_id).strip())
                and self._inputs_valid
                and not self._active_inputs
                and self._motion_stopped
            )
            if can_reset:
                self._latched = False
            return can_reset, self._snapshot_locked()

    def snapshot(self) -> ProtectiveStopSnapshot:
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> ProtectiveStopSnapshot:
        reset_conditions = (
            self.installed
            and self.enabled
            and self._inputs_valid
            and not self._active_inputs
            and self._motion_stopped
        )
        if not self.installed:
            reason = "edge hardware is not installed"
        elif not self.enabled:
            reason = "edge protection is disabled"
        elif not self._inputs_valid:
            reason = "edge input state is unavailable"
        elif self._active_inputs:
            reason = "one or more edge inputs are active"
        elif self._latched:
            reason = "protective stop is latched"
        else:
            reason = ""
        return ProtectiveStopSnapshot(
            self.installed,
            self.enabled,
            self._inputs_valid,
            self._latched,
            self._active_inputs,
            reset_conditions,
            self.enabled and self._latched,
            reason,
        )


@dataclass(frozen=True)
class ModuleSnapshot:
    module_id: str
    installation_state: int
    implementation_state: int
    enabled: bool
    configuration_valid: bool
    data_valid: bool
    operational_state: str
    capability_names: tuple[str, ...]
    errors: tuple[str, ...] = ()


INSTALLATION_UNKNOWN = 0
INSTALLATION_NOT_INSTALLED = 1
INSTALLATION_INSTALLED = 2
IMPLEMENTATION_UNKNOWN = 0
IMPLEMENTATION_NOT_IMPLEMENTED = 1
IMPLEMENTATION_NOT_ADMITTED = 2
IMPLEMENTATION_AVAILABLE = 3


def static_deferred_modules() -> tuple[ModuleSnapshot, ...]:
    return (
        ModuleSnapshot(
            "edge_protection",
            INSTALLATION_NOT_INSTALLED,
            IMPLEMENTATION_NOT_ADMITTED,
            False,
            False,
            False,
            "NOT_INSTALLED",
            ("protective_stop",),
            ("edge hardware is not installed",),
        ),
        ModuleSnapshot(
            "force_control",
            INSTALLATION_UNKNOWN,
            IMPLEMENTATION_NOT_IMPLEMENTED,
            False,
            False,
            False,
            "NOT_IMPLEMENTED",
            ("closed_loop_force_control",),
            ("closed-loop force control is deferred",),
        ),
        ModuleSnapshot(
            "battery_exchange",
            INSTALLATION_UNKNOWN,
            IMPLEMENTATION_NOT_IMPLEMENTED,
            False,
            False,
            False,
            "NOT_IMPLEMENTED",
            ("battery_exchange_control",),
            ("battery exchange control is deferred",),
        ),
    )


def require_edge_preview_only(enabled: bool) -> None:
    if enabled:
        raise ValueError(
            "edge execution is not implemented: DI acquisition, all-resource stop, "
            "authorized reset and restart latch persistence are still required"
        )


def main(args=None) -> None:
    import rclpy
    from robot_interfaces_qos import latched
    from robot_rt_control_interfaces.msg import (
        ModuleState,
        ModuleStateArray,
        MotionExecutionState,
        ProtectiveStopState,
        SensorStatus,
        SensorStatusArray,
    )
    from robot_rt_control_interfaces.srv import ResetProtectiveStop
    from robot_system_interfaces.msg import ErrorInfo
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from rt_control_interfaces.msg import PlcIoState
    from sensor_msgs.msg import BatteryState

    def set_uuid(message, value: uuid.UUID) -> None:
        message.uuid = list(value.bytes)

    def error_message(text: str, *, retryable: bool = True):
        message = ErrorInfo()
        message.code = 130
        message.message = text
        message.retryable = retryable
        message.severity = ErrorInfo.WARN
        message.source = "rt_control"
        message.detail = ""
        return message

    class ModuleStateAdapter(Node):
        def __init__(self) -> None:
            super().__init__("module_state_adapter")
            self.declare_parameter("plc_state_topic", "/plc/io_state")
            self.declare_parameter("battery_state_topic", "/battery_state")
            self.declare_parameter("sensor_status_topic", "/rt_control/sensors/status")
            self.declare_parameter("motion_state_topic", "/rt_control/motion/state")
            self.declare_parameter("module_state_topic", "/rt_control/modules/state")
            self.declare_parameter("protective_stop_topic", "/protective_stop/state")
            self.declare_parameter("protective_stop_reset_service", "/protective_stop/reset")
            self.declare_parameter("publish_period_s", 1.0)
            self.declare_parameter("plc_timeout_s", 1.5)
            self.declare_parameter("battery_timeout_s", 6.0)
            self.declare_parameter("ultrasonic_timeout_s", 2.5)
            self.declare_parameter("motion_state_timeout_s", 0.5)
            self.declare_parameter("edge.installed", False)
            self.declare_parameter("edge.protection_enabled", False)

            require_edge_preview_only(
                bool(self.get_parameter("edge.protection_enabled").value)
            )
            self._instance_id = uuid.uuid4()
            self._protective_core = ProtectiveStopCore(
                installed=bool(self.get_parameter("edge.installed").value),
                enabled=bool(self.get_parameter("edge.protection_enabled").value),
            )
            self._plc_message = None
            self._plc_received = None
            self._battery_message = None
            self._battery_received = None
            self._sensor_states = {}
            self._motion_states = {}
            self._publisher = self.create_publisher(
                ModuleStateArray,
                str(self.get_parameter("module_state_topic").value),
                latched(),
            )
            self._protective_publisher = self.create_publisher(
                ProtectiveStopState,
                str(self.get_parameter("protective_stop_topic").value),
                latched(),
            )
            self._plc_subscription = self.create_subscription(
                PlcIoState,
                str(self.get_parameter("plc_state_topic").value),
                self._on_plc,
                QoSProfile(depth=10),
            )
            self._battery_subscription = self.create_subscription(
                BatteryState,
                str(self.get_parameter("battery_state_topic").value),
                self._on_battery,
                QoSProfile(depth=10),
            )
            self._sensor_subscription = self.create_subscription(
                SensorStatusArray,
                str(self.get_parameter("sensor_status_topic").value),
                self._on_sensor_status,
                QoSProfile(depth=20),
            )
            self._motion_subscription = self.create_subscription(
                MotionExecutionState,
                str(self.get_parameter("motion_state_topic").value),
                self._on_motion_state,
                QoSProfile(depth=20),
            )
            self._reset_service = self.create_service(
                ResetProtectiveStop,
                str(self.get_parameter("protective_stop_reset_service").value),
                self._reset_protective_stop,
            )
            self._timer = self.create_timer(
                float(self.get_parameter("publish_period_s").value), self._publish
            )

        def _on_plc(self, message) -> None:
            self._plc_message = message
            self._plc_received = time.monotonic()

        def _on_battery(self, message) -> None:
            self._battery_message = message
            self._battery_received = time.monotonic()

        def _on_sensor_status(self, message) -> None:
            received = time.monotonic()
            for sensor in message.sensors:
                self._sensor_states[str(sensor.sensor_id)] = (sensor, received)

        def _on_motion_state(self, message) -> None:
            self._motion_states[str(message.resource_id)] = (message, time.monotonic())

        @staticmethod
        def _fresh(received, timeout) -> bool:
            if received is None:
                return False
            age = time.monotonic() - received
            return 0.0 <= age <= timeout

        def _vacuum_snapshot(self) -> ModuleSnapshot:
            fresh = self._fresh(
                self._plc_received, float(self.get_parameter("plc_timeout_s").value)
            )
            message = self._plc_message
            configured = bool(message is not None and message.hardware_configured)
            valid = bool(message is not None and fresh and message.data_fresh)
            errors = () if valid else (
                str(message.error) if message is not None and message.error else
                "vacuum hardware configuration or fresh data is unavailable",
            )
            return ModuleSnapshot(
                "vacuum_io",
                INSTALLATION_INSTALLED,
                IMPLEMENTATION_AVAILABLE,
                configured,
                configured,
                valid,
                "AVAILABLE" if valid else "NOT_ADMITTED",
                ("vacuum_pump", "vacuum_valves", "vacuum_observation"),
                errors,
            )

        def _battery_snapshot(self) -> ModuleSnapshot:
            fresh = self._fresh(
                self._battery_received,
                float(self.get_parameter("battery_timeout_s").value),
            )
            valid = bool(
                self._battery_message is not None
                and fresh
                and self._battery_message.present
            )
            return ModuleSnapshot(
                "battery_state",
                INSTALLATION_INSTALLED if valid else INSTALLATION_UNKNOWN,
                IMPLEMENTATION_AVAILABLE,
                True,
                True,
                valid,
                "AVAILABLE" if valid else "STALE_OR_UNCONFIRMED",
                ("battery_observation",),
                () if valid else ("battery observation is unavailable or stale",),
            )

        def _sensor_module_snapshots(self) -> Iterable[ModuleSnapshot]:
            ultrasonic_timeout = float(
                self.get_parameter("ultrasonic_timeout_s").value
            )
            groups = {
                "ultrasonic_unit1": [],
                "ultrasonic_unit6": [],
                "lpms_nav3_can": [],
                "left_wrist_force_sensor": [],
                "right_wrist_force_sensor": [],
            }
            for sensor, received in self._sensor_states.values():
                if sensor.module_id in groups:
                    groups[sensor.module_id].append((sensor, received))
            expected_counts = {
                "ultrasonic_unit1": 4,
                "ultrasonic_unit6": 4,
                "lpms_nav3_can": 1,
                "left_wrist_force_sensor": 1,
                "right_wrist_force_sensor": 1,
            }
            for module_id, samples in groups.items():
                timeout = ultrasonic_timeout if module_id.startswith("ultrasonic") else math.nan
                fresh = (
                    math.isfinite(timeout)
                    and len(samples) == expected_counts[module_id]
                    and all(self._fresh(received, timeout) for _, received in samples)
                )
                valid_states = {
                    SensorStatus.MEASUREMENT_VALID,
                    SensorStatus.MEASUREMENT_NO_TARGET,
                }
                valid = fresh and all(
                    int(sample.measurement_state) in valid_states for sample, _ in samples
                )
                observed = bool(samples)
                yield ModuleSnapshot(
                    module_id,
                    INSTALLATION_INSTALLED if observed else INSTALLATION_UNKNOWN,
                    IMPLEMENTATION_AVAILABLE if module_id.startswith("ultrasonic") else
                    IMPLEMENTATION_NOT_ADMITTED,
                    observed,
                    observed,
                    valid,
                    "AVAILABLE" if valid else "NOT_ADMITTED",
                    ("sensor_observation",),
                    () if valid else ("sensor configuration or fresh data is unavailable",),
                )

        def _motion_module_snapshots(self) -> Iterable[ModuleSnapshot]:
            timeout = float(self.get_parameter("motion_state_timeout_s").value)
            capabilities = {
                "left_pp": "left_pp_motion",
                "right_pp": "right_pp_motion",
                "updown": "updown_motion",
                "head": "head_motion",
            }
            for resource_id, capability in capabilities.items():
                entry = self._motion_states.get(resource_id)
                if entry is None:
                    yield ModuleSnapshot(
                        resource_id,
                        INSTALLATION_UNKNOWN,
                        IMPLEMENTATION_NOT_ADMITTED,
                        False,
                        False,
                        False,
                        "NOT_CONFIGURED",
                        (capability,),
                        (f"{resource_id} action configuration is unavailable",),
                    )
                    continue
                message, received = entry
                fresh = self._fresh(received, timeout)
                configured = int(message.execution_state) != 0
                data_valid = fresh and bool(message.feedback_observation.valid)
                yield ModuleSnapshot(
                    resource_id,
                    INSTALLATION_INSTALLED if configured else INSTALLATION_UNKNOWN,
                    IMPLEMENTATION_AVAILABLE if configured else IMPLEMENTATION_NOT_ADMITTED,
                    configured,
                    configured,
                    data_valid,
                    "AVAILABLE" if configured and data_valid else "NOT_ADMITTED",
                    (capability,),
                    () if configured and data_valid else
                    (f"{resource_id} configuration or fresh feedback is unavailable",),
                )

        def _to_message(self, snapshot: ModuleSnapshot):
            message = ModuleState()
            message.module_id = snapshot.module_id
            message.installation_state = snapshot.installation_state
            message.implementation_state = snapshot.implementation_state
            message.enabled = snapshot.enabled
            message.configuration_valid = snapshot.configuration_valid
            message.data_valid = snapshot.data_valid
            message.operational_state = snapshot.operational_state
            message.capability_names = list(snapshot.capability_names)
            message.errors = [error_message(item) for item in snapshot.errors]
            return message

        def _publish(self) -> None:
            message = ModuleStateArray()
            message.header.stamp = self.get_clock().now().to_msg()
            set_uuid(message.producer_instance_id, self._instance_id)
            snapshots = [self._vacuum_snapshot(), self._battery_snapshot()]
            snapshots.extend(self._sensor_module_snapshots())
            snapshots.extend(self._motion_module_snapshots())
            snapshots.extend(static_deferred_modules())
            message.modules = [self._to_message(item) for item in snapshots]
            self._publisher.publish(message)

            protective_snapshot = self._protective_core.snapshot()
            protective = ProtectiveStopState()
            protective.header = message.header
            protective.producer_instance_id = message.producer_instance_id
            protective.edge_installed = protective_snapshot.installed
            protective.edge_protection_enabled = protective_snapshot.enabled
            protective.input_state_valid = protective_snapshot.inputs_valid
            protective.latched = protective_snapshot.latched
            protective.active_input_ids = list(protective_snapshot.active_inputs)
            protective.all_required_motion_stopped = False
            protective.stop_confirmation_valid = False
            protective.reset_conditions_met = protective_snapshot.reset_conditions_met
            if protective_snapshot.reason:
                protective.reasons = [protective_snapshot.reason]
                protective.errors = [error_message(protective_snapshot.reason)]
            self._protective_publisher.publish(protective)

        def _reset_protective_stop(self, request, response):
            cleared, snapshot = self._protective_core.reset(
                str(request.operator_confirmation_id)
            )
            response.accepted = cleared
            response.latch_cleared = cleared
            response.state.edge_installed = snapshot.installed
            response.state.edge_protection_enabled = snapshot.enabled
            response.state.input_state_valid = snapshot.inputs_valid
            response.state.latched = snapshot.latched
            response.state.active_input_ids = list(snapshot.active_inputs)
            response.state.reset_conditions_met = snapshot.reset_conditions_met
            if snapshot.reason:
                response.state.reasons = [snapshot.reason]
            response.error = (
                error_message(snapshot.reason or "protective stop reset rejected")
                if not cleared else ErrorInfo()
            )
            return response

    rclpy.init(args=args)
    node = ModuleStateAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
