from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


OK = 0
WARN = 1
ERROR = 2
STALE = 3

ETHERCAT_SLAVE_NAMES = tuple(
    f"/robot/rt_control/ethercat/slave_{position}"
    for position in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15)
)
CANOPEN_NODE_NAMES = (
    "/robot/rt_control/canopen/node_2",
    "/robot/rt_control/canopen/node_3",
)
ENABLE_MANAGER_NAME = "/robot/rt_control/enable_manager"
ETHERCAT_MASTER_NAME = "/robot/rt_control/ethercat/master"


@dataclass(frozen=True)
class ComponentSnapshot:
    name: str
    level: int
    fresh: bool
    message: str
    values: Mapping[str, str]


@dataclass(frozen=True)
class PlcHealthSnapshot:
    connected: bool
    data_fresh: bool
    error: str = ""


@dataclass(frozen=True)
class BatteryHealthSnapshot:
    present: bool
    fresh: bool


@dataclass(frozen=True)
class SafetySummary:
    safe_to_start_motion: bool
    control_enabled: bool
    enable_manager_ok: bool
    ethercat_ok: bool
    canopen_ok: bool
    plc_ok: bool
    bms_ok: bool
    state: str
    active_faults: tuple[str, ...]
    sources: Mapping[str, Any]

    @property
    def readiness_reason(self) -> str:
        if self.safe_to_start_motion:
            return "ready"
        if not self.control_enabled and not self.active_faults:
            return "control disabled"
        if self.active_faults:
            return "; ".join(self.active_faults)
        return "not ready"


def _component_ok(component: ComponentSnapshot) -> bool:
    return bool(component.fresh) and int(component.level) == OK


def _diagnostic_level(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            return ERROR
        return int(value[0])
    return int(value)


def _fault(label: str, component: ComponentSnapshot) -> str:
    if not component.fresh:
        return f"{label}: stale or missing"
    if component.message:
        return f"{component.name}: {component.message}"
    return f"{component.name}: diagnostic level {component.level}"


def build_safety_summary(
    *,
    enable_manager: ComponentSnapshot,
    ethercat_master: ComponentSnapshot,
    ethercat_slaves: Sequence[ComponentSnapshot],
    canopen_nodes: Sequence[ComponentSnapshot],
    plc: PlcHealthSnapshot,
    bms: BatteryHealthSnapshot,
) -> SafetySummary:
    enable_state = str(enable_manager.values.get("state", ""))
    enable_stage = str(enable_manager.values.get("stage", ""))
    control_enabled = _component_ok(enable_manager) and enable_state == "ENABLED"
    enable_manager_ok = (
        bool(enable_manager.fresh)
        and int(enable_manager.level) == OK
        and enable_state != "FAILED"
        and enable_stage != "restart_required"
    )

    ethercat_ok = _component_ok(ethercat_master) and len(ethercat_slaves) == len(
        ETHERCAT_SLAVE_NAMES
    ) and all(_component_ok(slave) for slave in ethercat_slaves)
    canopen_ok = len(canopen_nodes) == len(CANOPEN_NODE_NAMES) and all(
        _component_ok(node) for node in canopen_nodes
    )
    plc_ok = bool(plc.connected) and bool(plc.data_fresh)
    bms_ok = bool(bms.present) and bool(bms.fresh)

    active_faults: list[str] = []
    if not enable_manager_ok:
        active_faults.append(_fault("enable_manager", enable_manager))
    if not ethercat_ok:
        if not _component_ok(ethercat_master):
            active_faults.append(_fault("ethercat_master", ethercat_master))
        for slave in ethercat_slaves:
            if not _component_ok(slave):
                active_faults.append(_fault(slave.name, slave))
                break
        if len(ethercat_slaves) != len(ETHERCAT_SLAVE_NAMES):
            active_faults.append("ethercat_slaves: incomplete diagnostic set")
    if not canopen_ok:
        for node in canopen_nodes:
            if not _component_ok(node):
                active_faults.append(_fault(node.name, node))
                break
        if len(canopen_nodes) != len(CANOPEN_NODE_NAMES):
            active_faults.append("canopen_nodes: incomplete diagnostic set")
    if not plc_ok:
        active_faults.append(
            f"plc: {'stale or disconnected' if not plc.error else plc.error}"
        )
    if not bms_ok:
        active_faults.append("bms: stale or missing")

    safe_to_start_motion = (
        control_enabled
        and enable_manager_ok
        and ethercat_ok
        and canopen_ok
        and plc_ok
        and bms_ok
    )
    if safe_to_start_motion:
        state = "READY"
    elif enable_manager_ok and not control_enabled and ethercat_ok and canopen_ok and plc_ok and bms_ok:
        state = "DISABLED"
    else:
        state = "NOT_READY"

    return SafetySummary(
        safe_to_start_motion=safe_to_start_motion,
        control_enabled=control_enabled,
        enable_manager_ok=enable_manager_ok,
        ethercat_ok=ethercat_ok,
        canopen_ok=canopen_ok,
        plc_ok=plc_ok,
        bms_ok=bms_ok,
        state=state,
        active_faults=tuple(active_faults),
        sources={
            "enable_manager": enable_manager,
            "ethercat_master": ethercat_master,
            "ethercat_slaves": list(ethercat_slaves),
            "canopen_nodes": list(canopen_nodes),
            "plc": plc,
            "bms": bms,
        },
    )


def main(args=None) -> None:
    import rclpy
    from robot_rt_control_interfaces.msg import SafetyState
    from robot_system_interfaces.msg import DomainReadiness
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rt_control_interfaces.msg import PlcIoState
    from sensor_msgs.msg import BatteryState

    class RtStatusAdapterNode(Node):
        def __init__(self) -> None:
            super().__init__("rt_status_adapter")
            self.declare_parameter("diagnostics_topic", "/diagnostics")
            self.declare_parameter("plc_state_topic", "/plc/io_state")
            self.declare_parameter("battery_state_topic", "/battery_state")
            self.declare_parameter("safety_state_topic", "/control/safety_state")
            self.declare_parameter("readiness_topic", "/rt_control/readiness")
            self.declare_parameter("diagnostic_timeout_s", 3.0)
            self.declare_parameter("plc_timeout_s", 1.5)
            self.declare_parameter("bms_timeout_s", 6.0)
            self.declare_parameter("safety_publish_period_s", 0.1)
            self.declare_parameter("readiness_publish_period_s", 1.0)
            self.declare_parameter("version", "0.1.0")
            self.declare_parameter(
                "config_summary",
                "14-axis EtherCAT, CANopen tracks, PLC vacuum, BMS",
            )

            self._callback_group = ReentrantCallbackGroup()
            self._diagnostic_timeout_s = float(
                self.get_parameter("diagnostic_timeout_s").value
            )
            self._plc_timeout_s = float(self.get_parameter("plc_timeout_s").value)
            self._bms_timeout_s = float(self.get_parameter("bms_timeout_s").value)
            self._version = str(self.get_parameter("version").value)
            self._config_summary = str(self.get_parameter("config_summary").value)
            self._diagnostics: dict[str, tuple[Any, float]] = {}
            self._plc_message = None
            self._plc_received_s: float | None = None
            self._bms_message = None
            self._bms_received_s: float | None = None
            self._last_readiness_signature: tuple[bool, str, str] | None = None
            self._last_readiness_publish_s = 0.0

            self._diagnostics_subscription = self.create_subscription(
                DiagnosticArray,
                str(self.get_parameter("diagnostics_topic").value),
                self._on_diagnostics,
                QoSProfile(depth=50),
                callback_group=self._callback_group,
            )
            self._plc_subscription = self.create_subscription(
                PlcIoState,
                str(self.get_parameter("plc_state_topic").value),
                self._on_plc,
                QoSProfile(depth=10),
                callback_group=self._callback_group,
            )
            self._bms_subscription = self.create_subscription(
                BatteryState,
                str(self.get_parameter("battery_state_topic").value),
                self._on_bms,
                QoSProfile(depth=10),
                callback_group=self._callback_group,
            )
            self._safety_publisher = self.create_publisher(
                SafetyState,
                str(self.get_parameter("safety_state_topic").value),
                QoSProfile(depth=10),
            )
            readiness_qos = QoSProfile(depth=1)
            readiness_qos.reliability = ReliabilityPolicy.RELIABLE
            readiness_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._readiness_publisher = self.create_publisher(
                DomainReadiness,
                str(self.get_parameter("readiness_topic").value),
                readiness_qos,
            )
            self._timer = self.create_timer(
                float(self.get_parameter("safety_publish_period_s").value),
                self._publish,
                callback_group=self._callback_group,
            )

        def _on_diagnostics(self, message: DiagnosticArray) -> None:
            received = time.monotonic()
            for status in message.status:
                self._diagnostics[status.name] = (status, received)

        def _on_plc(self, message: PlcIoState) -> None:
            self._plc_message = message
            self._plc_received_s = time.monotonic()

        def _on_bms(self, message: BatteryState) -> None:
            self._bms_message = message
            self._bms_received_s = time.monotonic()

        def _publish(self) -> None:
            summary = self._build_summary()
            safety = SafetyState()
            safety.header.stamp = self.get_clock().now().to_msg()
            safety.header.frame_id = "rt_control"
            safety.safe_to_start_motion = summary.safe_to_start_motion
            safety.control_enabled = summary.control_enabled
            safety.enable_manager_ok = summary.enable_manager_ok
            safety.ethercat_ok = summary.ethercat_ok
            safety.canopen_ok = summary.canopen_ok
            safety.plc_ok = summary.plc_ok
            safety.bms_ok = summary.bms_ok
            safety.state = summary.state
            safety.active_faults = list(summary.active_faults)
            self._safety_publisher.publish(safety)

            now_s = time.monotonic()
            signature = (
                summary.safe_to_start_motion,
                summary.state,
                summary.readiness_reason,
            )
            period_s = float(self.get_parameter("readiness_publish_period_s").value)
            if (
                signature != self._last_readiness_signature
                or now_s - self._last_readiness_publish_s >= period_s
            ):
                readiness = DomainReadiness()
                readiness.header = safety.header
                readiness.ready = summary.safe_to_start_motion
                readiness.state = summary.state
                readiness.reason = summary.readiness_reason
                readiness.version = self._version
                readiness.config_summary = self._config_summary
                self._readiness_publisher.publish(readiness)
                self._last_readiness_signature = signature
                self._last_readiness_publish_s = now_s

        def _build_summary(self) -> SafetySummary:
            return build_safety_summary(
                enable_manager=self._component(ENABLE_MANAGER_NAME),
                ethercat_master=self._component(ETHERCAT_MASTER_NAME),
                ethercat_slaves=[
                    self._component(name) for name in ETHERCAT_SLAVE_NAMES
                ],
                canopen_nodes=[self._component(name) for name in CANOPEN_NODE_NAMES],
                plc=self._plc_health(),
                bms=self._bms_health(),
            )

        def _component(self, name: str) -> ComponentSnapshot:
            status_and_time = self._diagnostics.get(name)
            if status_and_time is None:
                return ComponentSnapshot(name, STALE, False, "missing", {})
            status, received = status_and_time
            fresh = time.monotonic() - received <= self._diagnostic_timeout_s
            return ComponentSnapshot(
                name=name,
                level=_diagnostic_level(status.level),
                fresh=fresh,
                message=str(status.message),
                values={item.key: item.value for item in status.values},
            )

        def _plc_health(self) -> PlcHealthSnapshot:
            message = self._plc_message
            received = self._plc_received_s
            if message is None or received is None:
                return PlcHealthSnapshot(False, False, "missing")
            fresh_by_age = time.monotonic() - received <= self._plc_timeout_s
            return PlcHealthSnapshot(
                connected=bool(message.connected),
                data_fresh=bool(message.data_fresh) and fresh_by_age,
                error=str(message.error),
            )

        def _bms_health(self) -> BatteryHealthSnapshot:
            message = self._bms_message
            received = self._bms_received_s
            if message is None or received is None:
                return BatteryHealthSnapshot(False, False)
            age_s = time.monotonic() - received
            voltage_ok = math.isfinite(float(message.voltage))
            percentage_ok = math.isfinite(float(message.percentage))
            return BatteryHealthSnapshot(
                present=bool(message.present) and voltage_ok and percentage_ok,
                fresh=0.0 <= age_s <= self._bms_timeout_s,
            )

    rclpy.init(args=args)
    node = RtStatusAdapterNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
