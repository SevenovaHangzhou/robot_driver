from __future__ import annotations

from typing import Any, Protocol

from .battery import BatteryDisplay, project_battery
from .diagnostic_adapter import DiagnosticFaultProjector
from .fault_catalog import FaultCatalog
from .fault_log import FaultEvent


def _diagnostic_level(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        return int(value[0]) if len(value) == 1 else 3
    return int(value)


def _whole_number(values: dict[str, str], key: str) -> str:
    try:
        return str(int(float(values[key])))
    except (KeyError, TypeError, ValueError):
        return "--"


class OperatorView(Protocol):
    def ingest_fault(self, event: FaultEvent) -> None: ...

    def update_battery(self, display: BatteryDisplay) -> None: ...

    def set_overall_state(self, state: str) -> None: ...

    def set_bus_state(self, text: str) -> None: ...

    def set_command_availability(
        self, *, reset: bool, enable: bool, stop: bool
    ) -> None: ...


class OperatorPresenter:
    def __init__(self, view: OperatorView, catalog: FaultCatalog) -> None:
        self._view = view
        self._fault_projector = DiagnosticFaultProjector(catalog)
        self._battery_message: Any | None = None
        self._battery_received_s: float | None = None
        self._ethercat_master_healthy = False
        self._ethercat_master_values: dict[str, str] = {}
        self._enable_stage = ""

    def on_diagnostics(self, message: Any, timestamp_s: float) -> None:
        for status in getattr(message, "status", ()):
            values = {
                str(item.key): str(item.value)
                for item in getattr(status, "values", ())
            }
            if status.name == "/robot/rt_control/ethercat/master":
                self._ethercat_master_healthy = _diagnostic_level(status.level) == 0
                self._ethercat_master_values = values
            elif status.name == "/robot/rt_control/enable_manager":
                self._enable_stage = values.get("stage", "")
            event = self._fault_projector.project(status, timestamp_s)
            if event is not None:
                self._view.ingest_fault(event)

    def on_safety(self, message: Any) -> None:
        self._view.set_overall_state(str(message.state))
        canopen = "正常" if bool(message.canopen_ok) else "异常"
        enabled = "已使能" if bool(message.control_enabled) else "已失能"
        link_value = self._ethercat_master_values.get("link")
        if link_value is None:
            link = "正常" if bool(message.ethercat_ok) else "异常"
        else:
            try:
                link = "UP" if float(link_value) >= 0.5 else "DOWN"
            except ValueError:
                link = "--"
        slaves = _whole_number(self._ethercat_master_values, "slaves_responding")
        wc_errors = _whole_number(self._ethercat_master_values, "wc_error_count")
        self._view.set_bus_state(
            f"EtherCAT {link} · slaves {slaves} · WCerr {wc_errors} · "
            f"CAN {canopen} · {enabled}"
        )
        state = str(message.state).upper()
        control_enabled = bool(message.control_enabled)
        self._view.set_command_availability(
            reset=(
                not control_enabled
                and self._ethercat_master_healthy
                and self._enable_stage == "fault_requires_reset"
            ),
            enable=(
                not control_enabled
                and bool(getattr(message, "enable_manager_ok", False))
                and bool(message.ethercat_ok)
                and bool(message.canopen_ok)
            ),
            stop=state != "OFFLINE",
        )

    def on_battery(self, message: Any, received_s: float) -> None:
        self._battery_message = message
        self._battery_received_s = float(received_s)

    def refresh_battery(self, now_s: float) -> None:
        message = self._battery_message
        received_s = self._battery_received_s
        if message is None or received_s is None:
            self._view.update_battery(project_battery(False, 0.0, float("nan"), 0.0))
            return
        self._view.update_battery(
            project_battery(
                bool(message.present),
                float(message.percentage),
                float(message.voltage),
                float(now_s) - received_s,
            )
        )
