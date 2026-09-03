import sys
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rt_control_operator_ui.fault_catalog import FaultCatalog  # noqa: E402
from rt_control_operator_ui.presenter import OperatorPresenter  # noqa: E402


class FakeWindow:
    def __init__(self) -> None:
        self.faults = []
        self.battery = None
        self.overall_state = ""
        self.bus_state = ""
        self.command_availability = None

    def ingest_fault(self, event) -> None:
        self.faults.append(event)

    def update_battery(self, display) -> None:
        self.battery = display

    def set_overall_state(self, state: str) -> None:
        self.overall_state = state

    def set_bus_state(self, text: str) -> None:
        self.bus_state = text

    def set_command_availability(self, *, reset: bool, enable: bool, stop: bool) -> None:
        self.command_availability = (reset, enable, stop)


def value(key: str, item: str):
    return SimpleNamespace(key=key, value=item)


def test_presenter_maps_diagnostics_safety_and_battery_to_view() -> None:
    window = FakeWindow()
    catalog = FaultCatalog.load_directory(PACKAGE_ROOT / "config" / "fault_catalog")
    presenter = OperatorPresenter(window, catalog)
    diagnostic = SimpleNamespace(
        status=[
            SimpleNamespace(
                name="/robot/rt_control/ethercat/slave_8",
                level=2,
                message="CiA402 fault",
                values=[
                    value("joint", "left_joint2"),
                    value("position", "8"),
                    value("vendor", "TI5"),
                    value("status_word_hex", "0x1218"),
                    value("error_code_hex", "0x7500"),
                ],
            )
        ]
    )

    presenter.on_diagnostics(diagnostic, timestamp_s=10.0)
    presenter.on_safety(
        SimpleNamespace(
            state="FAULT",
            ethercat_ok=False,
            canopen_ok=True,
            control_enabled=False,
        )
    )
    presenter.on_battery(
        SimpleNamespace(present=True, percentage=0.8, voltage=50.5),
        received_s=20.0,
    )
    presenter.refresh_battery(now_s=21.0)

    assert len(window.faults) == 1
    assert window.faults[0].message == "通信错误"
    assert window.overall_state == "FAULT"
    assert window.bus_state == (
        "EtherCAT 异常 · slaves -- · WCerr -- · CAN 正常 · 已失能"
    )
    assert window.battery.percent_text == "80%"


def test_presenter_marks_battery_stale_without_receiving_a_new_message() -> None:
    window = FakeWindow()
    catalog = FaultCatalog.load_directory(PACKAGE_ROOT / "config" / "fault_catalog")
    presenter = OperatorPresenter(window, catalog)
    presenter.on_battery(
        SimpleNamespace(present=True, percentage=0.5, voltage=48.0),
        received_s=1.0,
    )

    presenter.refresh_battery(now_s=8.0)

    assert window.battery.state == "stale"


def test_presenter_gates_reset_from_structured_enable_stage_and_master_health() -> None:
    window = FakeWindow()
    catalog = FaultCatalog.load_directory(PACKAGE_ROOT / "config" / "fault_catalog")
    presenter = OperatorPresenter(window, catalog)
    diagnostics = SimpleNamespace(
        status=[
            SimpleNamespace(
                name="/robot/rt_control/ethercat/master",
                level=0,
                message="healthy",
                values=[],
            ),
            SimpleNamespace(
                name="/robot/rt_control/enable_manager",
                level=2,
                message="failed",
                values=[value("state", "FAILED"), value("stage", "fault_requires_reset")],
            ),
        ]
    )
    presenter.on_diagnostics(diagnostics, timestamp_s=1.0)

    presenter.on_safety(
        SimpleNamespace(
            state="NOT_READY",
            ethercat_ok=False,
            canopen_ok=True,
            control_enabled=False,
            enable_manager_ok=False,
            active_faults=["drive fault"],
        )
    )

    assert window.command_availability == (True, False, True)


def test_presenter_exposes_master_link_slave_count_and_wc_errors() -> None:
    window = FakeWindow()
    catalog = FaultCatalog.load_directory(PACKAGE_ROOT / "config" / "fault_catalog")
    presenter = OperatorPresenter(window, catalog)
    diagnostics = SimpleNamespace(
        status=[
            SimpleNamespace(
                name="/robot/rt_control/ethercat/master",
                level=b"\x00",
                message="healthy",
                values=[
                    value("link", "1.000000"),
                    value("slaves_responding", "16.000000"),
                    value("wc_error_count", "42.000000"),
                ],
            )
        ]
    )

    presenter.on_diagnostics(diagnostics, timestamp_s=1.0)
    presenter.on_safety(
        SimpleNamespace(
            state="READY",
            ethercat_ok=True,
            canopen_ok=True,
            control_enabled=True,
            enable_manager_ok=True,
        )
    )

    assert window.bus_state == "EtherCAT UP · slaves 16 · WCerr 42 · CAN 正常 · 已使能"
