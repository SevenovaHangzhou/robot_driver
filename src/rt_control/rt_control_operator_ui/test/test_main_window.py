import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from python_qt_binding.QtWidgets import (  # noqa: E402
    QApplication,
    QLabel,
    QListWidget,
    QProgressBar,
    QPushButton,
    QTableView,
    QTextBrowser,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rt_control_operator_ui.battery import project_battery  # noqa: E402
from rt_control_operator_ui.fault_log import FaultEvent  # noqa: E402
from rt_control_operator_ui.main_window import OperatorConsoleWindow  # noqa: E402


APPLICATION = QApplication.instance() or QApplication([])


def app() -> QApplication:
    return APPLICATION


def fault(timestamp_s: float) -> FaultEvent:
    return FaultEvent(
        timestamp_s=timestamp_s,
        source="ethercat/slave_8",
        joint="left_joint2",
        address=8,
        vendor="TI5",
        status_word=0x1218,
        error_code=0x7500,
        message="通信错误",
        severity="error",
        reset_policy="reset_once_after_link_recovered",
        possible_causes=("EtherCAT 链路中断",),
        operator_actions=("确认 EtherCAT Link UP",),
        manual_title="钛虎 C1 关节模组通讯使用说明",
        manual_version="1.1.1",
        manual_pages="77-78",
        source_reference="sha256:test",
    )


def test_window_has_compact_operator_sections_without_joint_table() -> None:
    app()
    window = OperatorConsoleWindow(domain_id="7")

    assert window.findChild(QLabel, "overallStateLabel") is not None
    assert window.findChild(QLabel, "domainIdLabel").text() == "Domain 7"
    assert window.findChild(QLabel, "cpu14Label") is not None
    assert window.findChild(QLabel, "batteryLabel") is not None
    assert window.findChild(QLabel, "firstFaultLabel") is not None
    assert window.findChild(QListWidget, "faultLog") is not None
    assert window.findChild(QTableView, "jointStatusTable") is None
    window.close()


def test_duplicate_fault_updates_one_scrollable_row_and_pins_first_fault() -> None:
    app()
    window = OperatorConsoleWindow()

    window.ingest_fault(fault(1.0))
    window.ingest_fault(fault(2.0))

    fault_log = window.findChild(QListWidget, "faultLog")
    first_fault = window.findChild(QLabel, "firstFaultLabel")
    assert fault_log.count() == 1
    assert "×2" in fault_log.item(0).text()
    assert "left_joint2" in first_fault.text()
    assert "0x7500" in first_fault.text()
    assert fault_log.currentRow() == 0
    assert "复位策略" in window.findChild(QTextBrowser, "faultDetail").toPlainText()
    detail = window.findChild(QTextBrowser, "faultDetail").toPlainText()
    assert "EtherCAT 链路中断" in detail
    assert "确认 EtherCAT Link UP" in detail
    assert "1.1.1" in detail
    assert "77-78" in detail
    window.close()


def test_battery_label_uses_projected_battery_state() -> None:
    app()
    window = OperatorConsoleWindow()

    window.update_battery(project_battery(True, 0.8, 50.5, 1.0))

    battery = window.findChild(QLabel, "batteryLabel")
    battery_level = window.findChild(QProgressBar, "batteryLevel")
    assert battery.text() == "80%"
    assert battery_level.value() == 80
    assert "50.5 V" in battery.toolTip()
    window.close()


def test_command_buttons_emit_directly_without_confirmation_and_debounce_double_click() -> None:
    application = app()
    window = OperatorConsoleWindow()
    requested: list[str] = []
    window.fault_reset_requested.connect(lambda: requested.append("reset_fault"))
    window.enable_requested.connect(lambda: requested.append("enable"))
    window.software_stop_requested.connect(lambda: requested.append("stop"))
    window.set_command_availability(reset=True, enable=True, stop=True)

    reset = window.findChild(QPushButton, "faultResetButton")
    enable = window.findChild(QPushButton, "enableButton")
    stop = window.findChild(QPushButton, "softwareStopButton")
    reset.click()
    reset.click()
    application.processEvents()

    assert requested == ["reset_fault"]
    assert not reset.isEnabled()
    assert not enable.isEnabled()
    assert not stop.isEnabled()
    window.finish_command("复位失败：未自动重试")
    assert reset.isEnabled()
    assert enable.isEnabled()
    assert stop.isEnabled()
    window.close()
