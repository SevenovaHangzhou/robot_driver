from __future__ import annotations

from datetime import datetime

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QSplitter,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)

from .battery import BatteryDisplay
from .cpu_metrics import CpuUsageDisplay
from .fault_log import FaultEvent, FaultIncidentModel, FaultRow


_STYLE = """
QMainWindow, QWidget { background: #12171C; color: #E8EDF2; }
QFrame#topBar, QFrame#firstFaultCard, QFrame#commandStrip {
  background: #1B232B; border: 1px solid #2B3945; border-radius: 5px;
}
QLabel#overallStateLabel { color: #36D399; font-size: 23px; font-weight: 700; }
QLabel#firstFaultEyebrow { color: #FF817A; font-size: 11px; font-weight: 700; }
QLabel#firstFaultLabel { color: #FFFFFF; font-size: 16px; font-weight: 600; }
QLabel#cpu14Label, QLabel#batteryLabel, QLabel#busStateLabel, QLabel#domainIdLabel {
  color: #D6E1E9; font-family: "DejaVu Sans Mono"; font-size: 13px;
}
QProgressBar#batteryLevel {
  border: 1px solid #82939F; border-radius: 2px; background: #11161A;
}
QProgressBar#batteryLevel::chunk { background: #36D399; }
QLabel#commandStateLabel { color: #8FA0AE; }
QPushButton {
  background: #25313A; border: 1px solid #405260; border-radius: 4px;
  color: #E8EDF2; min-height: 34px; min-width: 112px; padding: 0 12px;
}
QPushButton:hover { background: #30404C; }
QPushButton:disabled { color: #667682; background: #1A2228; border-color: #293640; }
QPushButton#faultResetButton { border-color: #A97928; }
QPushButton#enableButton { border-color: #278A68; }
QPushButton#softwareStopButton { background: #8F3030; border-color: #D95B56; }
QPushButton#softwareStopButton:hover { background: #A83A38; }
QListWidget#faultLog {
  background: #151C22; border: 1px solid #2B3945; outline: 0;
  alternate-background-color: #192129; font-family: "DejaVu Sans Mono";
}
QListWidget#faultLog::item { padding: 8px; border-bottom: 1px solid #26323C; }
QListWidget#faultLog::item:selected { background: #2B3945; color: #FFFFFF; }
QTextBrowser#faultDetail {
  background: #151C22; border: 1px solid #2B3945; color: #D6E1E9;
}
"""


class OperatorConsoleWindow(QMainWindow):
    fault_reset_requested = Signal()
    enable_requested = Signal()
    software_stop_requested = Signal()

    def __init__(self, domain_id: str | int = "0") -> None:
        super().__init__()
        self.setWindowTitle("RT-Control 操作员控制台")
        self.resize(1120, 760)
        self.setStyleSheet(_STYLE)
        self._faults = FaultIncidentModel(max_rows=500)
        self._command_pending = False
        self._command_availability = {"reset": False, "enable": False, "stop": False}

        central = QWidget(self)
        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)
        self.setCentralWidget(central)

        top = QFrame()
        top.setObjectName("topBar")
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(14, 10, 14, 10)
        self._overall_state = QLabel("OFFLINE")
        self._overall_state.setObjectName("overallStateLabel")
        self._bus_state = QLabel("EtherCAT -- · WC --")
        self._bus_state.setObjectName("busStateLabel")
        self._domain = QLabel(f"Domain {domain_id}")
        self._domain.setObjectName("domainIdLabel")
        self._cpu = QLabel("CPU14 --%")
        self._cpu.setObjectName("cpu14Label")
        battery_widget = QWidget()
        battery_layout = QHBoxLayout(battery_widget)
        battery_layout.setContentsMargins(0, 0, 0, 0)
        battery_layout.setSpacing(6)
        self._battery_level = QProgressBar()
        self._battery_level.setObjectName("batteryLevel")
        self._battery_level.setRange(0, 100)
        self._battery_level.setValue(0)
        self._battery_level.setTextVisible(False)
        self._battery_level.setFixedSize(34, 14)
        self._battery = QLabel("--%")
        self._battery.setObjectName("batteryLabel")
        battery_layout.addWidget(self._battery_level)
        battery_layout.addWidget(self._battery)
        top_layout.addWidget(self._overall_state)
        top_layout.addStretch(1)
        top_layout.addWidget(self._domain)
        top_layout.addSpacing(20)
        top_layout.addWidget(self._bus_state)
        top_layout.addSpacing(20)
        top_layout.addWidget(self._cpu)
        top_layout.addSpacing(20)
        top_layout.addWidget(battery_widget)
        root.addWidget(top)

        command_strip = QFrame()
        command_strip.setObjectName("commandStrip")
        command_layout = QHBoxLayout(command_strip)
        command_layout.setContentsMargins(14, 7, 14, 7)
        self._command_state = QLabel("等待状态数据")
        self._command_state.setObjectName("commandStateLabel")
        self._reset_button = QPushButton("Fault Reset")
        self._reset_button.setObjectName("faultResetButton")
        self._enable_button = QPushButton("Enable")
        self._enable_button.setObjectName("enableButton")
        self._stop_button = QPushButton("软件停机（非安全级）")
        self._stop_button.setObjectName("softwareStopButton")
        self._reset_button.clicked.connect(lambda: self._request_command("reset"))
        self._enable_button.clicked.connect(lambda: self._request_command("enable"))
        self._stop_button.clicked.connect(lambda: self._request_command("stop"))
        command_layout.addWidget(self._command_state)
        command_layout.addStretch(1)
        command_layout.addWidget(self._reset_button)
        command_layout.addSpacing(8)
        command_layout.addWidget(self._enable_button)
        command_layout.addSpacing(22)
        command_layout.addWidget(self._stop_button)
        root.addWidget(command_strip)
        self._apply_command_availability()

        first_card = QFrame()
        first_card.setObjectName("firstFaultCard")
        first_layout = QVBoxLayout(first_card)
        first_layout.setContentsMargins(14, 9, 14, 10)
        eyebrow = QLabel("第一故障")
        eyebrow.setObjectName("firstFaultEyebrow")
        self._first_fault = QLabel("当前没有记录到故障")
        self._first_fault.setObjectName("firstFaultLabel")
        self._first_fault.setTextInteractionFlags(Qt.TextSelectableByMouse)
        first_layout.addWidget(eyebrow)
        first_layout.addWidget(self._first_fault)
        root.addWidget(first_card)

        splitter = QSplitter(Qt.Vertical)
        self._fault_log = QListWidget()
        self._fault_log.setObjectName("faultLog")
        self._fault_log.setAlternatingRowColors(True)
        self._fault_log.currentRowChanged.connect(self._show_detail)
        self._detail = QTextBrowser()
        self._detail.setObjectName("faultDetail")
        self._detail.setPlainText("选择一条故障查看原始状态和处理策略。")
        splitter.addWidget(self._fault_log)
        splitter.addWidget(self._detail)
        splitter.setSizes([360, 180])
        root.addWidget(splitter, 1)

    def ingest_fault(self, event: FaultEvent) -> None:
        scrollbar = self._fault_log.verticalScrollBar()
        follow_latest = scrollbar.value() >= scrollbar.maximum()
        self._faults.ingest(event)
        selected_fingerprint = None
        selected = self._fault_log.currentRow()
        if 0 <= selected < len(self._faults.rows):
            selected_fingerprint = self._faults.rows[selected].event.fingerprint

        self._fault_log.clear()
        selected_row = -1
        for index, row in enumerate(self._faults.rows):
            self._fault_log.addItem(self._format_row(row))
            if row.event.fingerprint == selected_fingerprint:
                selected_row = index
        if selected_row >= 0:
            self._fault_log.setCurrentRow(selected_row)
        elif follow_latest and self._fault_log.count() > 0:
            self._fault_log.setCurrentRow(self._fault_log.count() - 1)
            self._fault_log.scrollToBottom()

        first = self._faults.first_fault
        if first is not None:
            self._first_fault.setText(
                f"{self._time(first.timestamp_s)}  {first.joint or first.source}  "
                f"0x{first.error_code:04X}  {first.message}"
            )

    def update_battery(self, display: BatteryDisplay) -> None:
        self._battery_level.setValue(
            int(round((display.fraction or 0.0) * 100.0))
        )
        self._battery.setText(display.percent_text)
        self._battery.setToolTip(display.tooltip)
        self._battery_level.setToolTip(display.tooltip)

    def update_cpu(self, display: CpuUsageDisplay) -> None:
        self._cpu.setText(
            f"CPU14 {display.current_percent:.1f}% · 10s {display.average_10s_percent:.1f}% "
            f"· peak {display.peak_60s_percent:.1f}%"
        )
        self._cpu.setProperty("level", display.level)

    def set_overall_state(self, state: str) -> None:
        self._overall_state.setText(state)

    def set_bus_state(self, text: str) -> None:
        self._bus_state.setText(text)

    def set_command_availability(self, *, reset: bool, enable: bool, stop: bool) -> None:
        self._command_availability = {
            "reset": bool(reset),
            "enable": bool(enable),
            "stop": bool(stop),
        }
        self._apply_command_availability()

    def finish_command(self, result_text: str) -> None:
        self._command_pending = False
        self._command_state.setText(result_text)
        self._apply_command_availability()

    def _request_command(self, command: str) -> None:
        if self._command_pending or not self._command_availability[command]:
            return
        self._command_pending = True
        labels = {"reset": "正在复位故障…", "enable": "正在使能…", "stop": "正在软件停机…"}
        self._command_state.setText(labels[command])
        self._apply_command_availability()
        if command == "reset":
            self.fault_reset_requested.emit()
        elif command == "enable":
            self.enable_requested.emit()
        else:
            self.software_stop_requested.emit()

    def _apply_command_availability(self) -> None:
        self._reset_button.setEnabled(
            not self._command_pending and self._command_availability["reset"]
        )
        self._enable_button.setEnabled(
            not self._command_pending and self._command_availability["enable"]
        )
        self._stop_button.setEnabled(
            not self._command_pending and self._command_availability["stop"]
        )

    def _show_detail(self, index: int) -> None:
        if not 0 <= index < len(self._faults.rows):
            return
        row = self._faults.rows[index]
        event = row.event
        causes = "\n".join(f"  - {item}" for item in event.possible_causes) or "  - 未提供"
        actions = "\n".join(
            f"  {number}. {item}" for number, item in enumerate(event.operator_actions, 1)
        ) or "  - 未提供"
        self._detail.setPlainText(
            "\n".join(
                (
                    f"来源：{event.source}",
                    f"关节：{event.joint or '--'}",
                    f"地址：{event.address}",
                    f"厂商：{event.vendor}",
                    f"状态字：0x{event.status_word:04X}",
                    f"错误码：0x{event.error_code:04X}",
                    f"含义：{event.message}",
                    f"复位策略：{event.reset_policy}",
                    f"重复次数：{row.repeat_count}",
                    f"状态：{'活动' if row.active else '已恢复'}",
                    "",
                    "可能原因：",
                    causes,
                    "",
                    "操作步骤：",
                    actions,
                    "",
                    f"手册：{event.manual_title or '未收录'}",
                    f"版本：{event.manual_version or '未收录'}",
                    f"页码：{event.manual_pages or '未收录'}",
                    f"来源：{event.source_reference or 'unavailable'}",
                )
            )
        )

    @staticmethod
    def _format_row(row: FaultRow) -> str:
        event = row.event
        state = "活动" if row.active else "已恢复"
        return (
            f"{OperatorConsoleWindow._time(row.first_timestamp_s)}  "
            f"{event.joint or event.source}  0x{event.error_code:04X}  "
            f"{event.message}  ×{row.repeat_count}  {state}"
        )

    @staticmethod
    def _time(timestamp_s: float) -> str:
        return datetime.fromtimestamp(timestamp_s).strftime("%H:%M:%S.%f")[:-3]
