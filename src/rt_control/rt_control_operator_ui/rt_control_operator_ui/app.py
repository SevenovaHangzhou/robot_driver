from __future__ import annotations

import os
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtCore import QObject, QTimer, Signal
from python_qt_binding.QtWidgets import QApplication

from .command_gateway import CommandGateway
from .cpu_metrics import CpuUsageHistory, parse_proc_stat
from .fault_catalog import FaultCatalog
from .main_window import OperatorConsoleWindow
from .presenter import OperatorPresenter


class RosSignals(QObject):
    diagnostics = Signal(object, float)
    safety = Signal(object)
    battery = Signal(object, float)
    command_result = Signal(int, bool, str)
    shutdown_requested = Signal()


def spin_ros_until_stopped(
    executor: Any,
    shutdown_exception: type[BaseException],
    on_stopped: Callable[[], None],
) -> None:
    try:
        executor.spin()
    except shutdown_exception:
        pass
    finally:
        on_stopped()


class RosCommandDispatcher:
    def __init__(self, node, signals, rt_enable_type, set_enabled_type) -> None:
        self._node = node
        self._signals = signals
        self._rt_enable_type = rt_enable_type
        self._set_enabled_type = set_enabled_type
        self._reset_client = node.create_client(rt_enable_type, "/rt/reset_fault")
        self._control_client = node.create_client(
            set_enabled_type, "/control/set_enabled"
        )
        self._lock = threading.Lock()
        self._timers: dict[int, threading.Timer] = {}

    def dispatch(self, command: str, operation_id: int) -> None:
        client = self._reset_client if command == "reset_fault" else self._control_client
        if not client.wait_for_service(timeout_sec=0.0):
            self._signals.command_result.emit(
                operation_id, False, f"{command} 服务不可用；未自动重试"
            )
            return

        if command == "reset_fault":
            request = self._rt_enable_type.Request()
        else:
            request = self._set_enabled_type.Request()
            request.enabled = command == "enable"
            request.reason = "rt-control-operator-ui"

        timeout = threading.Timer(
            45.0,
            lambda: self._signals.command_result.emit(
                operation_id, False, f"{command} 45 秒超时；未自动重试"
            ),
        )
        timeout.daemon = True
        with self._lock:
            self._timers[operation_id] = timeout
        timeout.start()
        future = client.call_async(request)
        future.add_done_callback(
            lambda completed: self._on_complete(operation_id, command, completed)
        )

    def _on_complete(self, operation_id: int, command: str, future) -> None:
        with self._lock:
            timer = self._timers.pop(operation_id, None)
        if timer is not None:
            timer.cancel()
        try:
            response = future.result()
        except Exception as exc:  # rclpy futures surface transport failures here.
            self._signals.command_result.emit(
                operation_id, False, f"{command} 调用失败：{exc}"
            )
            return
        if response is None:
            self._signals.command_result.emit(
                operation_id, False, f"{command} 没有返回结果"
            )
            return

        if command == "reset_fault":
            success = bool(response.ok)
            detail = f"stage={response.stage}"
            if response.failed_joint:
                detail += (
                    f" · {response.failed_joint} · status=0x{int(response.status_word):04X}"
                )
            text = f"故障复位{'成功' if success else '失败'} · {detail}"
        else:
            expected_enabled = command == "enable"
            success = bool(response.accepted) and bool(response.enabled) == expected_enabled
            action = "使能" if expected_enabled else "软件停机"
            message = str(response.error.message).strip()
            text = f"{action}{'成功' if success else '失败'}"
            if message:
                text += f" · {message}"
        self._signals.command_result.emit(operation_id, success, text)


def main(args=None) -> int:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from robot_interfaces_qos import diagnostic, state
    from robot_rt_control_interfaces.msg import SafetyState
    from robot_rt_control_interfaces.srv import SetControlEnabled
    from rt_control_interfaces.srv import RtEnable
    from sensor_msgs.msg import BatteryState

    application = QApplication.instance() or QApplication([])
    window = OperatorConsoleWindow(domain_id=os.environ.get("ROS_DOMAIN_ID", "0"))
    signals = RosSignals()
    signals.shutdown_requested.connect(application.quit)
    catalog_path = (
        Path(get_package_share_directory("rt_control_operator_ui"))
        / "config"
        / "fault_catalog"
    )
    presenter = OperatorPresenter(window, FaultCatalog.load_directory(catalog_path))
    signals.diagnostics.connect(presenter.on_diagnostics)
    signals.safety.connect(presenter.on_safety)
    signals.battery.connect(presenter.on_battery)

    rclpy.init(args=args)
    node = Node("rt_control_operator_ui")
    node.create_subscription(
        DiagnosticArray,
        "/diagnostics",
        lambda message: signals.diagnostics.emit(
            message, node.get_clock().now().nanoseconds / 1_000_000_000.0
        ),
        diagnostic(),
    )
    node.create_subscription(
        SafetyState, "/control/safety_state", signals.safety.emit, state()
    )
    node.create_subscription(
        BatteryState,
        "/battery_state",
        lambda message: signals.battery.emit(message, time.monotonic()),
        state(),
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    dispatcher = RosCommandDispatcher(node, signals, RtEnable, SetControlEnabled)
    gateway = CommandGateway(
        dispatch=dispatcher.dispatch,
        on_result=lambda _success, text: window.finish_command(text),
    )
    signals.command_result.connect(gateway.complete)
    window.fault_reset_requested.connect(lambda: gateway.request("reset_fault"))
    window.enable_requested.connect(lambda: gateway.request("enable"))
    window.software_stop_requested.connect(lambda: gateway.request("stop"))
    ros_thread = threading.Thread(
        target=lambda: spin_ros_until_stopped(
            executor, ExternalShutdownException, signals.shutdown_requested.emit
        ),
        name="operator-ui-ros",
        daemon=False,
    )
    ros_thread.start()

    cpu_history = CpuUsageHistory(max_samples=60)

    def refresh_non_ros_metrics() -> None:
        presenter.refresh_battery(time.monotonic())
        try:
            counters = parse_proc_stat(
                Path("/proc/stat").read_text(encoding="utf-8"), cpu=14
            )
            display = cpu_history.ingest(counters)
            if display is not None:
                window.update_cpu(display)
        except (OSError, ValueError) as exc:
            node.get_logger().warning(f"CPU14 metrics unavailable: {exc}")

    timer = QTimer(window)
    timer.setInterval(1000)
    timer.timeout.connect(refresh_non_ros_metrics)
    timer.start()
    refresh_non_ros_metrics()
    window.show()
    exit_code = application.exec_()

    timer.stop()
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    ros_thread.join(timeout=2.0)
    if ros_thread.is_alive():
        raise RuntimeError("operator UI ROS executor did not stop within 2 seconds")
    return int(exit_code)
