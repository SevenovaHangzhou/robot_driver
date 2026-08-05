from __future__ import annotations

import enum
import threading
import time
from dataclasses import dataclass
from typing import Protocol


class ErrorCode(enum.IntEnum):
    OK = 0
    RT_SERVICE_UNAVAILABLE = 1
    ENABLE_MANAGER_NOT_READY = 2
    RESET_FAILED = 3
    ENABLE_FAILED = 4
    DISABLE_FAILED = 5
    OPERATION_IN_PROGRESS = 6
    RESTART_REQUIRED = 7


@dataclass(frozen=True)
class RtServiceResult:
    ok: bool
    stage: str
    failed_batch: int = -1
    failed_joint: str = ""
    status_word: int = 0

    def summary(self) -> str:
        details = [f"ok={self.ok}", f"stage={self.stage}"]
        if self.failed_joint:
            details.append(f"failed_joint={self.failed_joint}")
        if self.failed_batch >= 0:
            details.append(f"failed_batch={self.failed_batch}")
        if self.status_word:
            details.append(f"status_word=0x{self.status_word:04x}")
        return ", ".join(details)


@dataclass(frozen=True)
class DiagnosticSnapshot:
    state: str
    stage: str
    message: str = ""
    failed_batch: int = -1
    failed_joint: str = ""
    failed_status_word: int = 0

    @property
    def resettable_fault(self) -> bool:
        return self.stage == "fault_requires_reset"

    @property
    def restart_required(self) -> bool:
        return self.stage == "restart_required" or (
            self.state == "FAILED" and self.stage != "fault_requires_reset"
        )

    @property
    def ready_for_enable_request(self) -> bool:
        return self.state in {"IDLE", "ENABLED"} or self.resettable_fault or self.restart_required

    def summary(self) -> str:
        details = [f"state={self.state}", f"stage={self.stage}"]
        if self.failed_joint:
            details.append(f"failed_joint={self.failed_joint}")
        if self.failed_batch >= 0:
            details.append(f"failed_batch={self.failed_batch}")
        if self.failed_status_word:
            details.append(f"failed_status_word=0x{self.failed_status_word:04x}")
        if self.message:
            details.append(f"message={self.message}")
        return ", ".join(details)


@dataclass(frozen=True)
class SetEnabledResult:
    accepted: bool
    enabled: bool
    error_code: ErrorCode
    message: str


class RtServices(Protocol):
    def call(self, operation: str) -> RtServiceResult:
        ...


class EnableManagerDiagnostics(Protocol):
    def wait_for_reset_ready(self) -> DiagnosticSnapshot:
        ...


class RtServiceUnavailable(RuntimeError):
    pass


class ControlEnableAdapterCore:
    """Translate the public enable service to the internal rt-control services.

    The adapter deliberately does not write control words.  All drive-state
    changes remain owned by enable_manager through /rt/reset_fault, /rt/enable
    and /rt/disable.
    """

    _ENABLE_SUCCESS_STAGES = {"success", "already_enabled"}
    _DISABLE_SUCCESS_STAGES = {"success", "already_disabled"}
    _RESET_SUCCESS_STAGES = {"success", "already_clear"}

    def __init__(
        self,
        rt_services: RtServices,
        diagnostics: EnableManagerDiagnostics,
    ) -> None:
        self._rt_services = rt_services
        self._diagnostics = diagnostics

    def set_enabled(self, enabled: bool, reason: str = "") -> SetEnabledResult:
        del reason
        if enabled:
            return self._enable_with_one_recovery_attempt()
        return self._disable()

    def _disable(self) -> SetEnabledResult:
        try:
            result = self._rt_services.call("disable")
        except RtServiceUnavailable as exc:
            return SetEnabledResult(False, False, ErrorCode.RT_SERVICE_UNAVAILABLE, str(exc))

        if result.ok and result.stage in self._DISABLE_SUCCESS_STAGES:
            return SetEnabledResult(True, False, ErrorCode.OK, f"/rt/disable accepted: {result.summary()}")
        return SetEnabledResult(
            False,
            False,
            self._error_code_for_stage(result.stage, ErrorCode.DISABLE_FAILED),
            f"/rt/disable rejected: {result.summary()}",
        )

    def _enable_with_one_recovery_attempt(self) -> SetEnabledResult:
        try:
            snapshot = self._diagnostics.wait_for_reset_ready()
        except TimeoutError as exc:
            return SetEnabledResult(False, False, ErrorCode.ENABLE_MANAGER_NOT_READY, str(exc))

        if snapshot.restart_required:
            return SetEnabledResult(
                False,
                False,
                ErrorCode.RESTART_REQUIRED,
                f"enable_manager requires restart/power-cycle before enable: {snapshot.summary()}",
            )

        if snapshot.resettable_fault:
            reset_result = self._call_reset_fault()
            if reset_result is not None:
                return reset_result

        try:
            enable_result = self._rt_services.call("enable")
        except RtServiceUnavailable as exc:
            return SetEnabledResult(False, False, ErrorCode.RT_SERVICE_UNAVAILABLE, str(exc))

        if enable_result.ok and enable_result.stage in self._ENABLE_SUCCESS_STAGES:
            return SetEnabledResult(
                True,
                True,
                ErrorCode.OK,
                f"/rt/enable accepted: {enable_result.summary()}",
            )
        return SetEnabledResult(
            False,
            False,
            self._error_code_for_stage(enable_result.stage, ErrorCode.ENABLE_FAILED),
            f"/rt/enable rejected: {enable_result.summary()}",
        )

    def _call_reset_fault(self) -> SetEnabledResult | None:
        try:
            reset_result = self._rt_services.call("reset_fault")
        except RtServiceUnavailable as exc:
            return SetEnabledResult(False, False, ErrorCode.RT_SERVICE_UNAVAILABLE, str(exc))

        if reset_result.ok and reset_result.stage in self._RESET_SUCCESS_STAGES:
            return None
        return SetEnabledResult(
            False,
            False,
            self._error_code_for_stage(reset_result.stage, ErrorCode.RESET_FAILED),
            f"/rt/reset_fault could not clear fault: {reset_result.summary()}",
        )

    @staticmethod
    def _error_code_for_stage(stage: str, fallback: ErrorCode) -> ErrorCode:
        if stage == "operation_in_progress":
            return ErrorCode.OPERATION_IN_PROGRESS
        if stage == "restart_required":
            return ErrorCode.RESTART_REQUIRED
        return fallback


def _parse_int(value: str, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _snapshot_from_status(status) -> DiagnosticSnapshot:
    values = {item.key: item.value for item in status.values}
    return DiagnosticSnapshot(
        state=values.get("state", ""),
        stage=values.get("stage", ""),
        message=status.message,
        failed_batch=_parse_int(values.get("failed_batch", "-1"), -1),
        failed_joint=values.get("failed_joint", ""),
        failed_status_word=_parse_int(values.get("failed_status_word", "0"), 0),
    )


def main(args=None) -> None:
    import rclpy
    from robot_control_interfaces.srv import SetControlEnabled
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rt_control_interfaces.srv import RtEnable

    class RosRtServices:
        def __init__(self, node: Node, callback_group: ReentrantCallbackGroup) -> None:
            self._node = node
            self._timeout_s = float(node.get_parameter("rt_service_timeout_s").value)
            self._clients = {
                operation: node.create_client(
                    RtEnable,
                    f"/rt/{operation}",
                    callback_group=callback_group,
                )
                for operation in ("enable", "disable", "reset_fault")
            }

        def call(self, operation: str) -> RtServiceResult:
            client = self._clients[operation]
            if not client.wait_for_service(timeout_sec=self._timeout_s):
                raise RtServiceUnavailable(f"/rt/{operation} is unavailable")

            future = client.call_async(RtEnable.Request())
            deadline = time.monotonic() + self._timeout_s
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                raise RtServiceUnavailable(f"/rt/{operation} timed out")
            exception = future.exception()
            if exception is not None:
                raise RtServiceUnavailable(f"/rt/{operation} failed: {exception}")

            response = future.result()
            if response is None:
                raise RtServiceUnavailable(f"/rt/{operation} returned no response")
            return RtServiceResult(
                ok=bool(response.ok),
                stage=str(response.stage),
                failed_batch=int(response.failed_batch),
                failed_joint=str(response.failed_joint),
                status_word=int(response.status_word),
            )

    class RosEnableManagerDiagnostics:
        def __init__(self, node: Node, callback_group: ReentrantCallbackGroup) -> None:
            self._node = node
            self._name = str(node.get_parameter("enable_manager_diagnostic_name").value)
            self._timeout_s = float(node.get_parameter("reset_ready_timeout_s").value)
            self._condition = threading.Condition()
            self._latest: DiagnosticSnapshot | None = None
            self._subscription = node.create_subscription(
                DiagnosticArray,
                str(node.get_parameter("diagnostics_topic").value),
                self._on_diagnostics,
                50,
                callback_group=callback_group,
            )

        def _on_diagnostics(self, message: DiagnosticArray) -> None:
            for status in message.status:
                if status.name != self._name:
                    continue
                snapshot = _snapshot_from_status(status)
                with self._condition:
                    self._latest = snapshot
                    self._condition.notify_all()
                return

        def wait_for_reset_ready(self) -> DiagnosticSnapshot:
            deadline = time.monotonic() + self._timeout_s
            with self._condition:
                while rclpy.ok():
                    if self._latest is not None and self._latest.ready_for_enable_request:
                        return self._latest
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self._condition.wait(timeout=min(0.2, remaining))
            raise TimeoutError(
                f"{self._name} did not report IDLE, ENABLED, fault_requires_reset "
                f"or restart_required within {self._timeout_s:.1f}s"
            )

    class ControlEnableAdapterNode(Node):
        def __init__(self) -> None:
            super().__init__("control_enable_adapter")
            self.declare_parameter("rt_service_timeout_s", 40.0)
            self.declare_parameter("reset_ready_timeout_s", 120.0)
            self.declare_parameter("diagnostics_topic", "/diagnostics")
            self.declare_parameter(
                "enable_manager_diagnostic_name",
                "/robot/rt_control/enable_manager",
            )

            self._callback_group = ReentrantCallbackGroup()
            self._diagnostics = RosEnableManagerDiagnostics(self, self._callback_group)
            self._rt_services = RosRtServices(self, self._callback_group)
            self._core = ControlEnableAdapterCore(self._rt_services, self._diagnostics)
            self._service = self.create_service(
                SetControlEnabled,
                "/control/set_enabled",
                self._handle_set_enabled,
                callback_group=self._callback_group,
            )

        def _handle_set_enabled(self, request, response):
            result = self._core.set_enabled(bool(request.enabled), str(request.reason))
            response.accepted = result.accepted
            response.enabled = result.enabled
            response.error_code = int(result.error_code)
            response.message = result.message
            if result.accepted:
                self.get_logger().info(result.message)
            else:
                self.get_logger().error(result.message)
            return response

    rclpy.init(args=args)
    node = ControlEnableAdapterNode()
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
