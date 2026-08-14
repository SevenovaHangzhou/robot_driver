import sys
from pathlib import Path
from types import SimpleNamespace

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from control_api_adapter.control_enable_adapter import (  # noqa: E402
    ControlEnableAdapterCore,
    DiagnosticSnapshot,
    RtServiceUnavailable,
    RtServiceResult,
)
from control_api_adapter.public_error import (  # noqa: E402
    PublicErrorCode,
    assign_error_info,
    error_info,
)


class FakeRtServices:
    def __init__(self, results):
        self._results = dict(results)
        self.calls = []

    def call(self, operation: str) -> RtServiceResult:
        self.calls.append(operation)
        result = self._results[operation]
        if isinstance(result, Exception):
            raise result
        return result


class FakeDiagnostics:
    def __init__(self, snapshot):
        self.snapshot = snapshot
        self.calls = 0

    def wait_for_reset_ready(self) -> DiagnosticSnapshot:
        self.calls += 1
        return self.snapshot


def test_error_info_uses_vendored_string_schema() -> None:
    message = SimpleNamespace(
        OK=0,
        WARN=1,
        FAULT=2,
        code="",
        message="",
        retryable=False,
        severity=99,
        source="",
        detail="unexpected",
    )

    assign_error_info(
        message,
        error_info(PublicErrorCode.RT_SERVICE_UNAVAILABLE, "service unavailable"),
    )

    assert message.code == "1100"
    assert message.message == "service unavailable"
    assert message.retryable
    assert message.severity == message.WARN
    assert message.source == "rt_control"
    assert message.detail == ""


def test_disable_only_calls_rt_disable():
    services = FakeRtServices({"disable": RtServiceResult(ok=True, stage="success")})
    adapter = ControlEnableAdapterCore(services, FakeDiagnostics(None))

    result = adapter.set_enabled(False, reason="maintenance")

    assert services.calls == ["disable"]
    assert result.accepted
    assert not result.enabled
    assert result.error.code == PublicErrorCode.SUCCESS
    assert not result.error.retryable


def test_enable_resets_resettable_fault_before_enable():
    services = FakeRtServices(
        {
            "reset_fault": RtServiceResult(ok=True, stage="success"),
            "enable": RtServiceResult(ok=True, stage="success"),
        }
    )
    diagnostics = FakeDiagnostics(
        DiagnosticSnapshot(state="FAILED", stage="fault_requires_reset")
    )
    adapter = ControlEnableAdapterCore(services, diagnostics)

    result = adapter.set_enabled(True, reason="operator")

    assert services.calls == ["reset_fault", "enable"]
    assert result.accepted
    assert result.enabled
    assert result.error.code == PublicErrorCode.SUCCESS


def test_enable_without_fault_calls_enable_directly():
    services = FakeRtServices({"enable": RtServiceResult(ok=True, stage="already_enabled")})
    diagnostics = FakeDiagnostics(DiagnosticSnapshot(state="IDLE", stage="success"))
    adapter = ControlEnableAdapterCore(services, diagnostics)

    result = adapter.set_enabled(True, reason="operator")

    assert services.calls == ["enable"]
    assert result.accepted
    assert result.enabled
    assert result.error.code == PublicErrorCode.SUCCESS


def test_enable_rejects_restart_required_without_reset():
    services = FakeRtServices({})
    diagnostics = FakeDiagnostics(DiagnosticSnapshot(state="FAILED", stage="restart_required"))
    adapter = ControlEnableAdapterCore(services, diagnostics)

    result = adapter.set_enabled(True, reason="operator")

    assert services.calls == []
    assert not result.accepted
    assert not result.enabled
    assert result.error.code == PublicErrorCode.RT_RESTART_REQUIRED
    assert not result.error.retryable
    assert "power-cycle" in result.error.message


def test_enable_stops_if_resettable_fault_does_not_clear():
    services = FakeRtServices(
        {"reset_fault": RtServiceResult(ok=False, stage="fault_detected")}
    )
    diagnostics = FakeDiagnostics(
        DiagnosticSnapshot(state="FAILED", stage="fault_requires_reset")
    )
    adapter = ControlEnableAdapterCore(services, diagnostics)

    result = adapter.set_enabled(True, reason="operator")

    assert services.calls == ["reset_fault"]
    assert not result.accepted
    assert not result.enabled
    assert result.error.code == PublicErrorCode.RT_RESET_FAILED
    assert not result.error.retryable
    assert "fault_detected" in result.error.message


def test_enable_reports_internal_service_unavailable():
    services = FakeRtServices({"enable": RtServiceUnavailable("/rt/enable timed out")})
    diagnostics = FakeDiagnostics(DiagnosticSnapshot(state="IDLE", stage="success"))
    adapter = ControlEnableAdapterCore(services, diagnostics)

    result = adapter.set_enabled(True, reason="operator")

    assert services.calls == ["enable"]
    assert not result.accepted
    assert result.error.code == PublicErrorCode.RT_SERVICE_UNAVAILABLE
    assert result.error.retryable
    assert result.error.origin == "rt_control"
    assert "/rt/enable timed out" in result.error.message
