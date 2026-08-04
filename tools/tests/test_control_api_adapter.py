import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/rt_control/control_api_adapter"))

from control_api_adapter.control_enable_adapter import (
    ControlEnableAdapterCore,
    DiagnosticSnapshot,
    ErrorCode,
    RtServiceUnavailable,
    RtServiceResult,
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


class ControlEnableAdapterTest(unittest.TestCase):
    def test_disable_only_calls_rt_disable(self):
        services = FakeRtServices(
            {"disable": RtServiceResult(ok=True, stage="success")}
        )
        adapter = ControlEnableAdapterCore(services, FakeDiagnostics(None))

        result = adapter.set_enabled(False, reason="maintenance")

        self.assertEqual(services.calls, ["disable"])
        self.assertTrue(result.accepted)
        self.assertFalse(result.enabled)
        self.assertEqual(result.error_code, ErrorCode.OK)

    def test_enable_resets_resettable_fault_before_enable(self):
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

        self.assertEqual(services.calls, ["reset_fault", "enable"])
        self.assertTrue(result.accepted)
        self.assertTrue(result.enabled)
        self.assertEqual(result.error_code, ErrorCode.OK)

    def test_enable_without_fault_calls_enable_directly(self):
        services = FakeRtServices(
            {"enable": RtServiceResult(ok=True, stage="already_enabled")}
        )
        diagnostics = FakeDiagnostics(DiagnosticSnapshot(state="IDLE", stage="success"))
        adapter = ControlEnableAdapterCore(services, diagnostics)

        result = adapter.set_enabled(True, reason="operator")

        self.assertEqual(services.calls, ["enable"])
        self.assertTrue(result.accepted)
        self.assertTrue(result.enabled)
        self.assertEqual(result.error_code, ErrorCode.OK)

    def test_enable_rejects_restart_required_without_reset(self):
        services = FakeRtServices({})
        diagnostics = FakeDiagnostics(
            DiagnosticSnapshot(state="FAILED", stage="restart_required")
        )
        adapter = ControlEnableAdapterCore(services, diagnostics)

        result = adapter.set_enabled(True, reason="operator")

        self.assertEqual(services.calls, [])
        self.assertFalse(result.accepted)
        self.assertFalse(result.enabled)
        self.assertEqual(result.error_code, ErrorCode.RESTART_REQUIRED)
        self.assertIn("power-cycle", result.message)

    def test_enable_stops_if_resettable_fault_does_not_clear(self):
        services = FakeRtServices(
            {"reset_fault": RtServiceResult(ok=False, stage="fault_detected")}
        )
        diagnostics = FakeDiagnostics(
            DiagnosticSnapshot(state="FAILED", stage="fault_requires_reset")
        )
        adapter = ControlEnableAdapterCore(services, diagnostics)

        result = adapter.set_enabled(True, reason="operator")

        self.assertEqual(services.calls, ["reset_fault"])
        self.assertFalse(result.accepted)
        self.assertFalse(result.enabled)
        self.assertEqual(result.error_code, ErrorCode.RESET_FAILED)
        self.assertIn("fault_detected", result.message)

    def test_enable_reports_internal_service_unavailable(self):
        services = FakeRtServices({"enable": RtServiceUnavailable("/rt/enable timed out")})
        diagnostics = FakeDiagnostics(DiagnosticSnapshot(state="IDLE", stage="success"))
        adapter = ControlEnableAdapterCore(services, diagnostics)

        result = adapter.set_enabled(True, reason="operator")

        self.assertEqual(services.calls, ["enable"])
        self.assertFalse(result.accepted)
        self.assertEqual(result.error_code, ErrorCode.RT_SERVICE_UNAVAILABLE)
        self.assertIn("/rt/enable timed out", result.message)


if __name__ == "__main__":
    unittest.main()
