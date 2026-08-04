import unittest

from control_api_adapter.control_enable_adapter import (
    ControlEnableAdapterCore,
    DiagnosticSnapshot,
    ErrorCode,
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


if __name__ == "__main__":
    unittest.main()
