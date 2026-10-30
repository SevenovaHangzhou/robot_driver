"""Web facade for the single vacuum IO service; no device connections in HTTP threads."""

from pathlib import Path
import runpy
import threading
import time

from .console import ConsoleError


class VacuumControl:
    def __init__(self, script: Path, clock=time.monotonic):
        self._tool = runpy.run_path(str(script))
        if not all(
            name in self._tool
            for name in ("service_status", "service_set", "ServiceError")
        ):
            raise RuntimeError("vacuum CLI must support the persistent IO service")
        self._lock = threading.Lock()

    @staticmethod
    def unknown(message):
        return {
            "configured": True,
            "valid": False,
            "busy": False,
            "outputs": {k: None for k in ("pump", "left", "right")},
            "sample_age_s": None,
            "message": message,
        }

    def status(self):
        try:
            # Cached daemon state only; the browser does not drive the hardware poll rate.
            return self._tool["service_status"]()
        except (OSError, RuntimeError, ValueError) as exc:
            return self.unknown(str(exc))

    def set_output(self, output, enabled, expected):
        if output not in ("pump", "left", "right"):
            raise ValueError("output must be pump, left or right")
        if type(enabled) is not bool or type(expected) is not bool:
            raise ValueError("enabled and expected must be booleans")
        if enabled == expected:
            raise ValueError("expected must be the opposite of enabled")
        if not self._lock.acquire(timeout=2.5):
            raise ConsoleError("vacuum_busy", "真空 IO 正在通信，请稍后重试")
        try:
            result = self._tool["service_set"](output, enabled, expected)
            return {"vacuum": result["vacuum"], "message": result["message"]}
        except self._tool["ServiceError"] as exc:
            raise ConsoleError(exc.code, str(exc)) from exc
        finally:
            self._lock.release()
