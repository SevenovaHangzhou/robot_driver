"""Console transaction for the opt-in controller-owned stop reference."""

from dataclasses import replace

from .console_types import ConsoleError


def request_controller_stop(console, execution) -> dict:
    # Called with the console RLock held. Replacing execution first fences the
    # original goal's ABORTED result, including callbacks arriving before the ACK.
    stopping = replace(execution, kind="stop", controller_stop=True, started=console._clock())
    console._execution = stopping
    console._last_result = "等待控制器完成制动参考"

    def done(success: bool, message: str) -> None:
        with console._lock:
            if console._execution is not stopping:
                return
            console._execution = None
            console._authority_outcome_at = console._clock()
            console._authority_unknown |= not success
            console._stop_unconfirmed |= not success
            console._last_result = (
                "控制器已确认制动参考结束" if success else f"停止未确认：{message}"
            )

    try:
        limits = console._limits(execution.gear, execution.plan.start, execution.plan.target)
        console._driver.stop_current(limits, done)
    except Exception as exc:
        done(False, str(exc))
        raise ConsoleError("stop_unconfirmed", f"停止请求失败：{exc}") from exc
    # No predicted duration, final position, or fallback to ordinary FJT cancel.
    return {"stopping": True, "duration_s": None}
