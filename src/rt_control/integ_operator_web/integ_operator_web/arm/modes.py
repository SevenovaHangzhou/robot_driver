"""Select the entry's controller while retaining the existing arm reservation."""

from contextlib import contextmanager

from .console_types import ConsoleError, STATE_STALE_S


class ModeError(ConsoleError):
    def __init__(self, code, message, *, uncertain=False):
        super().__init__(code, message)
        self.uncertain = uncertain


class ArmModes:
    def __init__(self, console, switch, available, matches):
        self.console = console
        self.switch, self.available, self.matches = switch, available, matches
        self.target, self.error = "", ""

    def can_select(self, target):
        c = self.console
        return (
            not c._mode_busy
            and not c._authority_unknown
            and (self.matches(target) or (self.available() and c.idle_probe(-float("inf"))))
        )

    def status(self):
        current = next((m for m in ("jtc", "rolling") if self.matches(m)), "unknown")
        return {
            "automatic": True,
            "current": current,
            "target": self.target,
            "switching": self.console._mode_busy,
            "error": self.error,
        }

    @contextmanager
    def select(self, connection, target):
        c = self.console
        with c._lock:
            permit = c._permit_for(connection)
            entry = {"jtc": "arm", "rolling": "jog"}[target]
            if permit is None or permit.entry != entry:
                raise ConsoleError("lease_required", "请先获取对应页面的双臂控制权")
            c.authority.require(permit)
            if (
                c._mode_busy
                or c._execution is not None
                or c._sequence is not None
                or c._jog_busy
                or c._enable_busy
                or c._scope_busy
                or c._authority_unknown
            ):
                raise ConsoleError("busy", "双臂已有运动或切换尚未结束")
            if c.robot.enable_state != "ENABLED" or not c._fresh(c.robot.enable_at, STATE_STALE_S):
                raise ConsoleError("not_ready", "手臂未使能")
            c._measured()
            if not c.idle_probe(-float("inf")):
                raise ConsoleError("authority_unverified", "双臂尚未确认停稳")
            needs_switch = not self.matches(target)
            if needs_switch and not self.available():
                raise ModeError("mode_unavailable", "控制模式切换服务不可用")
            c.authority.begin_motion(permit)
            c._authority_stop_sent = None
            c._mode_busy = True
            self.target, self.error = target, ""
        committed = False
        try:
            if needs_switch:
                self.switch(target)
            with c._lock:
                c.authority.require(permit)
                if not self.matches(target):
                    raise ModeError(
                        "mode_unconfirmed", "控制器模式未确认，未发送轨迹", uncertain=True
                    )
            yield
            committed = True
        except Exception as exc:
            with c._lock:
                self.error = str(exc)
                c._authority_unknown |= not isinstance(exc, ConsoleError) or getattr(
                    exc, "uncertain", False
                )
            raise
        finally:
            with c._lock:
                c._mode_busy = False
                if not committed:
                    c._authority_outcome_at = c._clock()
                    c.authority.request_stop(permit, release=False)
