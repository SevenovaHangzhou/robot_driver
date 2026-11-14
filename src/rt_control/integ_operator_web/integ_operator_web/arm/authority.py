"""Shared arm-resource arbitration for a single command gateway.

Both the joint console and Cartesian jog must use the SAME instance. This is
an in-process web command-admission component, not a distributed lock or ROS transport.
The arm and jog WebSocket entries share the instance owned by ArmConsole.
"""

from dataclasses import dataclass
import math
import secrets
import threading
import time

from .console_types import ConsoleError


@dataclass(frozen=True)
class ArmPermit:
    entry: str
    client: str
    generation: str


class ArmAuthority:
    """One owner for both arms, including two pages of the same login session.

    An expired idle lease can be released. An expired moving lease is fenced and
    retained until the execution adapter confirms stopped/finalized. The adapter
    must call poll(), act on stop_required, and provide controller stop evidence
    before calling finish_motion(); that method must not be a browser endpoint.
    """

    def __init__(self, lease_timeout, *, clock=time.monotonic):
        if not math.isfinite(lease_timeout) or lease_timeout <= 0:
            raise ValueError("lease_timeout must be positive")
        self.timeout, self.clock = lease_timeout, clock
        self.lock = threading.RLock()
        self.permit, self.phase, self.seen = None, "unverified", 0.0
        self.changed_at, self.release_pending = self.clock(), False

    def confirm_runtime_idle(self):
        """Backend only, after startup reconciliation with actual controller state.

        A gateway restart must not assume an older Rolling session has stopped.
        This call cannot clear a live reservation, including an unknown outcome.
        """
        with self.lock:
            if self.permit is not None:
                raise ConsoleError("authority_busy", "双臂仍有未确认结束的运动")
            self.phase = "idle"

    def invalidate_runtime(self):
        with self.lock:
            if self.permit is None:
                self.phase = "unverified"

    def _expire(self):
        if self.permit is not None and self.clock() - self.seen > self.timeout:
            if self.phase == "leased":
                self.permit, self.phase = None, "idle"
            else:
                self.phase, self.release_pending = "stopping", True

    def _owner(self, permit):
        self._expire()
        if permit is None or permit != self.permit:
            raise ConsoleError("authority_lost", "双臂控制权已失效")

    def _require(self, permit):
        self._owner(permit)
        if self.phase == "stopping":
            raise ConsoleError("authority_stopping", "双臂正在停车确认，拒绝继续发送目标")

    def acquire(self, entry, client):
        if not all(isinstance(v, str) and 0 < len(v) <= 128 for v in (entry, client)):
            raise ConsoleError("authority_identity", "控制页面身份无效")
        with self.lock:
            self._expire()
            if self.phase == "unverified":
                raise ConsoleError("authority_unverified", "双臂执行状态尚未完成启动对账")
            if self.permit is not None:
                if (self.permit.entry, self.permit.client) != (
                    entry,
                    client,
                ):
                    raise ConsoleError("lease_busy", "关节操作台或末端点动页面已持有双臂控制权")
                self._require(self.permit)
            else:
                self.permit = ArmPermit(entry, client, secrets.token_urlsafe(24))
                self.phase, self.release_pending = "leased", False
            self.seen = self.clock()
            return self.permit

    def renew(self, permit):
        with self.lock:
            self._require(permit)
            self.seen = self.clock()

    def require(self, permit):
        """Execution adapter checks before open, update and mode-changing work."""
        with self.lock:
            self._require(permit)

    def begin_motion(self, permit):
        """Reserve before submitting any action or opening a Rolling session."""
        with self.lock:
            self._require(permit)
            if self.phase != "leased":
                raise ConsoleError("authority_busy", "双臂运动资源已占用")
            self.phase, self.changed_at = "moving", self.clock()

    def request_stop(self, permit, *, release=True):
        with self.lock:
            self._owner(permit)
            # Keep the owner even if it is not known whether submission succeeded.
            self.phase = "stopping"
            self.release_pending |= release
            self.changed_at = self.clock()

    def release(self, permit):
        with self.lock:
            self._owner(permit)
            if self.phase != "leased":
                self.phase, self.release_pending = "stopping", True
                raise ConsoleError(
                    "authority_stop_pending", "停车和会话结束未确认，暂不交接双臂控制权"
                )
            self.permit, self.phase = None, "idle"

    def finish_motion(self, permit):
        """Backend only, after fresh controller idle evidence and resolved requests."""
        with self.lock:
            self._owner(permit)
            if self.release_pending:
                self.permit, self.phase = None, "idle"
            else:
                self.phase = "leased"

    def owner(self):
        """Backend identity; never serialize a permit into browser status."""
        with self.lock:
            self._expire()
            return self.permit

    def poll(self, permit=None):
        with self.lock:
            self._expire()
            return {
                "resource": "dual_arms",
                "phase": self.phase,
                "lease": "none"
                if self.permit is None
                else "you"
                if permit == self.permit
                else "other",
                "owner_entry": None if self.permit is None else self.permit.entry,
                "stop_required": self.phase == "stopping",
            }
