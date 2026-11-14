"""ArmConsole hooks for the one arm resource shared with the jog connection."""

from contextlib import contextmanager

from .authority import ArmAuthority
from .console_types import LEASE_TIMEOUT_S
from .console_types import ConsoleError


class ArmLease:
    def _init_lease(self, authority):
        self.authority = authority or ArmAuthority(LEASE_TIMEOUT_S, clock=self._clock)
        self._arm_permit = None  # Captured generation checked immediately before action send.
        self.jog_output = None
        self._jog_busy = False
        self.modes = None
        self._mode_busy = False
        self._authority_unknown = self._scope_busy = False
        self._authority_outcome_at = -float("inf")
        self._authority_stop_sent = None
        # Replaced by the ROS bridge's read-only controller evidence.
        self.idle_probe = lambda _since: False

    def _permit_for(self, connection):
        owner = self.authority.owner()
        return owner if owner is not None and owner.client == connection else None

    def _sync_authority(self):
        if self.jog_output is not None:
            self.jog_output.poll()
        owner = self.authority.owner()
        phase = self.authority.poll()["phase"]
        if (
            self._execution is not None
            or self._sequence is not None
            or self._enable_busy
            or self._scope_busy
            or self._jog_busy
            or self._mode_busy
        ):
            return
        since = max(self.authority.changed_at, self._authority_outcome_at)
        if self._authority_unknown or not self.idle_probe(since):
            if owner is None:
                self.authority.invalidate_runtime()
            return
        if owner is None:
            self.authority.confirm_runtime_idle()
        elif phase in ("moving", "stopping"):
            self.authority.finish_motion(owner)

    def _expire_lease(self):
        self._sync_authority()

    @property
    def lease_holder(self):
        with self._lock:
            self._sync_authority()
            owner = self.authority.owner()
            return owner.client if owner is not None and owner.entry == "arm" else None

    def _lease_status(self, connection):
        return self.authority.poll(self._permit_for(connection))["lease"]

    def acquire(self, connection):
        with self._lock:
            self._sync_authority()
            self._arm_permit = self.authority.acquire("arm", connection)

    def touch(self, connection):
        with self._lock:
            permit = self._permit_for(connection)
            if permit is not None and permit.entry == "arm":
                try:
                    self.authority.renew(permit)
                except ConsoleError:
                    pass  # An expired moving owner remains fenced until the stop completes.

    def _require_lease(self, connection):
        self._sync_authority()
        permit = self._permit_for(connection)
        if permit is None or permit.entry != "arm":
            raise ConsoleError("lease_required", "未持有双臂控制权")
        self.authority.require(permit)

    def _reserve_motion(self, connection, *, continuation=False):
        permit = self._permit_for(connection)
        if (continuation or self._mode_busy) and self.authority.poll(permit)["phase"] == "moving":
            self.authority.require(permit)
            return
        if not self.idle_probe(-float("inf")):
            raise ConsoleError("authority_unverified", "双臂控制器尚未确认空闲")
        self.authority.begin_motion(permit)
        self._authority_stop_sent = None

    @contextmanager
    def _select_mode(self, connection, target, *, continuation=False):
        if self.modes is None or continuation:
            yield
        else:
            with self.modes.select(connection, target):
                yield

    def release(self, connection):
        with self._lock:
            permit = self._permit_for(connection)
            if permit is None or permit.entry != "arm":
                return
            try:
                self.authority.release(permit)
            except ConsoleError as exc:
                if exc.code != "authority_stop_pending":
                    raise

    def disconnect(self, connection):
        self.release(connection)
        self.poll_authority()

    def _stop_authority(self):
        owner = self.authority.owner()
        if (
            owner is not None
            and owner.entry == "arm"
            and self.authority.poll()["phase"] in ("moving", "stopping")
        ):
            self.authority.request_stop(owner, release=False)

    def poll_authority(self):
        with self._lock:
            self._sync_authority()
            owner, status = self.authority.owner(), self.authority.poll()
            if (
                owner is None
                or owner.entry != "arm"
                or not status["stop_required"]
                or self._authority_stop_sent == owner.generation
            ):
                return
            self._authority_stop_sent = owner.generation
        try:
            self.stop()
        except Exception as exc:
            with self._lock:
                self._authority_unknown = True
                self._last_result = f"停车结果未确认：{exc}"

    def begin_scope_operation(self, entry, connection):
        """Reserve before a head/chassis operation that can change the whole scope."""
        with self._lock:
            # Explicit absence from a parsed scope capability document, not lack of feedback.
            if (
                getattr(self.robot, "scope_has_arms", None) is False
                and self.authority.owner() is None
            ):
                return None
            self._sync_authority()
            permit = self.authority.acquire(entry, connection)
            self.authority.begin_motion(permit)
            self._scope_busy = True
            return permit

    def end_scope_operation(self, permit, *, confirmed):
        with self._lock:
            self._scope_busy = False
            self._authority_unknown |= not confirmed
            self._authority_outcome_at = self._clock()
            self.authority.request_stop(permit)
