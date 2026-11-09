"""Jog authority entry sharing the arm page's authentication and WebSocket transport.

Motion remains disabled until the real Mink/Rolling adapter is commissioned.
Acquiring this entry reserves the SAME arm resource as joint moves and sequences.
"""

import os

from .errors import ConsoleError
from .page import ArmPage


class JogAccess:
    poses = None

    def __init__(self, control):
        self.control, self.authority = control, control.authority

    @property
    def lease_holder(self):
        owner = self.authority.owner()
        return owner.client if owner is not None and owner.entry == "jog" else None

    def touch(self, connection):
        owner = self.authority.owner()
        if owner is not None and owner.entry == "jog" and owner.client == connection:
            try:
                self.authority.renew(owner)
            except ConsoleError:
                pass

    def status(self, connection):
        with self.control._lock:
            self.control._sync_authority()
            permit = self.control._permit_for(connection)
            state = self.authority.poll(permit)
        return {
            "protocol": "shared_arm_authority_v1",
            "lease": state["lease"],
            "control_authority": {**state, "integrated": True},
            "motion_enabled": self.control.jog_output is not None,
            "motion_ready": self.control.jog_output.ready() if self.control.jog_output else False,
            "control_mode": self.control.modes.status() if self.control.modes else None,
            "positions": dict(self.control.robot.positions),
            "feedback_age_s": self.control._clock() - self.control.robot.feedback_at if self.control.robot.positions else None,
            "rolling": self.control.jog_output.snapshot() if self.control.jog_output else None,
            "runtime_identity": {
                "domain_id": int(os.environ["ROS_DOMAIN_ID"])
                if os.environ.get("ROS_DOMAIN_ID", "").isdigit()
                else None,
                "model_sha256": self.control.model.source_sha256 if self.control.model else None,
                "joint_names": list(self.control.joint_names),
                "controller": self.control.controller,
                "scope": (self.control.robot.capability or {}).get("scope"),
            },
        }

    def acquire(self, connection):
        with self.control._lock:
            self.control._sync_authority()
            self.authority.acquire("jog", connection, connection)

    def release(self, connection):
        owner = self.authority.owner()
        if owner is not None and owner.entry == "jog" and owner.client == connection:
            if self.control.jog_output is not None:
                self.control.jog_output.stop(connection, release=True)
            else:
                self.authority.release(owner)

    disconnect = release

    def stop(self, connection=None):
        if self.control.jog_output is not None:
            return self.control.jog_output.stop(connection)
        # A read-only entry cannot stop another page.
        return {"motion_enabled": False, "stopping": False}


class JogPage(ArmPage):
    entry = "jog"
    max_message_bytes = 256 * 14 * 2 * 32 + 8192

    def __init__(self, arm_page):
        self.console = JogAccess(arm_page.console)
        self._sessions, self._operators = arm_page._sessions, arm_page._operators
        self._audit, self._status_period = arm_page._audit, arm_page._status_period
        self._sockets = set()
        self._operators.register_page("jog", lambda: self.console.lease_holder)

    def add_routes(self, app):
        app.router.add_get("/ws/jog", self._websocket)
        app.on_shutdown.append(self._on_shutdown)

    async def _on_shutdown(self, _app):
        for ws in list(self._sockets):
            await ws.close(code=1001, message=b"server shutdown")

    def _command(self, kind, connection, body):
        if kind == "acquire":
            if not self._operators.may_acquire("jog", connection):
                raise ConsoleError("lease_busy", "另一位操作员正在操作其他页面")
            self.console.acquire(connection)
        elif kind == "release":
            self.console.release(connection)
        elif kind == "stop":
            return self.console.stop(connection)
        elif kind in ("begin_motion", "window") and self.console.control.jog_output is not None:
            output = self.console.control.jog_output
            return (output.begin if kind == "begin_motion" else output.publish)(connection, body)
        else:
            raise ConsoleError("jog_motion_disabled", "末端运动接口未在主操作台启用")
        return {"motion_enabled": False}
