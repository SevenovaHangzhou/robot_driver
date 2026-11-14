"""Head page: one WebSocket for status and commands; absolute targets on the declared topic.

Closing the socket that holds the lease holds the head at its measured position and frees
the lease. Enable and fault reset go through the public services and need the lease.
"""

from __future__ import annotations

import asyncio
import json
import threading
from pathlib import Path
from typing import Callable, Optional
import uuid

from aiohttp import WSMsgType, web

from ..audit import AuditLog
from ..operators import Operators
from ..session import PAGE_WS_HEARTBEAT_S, Sessions, same_origin
from .console import HeadConsole, HeadError

PAGE = "head"
STATUS_PERIOD_S = 0.05
MAX_MESSAGE_BYTES = 4 * 1024
CLOSE_LOGIN_REQUIRED = 4401
RESET_OK_STAGES = {"success", "already_clear"}


class HeadPage:
    def __init__(
        self,
        console: HeadConsole,
        sessions: Sessions,
        operators: Operators,
        web_dir: Path,
        set_enabled: Callable[[bool], dict],
        reset_fault: Callable[[], dict],
        audit: Optional[AuditLog] = None,
        status_period: float = STATUS_PERIOD_S,
        arm_control=None,
    ) -> None:
        # The former /head entry now opens the shared arm/head page.
        if not (web_dir.parent / "arm/index.html").is_file():
            raise ValueError(f"missing web asset: {web_dir.parent / 'arm/index.html'}")
        self.console = console
        self._sessions = sessions
        self._operators = operators
        self._web_dir = web_dir
        self._set_enabled = set_enabled
        self._reset_fault = reset_fault
        self._audit = audit or AuditLog(None)
        self._status_period = status_period
        self._operation = threading.Lock()
        self._arm_control = arm_control
        operators.register_page(PAGE, lambda: console.lease_holder)

    def add_routes(self, app: web.Application) -> None:
        app.router.add_get("/head", self._index)
        app.router.add_get("/ws/head", self._websocket)

    async def _index(self, _request: web.Request) -> web.StreamResponse:
        raise web.HTTPFound("/arm#head-controls")

    async def _websocket(self, request: web.Request) -> web.StreamResponse:
        if not same_origin(request):
            raise web.HTTPForbidden(text="cross-origin WebSocket rejected")
        ws = web.WebSocketResponse(heartbeat=PAGE_WS_HEARTBEAT_S, max_msg_size=MAX_MESSAGE_BYTES)
        await ws.prepare(request)
        login = self._sessions.session_of(request)
        if login is None:
            await ws.close(code=CLOSE_LOGIN_REQUIRED, message=b"login required")
            return ws
        client = request.remote or ""
        # One lease identity per connection: closing this socket holds the head.
        connection = uuid.uuid4().hex
        self._operators.connect(connection, login)
        pusher = asyncio.ensure_future(self._push_status(ws, connection))
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    break
                await self._handle(ws, connection, message.data, login, client)
        finally:
            pusher.cancel()
            try:
                await pusher
            except (asyncio.CancelledError, ConnectionResetError):
                pass
            try:
                self.console.disconnect(connection)
            except HeadError:
                pass  # no publisher yet: nothing was commanded
            self._operators.disconnect(connection)
        return ws

    async def _push_status(self, ws: web.WebSocketResponse, connection: str) -> None:
        while not ws.closed:
            status = self.console.status(connection)
            status["busy"] = self._operation.locked()
            await ws.send_json({"type": "status", **status})
            await asyncio.sleep(self._status_period)

    async def _handle(self, ws, connection, raw, login, client) -> None:
        try:
            message = json.loads(raw)
            if not isinstance(message, dict) or not isinstance(message.get("type"), str):
                raise ValueError("message must be an object with a type")
        except ValueError:
            await ws.send_json({"type": "result", "id": None, "ok": False, "error": "bad_json"})
            return
        kind, request_id = message.pop("type"), message.pop("id", None)
        try:
            result = await asyncio.get_running_loop().run_in_executor(
                None, self._command, kind, connection, message
            )
        except HeadError as exc:
            self._audit.record(
                f"head_{kind}",
                message,
                ok=False,
                error=exc.code,
                message=exc.message,
                session=login,
                client=client,
            )
            await ws.send_json(
                {
                    "type": "result",
                    "id": request_id,
                    "ok": False,
                    "error": exc.code,
                    "message": exc.message,
                }
            )
            return
        except (KeyError, TypeError, ValueError) as exc:
            self._audit.record(
                f"head_{kind}",
                message,
                ok=False,
                error="bad_request",
                message=str(exc),
                session=login,
                client=client,
            )
            await ws.send_json(
                {
                    "type": "result",
                    "id": request_id,
                    "ok": False,
                    "error": "bad_request",
                    "message": f"请求参数无效：{exc}",
                }
            )
            return
        self._audit.record(f"head_{kind}", message, ok=True, session=login, client=client)
        await ws.send_json({"type": "result", "id": request_id, "ok": True, **(result or {})})

    def _require_lease(self, connection: str) -> None:
        if self.console.lease_holder != connection:
            raise HeadError("lease_required", "未持有头部控制权")

    def _exclusive(self, call):
        if not self._operation.acquire(blocking=False):
            raise HeadError("busy", "上一项使能/复位操作尚未完成")
        try:
            return call()
        finally:
            self._operation.release()

    def _scope_call(self, connection, call):
        permit, confirmed = None, False
        try:
            if self._arm_control is not None:
                permit = self._arm_control.begin_scope_operation('head', connection)
            result = self._exclusive(call)
            confirmed = True
            return result
        except Exception as exc:
            if hasattr(exc, 'code') and not isinstance(exc, HeadError):
                raise HeadError(exc.code, str(exc)) from exc
            raise
        finally:
            if permit is not None:
                self._arm_control.end_scope_operation(permit, confirmed=confirmed)

    def _command(self, kind: str, connection: str, body: dict) -> Optional[dict]:
        console = self.console
        if kind == "acquire":
            if not self._operators.may_acquire(PAGE, connection):
                raise HeadError("lease_busy", "另一位操作员正在操作其他页面")
            console.acquire(connection)
        elif kind == "release":
            console.release(connection)
        elif kind == "move":
            if self._operation.locked():
                raise HeadError("busy", "正在切换使能")
            return console.move(connection, body["target"])
        elif kind == "hold":
            return console.hold()
        elif kind == "enable":
            enabled = body["enabled"]
            if not isinstance(enabled, bool):
                raise TypeError("enabled must be a boolean")
            self._require_lease(connection)
            result = self._scope_call(connection, lambda: self._set_enabled(enabled))
            if not result["accepted"]:
                raise HeadError("enable_rejected", result["message"] or "使能请求被拒绝")
            return result
        elif kind == "reset_fault":
            self._require_lease(connection)
            result = self._scope_call(connection, self._reset_fault)
            if not result["ok"] or result["stage"] not in RESET_OK_STAGES:
                raise HeadError("reset_rejected", f"故障复位未完成：{result['stage']}")
            return result
        else:
            raise HeadError("not_found", "未知操作")
        return None
