"""aiohttp HTTP/WebSocket front for the teleop arbiter.

Only whitelisted requests (see ``protocol``) reach the arbiter. The server
never exposes the ROS graph to the browser.
"""

from __future__ import annotations

import asyncio
import hmac
import uuid
from pathlib import Path
from typing import Any, Awaitable, Callable, Mapping, Optional
from urllib.parse import urlsplit

from aiohttp import WSMsgType, web

from .protocol import MAX_MESSAGE_BYTES, ProtocolError, parse_request
from .teleop import (
    CLIENT_HEARTBEAT_PERIOD_S,
    CLIENT_MIN_SEND_PERIOD_S,
    COMMAND_TIMEOUT_S,
    STICK_DEADZONE,
    STICK_EXPO,
    STICK_SNAP_DEG,
    DriveArbiter,
    TeleopError,
)

AUTH_TIMEOUT_S = 5.0
AUTH_FAILURE_DELAY_S = 1.0
STATUS_PERIOD_S = 0.05
WS_HEARTBEAT_S = 1.0
CLOSE_AUTH_FAILED = 4401
CLOSE_AUTH_TIMEOUT = 4408
SECURITY_HEADERS = {
    "Content-Security-Policy": (
        "default-src 'self'; connect-src 'self'; img-src 'self' data:; "
        "style-src 'self'; script-src 'self'; frame-ancestors 'none'"
    ),
    "X-Frame-Options": "DENY",
    "X-Content-Type-Options": "nosniff",
    "Referrer-Policy": "no-referrer",
    "Cache-Control": "no-store",
}

StatusProvider = Callable[[], Mapping[str, Any]]
# (op, modules) -> {"ok": bool, "stage": str, "modules": [...]}; may take tens of seconds.
ModuleOperation = Callable[[str, tuple], Awaitable[Mapping[str, Any]]]
StaticInfo = Mapping[str, Any]


def _same_origin(request: web.Request) -> bool:
    origin = request.headers.get("Origin")
    if origin is None:
        return True  # non-browser clients (tests, CLI) send no Origin
    return urlsplit(origin).netloc == request.host


class OperatorServer:
    def __init__(
        self,
        arbiter: DriveArbiter,
        token: str,
        status_provider: StatusProvider,
        static_dir: Path,
        static_info: Optional[StaticInfo] = None,
        model_provider: Optional[Callable[[], Optional[Mapping[str, Any]]]] = None,
        module_operation: Optional[ModuleOperation] = None,
        history_provider: Optional[Callable[[], list]] = None,
        auth_timeout: float = AUTH_TIMEOUT_S,
        auth_failure_delay: float = AUTH_FAILURE_DELAY_S,
        status_period: float = STATUS_PERIOD_S,
        drive_allowed: Optional[Callable[[], bool]] = None,
        steering_operation: Optional[Callable[..., Awaitable[Mapping[str, Any]]]] = None,
        steering_defaults: Optional[Callable[[], Awaitable[Mapping[str, Any]]]] = None,
    ) -> None:
        if not token:
            raise ValueError("an access token is required")
        index = static_dir / "index.html"
        if not index.is_file():
            raise ValueError(f"missing web asset: {index}")
        self._drive_allowed = drive_allowed or (lambda: True)
        self._steering_operation = steering_operation
        self._steering_defaults = steering_defaults
        self._arbiter = arbiter
        self._token = token.encode("utf-8")
        self._status_provider = status_provider
        self._static_info = dict(static_info or {})
        self._model_provider = model_provider or (lambda: None)
        self._module_operation = module_operation
        self._history_provider = history_provider or (lambda: [])
        self._module_op_task: Optional[asyncio.Task] = None
        self._static_dir = static_dir
        self._auth_timeout = auth_timeout
        self._auth_failure_delay = auth_failure_delay
        self._status_period = status_period
        self._sockets: set[web.WebSocketResponse] = set()

    def build_app(self) -> web.Application:
        app = web.Application(middlewares=[self._headers_middleware])
        app.router.add_get("/", self._index)
        app.router.add_get("/ws", self._websocket)
        app.router.add_static("/static/", self._static_dir, show_index=False)
        app.on_shutdown.append(self._on_shutdown)
        return app

    @web.middleware
    async def _headers_middleware(self, request: web.Request, handler):
        response = await handler(request)
        for name, value in SECURITY_HEADERS.items():
            response.headers.setdefault(name, value)
        return response

    async def _index(self, _request: web.Request) -> web.FileResponse:
        return web.FileResponse(self._static_dir / "index.html")

    async def _on_shutdown(self, _app: web.Application) -> None:
        self._arbiter.soft_stop("server_shutdown")
        for ws in list(self._sockets):
            await ws.close(code=1001, message=b"server shutdown")

    def _status(self, session: str) -> dict[str, Any]:
        snap = self._arbiter.snapshot()
        if snap.lease_holder is None:
            lease = "none"
        elif snap.lease_holder == session:
            lease = "you"
        else:
            lease = "other"
        vx, vy, wz = snap.command
        tx, ty, tz = snap.target
        return {
            "type": "status",
            "lease": lease,
            "gear": snap.gear,
            "moving": snap.moving,
            "stopping_fast": snap.stopping_fast,
            "target": {"vx": tx, "vy": ty, "wz": tz},
            "command": {"vx": vx, "vy": vy, "wz": wz},
            "stop_reason": snap.last_stop_reason,
            "robot": dict(self._status_provider()),
        }

    async def _authenticate(self, ws: web.WebSocketResponse) -> bool:
        try:
            message = await asyncio.wait_for(ws.receive(), self._auth_timeout)
        except asyncio.TimeoutError:
            await ws.close(code=CLOSE_AUTH_TIMEOUT, message=b"auth timeout")
            return False
        ok = False
        if message.type == WSMsgType.TEXT:
            try:
                request = parse_request(message.data)
                ok = request.type == "auth" and hmac.compare_digest(
                    request.token.encode("utf-8"), self._token
                )
            except ProtocolError:
                ok = False
        if not ok:
            await asyncio.sleep(self._auth_failure_delay)
            if not ws.closed:
                await ws.send_json({"type": "error", "code": "auth_failed", "message": "令牌无效"})
                await ws.close(code=CLOSE_AUTH_FAILED, message=b"auth failed")
        return ok

    async def _push_status(self, ws: web.WebSocketResponse, session: str) -> None:
        while not ws.closed:
            await ws.send_json(self._status(session))
            await asyncio.sleep(self._status_period)

    async def _websocket(self, request: web.Request) -> web.StreamResponse:
        if not _same_origin(request):
            raise web.HTTPForbidden(text="cross-origin WebSocket rejected")
        ws = web.WebSocketResponse(heartbeat=WS_HEARTBEAT_S, max_msg_size=MAX_MESSAGE_BYTES)
        await ws.prepare(request)
        if not await self._authenticate(ws):
            return ws
        session = uuid.uuid4().hex
        self._sockets.add(ws)
        await ws.send_json(
            {
                "type": "auth_ok",
                "session": session,
                "gears": {
                    name: {"linear": gear.linear, "angular": gear.angular}
                    for name, gear in self._arbiter.gears.items()
                },
                "min_send_period_ms": round(CLIENT_MIN_SEND_PERIOD_S * 1000),
                "heartbeat_period_ms": round(CLIENT_HEARTBEAT_PERIOD_S * 1000),
                "command_timeout_ms": round(COMMAND_TIMEOUT_S * 1000),
                "stick": {
                    "deadzone": STICK_DEADZONE,
                    "expo": STICK_EXPO,
                    "snap_deg": STICK_SNAP_DEG,
                },
                **self._static_info,
            }
        )
        pusher = asyncio.ensure_future(self._push_status(ws, session))
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    break
                await self._handle(ws, session, message.data)
        finally:
            self._arbiter.disconnect(session)
            self._sockets.discard(ws)
            pusher.cancel()
            try:
                await pusher
            except (asyncio.CancelledError, ConnectionResetError):
                pass
        return ws

    def _start_module_op(
        self, ws: web.WebSocketResponse, session: str, op: str, modules: tuple
    ) -> None:
        """Run enable/disable/reset in the background; one operation at a time."""
        if self._module_operation is None:
            raise TeleopError("module_op_unavailable", "当前运行环境不支持使能管理")
        if not self._arbiter.is_holder(session):
            raise TeleopError("lease_required", "未持有控制权")
        if self._module_op_task is not None and not self._module_op_task.done():
            raise TeleopError("module_op_busy", "上一项使能/复位操作尚未完成")

        async def run() -> None:
            try:
                result = await self._module_operation(op, tuple(modules))
            except TeleopError as exc:
                result = {"ok": False, "stage": exc.message, "modules": []}
            except Exception as exc:  # noqa: BLE001 - reported to the operator
                result = {"ok": False, "stage": f"internal_error: {exc}", "modules": []}
            if not ws.closed:
                await ws.send_json(
                    {
                        "type": "module_op_result",
                        "op": op,
                        "requested": list(modules),
                        **dict(result),
                    }
                )

        self._module_op_task = asyncio.ensure_future(run())

    async def _handle(self, ws: web.WebSocketResponse, session: str, raw: str) -> None:
        try:
            request = parse_request(raw)
            if request.type == "acquire":
                self._arbiter.acquire(session)
            elif request.type == "release":
                self._arbiter.release(session)
            elif request.type == "drive":
                if not self._drive_allowed():
                    raise TeleopError("chassis_not_enabled", "请先连接并使能底盘")
                self._arbiter.drive(session, request.x, request.y, request.yaw)
            elif request.type == "gear":
                self._arbiter.set_gear(session, request.gear)
            elif request.type == "halt":
                self._arbiter.halt(session)
            elif request.type == "soft_stop":
                self._arbiter.soft_stop()
            elif request.type == "module_op":
                self._start_module_op(ws, session, request.op, request.modules)
                return
            elif request.type == "steering_params":
                if self._steering_operation is None:
                    raise TeleopError("unsupported_operation", "当前运行环境不支持转向参数调整")
                if not self._arbiter.is_holder(session):
                    raise TeleopError("lease_required", "未持有控制权")
                if not hmac.compare_digest(request.token.encode("utf-8"), self._token):
                    await asyncio.sleep(self._auth_failure_delay)
                    raise TeleopError("token_mismatch", "令牌确认失败，转向参数未修改")
                result = await self._steering_operation(
                    request.x, request.y, request.z, request.w, request.release
                )
                await ws.send_json({"type": "steering_params_result", **dict(result)})
                return
            elif request.type == "steering_defaults":
                if self._steering_defaults is None:
                    raise TeleopError("unsupported_operation", "当前运行环境不支持恢复默认")
                if not self._arbiter.is_holder(session):
                    raise TeleopError("lease_required", "未持有控制权")
                if not hmac.compare_digest(request.token.encode("utf-8"), self._token):
                    await asyncio.sleep(self._auth_failure_delay)
                    raise TeleopError("token_mismatch", "令牌确认失败，转向参数未修改")
                result = await self._steering_defaults()
                await ws.send_json({"type": "steering_params_result", **dict(result)})
                return
            elif request.type == "get_fault_history":
                await ws.send_json({"type": "fault_history", "items": self._history_provider()})
                return
            elif request.type == "get_model":
                # Read-only; any authenticated client may fetch the top view.
                model = self._model_provider()
                await ws.send_json(dict(model) if model else {"type": "model", "version": None})
                return
            else:  # repeated auth after login
                raise ProtocolError("bad_request", "already authenticated")
        except (ProtocolError, TeleopError) as exc:
            # Fail closed: any rejected request leaves this session's motion stopped.
            self._arbiter.halt(session)
            await ws.send_json({"type": "error", "code": exc.code, "message": exc.message})
