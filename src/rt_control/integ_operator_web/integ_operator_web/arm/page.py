"""Arm page: model/mesh GETs and one WebSocket for status and commands.

The console logic is unchanged from rt_control_arm_console; only the transport moved from
POST + Server-Sent Events to WebSocket messages. Every command is audited. Closing the
socket that holds the lease stops the arm smoothly and releases it.
"""

from __future__ import annotations

import asyncio
import hashlib
import json
from pathlib import Path
from typing import Callable, Optional
import uuid

from aiohttp import WSMsgType, web

from ..audit import AuditLog
from ..operators import Operators
from ..session import Sessions, same_origin
from .console import ArmConsole, ConsoleError
from .model import MESH_URL_PREFIX, ModelError, load_arm_model, resolve_mesh

PAGE = "arm"
STATUS_PERIOD_S = 0.05
MAX_MESSAGE_BYTES = 16 * 1024
CLOSE_LOGIN_REQUIRED = 4401
# Whole-vehicle meshes are tens of MB; let the browser keep them for an hour.
MESH_CACHE = {"Cache-Control": "private, max-age=3600"}
RESET_OK_STAGES = {"success", "already_clear"}
RESET_HINTS = {
    "restart_required": "控制器状态无法确认，需要重启控制栈（工控机上 arm restart）",
    "operation_in_progress": "另一项操作进行中，稍后再试",
    "fault_reset_timeout": "复位后仍有轴未回到禁止合闸，检查首故障轴和实体安全链",
    "preempted_by_disable": "被失能请求打断，稍后再试",
}


class ArmPage:
    def __init__(
        self,
        console: ArmConsole,
        sessions: Sessions,
        operators: Operators,
        web_dir: Path,
        share_lookup: Callable[[str], str],
        set_enabled: Callable[[bool], dict],
        reset_fault: Optional[Callable[[], dict]] = None,
        audit: Optional[AuditLog] = None,
        view_model_share: Optional[Path] = None,
        vacuum=None,
        status_period: float = STATUS_PERIOD_S,
    ) -> None:
        if not (web_dir / "index.html").is_file():
            raise ValueError(f"missing web asset: {web_dir / 'index.html'}")
        self.console = console
        self._sessions = sessions
        self._operators = operators
        self._web_dir = web_dir
        self._share_lookup = share_lookup
        self._set_enabled = set_enabled
        self._reset_fault = reset_fault
        self._audit = audit or AuditLog(None)
        self._vacuum = vacuum
        self._status_period = status_period
        self._sockets: set[web.WebSocketResponse] = set()
        self._view_share = view_model_share.resolve() if view_model_share is not None else None
        self._view_text = self._view_prefix = self._view_metadata = None
        if self._view_share is not None:
            entry = self._view_share / "urdf/robot_alfav3_mobile.urdf"
            data = entry.read_bytes()
            self._view_text = data.decode("utf-8")
            manifest = self._view_share / "config/mesh_assets.json"
            digest = hashlib.sha256(
                data + (manifest.read_bytes() if manifest.is_file() else b"")
            ).hexdigest()
            self._view_prefix = f"/view-mesh/{digest}/"
            self._view_metadata = {
                "entry": "urdf/robot_alfav3_mobile.urdf",
                "urdf_sha256": hashlib.sha256(data).hexdigest(),
                "assets_version": digest,
            }
        operators.register_page(PAGE, lambda: console.lease_holder)

    def add_routes(self, app: web.Application) -> None:
        app.router.add_get("/arm", self._index)
        app.router.add_get("/api/arm/info", self._info)
        app.router.add_get("/api/arm/vacuum", self._vacuum_status)
        app.router.add_get(MESH_URL_PREFIX + "{path:.+}", self._mesh)
        if self._view_prefix:
            app.router.add_get(self._view_prefix + "{path:.+}", self._view_mesh)
        app.router.add_get("/ws/arm", self._websocket)
        app.on_shutdown.append(self._on_shutdown)

    # ------------------------------------------------------------------ GET
    async def _index(self, _request: web.Request) -> web.FileResponse:
        return web.FileResponse(self._web_dir / "index.html")

    def _require_login(self, request: web.Request) -> None:
        if self._sessions.session_of(request) is None:
            raise web.HTTPUnauthorized(
                text=json.dumps({"error": "login_required"}), content_type="application/json"
            )

    def _view_lookup(self, package: str) -> str:
        if package == "robot_description" and self._view_share is not None:
            return str(self._view_share)
        return self._share_lookup(package)

    async def _mesh(self, request: web.Request) -> web.StreamResponse:
        self._require_login(request)
        path = resolve_mesh(request.match_info["path"], self._share_lookup)
        if path is None:
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers=MESH_CACHE)

    async def _view_mesh(self, request: web.Request) -> web.StreamResponse:
        self._require_login(request)
        path = resolve_mesh(request.match_info["path"], self._view_lookup)
        if path is None:
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers=MESH_CACHE)

    async def _info(self, request: web.Request) -> web.Response:
        self._require_login(request)
        model = self.console.model
        if self._view_text is not None:
            try:
                model = load_arm_model(
                    self._view_text, self.console.joint_names, mesh_prefix=self._view_prefix
                )
            except (ModelError, ValueError, AttributeError, IndexError) as exc:
                return web.json_response(
                    {"error": "view_model_unavailable", "message": str(exc)}, status=503
                )
        return web.json_response(
            {
                **self.console.static_info(),
                "urdf": model.browser_urdf if model else None,
                "arm_links": list(model.arm_links) if model else [],
                "view_model": self._view_metadata,
            }
        )

    async def _vacuum_status(self, request: web.Request) -> web.Response:
        self._require_login(request)
        if self._vacuum is None:
            state = {
                "configured": False,
                "valid": False,
                "busy": False,
                "outputs": {},
                "message": "未配置真空控制工具",
            }
        else:
            state = await asyncio.get_running_loop().run_in_executor(None, self._vacuum.status)
        return web.json_response(state)

    # ------------------------------------------------------------------ WebSocket
    async def _on_shutdown(self, _app: web.Application) -> None:
        self.console.stop()
        for ws in list(self._sockets):
            await ws.close(code=1001, message=b"server shutdown")

    async def _websocket(self, request: web.Request) -> web.StreamResponse:
        if not same_origin(request):
            raise web.HTTPForbidden(text="cross-origin WebSocket rejected")
        ws = web.WebSocketResponse(heartbeat=1.0, max_msg_size=MAX_MESSAGE_BYTES)
        await ws.prepare(request)
        login = self._sessions.session_of(request)
        if login is None:
            await ws.close(code=CLOSE_LOGIN_REQUIRED, message=b"login required")
            return ws
        client = request.remote or ""
        # One lease identity per connection: closing this socket stops what it commanded.
        connection = uuid.uuid4().hex
        self._operators.connect(connection, login)
        self._sockets.add(ws)
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
            if self.console.lease_holder == connection:
                await asyncio.get_running_loop().run_in_executor(None, self.console.stop)
                self.console.release(connection)
            self._operators.disconnect(connection)
            self._sockets.discard(ws)
        return ws

    async def _push_status(self, ws: web.WebSocketResponse, connection: str) -> None:
        poses_version = None
        while not ws.closed:
            self.console.touch(connection)
            status = self.console.status(connection)
            poses = self.console.poses.list() if self.console.poses else []
            version = json.dumps(poses, sort_keys=True)
            if version != poses_version:
                status["poses"], poses_version = poses, version
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
        except ConsoleError as exc:
            self._audit.record(
                kind,
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
                kind,
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
        self._audit.record(kind, message, ok=True, session=login, client=client)
        await ws.send_json({"type": "result", "id": request_id, "ok": True, **(result or {})})

    def _command(self, kind: str, connection: str, body: dict) -> Optional[dict]:
        console = self.console
        if kind == "vacuum":
            if self._vacuum is None:
                raise ConsoleError("vacuum_unavailable", "未配置真空控制工具")
            status = console.status(connection)
            if status["lease"] != "you":
                raise ConsoleError("lease_required", "请先获取控制权")
            if status["moving"] or status["sequence"] or status["enable"]["busy"]:
                raise ConsoleError("busy", "请等待当前运动、序列或使能操作结束")
            return self._vacuum.set_output(body["output"], body["enabled"], body["expected"])
        if kind == "acquire":
            if not self._operators.may_acquire(PAGE, connection):
                raise ConsoleError("lease_busy", "另一位操作员正在操作其他页面")
            console.acquire(connection)
        elif kind == "release":
            console.release(connection)
        elif kind == "preview":
            return console.preview(body["target"], body["gear"])
        elif kind == "execute":
            return console.execute(connection, body["target"], body["gear"])
        elif kind == "stop":
            return console.stop()
        elif kind == "goto_pose":
            return console.goto_pose(connection, body["name"], body["gear"])
        elif kind == "run_sequence":
            return console.run_sequence(
                connection,
                body.get("names", []),
                body.get("gear", ""),
                body.get("dwell_s", 0),
                steps=body.get("steps"),
            )
        elif kind == "save_pose":
            console.save_pose(connection, body["name"], bool(body.get("overwrite", False)))
        elif kind == "delete_pose":
            console.delete_pose(connection, body["name"])
        elif kind == "enable":
            enabled = body["enabled"]
            if not isinstance(enabled, bool):
                raise TypeError("enabled must be a boolean")
            console.begin_enable(connection)
            try:
                result = self._set_enabled(enabled)
            finally:
                console.end_enable()
            if not result["accepted"]:
                raise ConsoleError("enable_rejected", result["message"] or "使能请求被拒绝")
            return result
        elif kind == "reset_fault":
            if self._reset_fault is None:
                raise ConsoleError("reset_unavailable", "故障复位不可用")
            console.begin_enable(connection)
            try:
                result = self._reset_fault()
            finally:
                console.end_enable()
            if not result["ok"] or result["stage"] not in RESET_OK_STAGES:
                hint = RESET_HINTS.get(result["stage"], "")
                joint = f"，关节 {result['failed_joint']}" if result.get("failed_joint") else ""
                raise ConsoleError(
                    "reset_rejected",
                    f"故障复位未完成：{result['stage']}{joint}。{hint}".rstrip("。") + "。",
                )
            return result
        else:
            raise ConsoleError("not_found", "未知操作")
        return None
