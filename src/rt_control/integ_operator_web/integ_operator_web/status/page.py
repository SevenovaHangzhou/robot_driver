"""Status page: the read-only home page. One WebSocket pushes the snapshot from ``feed.document``.

No lease, no command, no control ownership: the page only displays public data and never takes part
in the "one operator at a time" rule.
"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Callable, Optional

from aiohttp import WSMsgType, web

from ..session import PAGE_WS_HEARTBEAT_S, Sessions, same_origin
from . import feed as feed_module
from .logstation import LogStation, evidence

STATUS_PERIOD_S = 1.0
CLOSE_LOGIN_REQUIRED = 4401
MESH_CACHE = {"Cache-Control": "public, max-age=3600"}


class StatusPage:
    def __init__(
        self,
        feed: feed_module.Feed,
        sessions: Sessions,
        web_dir: Path,
        *,
        camera_base: str = "",
        robot_label: str = "",
        roles: Optional[dict] = None,
        log_station: Optional[LogStation] = None,
        model_file: Optional[Path] = None,
        mesh_lookup: Optional[Callable[[str], str]] = None,
        status_period: float = STATUS_PERIOD_S,
    ) -> None:
        if not (web_dir / "index.html").is_file():
            raise ValueError(f"missing web asset: {web_dir / 'index.html'}")
        self._feed = feed
        self._sessions = sessions
        self._web_dir = web_dir
        self._camera_base = camera_base.rstrip("/")
        self._robot_label = robot_label
        self._roles = dict(roles or {})
        self._log = log_station or LogStation("")
        self._model_file = model_file
        self._mesh_lookup = mesh_lookup
        self._status_period = status_period

    def start_background(self) -> None:
        self._log.start()

    def add_routes(self, app: web.Application) -> None:
        app.router.add_get("/", self._index)
        app.router.add_get("/api/status/model", self._model)
        app.router.add_get("/status-mesh/{path:.+}", self._mesh)
        app.router.add_get("/ws/status", self._websocket)

    async def _index(self, _request: web.Request) -> web.FileResponse:
        return web.FileResponse(self._web_dir / "index.html")

    def _require_login(self, request: web.Request) -> Optional[str]:
        session = self._sessions.session_of(request)
        if session is None:
            raise web.HTTPUnauthorized(
                text=json.dumps({"error": "login_required"}), content_type="application/json"
            )
        return session

    async def _model(self, request: web.Request) -> web.Response:
        self._require_login(request)
        if self._model_file is None or not self._model_file.is_file():
            raise web.HTTPNotFound()
        return web.Response(
            text=self._model_file.read_text(encoding="utf-8"), content_type="application/xml"
        )

    async def _mesh(self, request: web.Request) -> web.StreamResponse:
        self._require_login(request)
        path = self._mesh_lookup(request.match_info["path"]) if self._mesh_lookup else None
        if path is None:
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers=MESH_CACHE)

    def snapshot(self) -> dict:
        document = feed_module.document(
            self._feed, state_period_s=self._status_period, roles=self._roles
        )
        log = self._log.snapshot()
        document["log"] = log
        document["alerts"] = evidence(document["alerts"], log)
        document["camera_base"] = self._camera_base
        document["robot_label"] = self._robot_label
        document["model"] = self._model_file is not None and self._model_file.is_file()
        return document

    async def _websocket(self, request: web.Request) -> web.WebSocketResponse:
        session = self._sessions.session_of(request)
        if session is None or not same_origin(request):
            raise web.HTTPUnauthorized()
        ws = web.WebSocketResponse(heartbeat=PAGE_WS_HEARTBEAT_S)
        await ws.prepare(request)
        try:
            while not ws.closed:
                await ws.send_json(self.snapshot())
                try:
                    message = await asyncio.wait_for(ws.receive(), timeout=self._status_period)
                except asyncio.TimeoutError:
                    continue
                if message.type in (WSMsgType.CLOSE, WSMsgType.CLOSED, WSMsgType.ERROR):
                    break
        finally:
            await ws.close()
        return ws
