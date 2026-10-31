"""Diagnostics page: read-only. Self-check reports, R-OUT-09 readiness and raw /diagnostics.

No lease, no command: the page only shows what integ_diagnostics and the launch wrote or
published. Self-check reports remain readable when no runtime stack is up.
"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Callable

from aiohttp import WSMsgType, web

from ..session import Sessions, same_origin
from .feed import DiagnosticsFeed

STATUS_PERIOD_S = 1.0
CLOSE_LOGIN_REQUIRED = 4401


def self_check_reports(directory: Path) -> list[dict]:
    """Latest report per control scope, newest first; unreadable files are reported as such."""
    reports = []
    paths = sorted(directory.glob("*.json"), key=lambda path: path.stat().st_mtime, reverse=True)
    for path in paths:
        try:
            reports.append(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, ValueError) as exc:
            reports.append({"control_scope": path.stem, "error": f"报告无法读取：{exc}"})
    return reports


class DiagPage:
    def __init__(
        self,
        feed: DiagnosticsFeed,
        sessions: Sessions,
        web_dir: Path,
        reports_dir: Callable[[], Path],
        status_period: float = STATUS_PERIOD_S,
    ) -> None:
        if not (web_dir / "index.html").is_file():
            raise ValueError(f"missing web asset: {web_dir / 'index.html'}")
        self._feed = feed
        self._sessions = sessions
        self._web_dir = web_dir
        self._reports_dir = reports_dir
        self._status_period = status_period

    def add_routes(self, app: web.Application) -> None:
        app.router.add_get("/diag", self._index)
        app.router.add_get("/ws/diag", self._websocket)

    async def _index(self, _request: web.Request) -> web.FileResponse:
        return web.FileResponse(self._web_dir / "index.html")

    def status(self) -> dict:
        return {
            "type": "status",
            **self._feed.snapshot(),
            "self_checks": self_check_reports(self._reports_dir()),
        }

    async def _websocket(self, request: web.Request) -> web.StreamResponse:
        if not same_origin(request):
            raise web.HTTPForbidden(text="cross-origin WebSocket rejected")
        ws = web.WebSocketResponse(heartbeat=1.0, max_msg_size=1024)
        await ws.prepare(request)
        if self._sessions.session_of(request) is None:
            await ws.close(code=CLOSE_LOGIN_REQUIRED, message=b"login required")
            return ws
        loop = asyncio.get_running_loop()
        receiver = asyncio.ensure_future(ws.receive())
        try:
            while not ws.closed:
                status = await loop.run_in_executor(None, self.status)
                await ws.send_json(status)
                done, _ = await asyncio.wait({receiver}, timeout=self._status_period)
                if done:
                    message = receiver.result()
                    if message.type in (WSMsgType.CLOSE, WSMsgType.CLOSED, WSMsgType.ERROR):
                        break
                    receiver = asyncio.ensure_future(ws.receive())
        finally:
            receiver.cancel()
        return ws
