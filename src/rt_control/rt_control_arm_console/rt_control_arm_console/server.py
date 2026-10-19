"""Standard-library HTTP front: static page, JSON commands, Server-Sent Events status.

The browser never sees the ROS graph. Every command passes a session cookie
issued after token login, a same-origin check and the console's own checks.
"""

from __future__ import annotations

import hmac
from http import HTTPStatus
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import secrets
import threading
import time
from typing import Callable, Optional
from urllib.parse import urlsplit

from .audit import AuditLog
from .console import ArmConsole, ConsoleError
from .model import MESH_URL_PREFIX, resolve_mesh

RESET_OK_STAGES = {"success", "already_clear"}
RESET_HINTS = {
    "restart_required": "控制器状态无法确认，需要重启控制栈（工控机上 arm restart）",
    "operation_in_progress": "另一项操作进行中，稍后再试",
    "fault_reset_timeout": "复位后仍有轴未回到禁止合闸，检查首故障轴和实体安全链",
    "preempted_by_disable": "被失能请求打断，稍后再试",
}
MAX_BODY_BYTES = 16 * 1024
STATUS_PERIOD_S = 0.05
LOGIN_FAILURE_DELAY_S = 1.0
COOKIE = "arm_console_session"
SECURITY_HEADERS = {
    "Content-Security-Policy": (
        "default-src 'self'; connect-src 'self'; img-src 'self' data: blob:; "
        "style-src 'self'; script-src 'self'; frame-ancestors 'none'"
    ),
    "X-Frame-Options": "DENY",
    "X-Content-Type-Options": "nosniff",
    "Referrer-Policy": "no-referrer",
}
mimetypes.add_type("text/javascript", ".js")


class ConsoleServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        address,
        console: ArmConsole,
        token: str,
        web_dir: Path,
        share_lookup: Callable[[str], str],
        set_enabled: Callable[[bool], dict],
        audit: Optional[AuditLog] = None,
        reset_fault: Optional[Callable[[], dict]] = None,
    ) -> None:
        if not token:
            raise ValueError("an access token is required")
        if not (web_dir / "index.html").is_file():
            raise ValueError(f"missing web asset: {web_dir / 'index.html'}")
        super().__init__(address, Handler)
        self.console = console
        self.token = token.encode("utf-8")
        self.web_dir = web_dir.resolve()
        self.share_lookup = share_lookup
        self.set_enabled = set_enabled
        self.audit = audit or AuditLog(None)
        self.reset_fault = reset_fault
        self.sessions: set[str] = set()
        self.lock = threading.Lock()
        self.stopping = threading.Event()


class Handler(BaseHTTPRequestHandler):
    server: ConsoleServer
    protocol_version = "HTTP/1.1"

    def log_message(self, *_args) -> None:  # quiet; ROS logging carries the facts
        return

    # ------------------------------------------------------------------ helpers
    def _headers(
        self, status: int, content_type: str, length: Optional[int], extra: Optional[dict] = None
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        if length is not None:
            self.send_header("Content-Length", str(length))
        headers = {"Cache-Control": "no-store", **SECURITY_HEADERS, **(extra or {})}
        for name, value in headers.items():
            self.send_header(name, value)
        self.end_headers()

    def _json(self, status: int, document: dict, extra: Optional[dict] = None) -> None:
        body = json.dumps(document, ensure_ascii=False).encode("utf-8")
        self._headers(status, "application/json; charset=utf-8", len(body), extra)
        self.wfile.write(body)

    def _session(self) -> Optional[str]:
        cookie = SimpleCookie(self.headers.get("Cookie", ""))
        morsel = cookie.get(COOKIE)
        if morsel is None:
            return None
        with self.server.lock:
            return morsel.value if morsel.value in self.server.sessions else None

    def _same_origin(self) -> bool:
        origin = self.headers.get("Origin")
        return origin is None or urlsplit(origin).netloc == self.headers.get("Host")

    def _file(self, path: Optional[Path], cache: bool = False) -> None:
        if path is None or not path.is_file():
            self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
            return
        data = path.read_bytes()
        kind = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        # Whole-vehicle meshes are tens of MB; let the browser keep them for an hour.
        extra = {"Cache-Control": "private, max-age=3600"} if cache else None
        self._headers(HTTPStatus.OK, kind, len(data), extra)
        self.wfile.write(data)

    def _static(self, relative: str) -> Optional[Path]:
        # Contain the lexical path, not the symlink target: a symlink install points every
        # installed asset back into the source tree, and resolve() would reject all of them.
        path = Path(os.path.normpath(self.server.web_dir / relative))
        return path if path.is_relative_to(self.server.web_dir) else None

    # ------------------------------------------------------------------ GET
    def do_GET(self) -> None:  # noqa: N802 - http.server API
        path = urlsplit(self.path).path
        if path == "/":
            self._file(self.server.web_dir / "index.html")
        elif path.startswith("/static/"):
            self._file(self._static(path[len("/static/") :]))
        elif path.startswith(MESH_URL_PREFIX):
            if self._session() is None:
                self._json(HTTPStatus.UNAUTHORIZED, {"error": "login_required"})
                return
            self._file(
                resolve_mesh(path[len(MESH_URL_PREFIX) :], self.server.share_lookup), cache=True
            )
        elif path == "/api/info":
            if self._session() is None:
                self._json(HTTPStatus.UNAUTHORIZED, {"error": "login_required"})
                return
            console = self.server.console
            self._json(
                HTTPStatus.OK,
                {
                    **console.static_info(),
                    "urdf": console.model.browser_urdf if console.model else None,
                },
            )
        elif path == "/api/events":
            self._events()
        else:
            self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

    def _events(self) -> None:
        session = self._session()
        if session is None:
            self._json(HTTPStatus.UNAUTHORIZED, {"error": "login_required"})
            return
        self._headers(HTTPStatus.OK, "text/event-stream", None, {"Connection": "close"})
        self.close_connection = True
        console, pose_version = self.server.console, None
        try:
            while not self.server.stopping.is_set():
                console.touch(session)
                status = console.status(session)
                poses = console.poses.list() if console.poses else []
                version = json.dumps(poses, sort_keys=True)
                if version != pose_version:
                    status["poses"], pose_version = poses, version
                payload = json.dumps(status, ensure_ascii=False)
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
                time.sleep(STATUS_PERIOD_S)
        except (BrokenPipeError, ConnectionResetError):
            pass

    # ------------------------------------------------------------------ POST
    def do_POST(self) -> None:  # noqa: N802 - http.server API
        path = urlsplit(self.path).path
        if not self._same_origin():
            self._json(HTTPStatus.FORBIDDEN, {"error": "cross_origin"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = -1
        if not 0 <= length <= MAX_BODY_BYTES:
            self._json(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, {"error": "body_too_large"})
            return
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(body, dict):
                raise ValueError
        except ValueError:
            self._json(HTTPStatus.BAD_REQUEST, {"error": "bad_json", "message": "请求格式无效"})
            return
        if path == "/api/login":
            self._login(body)
            return
        session = self._session()
        if session is None:
            self._json(HTTPStatus.UNAUTHORIZED, {"error": "login_required", "message": "请先登录"})
            return
        action = path.removeprefix("/api/")
        audit = self.server.audit
        client = self.client_address[0] if self.client_address else ""
        try:
            result = self._command(path, session, body) or {}
        except ConsoleError as exc:
            audit.record(
                action,
                body,
                ok=False,
                error=exc.code,
                message=exc.message,
                session=session,
                client=client,
            )
            self._json(
                HTTPStatus.CONFLICT, {"ok": False, "error": exc.code, "message": exc.message}
            )
            return
        except (KeyError, TypeError, ValueError) as exc:
            audit.record(
                action,
                body,
                ok=False,
                error="bad_request",
                message=str(exc),
                session=session,
                client=client,
            )
            self._json(
                HTTPStatus.BAD_REQUEST,
                {"ok": False, "error": "bad_request", "message": f"请求参数无效：{exc}"},
            )
            return
        audit.record(action, body, ok=True, session=session, client=client)
        self._json(HTTPStatus.OK, {"ok": True, **result})

    def _login(self, body: dict) -> None:
        token = body.get("token")
        if not isinstance(token, str) or not hmac.compare_digest(
            token.encode("utf-8"), self.server.token
        ):
            time.sleep(LOGIN_FAILURE_DELAY_S)
            self.server.audit.record(
                "login", {}, ok=False, error="auth_failed", client=self.client_address[0]
            )
            self._json(HTTPStatus.UNAUTHORIZED, {"error": "auth_failed", "message": "令牌无效"})
            return
        session = secrets.token_urlsafe(24)
        self.server.audit.record(
            "login", {}, ok=True, session=session, client=self.client_address[0]
        )
        with self.server.lock:
            self.server.sessions.add(session)
        cookie = f"{COOKIE}={session}; HttpOnly; SameSite=Strict; Path=/"
        self._json(HTTPStatus.OK, {"ok": True}, {"Set-Cookie": cookie})

    def _command(self, path: str, session: str, body: dict) -> Optional[dict]:
        console = self.server.console
        if path == "/api/acquire":
            console.acquire(session)
        elif path == "/api/release":
            console.release(session)
        elif path == "/api/preview":
            return console.preview(body["target"], body["gear"])
        elif path == "/api/execute":
            return console.execute(session, body["target"], body["gear"])
        elif path == "/api/stop":
            return console.stop()
        elif path == "/api/goto_pose":
            return console.goto_pose(session, body["name"], body["gear"])
        elif path == "/api/run_sequence":
            return console.run_sequence(session, body["names"], body["gear"])
        elif path == "/api/save_pose":
            console.save_pose(session, body["name"], bool(body.get("overwrite", False)))
        elif path == "/api/delete_pose":
            console.delete_pose(session, body["name"])
        elif path == "/api/enable":
            enabled = body["enabled"]
            if not isinstance(enabled, bool):
                raise TypeError("enabled must be a boolean")
            console.begin_enable(session)
            try:
                result = self.server.set_enabled(enabled)
            finally:
                console.end_enable()
            if not result["accepted"]:
                raise ConsoleError("enable_rejected", result["message"] or "使能请求被拒绝")
            return result
        elif path == "/api/reset_fault":
            if self.server.reset_fault is None:
                raise ConsoleError("reset_unavailable", "故障复位不可用")
            console.begin_enable(session)
            try:
                result = self.server.reset_fault()
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
