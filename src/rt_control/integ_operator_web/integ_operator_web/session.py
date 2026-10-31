"""Token login -> HttpOnly session cookie; WebSocket handshakes check the cookie, not a token.

Sessions live in memory for the server process; a restart logs everyone out.
"""

from __future__ import annotations

import asyncio
import hmac
import json
import secrets
from typing import Optional
from urllib.parse import urlsplit

from aiohttp import web

COOKIE = "integ_operator_session"
LOGIN_FAILURE_DELAY_S = 1.0
MAX_LOGIN_BYTES = 1024


def same_origin(request: web.Request) -> bool:
    origin = request.headers.get("Origin")
    if origin is None:
        return True  # non-browser clients (tests, CLI) send no Origin
    return urlsplit(origin).netloc == request.host


class Sessions:
    def __init__(self, token: str, failure_delay: float = LOGIN_FAILURE_DELAY_S) -> None:
        if not token:
            raise ValueError("an access token is required")
        self._token = token.encode("utf-8")
        self._failure_delay = failure_delay
        self._ids: set[str] = set()

    def token_matches(self, value: str) -> bool:
        return hmac.compare_digest(value.encode("utf-8"), self._token)

    async def reject_slowly(self) -> None:
        await asyncio.sleep(self._failure_delay)

    def session_of(self, request: web.Request) -> Optional[str]:
        value = request.cookies.get(COOKIE)
        return value if value in self._ids else None

    async def login(self, request: web.Request) -> web.Response:
        if not same_origin(request):
            raise web.HTTPForbidden(text="cross-origin login rejected")
        body = await request.read()
        try:
            token = json.loads(body)["token"] if len(body) <= MAX_LOGIN_BYTES else ""
        except (ValueError, KeyError, TypeError):
            token = ""
        if not isinstance(token, str) or not self.token_matches(token):
            await self.reject_slowly()
            return web.json_response({"ok": False, "message": "令牌无效"}, status=401)
        session = secrets.token_urlsafe(24)
        self._ids.add(session)
        response = web.json_response({"ok": True})
        response.set_cookie(COOKIE, session, httponly=True, samesite="Strict", path="/")
        return response

    async def logout(self, request: web.Request) -> web.Response:
        self._ids.discard(request.cookies.get(COOKIE, ""))
        response = web.json_response({"ok": True})
        response.del_cookie(COOKIE, path="/")
        return response
