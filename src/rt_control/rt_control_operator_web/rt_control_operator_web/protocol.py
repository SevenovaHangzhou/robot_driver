"""Whitelisted WebSocket message parsing."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

MAX_MESSAGE_BYTES = 1024
ALLOWED_TYPES = frozenset(
    {"auth", "acquire", "release", "drive", "halt", "gear", "soft_stop", "get_model"}
)


class ProtocolError(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class Request:
    type: str
    token: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: int = 0
    gear: str = ""


def _short_string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not 0 < len(value) <= 256:
        raise ProtocolError("bad_request", f"{field} must be a non-empty string")
    return value


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError("bad_request", f"{field} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ProtocolError("bad_request", f"{field} must be finite")
    return number


def parse_request(raw: str) -> Request:
    if len(raw.encode("utf-8")) > MAX_MESSAGE_BYTES:
        raise ProtocolError("bad_request", "message too large")
    try:
        document = json.loads(raw)
    except (json.JSONDecodeError, ValueError) as exc:
        raise ProtocolError("bad_request", "invalid JSON") from exc
    if not isinstance(document, dict):
        raise ProtocolError("bad_request", "message must be an object")
    kind = document.get("type")
    if kind not in ALLOWED_TYPES:
        raise ProtocolError("unknown_type", "message type is not allowed")
    if kind == "auth":
        return Request(type=kind, token=_short_string(document.get("token"), "token"))
    if kind == "drive":
        yaw = document.get("yaw", 0)
        if isinstance(yaw, bool) or yaw not in (-1, 0, 1):
            raise ProtocolError("bad_request", "yaw must be -1, 0 or 1")
        return Request(
            type=kind,
            x=_number(document.get("x"), "x"),
            y=_number(document.get("y"), "y"),
            yaw=int(yaw),
        )
    if kind == "gear":
        return Request(type=kind, gear=_short_string(document.get("gear"), "gear"))
    return Request(type=kind)
