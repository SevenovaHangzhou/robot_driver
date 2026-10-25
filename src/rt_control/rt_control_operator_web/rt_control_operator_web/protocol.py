"""Whitelisted WebSocket message parsing."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

MAX_MESSAGE_BYTES = 1024
MODULE_OPERATIONS = frozenset({"enable", "disable", "reset"})
MAX_MODULES = 8
ALLOWED_TYPES = frozenset(
    {
        "auth",
        "acquire",
        "release",
        "drive",
        "halt",
        "gear",
        "soft_stop",
        "get_model",
        "module_op",
        "get_fault_history",
        "steering_params",
        "steering_defaults",
    }
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
    z: float = 0.0
    w: float = 0.0
    release: float = 0.0
    gear: str = ""
    op: str = ""
    modules: tuple[str, ...] = ()


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
    if kind == "steering_params":
        # ELECTRI-127: operator-tuned steering motion; the token is re-confirmed per change.
        return Request(
            type=kind,
            token=_short_string(document.get("token"), "token"),
            x=_number(document.get("slew_deg_s"), "slew_deg_s"),
            y=_number(document.get("accel_deg_s2"), "accel_deg_s2"),
            z=_number(document.get("jerk_deg_s3"), "jerk_deg_s3"),
            w=_number(document.get("alignment_deg"), "alignment_deg"),
            # Only the align drive mode has a release angle; absent means not applicable.
            release=_number(document.get("release_deg", 0.0), "release_deg"),
        )
    if kind == "steering_defaults":
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
    if kind == "module_op":
        op = document.get("op")
        if op not in MODULE_OPERATIONS:
            raise ProtocolError("bad_request", "op must be enable, disable or reset")
        modules = document.get("modules")
        if not isinstance(modules, list) or not 0 < len(modules) <= MAX_MODULES:
            raise ProtocolError("bad_request", "modules must be a non-empty short list")
        return Request(type=kind, op=op, modules=tuple(_short_string(m, "module") for m in modules))
    if kind == "gear":
        return Request(type=kind, gear=_short_string(document.get("gear"), "gear"))
    return Request(type=kind)
