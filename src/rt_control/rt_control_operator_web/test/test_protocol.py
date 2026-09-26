import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rt_control_operator_web.protocol import (  # noqa: E402
    MAX_MESSAGE_BYTES,
    ProtocolError,
    parse_request,
)


def test_auth_message() -> None:
    request = parse_request(json.dumps({"type": "auth", "token": "abc"}))
    assert request.type == "auth" and request.token == "abc"


def test_drive_message() -> None:
    request = parse_request(json.dumps({"type": "drive", "x": 0.5, "y": -0.25, "yaw": 0}))
    assert (request.x, request.y, request.yaw) == (0.5, -0.25, 0)


def test_drive_yaw_defaults_to_zero() -> None:
    assert parse_request(json.dumps({"type": "drive", "x": 0, "y": 0})).yaw == 0


def test_gear_message() -> None:
    assert parse_request(json.dumps({"type": "gear", "gear": "sport"})).gear == "sport"


@pytest.mark.parametrize("kind", ["acquire", "release", "halt", "soft_stop"])
def test_simple_messages(kind: str) -> None:
    assert parse_request(json.dumps({"type": kind})).type == kind


@pytest.mark.parametrize(
    "raw, code",
    [
        ("not json", "bad_request"),
        ("[]", "bad_request"),
        (json.dumps({"type": "enable"}), "unknown_type"),
        (json.dumps({"type": "jog", "keys": ["left"]}), "unknown_type"),
        (json.dumps({"type": "publish", "topic": "/cmd_vel_safe"}), "unknown_type"),
        (json.dumps({"type": "auth"}), "bad_request"),
        (json.dumps({"type": "drive", "x": "1", "y": 0}), "bad_request"),
        (json.dumps({"type": "drive", "x": True, "y": 0}), "bad_request"),
        (json.dumps({"type": "drive", "y": 0}), "bad_request"),
        ('{"type": "drive", "x": NaN, "y": 0}', "bad_request"),
        ('{"type": "drive", "x": Infinity, "y": 0}', "bad_request"),
        (json.dumps({"type": "drive", "x": 0, "y": 0, "yaw": 0.5}), "bad_request"),
        (json.dumps({"type": "drive", "x": 0, "y": 0, "yaw": True}), "bad_request"),
        (json.dumps({"type": "gear"}), "bad_request"),
        ("x" * (MAX_MESSAGE_BYTES + 1), "bad_request"),
    ],
)
def test_rejected_messages(raw: str, code: str) -> None:
    with pytest.raises(ProtocolError) as err:
        parse_request(raw)
    assert err.value.code == code
