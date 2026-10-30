"""HTTP authorization and IO routing tests; all hardware is fake."""

import json

import pytest

from test_console import setup  # noqa: F401
from test_server import server, request, login  # noqa: F401
from rt_control_arm_console.console import ConsoleError


class FakeVacuum:
    def __init__(self):
        self.reads = 0
        self.writes = []
        self.failure = False

    def status(self):
        self.reads += 1
        return {
            "configured": True,
            "valid": True,
            "busy": False,
            "outputs": {"pump": False, "left": False, "right": False},
        }

    def set_output(self, output, enabled, expected):
        if (
            output not in ("pump", "left", "right")
            or type(enabled) is not bool
            or type(expected) is not bool
        ):
            raise ValueError("invalid output request")
        self.writes.append((output, enabled, expected))
        if self.failure:
            raise ConsoleError("vacuum_unconfirmed", "设备响应丢失")
        snapshot = self.status()
        snapshot["outputs"][output] = enabled
        return {"vacuum": snapshot, "message": "confirmed"}


def test_reads_require_login_and_never_write(server):
    instance, _, driver = server
    instance.vacuum = FakeVacuum()
    response, _ = request(instance, "GET", "/api/vacuum")
    assert response.status == 401 and instance.vacuum.reads == 0
    cookie = login(instance)
    response, body = request(instance, "GET", "/api/vacuum", cookie=cookie)
    assert response.status == 200 and json.loads(body)["valid"]
    assert not instance.vacuum.writes and not driver.sent


def test_write_requires_login_same_origin_and_control_lease(server):
    instance, console, driver = server
    instance.vacuum = FakeVacuum()
    target = {"output": "pump", "enabled": True, "expected": False}
    response, _ = request(instance, "POST", "/api/vacuum", target)
    assert response.status == 401
    cookie = login(instance)
    response, _ = request(instance, "POST", "/api/vacuum", target, cookie, "http://evil.test")
    assert response.status == 403
    response, body = request(instance, "POST", "/api/vacuum", target, cookie)
    assert response.status == 409 and json.loads(body)["error"] == "lease_required"
    assert not instance.vacuum.writes
    console.release("s1")
    request(instance, "POST", "/api/acquire", {}, cookie)
    response, body = request(instance, "POST", "/api/vacuum", target, cookie)
    assert response.status == 200 and json.loads(body)["vacuum"]["outputs"]["pump"]
    assert instance.vacuum.writes == [("pump", True, False)]
    assert not driver.sent and not driver.cancelled


def test_absent_tool_is_reported_and_commands_are_rejected(server):
    instance, _, _ = server
    cookie = login(instance)
    response, body = request(instance, "GET", "/api/vacuum", cookie=cookie)
    assert response.status == 200 and not json.loads(body)["configured"]
    response, body = request(instance, "POST", "/api/vacuum", {}, cookie)
    assert response.status == 409 and json.loads(body)["error"] == "vacuum_unavailable"


def test_moving_arm_rejects_io_command(server):
    instance, console, _ = server
    instance.vacuum = FakeVacuum()
    console.release("s1")
    cookie = login(instance)
    request(instance, "POST", "/api/acquire", {}, cookie)
    request(
        instance,
        "POST",
        "/api/execute",
        {"target": {"left_joint1": 0.4, "left_joint2": 0.2}, "gear": "leisure"},
        cookie,
    )
    response, body = request(
        instance,
        "POST",
        "/api/vacuum",
        {"output": "left", "enabled": True, "expected": False},
        cookie,
    )
    assert response.status == 409 and json.loads(body)["error"] == "busy"
    assert not instance.vacuum.writes


def test_device_failure_is_returned_as_unconfirmed(server):
    instance, console, _ = server
    instance.vacuum = FakeVacuum()
    instance.vacuum.failure = True
    console.release("s1")
    cookie = login(instance)
    request(instance, "POST", "/api/acquire", {}, cookie)
    response, body = request(
        instance,
        "POST",
        "/api/vacuum",
        {"output": "right", "enabled": True, "expected": False},
        cookie,
    )
    assert response.status == 409 and json.loads(body)["error"] == "vacuum_unconfirmed"
    assert len(instance.vacuum.writes) == 1


@pytest.mark.parametrize("body", [{}, {"output": "pump", "enabled": "yes", "expected": False}])
def test_invalid_requests_are_bad_requests_without_writes(server, body):
    instance, console, _ = server
    instance.vacuum = FakeVacuum()
    console.release("s1")
    cookie = login(instance)
    request(instance, "POST", "/api/acquire", {}, cookie)
    response, _ = request(instance, "POST", "/api/vacuum", body, cookie)
    assert response.status == 400 and not instance.vacuum.writes
