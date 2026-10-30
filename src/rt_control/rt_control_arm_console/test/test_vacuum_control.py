from pathlib import Path

import pytest

from rt_control_arm_console.console import ConsoleError
from rt_control_arm_console.vacuum_control import VacuumControl


class ServiceError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


@pytest.fixture
def io_control(monkeypatch):
    state = {
        "outputs": {"pump": False, "left": False, "right": False},
        "calls": [],
        "failure": None,
    }

    def status():
        state["calls"].append(("status",))
        if state["failure"]:
            raise ServiceError(*state["failure"])
        return {
            "configured": True,
            "valid": True,
            "busy": False,
            "outputs": state["outputs"].copy(),
            "sample_age_s": 0.01,
            "message": "",
            "owner_pid": 321,
        }

    def change(output, enabled, expected):
        state["calls"].append(("set", output, enabled, expected))
        if state["failure"]:
            raise ServiceError(*state["failure"])
        if state["outputs"][output] != expected:
            raise ServiceError("vacuum_changed", "changed")
        state["outputs"][output] = enabled
        return {"vacuum": status(), "message": "confirmed"}

    tool = {
        "service_status": status,
        "service_set": change,
        "ServiceError": ServiceError,
    }
    monkeypatch.setattr(
        "rt_control_arm_console.vacuum_control.runpy.run_path", lambda _: tool
    )
    return VacuumControl(Path("fake-vacuum")), state


def test_construction_never_contacts_device_and_status_only_queries_service(io_control):
    c, s = io_control
    assert s["calls"] == []
    assert c.status()["owner_pid"] == 321
    assert s["calls"] == [("status",)]


@pytest.mark.parametrize("output", ["pump", "left", "right"])
def test_manual_target_is_one_service_command(io_control, output):
    c, s = io_control
    assert c.set_output(output, True, False)["vacuum"]["outputs"][output]
    assert s["calls"][0] == ("set", output, True, False)
    assert all(v == (k == output) for k, v in s["outputs"].items())


@pytest.mark.parametrize(
    "args", [("bad", True, False), ("pump", 1, False), ("left", True, True)]
)
def test_invalid_request_never_contacts_service(io_control, args):
    c, s = io_control
    with pytest.raises(ValueError):
        c.set_output(*args)
    assert s["calls"] == []


def test_old_browser_state_is_rejected(io_control):
    c, s = io_control
    s["outputs"]["pump"] = True
    with pytest.raises(ConsoleError) as e:
        c.set_output("pump", True, False)
    assert e.value.code == "vacuum_changed"


def test_read_failure_shows_unknown_never_false(io_control):
    c, s = io_control
    s["outputs"]["pump"] = True
    assert c.status()["outputs"]["pump"]
    s["failure"] = ("vacuum_unavailable", "service stopped")
    v = c.status()
    assert not v["valid"] and all(x is None for x in v["outputs"].values())


def test_uncertain_write_is_not_retried(io_control):
    c, s = io_control
    s["failure"] = ("vacuum_unconfirmed", "response lost")
    with pytest.raises(ConsoleError) as e:
        c.set_output("pump", True, False)
    assert e.value.code == "vacuum_unconfirmed"
    assert s["calls"] == [("set", "pump", True, False)]
