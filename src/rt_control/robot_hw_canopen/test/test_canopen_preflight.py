"""CAN owner preflight is read-only and rejects a down or bus-off interface."""

import json

import pytest

from robot_hw_canopen.rt_control_renderer import preflight

TOPOLOGY = {"node_ids": [11], "can_interface": "can0", "preflight_required": True}


def link(flags, state):
    return json.dumps([{"flags": flags, "linkinfo": {"info_data": {"state": state}}}])


def test_up_and_error_active_interface_is_admitted():
    calls = []
    preflight(TOPOLOGY, run=lambda argv, **_: calls.append(argv) or link(["UP"], "ERROR-ACTIVE"))
    assert calls == [["ip", "-details", "-json", "link", "show", "can0"]]


@pytest.mark.parametrize(
    "flags,state,message",
    [(["NO-CARRIER"], "ERROR-ACTIVE", "not up"), (["UP"], "BUS-OFF", "not error-active")],
)
def test_down_or_bus_off_interface_is_rejected(flags, state, message):
    with pytest.raises(RuntimeError, match=message):
        preflight(TOPOLOGY, run=lambda argv, **_: link(flags, state))


def test_mock_topology_runs_no_command():
    preflight({**TOPOLOGY, "preflight_required": False}, run=lambda *_: pytest.fail("ran"))
