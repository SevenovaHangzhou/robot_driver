"""Head CAN preflight: interface admission, then the damiao_preflight tool's per-motor replies."""

import json

import pytest

from hw_can_damiao import rt_control_renderer as head

TOPOLOGY = {
    "preflight_required": True,
    "can_interface": "can2",
    "motors": [{"can_id": 1, "master_id": 17}, {"can_id": 2, "master_id": 18}],
    "timeout_ms": 100,
}


def bus(replies, state="ERROR-ACTIVE"):
    calls = []

    def run(argv, **kwargs):
        calls.append(argv)
        if argv[0] == "ip":
            return json.dumps([{"flags": ["UP"], "linkinfo": {"info_data": {"state": state}}}])
        return "".join(json.dumps(reply) + "\n" for reply in replies)

    return run, calls


def test_answering_motors_without_err_pass_and_the_tool_gets_every_motor():
    run, calls = bus(
        [{"can_id": 1, "answered": True, "status": 0}, {"can_id": 2, "answered": True, "status": 0}]
    )
    assert head.preflight(TOPOLOGY, run=run) == []
    assert calls[1][1:] == ["can2", "100", "1:17", "2:18"]


@pytest.mark.parametrize(
    ("reply", "problem", "code"),
    [
        ({"can_id": 2, "answered": False, "status": 0}, "offline", None),
        ({"can_id": 2, "answered": True, "status": 9}, "fault_code", 9),
    ],
)
def test_silent_or_faulted_motor_blocks_at_its_can_id(reply, problem, code):
    run, _ = bus([{"can_id": 1, "answered": True, "status": 0}, reply])
    [finding] = head.preflight(TOPOLOGY, run=run)
    assert (finding["address"], finding["problem"], finding["blocking"]) == (
        {"can_id": 2},
        problem,
        True,
    )
    assert finding.get("fault_code") == code


def test_interface_that_is_not_error_active_blocks_before_any_frame():
    run, calls = bus([], state="BUS-OFF")
    [finding] = head.preflight(TOPOLOGY, run=run)
    assert finding["address"] == {"can_interface": "can2"} and finding["problem"] == "offline"
    assert len(calls) == 1


def test_mock_hardware_skips_the_preflight():
    assert head.preflight({**TOPOLOGY, "preflight_required": False}, run=None) == []
