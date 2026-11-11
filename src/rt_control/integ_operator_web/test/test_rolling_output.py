"""Command admission and stop fencing against the actual shared authority."""

import json
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.arm.jog import JogAccess
from integ_operator_web.arm.rolling_output import RollingOutput
from integ_operator_web.arm.console_types import ConsoleError
from test_arm_console import setup  # noqa: F401
from test_arm_page import build, login, run
from integ_operator_web.arm.jog import JogPage


class Port:
    def __init__(self):
        self.phase, self.sequence, self.sent = 0, 0, []

    def status(self):
        return {"phase": self.phase, "ros_time_ns": 10**9, "last_sent_sequence": self.sequence}

    def reset(self):
        return self.phase in (0, 3, 4, 5)

    def send(self, window):
        self.sequence += 1
        self.phase = 1
        self.sent.append(window)
        return 0


def ready(setup):
    console, _, _ = setup
    console.release("s1")
    port = Port()
    parameters = {
        "joints": list(console.joint_names),
        "buffer_capacity": 64,
        "joint_acceleration_limits": [1.0] * len(console.joint_names),
        "replace_lead_ms": 4,
        "max_horizon_ms": 600,
    }
    evidence = NS(
        rolling_active=True,
        jtc_active=False,
        lock=threading.Lock(),
        clock=time.monotonic,
        rolling=NS(epoch=2, state=0, last_sequence=0, last_result=0, stop_reason=0),
        rolling_at=time.monotonic(),
    )
    bridge = NS(
        _readback=NS(evidence=evidence),
        rolling_parameters=parameters,
        rolling_parameters_at=console._clock(),
        rolling_hold=NS(query=lambda names: dict.fromkeys(names, 0.0)),
    )
    output = console.jog_output = RollingOutput(console, bridge, port)
    access = JogAccess(console)
    access.acquire("jog")
    identity = {
        "model_sha256": console.model.source_sha256,
        "joint_names": list(console.joint_names),
    }
    return console, port, output, access, identity


def window(count):
    return {
        "replace_from": {"sec": 1, "nanosec": 100_000_000},
        "final": True,
        "points": [
            {
                "time_from_start": {"sec": 0, "nanosec": t},
                "positions": [0.0] * count,
                "velocities": [0.0] * count,
            }
            for t in (0, 200_000_000)
        ],
    }


def test_browser_can_parse_status_before_first_rolling_feedback_and_after_it_arrives(setup):
    console, port, output, _, _ = ready(setup)
    evidence = output.bridge._readback.evidence
    feedback = evidence.rolling
    evidence.rolling, evidence.rolling_at = None, -math.inf
    evidence.rolling_active = False
    app, arm = build(console)
    JogPage(arm).add_routes(app)

    def reject_constant(value):
        raise ValueError(f"Invalid JSON constant: {value}")

    async def status(ws):
        for _ in range(100):
            payload = json.loads(await ws.receive_str(timeout=2), parse_constant=reject_constant)
            if payload.get("type") == "status":
                return payload
        raise AssertionError("status unavailable")

    async def scenario(client):
        await login(client)
        ws = await client.ws_connect("/ws/jog")
        initial = await status(ws)
        assert initial["rolling"]["state_age_s"] is None
        assert initial["rolling"]["controller"] is None
        evidence.rolling, evidence.rolling_at = feedback, time.monotonic()
        current = await status(ws)
        assert math.isfinite(current["rolling"]["state_age_s"])
        assert current["rolling"]["controller"]["epoch"] == feedback.epoch
        assert not port.sent

    run(app, scenario)


def test_jog_reservation_persists_before_first_window_and_until_actual_stop(setup):
    console, port, output, access, identity = ready(setup)
    output.begin("jog", identity)
    console._sync_authority()
    assert console.authority.poll()["phase"] == "moving"
    with pytest.raises(ConsoleError):
        console.acquire("arm")
    output.publish("jog", {"window": window(len(console.joint_names))})
    access.disconnect("jog")
    assert console.authority.poll()["phase"] == "stopping"
    with pytest.raises(ConsoleError):
        output.publish("jog", {"window": window(len(console.joint_names))})
    assert len(port.sent) == 1
    console.idle_probe = lambda _: False
    port.phase = 4
    console.poll_authority()
    assert console.authority.poll()["phase"] == "stopping"
    console.idle_probe = lambda _: True
    console.acquire("arm")
    assert console.authority.poll()["owner_entry"] == "arm"


def test_primary_rejects_wrong_tab_identity_and_nonrest_final(setup):
    console, port, output, _, identity = ready(setup)
    with pytest.raises(ConsoleError):
        output.begin("other", identity)
    with pytest.raises(ConsoleError):
        output.begin("jog", {**identity, "model_sha256": "wrong"})
    assert not port.sent
    output.begin("jog", identity)
    target = window(len(console.joint_names))
    target["points"][-1]["velocities"][0] = 0.001
    with pytest.raises(ValueError):
        output.publish("jog", {"window": target})
    assert not port.sent


def test_unknown_first_ack_keeps_the_resource_fenced(setup):
    console, _, output, access, identity = ready(setup)
    output.begin("jog", identity)
    output.publish("jog", {"window": window(len(console.joint_names))})
    output.sender.phase = 6
    access.disconnect("jog")
    console.poll_authority()
    with pytest.raises(ConsoleError):
        console.acquire("arm")
    assert console._authority_unknown and console.authority.owner().entry == "jog"


def test_begin_returns_controller_command_and_rejects_measured_position_as_first_knot(setup):
    console, port, output, _, identity = ready(setup)
    held = dict.fromkeys(console.joint_names, 0.0002)
    output.bridge.rolling_hold.query = lambda names: held
    result = output.begin("jog", identity)
    assert result["hold_reference"]["positions"] == held
    target = window(len(console.joint_names))
    with pytest.raises(ConsoleError, match="首点"):
        output.publish("jog", {"window": target})
    assert not port.sent
    for point in target["points"]:
        point["positions"] = list(held.values())
    output.publish("jog", {"window": target})
    assert port.sent == [target]


@pytest.mark.parametrize("change", ["epoch", "last_sequence", "stop_reason", "state", "stale"])
def test_changed_hold_is_rejected_before_the_first_publish(setup, change):
    console, port, output, _, identity = ready(setup)
    output.begin("jog", identity)
    evidence = output.bridge._readback.evidence
    if change == "stale":
        evidence.rolling_at -= 1
    else:
        setattr(evidence.rolling, change, getattr(evidence.rolling, change) + 1)
    with pytest.raises(ConsoleError, match="保持状态"):
        output.publish("jog", {"window": window(len(console.joint_names))})
    assert not port.sent


def test_first_segment_acceleration_is_rejected_before_motor_transport(setup):
    console, port, output, _, identity = ready(setup)
    output.begin("jog", identity)
    target = window(len(console.joint_names))
    target["points"][1]["positions"][0] = 0.1  # 15 rad/s² on the 200 ms Hermite segment
    with pytest.raises(ConsoleError, match="首段加速度"):
        output.publish("jog", {"window": target})
    assert not port.sent


def test_release_during_hold_read_cancels_begin_without_a_window(setup):
    console, port, output, access, identity = ready(setup)

    def query(names):
        access.disconnect("jog")
        return dict.fromkeys(names, 0.0)

    output.bridge.rolling_hold.query = query
    with pytest.raises(ConsoleError):
        output.begin("jog", identity)
    assert not console._jog_busy and not port.sent
