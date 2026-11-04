"""Head page logic: readiness checks, limits, lease and hold-on-disconnect. No ROS."""

import math
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.head.console import HeadConsole, HeadError  # noqa: E402

CONFIG = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/head.yaml").read_text())
JOINTS = ("H1_joint", "H2_joint")


class Clock:
    now = 10.0

    def __call__(self):
        return self.now


def ready_console():
    clock, sent = Clock(), []
    console = HeadConsole(CONFIG, sent.append, clock)
    console.set_model(JOINTS, {"H1_joint": (-math.pi, math.pi), "H2_joint": (-1.0, 1.0)})
    console.update(
        capability={
            "topic": "/rt/head/move",
            "controller": "head_position_controller",
        },
        positions={"H1_joint": 0.1, "H2_joint": -0.2},
        feedback_at=clock.now,
        enable_state="ENABLED",
        motion_allowed=True,
        device_fault=False,
        enable_at=clock.now,
    )
    console.acquire("c1")
    return console, sent, clock


def test_move_publishes_one_absolute_target_in_controller_joint_order():
    console, sent, _ = ready_console()
    console.move("c1", {"H2_joint": 0.3, "H1_joint": -0.5})
    assert sent == [[-0.5, 0.3]] and console.target == {"H1_joint": -0.5, "H2_joint": 0.3}


@pytest.mark.parametrize(
    "change, code",
    [
        ({"capability": None}, "capability_missing"),
        ({"enable_state": "IDLE"}, "not_enabled"),
        ({"enable_at": -100.0}, "not_enabled"),
        ({"device_fault": True}, "device_fault"),
        ({"motion_allowed": False}, "motion_not_allowed"),
        ({"feedback_at": 9.0}, "feedback_stale"),
    ],
)
def test_move_requires_every_readiness_condition(change, code):
    console, sent, _ = ready_console()
    console.update(**change)
    with pytest.raises(HeadError) as error:
        console.move("c1", {"H1_joint": 0.0, "H2_joint": 0.0})
    assert error.value.code == code and not sent


def test_targets_stay_two_degrees_inside_the_model_limits_and_must_be_complete():
    console, sent, _ = ready_console()
    inside = 1.0 - math.radians(2.0)
    console.move("c1", {"H1_joint": 0.0, "H2_joint": inside})
    for target, code in (
        ({"H1_joint": 0.0, "H2_joint": inside + 1e-3}, "target_out_of_range"),
        ({"H1_joint": 0.0, "H2_joint": float("nan")}, "target_out_of_range"),
        ({"H1_joint": 0.0}, "bad_target"),
        ({"H1_joint": 0.0, "H2_joint": True}, "bad_target"),
    ):
        with pytest.raises(HeadError) as error:
            console.move("c1", target)
        assert error.value.code == code
    assert len(sent) == 1


def test_only_the_lease_holder_moves_and_disconnect_holds_in_place():
    console, sent, _ = ready_console()
    with pytest.raises(HeadError) as error:
        console.move("c2", {"H1_joint": 0.0, "H2_joint": 0.0})
    assert error.value.code == "lease_required"
    with pytest.raises(HeadError):
        console.acquire("c2")
    console.disconnect("c2")  # not the holder: nothing happens
    assert sent == [] and console.lease_holder == "c1"
    console.disconnect("c1")
    assert sent == [[0.1, -0.2]] and console.lease_holder is None


def test_hold_needs_fresh_feedback_and_missing_model_limits_are_reported():
    console, sent, clock = ready_console()
    clock.now += 1.0
    with pytest.raises(HeadError) as error:
        console.hold()
    assert error.value.code == "feedback_stale" and not sent
    fresh = HeadConsole(CONFIG, sent.append, clock)
    fresh.set_model(JOINTS, {"H1_joint": (-1.0, 1.0)})
    assert "H2_joint" in fresh.model_error and fresh.joints == ()


def test_status_reports_lease_limits_and_enable_state():
    console, _, clock = ready_console()
    status = console.status("c1")
    assert status["lease"] == "you" and console.status("c2")["lease"] == "other"
    assert status["joints"][1]["upper"] == pytest.approx(1.0 - math.radians(2.0))
    assert status["enable"] == {"state": "ENABLED", "motion_allowed": True, "device_fault": False}
    clock.now += 10.0
    assert console.status("c1")["enable"]["state"] == "UNKNOWN"
