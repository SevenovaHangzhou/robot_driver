"""Handover needs resolved requests AND fresh actual controller evidence."""

from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.arm.idle import ArmIdleEvidence

NAMES = ("a", "b")


def ready():
    clock = [100.0]
    evidence = ArmIdleEvidence(0.2, 2.5, clock=lambda: clock[0])
    evidence.controllers([NS(name="jtc", type="JointTrajectoryController", state="active")], "jtc")
    evidence.set_tolerance(0.01)  # Simulated public parameter, never a production default.
    evidence.controller_state(NAMES, [0, 0], [0, 0], [0, 0])
    evidence.goal_status([])
    return evidence, clock


def test_startup_and_outcome_barrier():
    e, clock = ready()
    assert e.idle(NAMES, 99.0)
    assert not e.idle(NAMES, 100.0)
    e.submitted()
    assert not e.idle(NAMES, 99.0)
    e.resolved()
    clock[0] += 0.01
    e.controller_state(NAMES, [0, 0], [0, 0], [0, 0])
    assert e.idle(NAMES, 100.0)
    clock[0] += 0.3
    assert not e.idle(NAMES, 100.0)


@pytest.mark.parametrize(
    "bad",
    [
        "goal",
        "unknown",
        "nan",
        "desired",
        "moving",
        "missing",
        "duplicate",
        "tolerance",
        "controllers",
        "positions",
    ],
)
def test_incomplete_or_unsafe_evidence_cannot_release(bad):
    e, clock = ready()
    if bad == "goal":
        e.goal_status([2])
    elif bad == "unknown":
        e.submitted()
        e.resolved(unknown=True)
    elif bad == "nan":
        e.controller_state(NAMES, [0, 0], [float("nan"), 0], [0, 0])
    elif bad == "desired":
        e.controller_state(NAMES, [0, 0], [0, 0], [0.001, 0])
    elif bad == "moving":
        e.controller_state(NAMES, [0, 0], [0.02, 0], [0, 0])
    elif bad == "missing":
        e.controller_state(NAMES, [0, 0], [], [0, 0])
    elif bad == "duplicate":
        e.controller_state(["a", "a", "b"], [0] * 3, [0] * 3, [0] * 3)
    elif bad == "tolerance":
        e.set_tolerance(float("nan"))
    elif bad == "controllers":
        e.controllers([], "jtc")
    else:
        e.controller_state(NAMES, [float("inf"), 0], [0, 0], [0, 0])
    assert not e.idle(NAMES, 99.0)


def test_current_rolling_handover_needs_holding_and_position_window():
    e, clock = ready()
    e.controllers([NS(name="rolling", type="RollingTrajectoryController", state="active")], "jtc")
    e.set_stationary_policy(NAMES, [0.01, 0.02], 20)
    state = NS(epoch=3, state=1, last_sequence=2, last_result=0, stop_reason=0)
    for i in range(21):
        clock[0] += 0.008
        e.measured(NAMES, [0, 0], 1_000_000_000 + i * 8_000_000)
    e.rolling_state(state)
    assert not e.idle(NAMES, 99.0)
    state.state = 0
    assert e.idle(NAMES, 99.0)
    assert not e.idle(NAMES, clock[0] - 0.05)  # Old samples cannot prove a new stop.
    clock[0] += 0.008
    e.measured(NAMES, [0.02, 0], 1_168_000_000)
    assert not e.idle(NAMES, 99.0)


def test_invalid_and_discontinuous_feedback_restarts_stationary_window():
    e, clock = ready()
    e.controllers([NS(name="rolling", type="RollingTrajectoryController", state="active")], "jtc")
    e.set_stationary_policy(NAMES, [0.01, 0.02], 2)
    e.rolling_state(NS(state=0))
    for stamp in (10, 20, 30):
        e.measured(NAMES, [0, 0], stamp)
    assert e.idle(NAMES, 99.0)
    e.measured(NAMES, [0, 0], 30)  # Duplicate ROS stamp cannot renew stop evidence.
    assert not e.idle(NAMES, 99.0)
    e.measured(NAMES, [float("nan"), 0], 40)
    assert not e.samples


def inactive_ready():
    e, clock = ready()
    e.controllers(
        [
            NS(name="jtc", type="JointTrajectoryController", state="inactive"),
            NS(name="rolling", type="RollingTrajectoryController", state="inactive"),
        ],
        "jtc",
    )
    e.set_stationary_policy(NAMES, [0.01, 0.02], 20)
    for i in range(21):
        clock[0] += 0.008
        e.measured(NAMES, [0, 0], 1_000_000_000 + i * 8_000_000)
    return e, clock


def test_inactive_controllers_need_fresh_measured_stationary_window():
    e, clock = inactive_ready()
    assert e.idle(NAMES, 99.0)
    assert not e.idle(NAMES, clock[0] - 0.05)
    e.measured(NAMES, [0.02, 0], 1_168_000_000)
    assert not e.idle(NAMES, 99.0)


def test_jtc_only_scope_reconciles_with_fresh_declared_controller_configuration():
    e, clock = inactive_ready()
    controllers = [NS(name="jtc", type="JointTrajectoryController", state="inactive")]
    e.controllers(controllers, "jtc")
    assert not e.idle(NAMES, 99.0)
    e.set_motion_controllers("jtc", "")
    e.controllers(controllers, "jtc")
    assert e.idle(NAMES, 99.0)


@pytest.mark.parametrize("invalid", ["stale", "other_jtc", "rolling_expected"])
def test_jtc_only_scope_rejects_stale_or_inconsistent_configuration(invalid):
    e, clock = inactive_ready()
    e.set_motion_controllers(
        "other" if invalid == "other_jtc" else "jtc",
        "rolling" if invalid == "rolling_expected" else "",
    )
    controllers = [NS(name="jtc", type="JointTrajectoryController", state="inactive")]
    e.controllers(controllers, "jtc")
    if invalid == "stale":
        clock[0] += e.controller_age + 0.01
        e.controllers(controllers, "jtc")
    assert not e.idle(NAMES, 99.0)


def test_position_only_jtc_after_enable_uses_configured_position_window():
    e, clock = inactive_ready()
    # No action goal has ever been submitted: there is no initial action status message.
    e.controllers(
        [
            NS(name="jtc", type="JointTrajectoryController", state="active"),
            NS(name="rolling", type="RollingTrajectoryController", state="inactive"),
        ],
        "jtc",
    )
    e.controller_state(NAMES, [0, 0], [], [0, 0])
    assert e.idle(NAMES, 99.0)
    assert not e.idle(NAMES, clock[0] - 0.05)
    e.measured(NAMES, [0.02, 0], 1_168_000_000)
    assert not e.idle(NAMES, 99.0)


@pytest.mark.parametrize(
    "bad", ["policy", "goal", "desired", "partial_velocity", "nan_velocity", "unknown_status"]
)
def test_position_window_cannot_hide_unsafe_jtc_evidence(bad):
    e, clock = inactive_ready()
    e.controllers([NS(name="jtc", type="JointTrajectoryController", state="active")], "jtc")
    e.controller_state(NAMES, [0, 0], [], [0, 0])
    if bad == "policy":
        e.set_stationary_policy([], [], 20)
    elif bad == "goal":
        e.goal_status([2])
    elif bad == "desired":
        e.controller_state(NAMES, [0, 0], [], [0.001, 0])
    elif bad == "partial_velocity":
        e.controller_state(NAMES, [0, 0], [0], [0, 0])
    elif bad == "nan_velocity":
        e.controller_state(NAMES, [0, 0], [float("nan"), 0], [0, 0])
    else:
        e.status_seen = False
    assert not e.idle(NAMES, 99.0)


@pytest.mark.parametrize("bad", ["missing", "unconfigured", "active", "stale", "policy", "unknown"])
def test_inactive_startup_cannot_bypass_missing_or_uncertain_evidence(bad):
    e, clock = inactive_ready()
    if bad == "missing":
        e.controllers([NS(name="jtc", type="JointTrajectoryController", state="inactive")], "jtc")
    elif bad in ("unconfigured", "active"):
        e.controllers(
            [
                NS(name="jtc", type="JointTrajectoryController", state="inactive"),
                NS(name="rolling", type="RollingTrajectoryController", state=bad),
            ],
            "jtc",
        )
    elif bad == "stale":
        clock[0] += 3
    elif bad == "policy":
        e.set_stationary_policy([], [], 20)
    else:
        e.submitted()
        e.resolved(unknown=True)
    assert not e.idle(NAMES, 99.0)
