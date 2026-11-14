"""Resolved requests and controller standstill are both required for handover."""

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
    evidence.controller_state(NAMES, [0, 0])
    evidence.goal_status([])
    evidence.confirm_standstill(True, 99.9)
    return evidence, clock


def test_stop_confirmation_must_start_after_outcome_and_remain_fresh():
    e, clock = ready()
    assert e.idle(NAMES, 99.0)
    assert not e.idle(NAMES, 100.0)
    clock[0] += 0.01
    e.controller_state(NAMES, [0, 0])
    e.confirm_standstill(True, 100.0)  # Query already in flight at the barrier is insufficient.
    assert not e.idle(NAMES, 100.0)
    e.confirm_standstill(True, 100.001)
    assert e.idle(NAMES, 100.0)
    clock[0] += 0.21
    assert not e.idle(NAMES, 100.0)


@pytest.mark.parametrize(
    "bad", ["pending", "unknown", "goal", "desired", "joints", "controllers", "stop_failed"]
)
def test_no_handover_without_terminal_and_standstill_evidence(bad):
    e, _ = ready()
    if bad in ("pending", "unknown"):
        e.submitted()
        if bad == "unknown":
            e.resolved(unknown=True)
    elif bad == "goal":
        e.goal_status([2])
    elif bad == "desired":
        e.controller_state(NAMES, [0.001, 0])
    elif bad == "joints":
        e.controller_state(["a", "a"], [0, 0])
    elif bad == "controllers":
        e.controllers([], "jtc")
    else:
        e.confirm_standstill(False, 99.9)
    assert not e.idle(NAMES, 99.0)


def test_rolling_requires_holding_and_confirmed_actual_standstill():
    e, _ = ready()
    e.controllers([NS(name="rolling", type="RollingTrajectoryController", state="active")], "jtc")
    state = NS(state=1, HOLDING=0)
    e.rolling_state(state)
    assert not e.idle(NAMES, 99.0)
    state.state = state.HOLDING
    e.confirm_standstill(False, 99.9)
    assert not e.idle(NAMES, 99.0)
    e.confirm_standstill(True, 99.9)
    assert e.idle(NAMES, 99.0)


def test_startup_with_inactive_controllers_still_requires_actual_standstill():
    e, _ = ready()
    e.controllers(
        [
            NS(name="jtc", type="JointTrajectoryController", state="inactive"),
            NS(name="rolling", type="RollingTrajectoryController", state="inactive"),
        ],
        "jtc",
    )
    e.confirm_standstill(False, 99.9)
    assert not e.idle(NAMES, 99.0)
    e.confirm_standstill(True, 99.9)
    assert e.idle(NAMES, 99.0)


def test_late_query_response_cannot_confirm_a_new_stop():
    from concurrent.futures import Future
    from integ_operator_web.arm.readback import ArmReadback

    evidence, clock = ready()
    futures, removed = [], []

    def request(_):
        future = Future()
        futures.append(future)
        return future

    readback = ArmReadback.__new__(ArmReadback)
    readback.evidence, readback.pending = evidence, None
    readback.motion_joints, readback.motion_joints_at = NAMES, clock[0]
    readback.console = NS(joint_names=NAMES)
    readback.standstill_client = NS(
        service_is_ready=lambda: True, call_async=request, remove_pending_request=removed.append
    )
    assert not readback.probe(100.0)
    clock[0] += 3
    readback.motion_joints_at = clock[0]
    assert not readback.probe(100.0)
    assert removed == [futures[0]]
    futures[0].set_result(NS(success=True))
    assert not evidence.idle(NAMES, 100.0)
    evidence.controllers([NS(name="jtc", type="JointTrajectoryController", state="active")], "jtc")
    evidence.controller_state(NAMES, [0, 0])
    futures[1].set_result(NS(success=True))
    assert evidence.idle(NAMES, 100.0)

    # Standstill of another motion group must not unlock these controlled joints.
    readback.motion_joints = ("another",)
    assert not readback.probe(100.0)
    assert len(futures) == 2
