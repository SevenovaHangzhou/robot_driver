"""Actual ROS message construction and asynchronous request accounting, no ROS node."""

from concurrent.futures import Future
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS

from rclpy.time import Time
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.arm.idle import ArmIdleEvidence
from integ_operator_web.arm.ros_bridge import RosBridge
from integ_operator_web.arm.trajectory import TrajectoryPoint


class Action:
    def __init__(self):
        self.future = Future()
        self.goal = None

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.goal = goal
        return self.future


def bridge():
    evidence = ArmIdleEvidence(0.2, 2.5)
    action = Action()
    obj = NS(
        _action=action,
        _lock=threading.Lock(),
        _goal=None,
        _readback=NS(evidence=evidence),
        get_clock=lambda: NS(now=lambda: Time(seconds=100)),
    )
    done = []
    RosBridge.send(
        obj,
        ["a"],
        [TrajectoryPoint(0.1, (0.2,), (0.0,), (0.0,))],
        lambda ok, text: done.append((ok, text)),
        time.monotonic(),
    )
    return evidence, action, done


def test_rejected_request_is_resolved_without_unknown_outcome():
    evidence, action, done = bridge()
    assert evidence.pending == 1 and not done
    assert action.goal.trajectory.joint_names == ["a"]
    assert list(action.goal.trajectory.points[0].positions) == [0.2]
    action.future.set_result(NS(accepted=False))
    assert evidence.pending == 0 and not evidence.unknown and not done[0][0]


@pytest.mark.parametrize("start_delay", [0.0, 0.05])
def test_message_preparation_preserves_move_lead_and_stop_timeline(monkeypatch, start_delay):
    now = [100.0]
    monkeypatch.setattr("integ_operator_web.arm.ros_bridge.time.monotonic", lambda: now[0])
    action = Action()
    obj = NS(
        _action=action, _lock=threading.Lock(), _goal=None,
        _readback=NS(evidence=ArmIdleEvidence(0.2, 2.5)),
        get_clock=lambda: NS(now=lambda: Time(seconds=now[0] + 100.0)),
    )

    def slow_points():
        now[0] += 0.08
        yield TrajectoryPoint(0.0, (0.2,), (0.0,), (0.0,))

    actual_start = RosBridge.send(
        obj, ["a"], slow_points(), lambda *_: None, 100.05, start_delay=start_delay,
    )
    expected = 100.13 if start_delay else 100.05
    assert actual_start == pytest.approx(expected)
    stamp = action.goal.trajectory.header.stamp
    assert stamp.sec + stamp.nanosec / 1e9 == pytest.approx(expected + 100.0, abs=1e-8)
    assert obj._readback.evidence.pending == 1


@pytest.mark.parametrize("stage", ["accept", "result"])
def test_unknown_acceptance_or_result_retains_handover_barrier(stage):
    evidence, action, done = bridge()
    if stage == "accept":
        action.future.set_exception(RuntimeError("connection lost"))
    else:
        result = Future()
        action.future.set_result(NS(accepted=True, get_result_async=lambda: result))
        assert evidence.pending == 1 and not done
        result.set_exception(RuntimeError("connection lost"))
    assert evidence.pending == 0 and evidence.unknown and not done[0][0]


def test_terminal_result_alone_only_resolves_request_not_controller_idle():
    evidence, action, done = bridge()
    result = Future()
    action.future.set_result(NS(accepted=True, get_result_async=lambda: result))
    assert evidence.pending == 1
    result.set_result(NS(status=4, result=NS(error_code=0, error_string="")))
    assert evidence.pending == 0 and not evidence.unknown and done[0][0]
    assert not evidence.idle(["a"], -float("inf"))  # Actual readback is still absent.


def test_only_parsed_scope_absence_can_bypass_arm_resource():
    import json

    updates = []
    obj = NS(
        console=NS(update=lambda **kw: updates.append(kw)),
        _lock=threading.Lock(),
        _action_topic="/jtc",
    )
    RosBridge._capabilities(obj, NS(data=json.dumps({"capabilities": []})))
    assert updates[-1]["scope_has_arms"] is False
    RosBridge._capabilities(obj, NS(data="invalid"))
    assert updates[-1]["scope_has_arms"] is None
    RosBridge._capabilities(
        obj,
        NS(
            data=json.dumps(
                {
                    "capabilities": [
                        {"name": "arm_joint_trajectory", "topic": "/jtc", "controller": "jtc"}
                    ],
                    "use_mock_hardware": True,
                    "control_scope": "bench_dual_arm_head",
                }
            )
        ),
    )
    assert updates[-1]["scope_has_arms"] is True
