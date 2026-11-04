import asyncio
from concurrent.futures import Future
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.chassis_operation import ChassisOperation, validate_motion
from integ_operator_web.protocol import parse_request, ProtocolError
from integ_operator_web.teleop import TeleopError

LIMITS = dict(
    max_translation_velocity=0.05,
    max_translation_acceleration=0.1,
    max_yaw_velocity=0.1,
    max_yaw_acceleration=0.2,
    max_wheel_velocity=20.0,
    max_wheel_acceleration=20.0,
    max_duration=60.0,
)


def motion():
    return dict(dx=0.01, dy=0.0, dyaw_deg=0.0, **LIMITS)


def done(value):
    future = Future()
    future.set_result(value)
    return future


def snapshot(**changes):
    return NS(
        phase=5,
        progress=1.0,
        actual_dx=0.01,
        actual_dy=0.0,
        actual_dyaw=0.0,
        estimate_valid=True,
        fault_code=0,
        **changes,
    )


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancels = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        return done(NS(goals_canceling=[1]))

    def finish(self, status=4, code=0):
        self.result.set_result(NS(status=status, result=NS(code=code, state=snapshot())))


class Client:
    def __init__(self):
        self.pending = Future()
        self.goals = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback):
        self.goals.append(goal)
        self.feedback = feedback_callback
        return self.pending


def setup():
    client = Client()
    operation = ChassisOperation(
        client, lambda value: value, lambda: not operation.running, threading.RLock()
    )
    operation.configure(LIMITS)
    return operation, client


@pytest.mark.parametrize(
    "change",
    [
        dict(dx=0.7, dy=0.01),
        dict(dyaw_deg=15.01),
        dict(max_duration=0),
        dict(max_translation_velocity=0.051),
        dict(dx=float("nan")),
        dict(max_wheel_velocity=True),
    ],
)
def test_motion_rejects_invalid_or_excess_limits(change):
    with pytest.raises(TeleopError):
        validate_motion({**motion(), **change}, LIMITS)


def test_degree_conversion_and_roundoff_are_bounded_by_actual_limits():
    value = validate_motion(
        {**motion(), "dyaw_deg": -15, "max_yaw_velocity": math.nextafter(0.1, 1)}, LIMITS
    )
    assert value["dyaw"] == pytest.approx(-math.pi / 12)
    assert value["max_yaw_velocity"] == 0.1


def test_protocol_requires_explicit_confirmation_and_whitelisted_complete_fields():
    value = {"type": "operation_move", "confirm": True, **motion()}
    assert parse_request(json.dumps(value)).motion == motion()
    for change in ({"confirm": False}, {"dx": "1"}, {"extra": "unsafe"}):
        with pytest.raises(ProtocolError):
            parse_request(json.dumps({**value, **change}))


def test_mode_imu_fresh_limits_and_server_readiness_gate_execution():
    op, _ = setup()
    mode = dict(
        confirmed=True,
        mode="operation",
        phase=5,
        ready=True,
        stationary=True,
        imu_valid=True,
        fault_code=0,
        goal_active=False,
    )
    assert op.status(mode, True, True, False)["can_execute"]
    for change in (
        {"mode": "navigation"},
        {"stationary": False},
        {"imu_valid": False},
        {"fault_code": 1},
        {"goal_active": True},
        {"confirmed": False},
    ):
        assert not op.status({**mode, **change}, True, True, False)["can_execute"]
    op.limits_at -= 3
    assert not op.status(mode, True, True, False)["can_execute"]


def test_only_confirmed_success_completes_and_no_second_goal_is_submitted():
    async def run():
        op, client = setup()
        handle = Handle()
        task = asyncio.create_task(op.execute(motion()))
        await asyncio.sleep(0)
        with pytest.raises(TeleopError):
            await op.execute(motion())
        client.pending.set_result(handle)
        await asyncio.sleep(0.01)
        assert op.running and len(client.goals) == 1
        handle.finish()
        assert (await task)["ok"] and not op.running and not op.unknown

    asyncio.run(run())


def test_cancel_before_acceptance_is_sent_after_acceptance_and_waits_for_result():
    async def run():
        op, client = setup()
        handle = Handle()
        task = asyncio.create_task(op.execute(motion()))
        await asyncio.sleep(0)
        await op.cancel()
        client.pending.set_result(handle)
        await asyncio.sleep(0.01)
        assert handle.cancels == 1 and op.running and not task.done()
        handle.finish(status=5, code=1)
        result = await task
        assert not result["ok"] and "取消" in result["message"] and not op.unknown

    asyncio.run(run())


def test_aborted_action_never_becomes_success():
    async def run():
        op, client = setup()
        handle = Handle()
        client.pending.set_result(handle)
        handle.finish(status=6, code=2)
        assert not (await op.execute(motion()))["ok"]

    asyncio.run(run())


def test_canceled_web_task_cancels_late_acceptance_and_marks_result_unknown():
    async def run():
        op, client = setup()
        handle = Handle()
        task = asyncio.create_task(op.execute(motion()))
        await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        client.pending.set_result(handle)
        assert handle.cancels == 1 and op.unknown and not op.running

    asyncio.run(run())


def test_bad_feedback_is_json_safe_and_does_not_fabricate_actual_displacement():
    op, _ = setup()
    state = snapshot()
    state.actual_dx = float("nan")
    op.receive_state(state)
    assert not op.feedback["estimate_valid"] and op.feedback["actual_dx"] is None
    json.dumps(op.feedback, allow_nan=False)
