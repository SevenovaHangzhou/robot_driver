import asyncio
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integ_operator_web.chassis_mode import ChassisModeView  # noqa: E402
from integ_operator_web.teleop import TeleopError  # noqa: E402


def feedback(**changes):
    fields = dict(
        mode=1,
        phase=0,
        ready=True,
        stationary=False,
        imu_valid=True,
        goal_active=False,
        fault_code=0,
    )
    fields.update(changes)
    return SimpleNamespace(**fields)


def view_status(view, **changes):
    args = dict(connected=True, enabled=True, busy=False, service_ready=True)
    args.update(changes)
    return view.status(**args)


def test_fixed_navigation_uses_fresh_configuration_without_claiming_readback():
    now = [10.0]
    view = ChassisModeView(lambda: now[0])
    assert not view_status(view)["velocity_allowed"]
    view.configure(False)
    status = view_status(view)
    assert status["mode"] == "navigation" and not status["confirmed"]
    assert status["velocity_allowed"] and not status["can_switch"]
    now[0] += 3
    assert not view_status(view)["velocity_allowed"]


def test_navigation_allows_controlled_stop_without_stationary_flag():
    # The handoff performs deceleration and verifies the physical dwell.
    view = ChassisModeView()
    view.configure(True)
    view.receive(feedback(), 0)
    assert view_status(view)["can_switch"]
    assert view_status(view)["velocity_allowed"]


def test_invalid_imu_blocks_entry_to_operation_but_retains_navigation():
    view = ChassisModeView()
    view.configure(True)
    view.receive(feedback(imu_valid=False), 0)
    assert not view_status(view)["can_switch"]
    assert view_status(view)["velocity_allowed"]


@pytest.mark.parametrize(
    "changes",
    [
        dict(mode=2, phase=5, stationary=True),
        dict(mode=0, phase=1),
        dict(fault_code=7),
        dict(ready=False),
        dict(goal_active=True),
    ],
)
def test_non_navigation_feedback_blocks_velocity(changes):
    view = ChassisModeView()
    view.configure(True)
    view.receive(feedback(**changes), 0)
    assert not view_status(view)["velocity_allowed"]


@pytest.mark.parametrize(
    "changes",
    [
        dict(fault_code=7),
        dict(ready=False),
        dict(goal_active=True),
        dict(mode=0),
        dict(phase=1),
        dict(mode=2, phase=5, stationary=False),
    ],
)
def test_unsafe_feedback_blocks_mode_changes(changes):
    view = ChassisModeView()
    view.configure(True)
    view.receive(feedback(**changes), 0)
    assert not view_status(view)["can_switch"]


@pytest.mark.parametrize(
    "changes",
    [dict(connected=False), dict(enabled=False), dict(busy=True), dict(service_ready=False)],
)
def test_unavailable_backend_blocks_switch(changes):
    view = ChassisModeView()
    view.configure(True)
    view.receive(feedback(), 0)
    assert not view_status(view, **changes)["can_switch"]


def test_old_future_and_missing_mode_feedback_fail_closed():
    now = [10.0]
    view = ChassisModeView(lambda: now[0])
    view.configure(True)
    assert not view_status(view)["velocity_allowed"]
    view.receive(feedback(), 2)
    now[0] += 1
    view.configure(True)
    assert not view_status(view)["can_switch"]
    assert not view_status(view)["velocity_allowed"]
    for age in (-1, 3, float("nan")):
        view.receive(feedback(), age)
        assert not view_status(view)["can_switch"]


def test_delayed_reply_cannot_replace_newer_fault_feedback():
    view = ChassisModeView(lambda: 10.0)
    view.configure(True)
    view.receive(feedback(fault_code=7), 0)
    view.receive(feedback(), 0.1)
    assert not view_status(view)["velocity_allowed"]
    assert view_status(view)["fault_code"] == 7


def connection(monkeypatch, result=None, error=None):
    # No Node or executor is started; all service/command side effects are fakes.
    from integ_operator_web import node as module

    backend = module.ChassisConnection.__new__(module.ChassisConnection)
    backend.lock = threading.RLock()
    backend.busy = False
    backend.mode_busy = False
    backend.last_error = ""
    backend.node = SimpleNamespace(arbiter=Mock(), publish_zero=Mock())
    backend.mode_client = Mock()
    backend.chassis_mode = Mock()
    backend.received_chassis_mode = Mock()
    backend.status = lambda: {
        "chassis_mode": {"can_switch": True, "mode": "navigation", "message": "ready"}
    }

    async def service(client, request, timeout):
        assert backend.mode_busy
        assert request.mode == 2 and request.confirm is True
        if error:
            raise error
        return result

    monkeypatch.setattr(module, "call_service", service)
    return backend


@pytest.mark.parametrize("code, mode, ok", [(0, 2, True), (0, 1, False), (6, 2, False)])
def test_backend_requires_completed_response_and_matching_mode(monkeypatch, code, mode, ok):
    backend = connection(
        monkeypatch, SimpleNamespace(code=code, state=SimpleNamespace(mode=mode, fault_code=0))
    )
    result = asyncio.run(backend.set_chassis_mode("operation"))
    assert result["ok"] is ok
    assert not backend.mode_busy
    assert backend.received_chassis_mode.call_count == int(ok)
    assert backend.node.arbiter.reset_motion.call_count == 2
    backend.node.publish_zero.assert_called_once()


def test_backend_timeout_invalidates_confirmation_and_clears_inputs(monkeypatch):
    backend = connection(monkeypatch, error=asyncio.TimeoutError())
    result = asyncio.run(backend.set_chassis_mode("operation"))
    assert not result["ok"] and "未确认" in result["message"]
    backend.chassis_mode.invalidate.assert_called_once()
    assert not backend.mode_busy
    assert backend.node.arbiter.reset_motion.call_count == 2


def test_backend_unready_does_not_call_or_publish(monkeypatch):
    backend = connection(monkeypatch)
    backend.status = lambda: {"chassis_mode": {"can_switch": False, "message": "未启用"}}
    with pytest.raises(TeleopError):
        asyncio.run(backend.set_chassis_mode("operation"))
    backend.node.publish_zero.assert_not_called()


def test_mode_completion_cannot_clear_an_overlapping_disable_busy_flag(monkeypatch):
    backend = connection(
        monkeypatch, SimpleNamespace(code=0, state=SimpleNamespace(mode=2, fault_code=0))
    )
    backend.busy = True  # Disable was requested while the handoff was pending.
    assert asyncio.run(backend.set_chassis_mode("operation"))["ok"]
    assert backend.busy and not backend.mode_busy


def test_backend_does_not_report_success_for_faulted_matching_mode(monkeypatch):
    backend = connection(
        monkeypatch, SimpleNamespace(code=0, state=SimpleNamespace(mode=2, fault_code=1))
    )
    assert not asyncio.run(backend.set_chassis_mode("operation"))["ok"]
    backend.received_chassis_mode.assert_not_called()
