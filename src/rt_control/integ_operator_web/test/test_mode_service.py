"""Public mode messages, response classification and post-switch observation."""

from concurrent.futures import Future
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
from robot_rt_control_interfaces.msg import JointControlMode
from robot_rt_control_interfaces.srv import SetJointControlMode
from controller_manager_msgs.srv import ListControllers

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.arm.mode_service import ModeService
from integ_operator_web.arm.modes import ModeError
from test_arm_modes import arm_rig
from test_arm_console import setup  # noqa: F401


class ServiceClient:
    def __init__(self, response):
        self.response, self.requests, self.removed = response, [], []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        future = Future()
        if isinstance(self.response, Exception):
            future.set_exception(self.response)
        elif self.response is not None:
            future.set_result(self.response)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


def service(setup, *, accepted=True, mode=JointControlMode.FJT_READY):
    c, _, _, io = arm_rig(setup)
    response = SetJointControlMode.Response(accepted=accepted, mode=JointControlMode(value=mode))
    mode_client = ServiceClient(response)
    list_client = ServiceClient(ListControllers.Response())
    b = NS(
        _group=None,
        console=c,
        _controllers=list_client,
        _readback=NS(evidence=io.e),
        create_client=lambda *a, **kw: mode_client,
        _received_controllers=lambda _: io.switch("jtc"),
    )
    return ModeService(b), mode_client, list_client, c, io


def test_public_request_and_actual_mode_are_confirmed_before_success(setup):
    s, client, listed, _, io = service(setup)
    s.switch("jtc")
    assert client.requests[0].target_mode.value == JointControlMode.FJT_READY
    assert len(listed.requests) == 1 and io.e.mode_matches("jtc")


@pytest.mark.parametrize(
    "mode", [JointControlMode.ROLLING_READY, JointControlMode.RESTART_REQUIRED]
)
def test_wrong_success_response_is_unknown_and_never_accepted(setup, mode):
    s, _, listed, _, _ = service(setup, mode=mode)
    with pytest.raises(ModeError) as e:
        s.switch("jtc")
    assert e.value.uncertain and not listed.requests


@pytest.mark.parametrize(
    "mode,unknown",
    [(JointControlMode.ROLLING_READY, False), (JointControlMode.RESTART_REQUIRED, True)],
)
def test_rejection_preserves_restart_requirement(setup, mode, unknown):
    s, _, _, _, _ = service(setup, accepted=False, mode=mode)
    with pytest.raises(ModeError) as e:
        s.switch("jtc")
    assert e.value.uncertain is unknown


@pytest.mark.parametrize("failure", [None, RuntimeError("disconnected")])
def test_transport_timeout_or_exception_has_uncertain_outcome(setup, monkeypatch, failure):
    s, client, _, _, _ = service(setup)
    client.response = failure
    monkeypatch.setattr("integ_operator_web.arm.mode_service.MODE_RESPONSE_TIMEOUT_S", 0.001)
    with pytest.raises(ModeError) as e:
        s.switch("jtc")
    assert e.value.uncertain
    assert bool(client.removed) is (failure is None)


def test_success_without_new_stationary_feedback_does_not_complete(setup, monkeypatch):
    s, _, _, c, _ = service(setup)
    c.idle_probe = lambda _: False
    monkeypatch.setattr("integ_operator_web.arm.mode_service.MODE_READBACK_TIMEOUT_S", 0.001)
    with pytest.raises(ModeError, match="停稳反馈") as e:
        s.switch("jtc")
    assert not e.value.uncertain  # The mode response is known; feedback may recover.
