"""Lost service responses must retain ownership and never resume the plan."""
from copy import deepcopy

import numpy as np
import pytest

from gripper_jog.errors import JogError
from gripper_jog.rolling import RollingMotion, identifier
from gripper_jog.rolling_mock import MockRollingTransport
from test_rolling import rolling


class LostOpen(MockRollingTransport):
    def __init__(self, *args):
        super().__init__(*args)
        self.available = False
        self.requests = []
        self.response = None

    def open(self, request):
        self._advance()
        self.requests.append(deepcopy(request))
        if self.response is None:
            self.response = super().open(request)
        if not self.available:
            raise TimeoutError('open response unavailable; outcome unknown')
        assert request == self.requests[0]
        return deepcopy(self.response)


def setup_motion(rolling):
    jog, execution, clock, body = rolling
    plan = jog.planner.step(jog.q, 'x+', .0005, require_collision=False)
    transport = LostOpen(np.r_[jog.q + .1, jog.q], .07, clock)
    return plan, transport, clock


def test_lost_open_recovers_original_identity_and_only_cleans_up(rolling):
    plan, transport, clock = setup_motion(rolling)
    motion = RollingMotion(plan, transport, clock)
    assert motion.open_pending and motion.fault and motion.stopping
    assert motion.publication()['session_id'] is None
    for _ in range(20):
        clock.now += .02
        motion.state(0)
    assert transport.status == 4  # PRIME_TIMEOUT -> HOLDING
    assert not motion.complete and not transport.batches
    transport.available = True
    motion.state(0)
    assert motion.complete and motion.fault
    assert transport.status == 0
    assert transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    assert not transport.batches
    assert all(r == transport.requests[0] for r in transport.requests)


def test_unknown_open_remains_owned_across_web_stop_and_blocks_new_motion(rolling):
    jog, execution, clock, body = rolling
    transport = LostOpen(np.r_[jog.q + .1, jog.q], .07, clock)
    execution.transport = transport
    jog.dispatch('step', 'session', body)
    motion = jog.active
    jog.dispatch('stop', 'session', {})
    for _ in range(10):
        clock.now += .02
        jog.tick()
    assert jog.active is motion and not jog.armed and jog.stopping
    with pytest.raises(JogError, match='运动中'):
        jog.dispatch('arm', 'session', body)
    transport.available = True
    jog.tick()
    assert jog.active is None and not jog.armed
    assert motion.complete and not transport.batches


@pytest.mark.parametrize('operation', ['REQUEST_STOP', 'FINALIZE'])
def test_cleanup_response_loss_retries_same_close_request(rolling, operation):
    plan, transport, clock = setup_motion(rolling)
    motion = RollingMotion(plan, transport, clock)
    transport.available = True
    original = transport.close
    requests = []
    blocked = True

    def close(request):
        nonlocal blocked
        response = original(request)
        if request['operation'] == operation:
            requests.append(deepcopy(request))
            if blocked:
                raise TimeoutError('close outcome unknown')
        return response

    transport.close = close
    motion.state(0)
    assert not motion.complete
    blocked = False
    motion.state(0)
    assert motion.complete and transport.status == 0
    assert len(requests) >= 2 and all(r == requests[0] for r in requests)
    assert not transport.batches


def test_open_identity_mismatch_does_not_stop_another_client(rolling):
    plan, transport, clock = setup_motion(rolling)
    motion = RollingMotion(plan, transport, clock)
    transport.available = True
    transport.response['client_instance_id'] = identifier()
    for _ in range(3):
        motion.state(0)
    assert motion.open_pending and not motion.complete and motion.fault
    assert not transport.close_operations and not transport.batches


def test_initialization_error_keeps_cleanup_owner_when_stop_response_is_lost(rolling):
    plan, transport, clock = setup_motion(rolling)
    transport.available = True
    transport.negotiated['buffer_capacity'] = 8
    original = transport.close
    blocked = True

    def close(request):
        if blocked:
            raise TimeoutError('stop outcome unknown')
        return original(request)

    transport.close = close
    motion = RollingMotion(plan, transport, clock)
    assert motion.fault and not motion.complete and motion.stopping
    blocked = False
    for _ in range(3):
        motion.state(0)
    assert motion.complete and transport.status == 0 and not transport.batches


def test_confirmed_stop_waits_for_holding_without_reissuing_close(rolling):
    plan, transport, clock = setup_motion(rolling)
    transport.available = True
    motion = RollingMotion(plan, transport, clock)
    clock.now += .02
    motion.state(0)
    original = transport.close
    stops = []

    def close(request):
        if request['operation'] == 'REQUEST_STOP':
            stops.append(deepcopy(request))
            if len(stops) > 1:
                return {'accepted': False}  # RT/NRT stop state not yet aligned.
        return original(request)

    transport.close = close
    motion.stop()
    motion.stop()
    for _ in range(30):
        clock.now += .02
        motion.state(0)
        if motion.complete:
            break
    assert len(stops) == 1
    assert motion.complete and not motion.fault and transport.status == 0
