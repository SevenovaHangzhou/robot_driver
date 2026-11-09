from concurrent.futures import ThreadPoolExecutor
import threading

import numpy as np
import pytest

from gripper_jog.control import MockJog
from gripper_jog.errors import JogError


class Clock:
    now = 0.
    def __call__(self): return self.now


def make_jog(planner, q):
    clock = Clock()
    jog = MockJog(planner, q, clock=clock)
    identity = {'client': 'page-00000001'}
    jog.dispatch('acquire', 'session', identity)
    epoch = jog.dispatch('arm', 'session', identity)['epoch']
    return jog, clock, {**identity, 'epoch': epoch, 'seq': 1}


def test_tab_exclusion_stale_epoch_and_duplicate_command(planner, q):
    jog, clock, body = make_jog(planner, q)
    with pytest.raises(JogError):
        jog.dispatch('acquire', 'session', {'client': 'page-00000002'})
    jog.dispatch('heartbeat', 'session', body)
    with pytest.raises(JogError) as e:
        jog.dispatch('heartbeat', 'session', body)
    assert e.value.code == 'stale_command'
    jog.dispatch('stop', 'other-authenticated-session', {})
    with pytest.raises(JogError) as e:
        jog.dispatch('step', 'session', {**body, 'seq': 2, 'direction': 'x+', 'step_mm': .5})
    assert e.value.code == 'epoch_expired'


def test_independent_heartbeat_expiry_stops_and_does_not_resume(planner, q):
    jog, clock, body = make_jog(planner, q)
    jog.dispatch('step', 'session', {**body, 'direction': 'x+', 'step_mm': .5})
    clock.now += .2; jog.tick()
    clock.now += .21; jog.tick()
    assert not jog.armed
    assert jog.state('session', body['client'])['stopping']
    clock.now += 3.01; jog.tick()
    assert jog.owner is None and jog.active is None
    assert not jog.state('session', body['client'])['armed']
    assert np.linalg.norm(jog.q - q) > 0


def test_busy_does_not_queue_and_no_new_motion_on_reconnect(planner, q):
    jog, clock, body = make_jog(planner, q)
    jog.dispatch('step', 'session', {**body, 'direction': 'y+', 'step_mm': .5})
    with pytest.raises(JogError) as e:
        jog.dispatch('step', 'session', {**body, 'seq': 2, 'direction': 'x+', 'step_mm': .5})
    assert e.value.code == 'busy'
    clock.now += 5; jog.tick()
    assert jog.active is None and not jog.armed
    snapshot = jog.q.copy(); jog.state('session', body['client']); jog.tick()
    np.testing.assert_allclose(jog.q, snapshot)


def test_stop_during_planning_cannot_be_resurrected(planner, q):
    jog, clock, body = make_jog(planner, q)
    ready, resume = threading.Event(), threading.Event()
    plan = planner.step(q, 'x+', .0005, require_collision=False)
    def delayed(*args, **kwargs):
        ready.set(); assert resume.wait(3); return plan
    planner.step = delayed
    with ThreadPoolExecutor() as pool:
        pending = pool.submit(jog.dispatch, 'step', 'session', {**body, 'direction': 'x+', 'step_mm': .5})
        assert ready.wait(3)
        jog.dispatch('stop', 'session', {})
        resume.set()
        with pytest.raises(JogError) as e: pending.result(timeout=3)
        assert e.value.code == 'cancelled'
    assert jog.active is None
    np.testing.assert_allclose(jog.q, q)


def test_external_sequence_and_stale_feedback_refuse_arm(planner, q):
    jog, clock, body = make_jog(planner, q)
    jog.external_busy = True; jog.tick()
    assert not jog.armed
    with pytest.raises(JogError): jog.dispatch('arm', 'session', body)
    jog.external_busy = False; jog.feedback_fresh = False
    with pytest.raises(JogError): jog.dispatch('arm', 'session', body)


def test_release_rotates_epoch_without_automatic_motion(planner, q):
    jog, clock, body = make_jog(planner, q)
    response = jog.dispatch('halt', 'session', body)
    assert response['epoch'] != body['epoch'] and jog.armed and jog.active is None
    with pytest.raises(JogError) as error:
        jog.dispatch('step', 'session', {**body, 'seq': 2, 'direction': 'x+', 'step_mm': .5})
    assert error.value.code == 'epoch_expired'
    jog.dispatch('step', 'session', {**body, 'epoch': response['epoch'], 'seq': 1, 'direction': 'x+', 'step_mm': .5})
    assert jog.active is not None


def test_later_heartbeat_cannot_invalidate_in_flight_step(planner, q):
    jog, clock, body = make_jog(planner, q)
    jog.dispatch('heartbeat', 'session', {**body, 'seq': 200})
    jog.dispatch('step', 'session', {**body, 'seq': 1, 'direction': 'x+', 'step_mm': .5})
    assert jog.armed and jog.active is not None
    with pytest.raises(JogError) as error:
        jog.dispatch('heartbeat', 'session', {**body, 'seq': 199})
    assert error.value.code == 'stale_command'


@pytest.mark.parametrize('step_mm', [.5, 1, 2, 5, 10])
def test_selected_step_reaches_requested_tcp_displacement(planner, q, step_mm):
    jog, clock, body = make_jog(planner, q)
    start = planner.chain.fk(q)
    jog.dispatch('step', 'session', {**body, 'direction': 'x+', 'step_mm': step_mm})
    plan = jog.active
    actual = planner.chain.fk(plan.state(plan.duration)[0])
    np.testing.assert_allclose(actual[:3, 3] - start[:3, 3], [step_mm / 1000, 0, 0], atol=1e-7)
    np.testing.assert_allclose(actual[:3, :3], start[:3, :3], atol=1e-7)
    assert max(plan.ratios()) <= 1


@pytest.mark.parametrize('step_mm', [None, True, '1', 0, -1, .75, 3, 11, float('nan'), float('inf'), []])
def test_unsupported_step_is_rejected_without_motion(planner, q, step_mm):
    jog, clock, body = make_jog(planner, q)
    with pytest.raises(JogError) as error:
        jog.dispatch('step', 'session', {**body, 'direction': 'x+', 'step_mm': step_mm})
    assert error.value.code == 'bad_step'
    assert jog.active is None and not jog.planning
    np.testing.assert_allclose(jog.q, q)


@pytest.mark.parametrize('speed_mm_s', [.5, 1, 2, 5, 10, 20, 30, 50])
def test_speed_controls_time_law_without_changing_distance_or_shared_limits(planner, q, speed_mm_s):
    jog, clock, body = make_jog(planner, q)
    original_limits = planner.limits
    start = planner.chain.fk(q)
    result = jog.dispatch('step', 'session', {**body, 'direction': 'x+', 'speed_mm_s': speed_mm_s})
    plan = jog.active
    assert plan.limits.tcp_speed == speed_mm_s / 1000
    assert planner.limits is original_limits
    assert result['speed_mm_s'] == speed_mm_s
    assert result['distance_mm'] == 5
    assert max(plan.ratios()) <= 1
    for t in np.linspace(0, plan.duration, 31):
        positions, velocity, *_ = plan.state(t)
        jac = planner.chain.fk_jacobian(positions)[1]
        assert np.linalg.norm(jac[:3] @ velocity) <= speed_mm_s / 1000 * 1.00001
    np.testing.assert_allclose(planner.chain.fk(plan.state(plan.duration)[0])[:3, 3] - start[:3, 3],
                               [.005, 0, 0], atol=1e-7)


def test_higher_speed_reduces_duration(planner, q):
    plans = []
    for speed in (.5, 10):
        jog, _, body = make_jog(planner, q)
        plans.append(jog.dispatch('step', 'session', {**body, 'direction': 'x+', 'speed_mm_s': speed}))
    assert plans[1]['duration_s'] < plans[0]['duration_s']


@pytest.mark.parametrize('speed_mm_s', [None, True, '10', 0, -1, .75, 11, 100, float('nan'), float('inf'), []])
def test_unsupported_speed_is_rejected_without_motion(planner, q, speed_mm_s):
    jog, clock, body = make_jog(planner, q)
    with pytest.raises(JogError) as error:
        jog.dispatch('step', 'session', {**body, 'direction': 'x+', 'speed_mm_s': speed_mm_s})
    assert error.value.code == 'bad_speed'
    assert jog.active is None and not jog.planning
    np.testing.assert_allclose(jog.q, q)


def test_speed_and_step_cannot_be_combined(planner, q):
    jog, _, body = make_jog(planner, q)
    with pytest.raises(JogError) as error:
        jog.dispatch('step', 'session', {**body, 'direction': 'x+', 'speed_mm_s': 10, 'step_mm': 10})
    assert error.value.code == 'bad_step'
    assert jog.active is None


def test_command_speed_feedback_matches_fk_derivative_and_clears_at_rest(planner, q):
    jog, clock, body = make_jog(planner, q)
    jog.dispatch('step', 'session', {**body, 'direction': 'x+', 'speed_mm_s': 10})
    plan = jog.active
    clock.now = plan.duration / 2
    jog.input_at = clock.now
    state = jog.state('session', body['client'])
    dt = 1e-5
    before = planner.chain.fk(plan.state(clock.now - dt)[0])[:3, 3]
    after = planner.chain.fk(plan.state(clock.now + dt)[0])[:3, 3]
    assert state['tcp_command_speed_mm_s'] == pytest.approx(np.linalg.norm(after-before) / (2*dt) * 1000, rel=1e-5)
    clock.now = plan.duration + .01
    jog.input_at = clock.now
    assert jog.state('session', body['client'])['tcp_command_speed_mm_s'] == 0
