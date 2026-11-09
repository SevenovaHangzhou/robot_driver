"""Real Mink prediction and MuJoCo execution through the local Rolling contract."""
import json
import os
from pathlib import Path
import threading
import time

import numpy as np
import pytest

from gripper_jog.errors import JogError
from gripper_jog.mink_sim import build_mink
from gripper_jog.rolling import JOINT_NAMES
from gripper_jog.server import MockServer
from test_http import request


@pytest.fixture
def rolling_mink():
    root = Path(os.environ.get('JOG_TEST_REAL2SIM', Path(__file__).resolve().parents[3] / 'sim/real2sim_v3'))
    if not (root / 'real2sim/mink_demo.py').is_file():
        pytest.skip('Set JOG_TEST_REAL2SIM to the real2sim_v3 reference')
    now = [0.]
    jog, _ = build_mink(root, clock=lambda: now[0], autostart=False, rolling=True, gravity_mode='mujoco')
    body = {'client': 'page-mink-rolling-test'}
    jog.dispatch('acquire', 'session', body)
    yield jog, now, body
    jog.dispatch('stop', 'test-cleanup', {})
    for _ in range(100):
        now[0] += .02
        jog._cycle()
        if jog.active is None:
            break
    jog.close()


def begin(jog, body, side='left', direction='x+', speed=10):
    if side != jog.selected_arm:
        jog.dispatch('select_arm', 'session', {**body, 'arm': side})
    body = {**body, **jog.dispatch('arm', 'session', body), 'seq': 1,
            'robot_direction': direction, 'speed_mm_s': speed}
    jog.dispatch('velocity', 'session', body)
    return body


def advance(jog, now, body, count, heartbeat=True):
    for i in range(count):
        now[0] += jog.follower.dt
        if heartbeat and i % 10 == 0:
            jog.dispatch('heartbeat', 'session', {**body, 'seq': jog.heartbeat_seq + 1})
        jog._cycle()
        assert not jog.fault, jog.fault


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direction', ['x+', 'x-', 'y+', 'y-', 'z+', 'z-'])
def test_six_directions_cross_five_mm_without_new_sessions(rolling_mink, side, direction):
    jog, now, owner = rolling_mink
    before = jog.backend.q.copy()
    body = begin(jog, owner, side, direction)
    start = jog.planner.chain.fk(jog.q)[:3, 3]
    advance(jog, now, body, 400)
    motion = jog.last_motion
    assert jog.active is motion and jog.armed and not jog.stopping
    assert motion.sequence > 40 and len(motion.committed) <= motion.capacity
    assert not jog.transport.close_operations
    assert len({bytes(batch['session_id']['uuid']) for batch in jog.transport.batches}) == 1
    delta = jog.planner.chain.fk(jog.q)[:3, 3] - start
    axis = 'xyz'.index(jog.direction[0])
    assert delta[axis] * (1 if jog.direction[1] == '+' else -1) > .005
    other = np.array([i for i, name in enumerate(JOINT_NAMES) if not name.startswith(side)])
    np.testing.assert_allclose(jog.backend.q[other], before[other], atol=1e-6)
    state = jog.state('session', owner['client'])
    assert state['publication']['backend'] == 'mujoco_mink_rolling'
    assert state['publication']['feedback_source'] == 'mujoco_actual'
    assert state['tcp_actual_speed_mm_s'] > .5


@pytest.mark.parametrize('speed', [.5, 1, 2, 5, 10, 20, 30, 50])
def test_allowed_speeds_are_continuous_and_joint_bounded(rolling_mink, speed):
    jog, now, owner = rolling_mink
    body = begin(jog, owner, speed=speed)
    advance(jog, now, body, 250)
    source = jog.last_motion.source
    assert jog.active is jog.last_motion and not jog.stopping
    for knot in jog.last_motion.committed:
        assert np.all(np.abs(knot.v) <= source.speed + 1e-8)
        assert np.max(np.abs(knot.a)) <= source.limits.joint_acceleration + 1e-8
    assert jog.state('session', owner['client'])['tcp_command_speed_mm_s'] <= 50.001


def test_change_speed_and_direction_replaces_tail_of_same_session(rolling_mink):
    jog, now, owner = rolling_mink
    body = begin(jog, owner)
    advance(jog, now, body, 150)
    motion, session = jog.last_motion, jog.last_motion.identity['session_id']
    before = jog.transport.curve
    before_until = jog.transport.until
    jog.dispatch('velocity', 'session', {**body, 'seq': 2, 'robot_direction': 'z+', 'speed_mm_s': 5})
    advance(jog, now, body, 1)
    batch = jog.transport.batches[-1]
    seam = batch['replace_from_ns']
    assert jog.active is motion and motion.identity['session_id'] == session
    assert seam < before_until and seam >= jog.transport.execution + jog.transport.negotiated['replace_lead_ns']
    np.testing.assert_allclose(before(seam / 1e9), jog.transport.curve(seam / 1e9), atol=1e-12)
    np.testing.assert_allclose(before(seam / 1e9, 1), jog.transport.curve(seam / 1e9, 1), atol=1e-12)
    np.testing.assert_allclose(before(seam / 1e9, 2), jog.transport.curve(seam / 1e9, 2), atol=2e-7)
    at = np.linspace(jog.transport.execution / 1e9, seam / 1e9, 21)
    np.testing.assert_allclose(before(at), jog.transport.curve(at), atol=1e-12)
    advance(jog, now, body, 200)
    assert jog.active is motion and motion.replacements >= 2 and not jog.transport.close_operations


@pytest.mark.parametrize('moving_cycles', [5, 30, 80, 200])
def test_normal_release_reaches_zero_velocity_and_acceleration_before_close(rolling_mink, moving_cycles):
    jog, now, owner = rolling_mink
    body = begin(jog, owner, speed=20)
    advance(jog, now, body, moving_cycles)
    motion = jog.last_motion
    recorded = []
    original = jog.transport.close

    def close(request):
        if request['operation'] == 'REQUEST_STOP':
            recorded.append(np.asarray(jog.transport.state()['desired_velocities']))
        return original(request)

    jog.transport.close = close
    result = jog.dispatch('halt', 'session', {**body, 'seq': 2})
    with pytest.raises(JogError, match='代次'):
        jog.dispatch('velocity', 'session', {**body, 'seq': 3})
    body.update(result, seq=1)
    assert jog.stopping
    for _ in range(250):
        advance(jog, now, body, 1)
        if jog.active is None:
            break
    assert jog.active is None and motion.complete and jog.armed and not jog.stopping
    assert jog.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    assert len(recorded) == 1 and np.max(np.abs(recorded[0])) < 1e-9
    end = motion.source.stop_knots[motion.source.stop_until_ns]
    np.testing.assert_array_equal(end.v, np.zeros(14))
    np.testing.assert_array_equal(end.a, np.zeros(14))


def test_input_timeout_uses_controller_brake_without_resuming(rolling_mink):
    jog, now, owner = rolling_mink
    body = begin(jog, owner)
    advance(jog, now, body, 100)
    count = jog.last_motion.sequence
    for _ in range(200):
        now[0] += .01
        jog._cycle()
    assert not jog.armed and not jog.epoch and jog.active is None
    assert jog.last_motion.sequence < count + 7  # TTL permits only the next .4 s.
    assert jog.last_motion.source.stop_knots is None
    assert jog.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']


def test_transport_failure_keeps_worker_cleanup_owner(rolling_mink, monkeypatch):
    jog, now, owner = rolling_mink
    body = begin(jog, owner)
    advance(jog, now, body, 100)
    motion = jog.last_motion
    original = jog.transport.state
    monkeypatch.setattr(jog.transport, 'state', lambda: (_ for _ in ()).throw(JogError('rolling_state_stale', '状态超时')))
    now[0] += .01
    jog._cycle()
    assert not jog.armed and not jog.feedback_fresh and jog.active is motion and jog.fault
    count = motion.sequence
    monkeypatch.setattr(jog.transport, 'state', original)
    for _ in range(100):
        now[0] += .01
        jog._cycle()
    assert jog.active is None and motion.complete and motion.sequence == count
    assert jog.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']


def test_persistent_physics_failure_does_not_abandon_finalize(rolling_mink, monkeypatch):
    jog, now, owner = rolling_mink
    body = begin(jog, owner)
    advance(jog, now, body, 100)
    motion = jog.last_motion
    monkeypatch.setattr(jog.backend, 'step', lambda *args: (_ for _ in ()).throw(RuntimeError('physics failed')))
    for _ in range(100):
        now[0] += .01
        jog._cycle()
    assert jog.active is None and motion.complete and not jog.armed and jog.fault
    assert jog.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']


def test_close_during_motion_finishes_session_before_joining_worker(rolling_mink):
    jog, _, owner = rolling_mink
    jog.clock = jog.transport.clock = time.monotonic
    jog.dispatch('acquire', 'session', owner)
    body = begin(jog, owner)
    jog.start()
    time.sleep(.15)
    jog.close()
    assert jog.active is None and jog.last_motion.complete and not jog.worker.is_alive()
    assert jog.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']


def test_100_mm_s_and_step_api_do_not_open_session(rolling_mink):
    jog, _, owner = rolling_mink
    body = {**owner, **jog.dispatch('arm', 'session', owner), 'seq': 1}
    for action, extra in [('velocity', {'robot_direction': 'x+', 'speed_mm_s': 100}), ('step', {'direction': 'x+', 'step_mm': 5})]:
        body['seq'] += 1
        with pytest.raises(JogError):
            jog.dispatch(action, 'session', {**body, **extra})
    assert jog.active is None and not jog.transport.batches and not jog.transport.status


def test_http_with_real_worker_and_actual_model_motion(rolling_mink):
    jog, _, owner = rolling_mink
    jog.dispatch('release', 'session', owner)
    jog.clock = jog.transport.clock = time.monotonic
    server = MockServer(('127.0.0.1', 0), jog)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    jog.start()
    try:
        assert request(server, 'POST', '/api/login', {'token': 'mock'})[0] == 200
        _, _, cookie = request(server, 'POST', '/api/login', {'token': 'mock'})
        body = {'client': 'http-mink-rolling'}
        assert request(server, 'POST', '/api/jog/acquire', body, cookie)[0] == 200
        status, raw, _ = request(server, 'POST', '/api/jog/arm', body, cookie)
        assert status == 200
        body.update(epoch=json.loads(raw)['epoch'], seq=1)
        before = jog.backend.q.copy()
        assert request(server, 'POST', '/api/jog/velocity', {**body, 'robot_direction': 'x+', 'speed_mm_s': 5}, cookie)[0] == 200
        for sequence in range(1, 15):
            time.sleep(.1)
            assert request(server, 'POST', '/api/jog/heartbeat', {**body, 'seq': sequence}, cookie)[0] == 200
        status, raw, _ = request(server, 'GET', '/api/jog/state?client=http-mink-rolling', cookie=cookie)
        state = json.loads(raw)
        assert status == 200 and not state['fault'] and state['busy'] and state['armed']
        assert state['publication']['sequence'] > 10
        assert np.linalg.norm(jog.backend.q - before) > 1e-4
        assert state['publication']['feedback_source'] == 'mujoco_actual'
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)
