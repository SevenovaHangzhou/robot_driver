"""Exercise the real Mink QP, contact model and actuators, without ROS."""
from pathlib import Path
import os

import numpy as np
import pytest

from gripper_jog.errors import JogError
from gripper_jog.mink_sim import build_mink


@pytest.fixture
def mink_jog():
    root = Path(os.environ.get('JOG_TEST_REAL2SIM', Path(__file__).resolve().parents[3] / 'sim/real2sim_v3'))
    if not (root / 'real2sim/mink_demo.py').is_file():
        pytest.skip('Set JOG_TEST_REAL2SIM to the real2sim_v3 reference')
    now = [0.]
    jog, assets = build_mink(root, clock=lambda: now[0], autostart=False, gravity_mode='mujoco')
    body = {'client': 'page-mink-test'}
    jog.dispatch('acquire', 'session', body)
    yield jog, assets, now, body
    jog.close()


def arm(jog, body, side='left'):
    if side != jog.selected_arm:
        jog.dispatch('select_arm', 'session', {**body, 'arm': side})
    return {**body, **jog.dispatch('arm', 'session', body), 'seq': 1}


def advance(jog, now, body, steps, heartbeat=True):
    for i in range(steps):
        now[0] += jog.follower.dt
        if heartbeat and i % 10 == 0:
            jog.dispatch('heartbeat', 'session', {**body, 'seq': jog.heartbeat_seq + 1})
        jog._cycle()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direction', ['x+', 'x-', 'y+', 'y-', 'z+', 'z-'])
def test_continuous_six_body_directions_and_release(mink_jog, side, direction):
    jog, assets, now, owner = mink_jog
    body = arm(jog, owner, side)
    other = 'right' if side == 'left' else 'left'
    other_q = jog.backend.q[jog.indices[other]].copy()
    before = jog.state('session', body['client'])
    result = jog.dispatch('velocity', 'session', {**body, 'robot_direction': direction, 'speed_mm_s': 10})
    # Accepting a command cannot teleport actual joints.
    np.testing.assert_array_equal(before['q_rad'], jog.q)
    advance(jog, now, body, 100)
    after = jog.state('session', body['client'])
    delta = np.array(after['tcp_mm']) - before['tcp_mm']
    axis = 'xyz'.index(result['direction'][0])
    sign = 1 if result['direction'][1] == '+' else -1
    assert 7 < delta[axis] * sign < 11
    assert np.linalg.norm(np.delete(delta, axis)) < .4
    assert 9 < after['tcp_actual_speed_mm_s'] < 11
    assert after['busy'] and after['armed'] and not after['fault']
    assert after['publication']['mode'] == 'mink'
    assert after['publication']['feedback_source'] == 'mujoco_actual'
    assert after['servo']['collision_pairs'] >= 500
    assert assets.info(jog.planner.chain)['model_hash'] == after['model_hash']
    np.testing.assert_allclose(jog.backend.q[jog.indices[other]], other_q, atol=1e-6)
    old_epoch = body['epoch']
    result = jog.dispatch('halt', 'session', {**body, 'seq': 2})
    assert result['epoch'] != old_epoch
    with pytest.raises(JogError, match='代次'):
        jog.dispatch('velocity', 'session', {**body, 'seq': 3, 'robot_direction': direction, 'speed_mm_s': 10})
    body.update(result, seq=1)
    advance(jog, now, body, 150)
    stopped = jog.state('session', body['client'])
    assert not stopped['busy'] and stopped['armed']
    assert stopped['tcp_actual_speed_mm_s'] < .05


@pytest.mark.parametrize('speed', [.5, 1, 2, 5, 10, 20, 30, 50])
def test_speed_is_velocity_independent_of_fixed_distance(mink_jog, speed):
    jog, _, now, owner = mink_jog
    body = arm(jog, owner)
    jog.dispatch('velocity', 'session', {**body, 'robot_direction': 'x+', 'speed_mm_s': speed})
    # Include the configured 50 mm/s² acceleration ramp before measuring speed.
    advance(jog, now, body, 120 + int(np.ceil(speed / 50 * 100)))
    state = jog.state('session', body['client'])
    assert state['tcp_actual_speed_mm_s'] == pytest.approx(speed, rel=.07)
    assert state['busy'] and jog.direction == 'x+'


def test_100_mm_s_is_rejected_without_motion(mink_jog):
    jog, _, _, owner = mink_jog
    body = arm(jog, owner)
    before = jog.backend.q.copy()
    with pytest.raises(JogError) as error:
        jog.dispatch('velocity', 'session', {**body, 'robot_direction': 'x+', 'speed_mm_s': 100})
    assert error.value.code == 'bad_speed'
    assert jog.direction is None and jog.target_speed == 0 and jog.active is None
    np.testing.assert_array_equal(jog.backend.q, before)
    assert 100 not in jog.state('session', owner['client'])['speed_options_mm_s']


def test_expired_input_stops_without_polling_and_old_epoch_cannot_resume(mink_jog):
    jog, _, now, owner = mink_jog
    body = arm(jog, owner)
    jog.dispatch('velocity', 'session', {**body, 'robot_direction': 'x+', 'speed_mm_s': 10})
    advance(jog, now, body, 200, heartbeat=False)
    assert not jog.armed and not jog.epoch and jog.direction is None
    assert not jog.active
    with pytest.raises(JogError, match='代次'):
        jog.dispatch('heartbeat', 'session', {**body, 'seq': 2})


def test_collision_hold_cancels_target_and_does_not_resume_on_heartbeat(mink_jog, monkeypatch):
    jog, _, now, owner = mink_jog
    body = arm(jog, owner)
    jog.dispatch('velocity', 'session', {**body, 'robot_direction': 'x+', 'speed_mm_s': 10})
    monkeypatch.setattr(jog.follower, 'preview_is_clear', lambda *args: False)
    advance(jog, now, body, 1, heartbeat=False)
    assert not jog.armed and jog.direction is None and '避碰' in jog.fault
    assert not np.any(jog.cart_velocity)
    with pytest.raises(JogError):
        jog.dispatch('heartbeat', 'session', {**body, 'seq': 2})
    np.testing.assert_array_equal(jog.follower.last_velocity, 0)


def test_switch_and_competing_page_are_rejected_during_motion(mink_jog):
    jog, _, _, owner = mink_jog
    body = arm(jog, owner)
    jog.dispatch('velocity', 'session', {**body, 'robot_direction': 'x+', 'speed_mm_s': 1})
    with pytest.raises(JogError):
        jog.dispatch('select_arm', 'session', {**owner, 'arm': 'right'})
    with pytest.raises(JogError):
        jog.dispatch('velocity', 'session', {**body, 'client': 'other-page', 'robot_direction': 'x-', 'speed_mm_s': 10})
    with pytest.raises(JogError):
        jog.dispatch('step', 'session', {**body, 'direction': 'x+', 'step_mm': 1})


def test_state_reads_do_not_advance_simulation(mink_jog):
    jog, _, _, body = mink_jog
    before = jog.backend.data.time
    for _ in range(10):
        jog.state('session', body['client'])
    assert jog.backend.data.time == before


def test_deferred_physics_starts_once_and_closes(mink_jog):
    import time

    jog, _, _, _ = mink_jog
    assert jog.worker is None and jog.iterations == 0
    jog.start()
    worker = jog.worker
    jog.start()
    assert jog.worker is worker
    deadline = time.monotonic() + 3
    while jog.iterations < 2 and time.monotonic() < deadline:
        time.sleep(.01)
    jog.close()
    assert jog.iterations >= 2 and not worker.is_alive()
    assert jog.backend.data.time > 0
    jog.start()
    assert jog.worker is worker and not worker.is_alive()


def test_viewer_observes_actual_motion_after_initialization(mink_jog):
    from contextlib import contextmanager
    import threading
    from types import SimpleNamespace
    from gripper_jog.mink_sim import run_viewer

    jog, _, _, owner = mink_jog
    body = arm(jog, owner)
    jog.dispatch('velocity', 'session', {**body, 'robot_direction': 'x+', 'speed_mm_s': 10})
    initial = jog.backend.q.copy()
    frames = []
    stopped = threading.Event()

    @contextmanager
    def viewer(model, data):
        assert jog.worker is None
        assert model is jog.backend.model and data is jog.backend.data
        yield SimpleNamespace(is_running=lambda: len(frames) < 10,
                              sync=lambda: frames.append(data.qpos.copy()))

    jog.managed_viewer = viewer
    server = SimpleNamespace(jog=jog, serve_forever=stopped.wait, shutdown=stopped.set)
    try:
        run_viewer(server)
    finally:
        jog.close()
    assert stopped.is_set() and len(frames) == 10
    assert jog.iterations >= 10
    assert np.linalg.norm(jog.backend.q - initial) > 1e-5
    assert np.linalg.norm(frames[-1] - frames[0]) > 1e-5
    assert not jog.fault
