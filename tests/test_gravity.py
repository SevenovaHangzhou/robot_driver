"""Static library to actual actuator tests, using the calibrated simulation."""
import json
import os
from pathlib import Path

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from gripper_jog.errors import JogError
from gripper_jog.gravity import GravityBackend, load_static_gravity
from gripper_jog.mink_sim import build_mink


@pytest.fixture
def gravity_jog():
    root = Path(os.environ.get('JOG_TEST_REAL2SIM', Path(__file__).resolve().parents[3] / 'sim/real2sim_v3'))
    if not (root / 'real2sim/mink_demo.py').is_file():
        pytest.skip('Set JOG_TEST_REAL2SIM to the real2sim_v3 reference')
    jog, _ = build_mink(root, autostart=False, rolling=True, gravity_mode='static')
    yield jog
    jog.close()


def test_static_matches_mujoco_for_random_angles_gravity_and_base_rotations(gravity_jog):
    b = gravity_jog.backend
    rng = np.random.default_rng(309)
    data = mujoco.MjData(b.model)
    before = b.data.qpos.copy()
    for _ in range(100):
        data.qpos[:] = b.data.qpos
        data.qpos[b.qadr] = rng.uniform(b.lower, b.upper)
        data.qvel[:] = 0
        mujoco.mj_forward(b.model, data)
        np.testing.assert_allclose(b._torques(data), data.qfrc_bias[b.vadr], atol=3e-12)
        # Rotating the entire arm base is equivalent to rotating gravity back
        # into the original world. Exercise the C ABI's non-default base path.
        rotation = Rotation.from_rotvec(rng.normal(size=3)).as_matrix()
        gravity = rng.normal(size=3) * 9.81
        bases = np.array([data.body(name).xmat.reshape(3, 3) for name in b.base_names])
        q = data.qpos[b.qadr]
        rotated = b.calculator.torques(q, gravity=gravity, bases=rotation @ bases)
        expected = b.calculator.torques(q, gravity=rotation.T @ gravity, bases=bases)
        np.testing.assert_allclose(rotated, expected, atol=3e-12)
    np.testing.assert_array_equal(b.data.qpos, before)
    assert b.data.time == 0


def test_actual_joint_order_and_no_bias_double_compensation(gravity_jog):
    b = gravity_jog.backend
    b.data.qvel[b.vadr] = np.linspace(-.4, .7, 14)
    mujoco.mj_forward(b.model, b.data)
    actual = b.q
    desired = np.clip(actual + np.linspace(-.01, .01, 14), b.lower, b.upper)
    velocity = np.linspace(-.02, .03, 14)
    static = b.calculator.torques(actual)
    # Nonzero speed separates static gravity from the old bias compensation.
    assert np.max(np.abs(static - b.data.qfrc_bias[b.vadr])) > 1e-3
    assert np.max(np.abs(static - b.calculator.torques(desired))) > .01
    b.command(b.data, desired, velocity, record=True)
    mujoco.mj_forward(b.model, b.data)
    np.testing.assert_allclose(b.last_torques, static, atol=1e-10)
    expected = b.kp * (desired - actual) + b.kv * (velocity - b.v) + static
    # The actual force is PD plus exactly one static torque, in right/left order.
    np.testing.assert_allclose(b.data.actuator_force[b.aids], expected, atol=1e-10)
    assert tuple(b.names[:7]) == tuple(f'right_joint{i}' for i in range(1, 8))


def test_static_holds_both_arms_without_moving_physics_on_status(gravity_jog):
    jog, b = gravity_jog, gravity_jog.backend
    start = b.q.copy()
    for _ in range(300):
        jog._cycle()
    state = jog.state('test', 'test')['gravity_compensation']
    assert state['mode'] == 'static' and state['enabled'] and not state['fault']
    assert state['period_s'] == .002 and state['evaluations'] == 1500
    assert not state['hardware_connected'] and state['validation_max_error_nm'] < 1e-7
    assert len(state['joint_names']) == len(state['torques_nm']) == 14
    assert max(abs(t) for t in state['torques_nm']) > 80
    np.testing.assert_allclose(b.q, start, atol=1e-9)
    assert np.max(np.abs(b.v)) < 1e-9 and not jog.fault
    at = b.data.time
    jog.state('test', 'test')
    assert b.data.time == at
    json.dumps(state, allow_nan=False)


@pytest.mark.parametrize('mode', ['mujoco', 'off'])
def test_comparison_modes_are_explicit_and_exclusive(gravity_jog, mode):
    b = GravityBackend(gravity_jog.backend.original, mode)
    q = b.q
    mujoco.mj_forward(b.model, b.data)
    expected = b.data.qfrc_bias[b.vadr].copy() if mode == 'mujoco' else np.zeros(14)
    b.command(b.data, q, np.zeros(14), record=True)
    np.testing.assert_allclose(b.data.ctrl[b.aids], q + expected / b.kp, atol=1e-12)
    assert b.gravity_state()['enabled'] == (mode != 'off')


def test_preview_uses_static_servo_without_mutating_live_commands(gravity_jog, monkeypatch):
    f, b = gravity_jog.follower, gravity_jog.backend
    q = b.q.copy()
    before = b.data.ctrl.copy()
    calls = []
    original = b.command

    def command(data, target, velocity, **kwargs):
        calls.append(data)
        return original(data, target, velocity, **kwargs)

    monkeypatch.setattr(b, 'command', command)
    assert f.preview_is_clear(q, np.zeros(14))
    assert len(calls) == f.preview_steps and all(data is f.preview_data for data in calls)
    assert b.evaluations == 0 and b.data.time == 0
    np.testing.assert_array_equal(b.data.ctrl, before)
    np.testing.assert_array_equal(b.q, q)


@pytest.mark.parametrize('result', [np.full(14, np.nan), np.zeros(7), 'exception'])
def test_gravity_failure_latches_revokes_and_finalizes_rolling(gravity_jog, monkeypatch, result):
    jog, b = gravity_jog, gravity_jog.backend
    now = [0.]
    jog.clock = jog.transport.clock = lambda: now[0]
    owner = {'client': 'gravity-fault-test'}
    jog.dispatch('acquire', 'session', owner)
    command = {**owner, **jog.dispatch('arm', 'session', owner), 'seq': 1,
               'robot_direction': 'x+', 'speed_mm_s': 5}
    jog.dispatch('velocity', 'session', command)
    for i in range(25):
        now[0] += .01
        jog._cycle()
    motion = jog.last_motion

    def failed(*args, **kwargs):
        if isinstance(result, str):
            raise ValueError('library rejected input')
        return result

    monkeypatch.setattr(b.calculator, 'torques', failed)
    for _ in range(100):
        now[0] += .01
        jog._cycle()
    assert jog.active is None and motion.complete and not jog.armed
    assert not jog.feedback_fresh and jog.fault and b.gravity_fault
    assert jog.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    np.testing.assert_array_equal(b.last_torques, np.zeros(14))
    np.testing.assert_array_equal(b.data.ctrl[b.aids], b.q)
    monkeypatch.undo()
    with pytest.raises(RuntimeError, match='需重启'):
        b.step(b.q)


def test_scene_inertia_mismatch_is_rejected_before_worker_start(gravity_jog):
    b = gravity_jog.backend
    body = b.model.body('left_link3').id
    original = b.model.body_mass[body]
    try:
        b.model.body_mass[body] *= 1.2
        with pytest.raises(JogError, match='不一致'):
            GravityBackend(b.original, 'static', b.calculator)
    finally:
        b.model.body_mass[body] = original
    assert gravity_jog.worker is None


def test_missing_static_source_fails_without_falling_back(gravity_jog, tmp_path):
    with pytest.raises(JogError, match='源码不完整'):
        load_static_gravity(tmp_path, tmp_path / 'missing.urdf', gravity_jog.backend.names)


def test_build_keeps_original_parameters_and_rejects_corrupt_cache(gravity_jog, tmp_path):
    root = Path(os.environ.get('JOG_TEST_REAL2SIM', Path(__file__).resolve().parents[3] / 'sim/real2sim_v3'))
    source = root.parent / 'Dynamic_Parameter_Identification/static_gravity'
    original_parameters = (source / 'parameters.json').read_bytes()
    urdf = root / 'urdf/robot.urdf'
    names = gravity_jog.backend.names
    calc = load_static_gravity(source, urdf, names, cache=tmp_path)
    assert calc.parameters['source_urdf_sha256'] == gravity_jog.backend.calculator.parameters['source_urdf_sha256']
    assert (source / 'parameters.json').read_bytes() == original_parameters
    package = next(tmp_path.glob('*/static_gravity'))
    parameters = json.loads((package / 'parameters.json').read_text())
    parameters['parameter_signature'] = 'stale'
    (package / 'parameters.json').write_text(json.dumps(parameters))
    with pytest.raises(JogError, match='缓存或模型校验失败'):
        load_static_gravity(source, urdf, names, cache=tmp_path)
