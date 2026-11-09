import json
import threading

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from gripper_jog.control import MockJog
from gripper_jog.errors import JogError
from gripper_jog.model import Chain
from gripper_jog.planner import Planner
from gripper_jog.rolling import AXIS_SET_HASH, JOINT_NAMES, RollingMotion, identifier
from gripper_jog.rolling_mock import MockRollingTransport, RollingExecutor
from gripper_jog.server import ROOT, build_mock, MockServer
from test_control import Clock
from test_http import request


@pytest.fixture
def rolling(xml, q):
    left, right = xml, xml
    for i in range(1, 8):
        left = left.replace(f'"j{i}"', f'"left_joint{i}"')
        right = right.replace(f'"j{i}"', f'"right_joint{i}"').replace(f'"l{i}"', f'"r{i}"')
    source = left.replace('</robot>', right[right.index('<link name="r1"'):])
    planner = Planner(Chain(source, 'base', 'l7'))
    other = Planner(Chain(source, 'base', 'r7'))
    clock = Clock()
    execution = RollingExecutor(clock)
    jog = MockJog(planner, q, clock=clock, arms={'left': (planner, q), 'right': (other, q + .1)},
                  selected_arm='left', execution=execution)
    body = {'client': 'page-00000001'}
    jog.dispatch('acquire', 'session', body)
    epoch = jog.dispatch('arm', 'session', body)['epoch']
    body.update(epoch=epoch, seq=1, direction='x+', step_mm=.5)
    return jog, execution, clock, body


def advance(jog, clock, body, count=1):
    for _ in range(count):
        clock.now += .02
        jog.tick()
        if jog.armed:
            jog.dispatch('heartbeat', 'session', {**body, 'seq': int(clock.now * 1000) + 100})


@pytest.mark.parametrize('speed_mm_s', [.5, 1, 2, 5, 10, 20, 30, 50])
def test_speed_request_executes_rolling_and_preserves_pose(rolling, speed_mm_s):
    jog, execution, clock, body = rolling
    body.pop('step_mm')
    body['speed_mm_s'] = speed_mm_s
    start = jog.planner.chain.fk(jog.q)
    result = jog.dispatch('step', 'session', body)
    assert result['speed_mm_s'] == speed_mm_s
    for _ in range(int(result['duration_s'] / .02) + 100):
        advance(jog, clock, body)
        if jog.active is None:
            break
    assert jog.active is None and not execution.motion.fault
    assert execution.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    pose = jog.planner.chain.fk(jog.q)
    np.testing.assert_allclose(pose[:3, 3] - start[:3, 3], [.005, 0, 0], atol=1e-7)
    np.testing.assert_allclose(pose[:3, :3], start[:3, :3], atol=1e-7)


def test_v3_axis_identity():
    assert AXIS_SET_HASH == list(bytes.fromhex('f4c8ff8a32183d1733032494600c9f5d1e7e625673a4ad51556b3ac6c35adee4'))
    assert JOINT_NAMES[:7] == tuple(f'right_joint{i}' for i in range(1, 8))


def test_windows_are_bounded_acknowledged_and_hold_the_other_arm(rolling):
    jog, execution, clock, body = rolling
    initial = jog.q.copy()
    jog.dispatch('step', 'session', body)
    advance(jog, clock, body, 80)
    transport = execution.transport
    assert jog.active is None and jog.armed
    assert transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    assert transport.status == 0 and len(transport.batches) > 5
    assert [b['sequence'] for b in transport.batches] == list(range(1, len(transport.batches) + 1))
    assert len({bytes(b['session_id']['uuid']) for b in transport.batches}) == 1
    for batch in transport.batches:
        assert 2 <= len(batch['points']) <= 32
        assert batch['replace_from_ns'] == batch['points'][0]['time_from_session_start_ns']
        times = [p['time_from_session_start_ns'] for p in batch['points']]
        assert all(a < b for a, b in zip(times, times[1:]))
        for point in batch['points']:
            assert len(point['positions']) == len(point['velocities']) == 14
            np.testing.assert_array_equal(point['positions'][:7], initial + .1)
            np.testing.assert_array_equal(point['velocities'][:7], np.zeros(7))
    np.testing.assert_allclose(jog.q[:3] - initial[:3], [.0005, 0, 0], atol=1e-8)
    # A later step uses a new session only after FINALIZE completed.
    old_session = execution.motion.identity['session_id']
    jog.dispatch('step', 'session', {**body, 'seq': 2})
    assert execution.motion.identity['session_id'] != old_session


def test_release_stops_with_controller_braking_then_finalizes(rolling):
    jog, execution, clock, body = rolling
    jog.dispatch('step', 'session', body)
    advance(jog, clock, body, 12)
    start_q, start_v = jog.q.copy(), execution.motion.last_v.copy()
    assert np.linalg.norm(start_v) > 0
    response = jog.dispatch('halt', 'session', {**body, 'seq': 2})
    assert response['epoch'] != body['epoch'] and jog.stopping
    stop_request = execution.motion.stop_request.copy()
    execution.motion.brake(0)
    assert execution.motion.stop_request == stop_request
    advance(jog, clock, {**body, 'epoch': response['epoch']}, 20)
    assert jog.active is None and jog.armed and not jog.stopping
    assert execution.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    assert np.linalg.norm(jog.q - start_q) > 0
    assert np.linalg.norm(execution.motion.last_v) == 0


@pytest.mark.parametrize('fault', ['rejected', 'restart', 'late', 'missing_ack', 'stale_state', 'nonfinite'])
def test_protocol_fault_stops_and_never_resumes(rolling, monkeypatch, fault):
    jog, execution, clock, body = rolling
    jog.dispatch('step', 'session', body)
    transport = execution.transport
    original = transport.state
    def faulty_state():
        state = original()
        if fault == 'rejected':
            state['last_rejected_sequence'] = execution.motion.sequence
            state['last_reject'] = 18
        elif fault == 'restart':
            state['controller_boot_id'] = identifier()
        elif fault == 'late':
            state['replaceable_from_ns'] = execution.motion.until + 1
        elif fault == 'stale_state':
            raise JogError('rolling_state_stale', 'Rolling 状态超时')
        elif fault == 'nonfinite':
            state['desired_positions'][0] = float('nan')
        else:
            state['last_accepted_sequence'] = 0
        return state
    monkeypatch.setattr(transport, 'state', faulty_state)
    advance(jog, clock, body, 20)
    assert not jog.armed and execution.motion.fault
    assert transport.close_operations[0] == 'REQUEST_STOP'
    assert len(transport.batches) == 1
    count = len(transport.batches)
    advance(jog, clock, body, 20)
    assert len(transport.batches) == count


def test_input_timeout_stops_stream_without_browser_polling(rolling):
    jog, execution, clock, body = rolling
    jog.dispatch('step', 'session', body)
    for _ in range(30):
        clock.now += .02
        jog.tick()
    assert not jog.armed and jog.active is None
    assert execution.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']


def test_wrong_axis_identity_refuses_to_open(rolling, monkeypatch):
    jog, execution, clock, body = rolling
    plan = jog.planner.step(jog.q, 'x+', .0005, require_collision=False)
    transport = MockRollingTransport(np.r_[jog.q + .1, jog.q], .07, clock)
    original = transport.state
    monkeypatch.setattr(transport, 'state', lambda: {**original(), 'axis_set_hash': [0] * 32})
    with pytest.raises(JogError, match='身份不匹配'):
        RollingMotion(plan, transport, clock)
    assert transport.status == 0 and not transport.batches


def test_initialization_failure_closes_and_finalizes_the_open_session(rolling):
    jog, execution, clock, body = rolling
    plan = jog.planner.step(jog.q, 'x+', .0005, require_collision=False)
    transport = MockRollingTransport(np.r_[jog.q + .1, jog.q], .07, clock)
    transport.negotiated['buffer_capacity'] = 8
    with pytest.raises(JogError, match='协商窗口'):
        RollingMotion(plan, transport, clock)
    assert transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    assert transport.status == 0


def test_switch_arm_after_rolling_finalize_keeps_controller_boot_and_other_pose(rolling):
    jog, execution, clock, body = rolling
    jog.dispatch('step', 'session', body)
    advance(jog, clock, body, 80)
    boot, left_q = execution.transport.boot, jog.q.copy()
    jog.dispatch('stop', 'session', {})
    jog.dispatch('select_arm', 'session', {**body, 'arm': 'right'})
    epoch = jog.dispatch('arm', 'session', body)['epoch']
    body = {**body, 'epoch': epoch}
    jog.dispatch('step', 'session', body)
    advance(jog, clock, body, 80)
    assert execution.transport.boot == boot
    np.testing.assert_allclose(execution.transport.q[7:], left_q, atol=1e-12)


def test_http_rolling_lifecycle_and_publication_feedback(rolling):
    jog, execution, clock, body = rolling
    jog.dispatch('release', 'session', body)
    server = MockServer(('127.0.0.1', 0), jog)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        status, _, cookie = request(server, 'POST', '/api/login', {'token': 'mock'})
        assert status == 200
        identity = {'client': 'http-rolling-page'}
        assert request(server, 'POST', '/api/jog/acquire', identity, cookie)[0] == 200
        status, raw, _ = request(server, 'POST', '/api/jog/arm', identity, cookie)
        assert status == 200
        command = {**identity, 'epoch': json.loads(raw)['epoch'], 'seq': 1, 'direction': 'x+', 'step_mm': .5}
        assert request(server, 'POST', '/api/jog/step', command, cookie)[0] == 200
        for sequence in range(1, 9):
            clock.now += .02
            assert request(server, 'POST', '/api/jog/heartbeat', {**command, 'seq': sequence}, cookie)[0] == 200
        status, raw, _ = request(server, 'GET', '/api/jog/state?client=http-rolling-page', cookie=cookie)
        state = json.loads(raw)
        assert status == 200 and state['publication']['mode'] == 'rolling'
        assert state['publication']['sequence'] > 1 and state['hardware_connected'] is False
        status, raw, _ = request(server, 'POST', '/api/jog/halt', {**command, 'seq': 2}, cookie)
        assert status == 200
        clock.now += .1
        jog.tick()
        assert jog.active is None and execution.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)


@pytest.mark.parametrize('step_mm', [.5, 1, 2, 5, 10])
@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direction', ['x+', 'x-', 'y+', 'y-', 'z+', 'z-'])
def test_authoritative_model_rolling_paths(actual_chain, tmp_path, side, direction, step_mm):
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text(actual_chain.source_xml)
    config = json.loads((ROOT / 'config/mock.json').read_text())
    config['selected_arm'] = side
    jog = build_mock(urdf, config)
    clock = Clock()
    jog.clock = clock
    jog.execution.clock = clock
    body = {'client': 'page-00000001'}
    jog.dispatch('acquire', 'session', body)
    body.update(epoch=jog.dispatch('arm', 'session', body)['epoch'], seq=1, direction=direction, step_mm=step_mm)
    start = jog.planner.chain.fk(jog.q)
    # The default left-arm fixture exceeds the existing joint-travel bound
    # for 10 mm XY moves. Adding distance options must retain that rejection.
    if step_mm == 10 and side == 'left' and direction[0] in 'xy':
        with pytest.raises(JogError) as error:
            jog.dispatch('step', 'session', body)
        assert error.value.code == 'joint_jump'
        assert jog.active is None and not jog.planning
        np.testing.assert_allclose(jog.planner.chain.fk(jog.q), start)
        return
    jog.dispatch('step', 'session', body)
    max_error = 0.
    for _ in range(int(jog.active.duration / .02) + 100):
        advance(jog, clock, body)
        current = jog.planner.chain.fk(jog.q)
        delta = current[:3, 3] - start[:3, 3]
        axis = 'xyz'.index(direction[0])
        max_error = max(max_error, float(np.linalg.norm(np.delete(delta, axis))))
        assert Rotation.from_matrix(current[:3, :3] @ start[:3, :3].T).magnitude() < jog.planner.limits.orientation_tolerance
        if jog.active is None:
            break
    assert not jog.execution.motion.fault
    assert jog.active is None
    delta = jog.planner.chain.fk(jog.q)[:3, 3] - start[:3, 3]
    assert abs(delta['xyz'.index(direction[0])] - (step_mm / 1000 if direction[1] == '+' else -step_mm / 1000)) < 2e-7
    assert max_error < jog.planner.limits.position_tolerance


@pytest.mark.parametrize('speed_mm_s', [.5, 10])
@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direction', ['x+', 'x-', 'y+', 'y-', 'z+', 'z-'])
def test_authoritative_speed_profile_through_rolling(actual_chain, tmp_path, side, direction, speed_mm_s):
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text(actual_chain.source_xml)
    config = json.loads((ROOT / 'config/mock.json').read_text())
    config['selected_arm'] = side
    jog = build_mock(urdf, config)
    clock = Clock()
    jog.clock = jog.execution.clock = clock
    body = {'client': 'page-speed-tests'}
    jog.dispatch('acquire', 'session', body)
    body.update(epoch=jog.dispatch('arm', 'session', body)['epoch'], seq=1,
                direction=direction, speed_mm_s=speed_mm_s)
    start = jog.planner.chain.fk(jog.q)
    result = jog.dispatch('step', 'session', body)
    for _ in range(int(result['duration_s'] / .02) + 100):
        advance(jog, clock, body)
        if jog.active is None:
            break
    assert jog.active is None and not jog.execution.motion.fault
    assert jog.execution.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    assert max(jog.execution.motion.plan.ratios(samples=2001)) <= 1
    pose = jog.planner.chain.fk(jog.q)
    delta = np.zeros(3)
    delta['xyz'.index(direction[0])] = .005 if direction[1] == '+' else -.005
    np.testing.assert_allclose(pose[:3, 3] - start[:3, 3], delta, atol=2e-7)
    np.testing.assert_allclose(pose[:3, :3], start[:3, :3], atol=1e-6)


def test_speed_motion_release_stops_before_endpoint(rolling):
    jog, execution, clock, body = rolling
    body.pop('step_mm')
    body['speed_mm_s'] = 10
    start = jog.q.copy()
    jog.dispatch('step', 'session', body)
    advance(jog, clock, body, 15)
    body['epoch'] = jog.dispatch('halt', 'session', {**body, 'seq': 2})['epoch']
    advance(jog, clock, body, 50)
    assert jog.active is None and not execution.motion.fault
    assert 0 < jog.q[0] - start[0] < .005
    assert jog.state('session', body['client'])['tcp_command_speed_mm_s'] == 0
    assert execution.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
