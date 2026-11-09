import json
from pathlib import Path

import numpy as np
import pytest

from gripper_jog.control import MockJog
from gripper_jog.errors import JogError
from gripper_jog.model import Chain
from gripper_jog.planner import Planner
from gripper_jog.server import build_mock, ROOT
from test_control import Clock


@pytest.fixture
def dual(xml, q):
    # Two disjoint chains attached to the same base, without copying robot geometry.
    right = xml[xml.index('<link name="l1"'):xml.index('</robot>')]
    for i in range(1, 8):
        right = right.replace(f'"l{i}"', f'"r{i}"').replace(f'"j{i}"', f'"k{i}"')
    source = xml.replace('</robot>', right + '</robot>')
    left, right = Planner(Chain(source, 'base', 'l7')), Planner(Chain(source, 'base', 'r7'))
    clock = Clock()
    jog = MockJog(left, q, clock=clock, arms={'left': (left, q), 'right': (right, q)}, selected_arm='left')
    body = {'client': 'page-00000001'}
    return jog, clock, body


def test_switch_requires_owner_and_idle_disarmed_state(dual):
    jog, clock, body = dual
    with pytest.raises(JogError) as e:
        jog.dispatch('select_arm', 'session', {**body, 'arm': 'right'})
    assert e.value.code == 'lease_required'
    jog.dispatch('acquire', 'session', body)
    epoch = jog.dispatch('arm', 'session', body)['epoch']
    command = {**body, 'epoch': epoch, 'seq': 1, 'direction': 'x+', 'step_mm': .5}
    with pytest.raises(JogError) as e:
        jog.dispatch('select_arm', 'session', {**body, 'arm': 'right'})
    assert e.value.code == 'not_ready'
    jog.dispatch('step', 'session', command)
    clock.now += .2
    jog.dispatch('stop', 'session', {})
    with pytest.raises(JogError) as e:
        jog.dispatch('select_arm', 'session', {**body, 'arm': 'right'})
    assert e.value.code == 'not_ready'
    clock.now += 1
    jog.dispatch('select_arm', 'session', {**body, 'arm': 'right'})
    state = jog.state('session', body['client'])
    assert state['selected_arm'] == 'right' and not state['armed']
    assert state['joint_names'] == [f'k{i}' for i in range(1, 8)]
    with pytest.raises(JogError) as e:
        jog.dispatch('step', 'session', command)
    assert e.value.code == 'epoch_expired'


def test_switch_keeps_each_arm_pose_and_feedback_separate(dual):
    jog, clock, body = dual
    initial = jog.q.copy()
    jog.dispatch('acquire', 'session', body)
    epoch = jog.dispatch('arm', 'session', body)['epoch']
    jog.dispatch('step', 'session', {**body, 'epoch': epoch, 'seq': 1, 'direction': 'x+', 'step_mm': .5})
    clock.now += .2
    jog.dispatch('stop', 'session', {})
    clock.now += 1
    jog.tick()
    left_end = jog.q.copy()
    assert np.linalg.norm(left_end - initial) > 0
    jog.dispatch('select_arm', 'session', {**body, 'arm': 'right'})
    np.testing.assert_array_equal(jog.q, initial)
    state = jog.state('session', body['client'])
    assert len(state['model_joint_names']) == 14
    np.testing.assert_array_equal(state['model_q_rad'][:7], left_end)
    np.testing.assert_array_equal(state['model_q_rad'][7:], initial)
    assert state['last_plan'] is None
    with pytest.raises(JogError):
        jog.dispatch('select_arm', 'session', {**body, 'arm': 'unknown'})
    jog.dispatch('select_arm', 'session', {**body, 'arm': 'left'})
    np.testing.assert_array_equal(jog.q, left_end)


@pytest.mark.parametrize('direction', ['x+', 'x-', 'y+', 'y-', 'z+', 'z-'])
def test_actual_right_arm_config_and_six_directions(actual_chain, direction):
    import tempfile
    with tempfile.TemporaryDirectory() as folder:
        source = Path(folder) / 'robot.urdf'
        source.write_text(actual_chain.source_xml)
        mock = build_mock(source, json.loads((ROOT/'config/mock.json').read_text()))
    body = {'client': 'page-00000001'}
    mock.dispatch('acquire', 'session', body)
    mock.dispatch('select_arm', 'session', {**body, 'arm': 'right'})
    assert mock.planner.chain.tip == 'right_tool0'
    assert all(name.startswith('right_') for name in mock.planner.chain.names)
    plan = mock.planner.step(mock.q, direction, .0005, require_collision=False)
    delta = mock.planner.chain.fk(plan.state(plan.duration)[0])[:3, 3] - mock.planner.chain.fk(mock.q)[:3, 3]
    expected = np.zeros(3)
    expected['xyz'.index(direction[0])] = .0005 * (1 if direction[1] == '+' else -1)
    np.testing.assert_allclose(delta, expected, atol=2e-7)
