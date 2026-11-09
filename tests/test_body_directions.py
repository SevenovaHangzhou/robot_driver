import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from gripper_jog.control import MockJog
from gripper_jog.errors import JogError
from gripper_jog.model import Chain, robot_direction_map
from gripper_jog.planner import DIRECTIONS, Planner
from gripper_jog.server import ROOT, build_mock
from test_control import Clock


def with_body(xml, kind='prismatic', rpy='3.141592653589793 0 0'):
    return xml.replace('<link name="base"/>', f'''<link name="body"/><link name="base"/>
        <joint name="lift" type="{kind}"><parent link="body"/><child link="base"/>
        <origin rpy="{rpy}"/><axis xyz="0 0 1"/></joint>''')


def test_body_map_flips_carriage_y_and_z_but_not_x(xml):
    mapping = robot_direction_map(with_body(xml), 'body', 'base')
    assert mapping == {'x+': 'x+', 'x-': 'x-', 'y+': 'y-', 'y-': 'y+', 'z+': 'z-', 'z-': 'z+'}


@pytest.mark.parametrize('kind,rpy', [('revolute', '0 0 0'), ('fixed', '0 0 .4')])
def test_unsupported_direction_frames_are_rejected(xml, kind, rpy):
    with pytest.raises(JogError) as error:
        robot_direction_map(with_body(xml, kind, rpy), 'body', 'base')
    assert error.value.code == 'direction_frame_invalid'


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direction', DIRECTIONS)
def test_body_directions_move_both_arms_in_same_body_direction(xml, q, side, direction):
    source = with_body(xml)
    planner = Planner(Chain(source, 'base', 'l7'))
    jog = MockJog(planner, q, clock=Clock(), arms={'left': (planner, q), 'right': (planner, q)},
                  direction_frame='body', selected_arm='left')
    body = {'client': 'page-00000001'}
    jog.dispatch('acquire', 'session', body)
    if side == 'right':
        jog.dispatch('select_arm', 'session', {**body, 'arm': side})
    epoch = jog.dispatch('arm', 'session', body)['epoch']
    start = planner.chain.fk(jog.q)
    result = jog.dispatch('step', 'session', {**body, 'epoch': epoch, 'seq': 1,
                                           'robot_direction': direction, 'step_mm': .5})
    local_delta = planner.chain.fk(result['target_q'])[:3, 3] - start[:3, 3]
    body_delta = np.diag([1., -1., -1.]) @ local_delta
    np.testing.assert_allclose(body_delta, np.array(DIRECTIONS[direction]) * .0005, atol=2e-7)
    state = jog.state('session', body['client'])
    assert state['direction_frame'] == 'body'
    assert state['direction_mapping'][direction] == result['direction']


@pytest.mark.parametrize('direction', [None, 'wrong', [], True])
def test_invalid_body_direction_never_starts(planner, q, direction):
    jog = MockJog(planner, q, clock=Clock())
    body = {'client': 'page-00000001'}
    jog.dispatch('acquire', 'session', body)
    epoch = jog.dispatch('arm', 'session', body)['epoch']
    with pytest.raises(JogError) as error:
        jog.dispatch('step', 'session', {**body, 'epoch': epoch, 'seq': 1,
                                       'robot_direction': direction, 'step_mm': .5})
    assert error.value.code == 'bad_direction'
    assert jog.active is None and not jog.planning


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direction', DIRECTIONS)
def test_authoritative_robot_body_directions(actual_chain, tmp_path, side, direction):
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text(actual_chain.source_xml)
    config = json.loads((ROOT / 'config/mock.json').read_text())
    config['selected_arm'] = side
    jog = build_mock(urdf, config)
    jog.clock = Clock()
    jog.execution.clock = jog.clock
    body = {'client': 'page-00000001'}
    jog.dispatch('acquire', 'session', body)
    epoch = jog.dispatch('arm', 'session', body)['epoch']
    start = jog.planner.chain.fk(jog.q)
    result = jog.dispatch('step', 'session', {**body, 'epoch': epoch, 'seq': 1,
                                           'robot_direction': direction, 'step_mm': .5})
    local_delta = jog.planner.chain.fk(result['target_q'])[:3, 3] - start[:3, 3]
    root = ET.fromstring(actual_chain.source_xml)
    lift = root.find("joint[@name='updown']")
    rotation = Rotation.from_euler('xyz', [float(x) for x in lift.find('origin').get('rpy').split()]).as_matrix()
    np.testing.assert_allclose(rotation @ local_delta, np.array(DIRECTIONS[direction]) * .0005, atol=2e-7)
    # The named left arm must be on the positive body-Y side of the right arm.
    left = np.fromstring(root.find("joint[@name='left_joint1']/origin").get('xyz'), sep=' ')
    right = np.fromstring(root.find("joint[@name='right_joint1']/origin").get('xyz'), sep=' ')
    assert (rotation @ (left - right))[1] > 0
