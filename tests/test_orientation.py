import asyncio
import math
import time
import xml.etree.ElementTree as ET
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from gripper_jog.cartesian_input import velocity_command
from gripper_jog.errors import JogError
from gripper_jog.real_keyboard import RealKeyboard
from gripper_jog.real_model import ArtifactMink
from gripper_jog.real_source import RealRollingSource
from gripper_jog.model import Chain
from gripper_jog.real_observe import ReadOnlyJog
from test_real_collision import collision_model, artifact_model, observer, POLICY  # noqa: F401
from test_current_stream import PARAMETERS, snapshot
from gripper_jog.current_stream import CurrentStream


MAPPING = {a+s: a+('-' if s=='+' else '+') for a in 'xyz' for s in '+-'}


@pytest.mark.parametrize('rotation,axis', [('yaw','z'), ('pitch','y'), ('roll','x')])
@pytest.mark.parametrize('sign', ['+', '-'])
def test_j7_rotation_ignores_body_axis_mapping(rotation, axis, sign):
    v, command = velocity_command({'robot_rotation': rotation+sign, 'speed_deg_s': 1}, MAPPING)
    assert command['rotation_axis'] == axis+sign
    assert command['rotation_frame'] == 'j7'
    expected = np.zeros(6)
    expected[3+'xyz'.index(axis)] = math.radians(1) * (1 if sign=='+' else -1)
    assert np.array_equal(v, expected)


@pytest.mark.parametrize('extra', [dict(robot_direction='x+'), dict(speed_mm_s=1),
    dict(direction='z+'), dict(step_mm=1), dict(rotation_frame='base_link'),
    dict(speed_deg_s=True), dict(speed_deg_s=float('nan')), dict(speed_deg_s=100)])
def test_ambiguous_or_invalid_rotation_rejected(extra):
    with pytest.raises(JogError):
        velocity_command({'robot_rotation':'yaw+', 'speed_deg_s':1, **extra}, MAPPING)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('speed', [.5, 1, 2, 5])
def test_j7_local_rotation_holds_offset_tcp_and_preserves_continuous_seams(collision_model, side, axis, sign, speed):
    original, positions = collision_model
    root = ET.fromstring(original.xml)
    # Rotated J7 and a nontrivial tool transform distinguish J7, TCP and base axes.
    fixed = root.find(f"joint[@name='{side}_tip']")
    ET.SubElement(fixed, 'origin', xyz='.04 .02 .03', rpy='.3 -.2 .4')
    model = ArtifactMink(ET.tostring(root, encoding='unicode'), original.parameters, collision_policy=POLICY)
    positions = positions | {side+'_joint4':.25, side+'_joint5':-.2, side+'_joint6':.3}
    source = RealRollingSource(model, positions, side)
    chain = model.chains[side]
    assert chain.rotation_frame == side+'_link7'
    initial = source.frame.as_matrix()
    initial_j7 = source.config.get_transform_frame_to_world(chain.rotation_frame, 'body').rotation().as_matrix()
    velocity = np.zeros(6)
    velocity[3+axis] = sign * math.radians(speed)
    stream = CurrentStream(source, PARAMETERS, 1_000_000_000)
    stream.acknowledge(snapshot())
    now, seq = 880_000_000, 0
    for _ in range(16):
        old = list(stream.committed)
        window = stream.propose(now, velocity)
        seq += 1
        stream.sent({'send_result':0, 'rolling':snapshot(seq)})
        stream.acknowledge(snapshot(seq))
        if old:
            common = next(k for k in old if k.time_ns == stream.committed[-21].time_ns)
            for key in ('q','v','a'):
                assert np.array_equal(getattr(common,key), getattr(stream.committed[-21],key))
        assert len(stream.committed)<64
        now += 40_000_000
    # Accumulated body-frame rotation aligns with J7, despite a rotated TCP.
    end = stream.committed[-1]
    cfg = model.configuration(dict(zip(model.names,end.q)))
    pose = cfg.get_transform_frame_to_world(chain.tip,'body').as_matrix()
    j7 = cfg.get_transform_frame_to_world(chain.rotation_frame,'body').rotation().as_matrix()
    delta = Rotation.from_matrix(initial_j7.T @ j7).as_rotvec()
    assert sign * delta[axis] > .001
    # Solver tracking tolerance at 20 ms, independent of hardware limits.
    assert np.linalg.norm(np.delete(delta,axis)) < math.radians(.002)
    assert np.linalg.norm(pose[:3,3]-initial[:3,3]) < 5e-5
    frozen = [i for i,n in enumerate(model.names) if n not in chain.names]
    assert np.array_equal(end.q[frozen], source.initial[frozen])
    # Final windows still decelerate to zero and pass full cubic/collision checks.
    while not stream.final_sent:
        window = stream.propose(now, velocity, finishing=True)
        seq += 1
        stream.sent({'send_result':0,'rolling':snapshot(seq)})
        stream.acknowledge(snapshot(seq)); now+=40_000_000
    assert window['points'][-1]['velocities'] == [0.]*14


@pytest.mark.parametrize('rotation', ['yaw+','yaw-','pitch+','pitch-','roll+','roll-'])
def test_keyboard_rotation_reaches_real_window_transport(collision_model, observer, rotation):
    model, positions = collision_model
    keyboard = RealKeyboard(model, observer[0])
    entry = NS(compatible=True,received_at=time.monotonic(),status={'lease':'you',
        'runtime_identity':{'joint_names':list(model.names)}})
    keyboard.entry, keyboard.epoch, keyboard.input_at = entry, 'input', time.monotonic()
    async def scenario():
        published=[]
        async def rpc(kind, body):
            if kind=='begin_motion':
                return {'positions':positions,'hold_reference':{'positions':positions},
                    'parameters':model.parameters|PARAMETERS,
                    'rolling':snapshot(phase=0)|{'ros_time_ns':1_000_000_000}}
            if kind=='window':
                published.append(body['window']);raise asyncio.CancelledError
            return {}
        result=await keyboard.command('velocity',entry,{'epoch':'input','seq':1,
            'robot_rotation':rotation,'rotation_frame':'j7','speed_deg_s':1},rpc)
        assert result['rotation_frame']=='j7' and result['direction']==rotation
        try:await keyboard.task
        except asyncio.CancelledError:pass
        assert len(published)==1, keyboard.fault
        assert published[0]['points'][0]['positions']==[positions[n] for n in model.names]
    asyncio.run(scenario())


def test_feedback_uses_j7_quaternion_and_measured_angular_velocity(observer):
    jog, sample, clock=observer
    assert jog.ingest(sample)
    for side in ('left','right'):
        jog.dispatch('select_arm','',{'arm':side})
        state=jog.state('','')
        assert state['rotation_frame']==side+'_link7'
        q=[jog.positions[n] for n in jog.planner.chain.names]
        pose,jac=jog.planner.chain.fk_jacobian(q)
        assert state['tcp_actual_angular_speed_deg_s']==pytest.approx(
            np.rad2deg(np.linalg.norm(jac[3:] @ [jog.velocities[n] for n in jog.planner.chain.names])))
    clock[0]+=1
    assert 'tcp_actual_angular_speed_deg_s' not in jog.state('','')


def test_feedback_removes_fixed_tcp_rotation_from_j7_axes(observer):
    jog,sample,_=observer
    root=ET.fromstring(jog.planner.chain.source_xml)
    for side in ('left','right'):
        ET.SubElement(root.find(f"joint[@name='{side}_tip']"),'origin',xyz='.02 .03 .1',rpy='.3 -.2 .4')
    xml=ET.tostring(root,encoding='unicode')
    changed=ReadOnlyJog(xml,78)
    assert changed.ingest(sample|{'model_sha256':changed.planner.chain.model_hash})
    for side in ('left','right'):
        changed.dispatch('select_arm','',{'arm':side})
        state=changed.state('','')
        j7=Chain(xml,'arm_carriage',side+'_link7',margin=0)
        q=[changed.positions[n] for n in j7.names]
        actual=Rotation.from_quat(state['rotation_orientation_xyzw']).as_matrix()
        assert np.max(np.abs(actual-j7.fk(q)[:3,:3])) < 1e-12
