"""Approved mounting geometry, lift datum and collision ownership contracts."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from model_geometry import frames
POLICY=json.loads((ROOT/'config/carriage_assembly/approval.json').read_text())


def model():
    return ET.parse(ROOT/'urdf/robot_alfav3_mobile.urdf').getroot()


def test_j1_reverse_axes_meet_at_cad_installation_point():
    robot=model()
    for q in (0., -.5, -1., .017):
        pose=frames(robot,{'updown':q})
        target=np.array([.443,0,1.66774990748])+pose['arm_carriage'][:3,:3]@np.array([0,0,-q])
        for side in ('left','right'):
            shoulder=pose[side+'_link1']
            delta=target-shoulder[:3,3]
            length=delta@shoulder[:3,2]
            assert length<0
            np.testing.assert_allclose(delta,length*shoulder[:3,2],atol=1e-10)


def test_lift_moves_whole_upper_assembly_not_chassis_or_wheels():
    robot=model()
    zero=frames(robot)
    assert POLICY['lift_reference_reading_m']==0.
    np.testing.assert_allclose(zero['base_link'][:3,3],[.07478297093,0,.37],atol=1e-12)
    assert 'up_and_down' not in zero and 'point_arm_carriage' not in zero
    assert robot.find("joint[@name='updown']/child").get('link') == 'arm_carriage'
    for q in (-1.,-.3,.017):
        current=frames(robot,{'updown':q})
        for link in ('arm_carriage','left_link1','right_link1','left_tool0','right_tool0','H1_Link','mid360_Link'):
            np.testing.assert_allclose(current[link][:3,3]-zero[link][:3,3],[0,0,q],atol=1e-7)
            np.testing.assert_allclose(current[link][:3,:3],zero[link][:3,:3],atol=1e-12)
        for link in ('base_link','steer1','steer2','steer3','steer4','wheel1','wheel2','wheel3','wheel4'):
            np.testing.assert_allclose(current[link],zero[link],atol=1e-12)


def test_upper_body_relative_kinematics_and_wheel_mount_alignment():
    source=ET.parse(ROOT/'config/carriage_assembly/assembled_reference.urdf').getroot()
    robot=model()
    rng=np.random.default_rng(24)
    for _ in range(8):
        q={j.get('name'):rng.uniform(-.4,.4) for j in robot.findall('joint') if j.get('type')=='revolute'}
        old,new=frames(source,q),frames(robot,q)
        for name in ('left_link1','right_link1','left_tool0','right_tool0','H1_Link','H2_Link','mid360_Link'):
            np.testing.assert_allclose(np.linalg.inv(old['arm_carriage'])@old[name],np.linalg.inv(new['arm_carriage'])@new[name],atol=1e-12)
        for i in range(1,5):
            quarter_turn=np.array([[0,-1,0],[1,0,0],[0,0,1]])
            steer=f'steer{i}'
            for name in (steer,f'wheel{i}'):
                expected_position=old[steer][:3,3]+[0,0,.035]+quarter_turn@(old[name][:3,3]-old[steer][:3,3])
                np.testing.assert_allclose(new[name][:3,3],expected_position,atol=1e-10)
                np.testing.assert_allclose(new[name][:3,:3],quarter_turn@old[name][:3,:3],atol=1e-12)


def test_approved_four_boxes_and_unchanged_spheres():
    original=ROOT/'config/carriage_assembly/approved_fixed_body_boxes.yaml'
    assert original.read_bytes()==(ROOT/'config/curobo/fixed_body_boxes.yaml').read_bytes()
    boxes=yaml.safe_load(original.read_text())
    assert boxes['frame']=='base_link'
    expected=[([-.074,0,-.102],[1,1,.37]),([-.2,0,.2],[.7,1,.4]),([-.33,0,.39],[.35,1,.21]),([.023,0,.796],[.294,.4,1.59])]
    for box,(center,size) in zip(boxes['cuboids'],expected):
        np.testing.assert_allclose(box['center'],center,atol=1e-12)
        np.testing.assert_allclose(box['dimensions'],size,atol=1e-12)
        np.testing.assert_allclose(box['rpy_rad'],0,atol=1e-12)
    # The exact approved sphere file predates assembly migration; no radius/center refit.
    assert hashlib.sha256((ROOT/'config/curobo/collision_spheres.yaml').read_bytes()).hexdigest()=='f57f7a7f23e7f5f7161b59c06eeb6c62fc85f5a77173bf7f89afd15a50c1fca9'


def test_original_frame_vocabulary_has_no_cad_auxiliary_frames():
    source=ET.parse(ROOT/'config/carriage_assembly/assembled_reference.urdf').getroot()
    robot=model()
    assert {l.get('name') for l in robot.findall('link')} == {l.get('name') for l in source.findall('link')} - {'up_and_down','point_arm_carriage'}
    names={j.get('name') for j in robot.findall('joint')}
    assert not names & {'arm_carriage','arm_carriage_fixed','mount_to_arm_carriage','preview_mount_to_arm_carriage'}
    assert 'updown' in names
    lift=robot.find("joint[@name='updown']")
    assert lift.find('parent').get('link')=='base_link'
    assert lift.find('child').get('link')=='arm_carriage'
    link=robot.find("link[@name='arm_carriage']")
    assert all(link.find(tag) is not None for tag in ('visual','collision','inertial'))
