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
    for q in ({},{'left_joint1':.3,'right_joint4':-.5}):
        a,b=frames(source,q),frames(robot,q)
        for link in [f'{side}_link{i}' for side in ('left','right') for i in range(1,8)]+['left_tool0','right_tool0']:
            np.testing.assert_allclose(np.linalg.inv(a['arm_carriage'])@a[link],np.linalg.inv(b['arm_carriage'])@b[link],atol=1e-12)



def test_approved_three_boxes_and_unchanged_arm_spheres():
    source=ROOT/'preview/chassis_head_20261124/config/curobo/fixed_body_boxes.yaml'
    assert source.read_bytes()==(ROOT/'config/curobo/fixed_body_boxes.yaml').read_bytes()
    boxes=yaml.safe_load(source.read_text())
    assert [b['name'] for b in boxes['cuboids']]==['base_box_001','base_box_002','base_box_004']
    assert boxes['frame']=='base_link'



def test_original_frame_vocabulary_has_no_cad_auxiliary_frames():
    source=ET.parse(ROOT/'config/carriage_assembly/assembled_reference.urdf').getroot()
    robot=model()
    assert {l.get('name') for l in robot.findall('link')} == {l.get('name') for l in source.findall('link')} - {'up_and_down','point_arm_carriage'} | {'chest_camera_left'}
    names={j.get('name') for j in robot.findall('joint')}
    assert not names & {'arm_carriage','arm_carriage_fixed','mount_to_arm_carriage','preview_mount_to_arm_carriage'}
    assert 'updown' in names
    lift=robot.find("joint[@name='updown']")
    assert lift.find('parent').get('link')=='base_link'
    assert lift.find('child').get('link')=='arm_carriage'
    link=robot.find("link[@name='arm_carriage']")
    assert all(link.find(tag) is not None for tag in ('visual','collision','inertial'))
