#!/usr/bin/env python3
"""Validate a portable robot bundle against actual cuRobo GPU FK/spheres.

This verifies coordinate consistency, not sphere-envelope coverage, planner
success or the separate fixed-body box collision backend.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from model_geometry import frames


def load_config(root):
    """Resolve package-relative assets without changing tracked YAML."""
    root = Path(root).resolve()
    robot = yaml.safe_load((root/'config/curobo/robot.yaml').read_text())
    k = robot['kinematics']
    k['urdf_path'] = str(root/k['urdf_path'])
    k['asset_root_path'] = str(root)
    spheres = root/'config/curobo'/k['collision_spheres']
    k['collision_spheres'] = yaml.safe_load(spheres.read_text())['collision_spheres']
    return robot


def validate(root, samples, all_joints=False):
    import torch
    from curobo.kinematics import Kinematics, KinematicsCfg
    from curobo._src.state.state_joint import JointState
    cfg = load_config(root)
    if all_joints:
        # Diagnostic-only full state: exercise head and steering/wheel joints
        # as well. The published planning preset remains 15-axis with locks.
        cfg['kinematics']['lock_joints'] = {}
        cfg['kinematics'].pop('cspace', None)
    kin = Kinematics(KinematicsCfg.from_robot_yaml_file(cfg))
    names = list(kin.joint_names)
    source = ET.parse(root/'urdf/robot_alfav3_mobile.urdf').getroot()
    joints = {j.get('name'):j for j in source.findall('joint')}
    rng = np.random.default_rng(20261112)
    qs = np.array([[rng.uniform(-np.pi,np.pi) if joints[n].get('type')=='continuous' else
                    rng.uniform(float(joints[n].find('limit').get('lower')),
                                float(joints[n].find('limit').get('upper'))) for n in names]
                   for _ in range(samples)])
    qs[0] = 0
    state = kin.compute_kinematics(JointState.from_position(
        torch.tensor(qs, dtype=torch.float32, device='cuda'), joint_names=names))
    observed_spheres = state.get_link_spheres().detach().cpu().numpy()[:,0]
    params = kin.config.kinematics_config
    max_error = 0.0
    checked = 0
    for i,q in enumerate(qs):
        fk = frames(source, dict(zip(names,q)))
        for link,spheres in cfg['kinematics']['collision_spheres'].items():
            indices = params.get_sphere_index_from_link_name(link).detach().cpu().numpy()
            t = fk[link]
            expected = np.array([[*(t[:3,:3]@s['center']+t[:3,3]),s['radius']] for s in spheres])
            actual = observed_spheres[i,indices]
            assert expected.shape == actual.shape, (link,expected.shape,actual.shape)
            max_error = max(max_error,float(np.max(np.abs(expected-actual))))
            checked += len(spheres)
    tool_error = 0.0
    rotation_error = 0.0
    # Query the stored GPU tool poses via the supported link-pose API.
    for link in cfg['kinematics']['tool_frames']:
        pose = kin.get_link_poses(torch.tensor(qs,dtype=torch.float32,device='cuda'),[link])
        matrices = np.tile(np.eye(4), (samples,1,1))
        matrices[:,:3,3] = pose.position.detach().cpu().numpy().reshape(samples,3)
        wxyz = pose.quaternion.detach().cpu().numpy().reshape(samples,4)
        matrices[:,:3,:3] = Rotation.from_quat(wxyz[:,[1,2,3,0]]).as_matrix()
        for i,q in enumerate(qs):
            expected = frames(source,dict(zip(names,q)))[link]
            tool_error = max(tool_error,float(np.max(np.abs(expected[:3,3]-matrices[i,:3,3]))))
            rotation_error = max(rotation_error,float(np.max(np.abs(expected[:3,:3]-matrices[i,:3,:3]))))
    assert max_error < 1e-5, max_error
    assert tool_error < 1e-5 and rotation_error < 1e-5, (tool_error,rotation_error)
    return {'status':'passed','samples':samples,'dof':len(names), 'sphere_checks':checked,
            'mode':'all_joints_diagnostic' if all_joints else 'published_planning_preset',
            'queried_frames':cfg['kinematics']['tool_frames'],
            'maximum_sphere_component_error_m':max_error,'maximum_tool_translation_error_m':tool_error,
            'maximum_tool_rotation_matrix_error':rotation_error,
            'scope':'GPU FK and sphere transforms vs independent CPU URDF FK; not collision coverage or planning acceptance'}


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--samples',type=int,default=64)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--all-joints',action='store_true',help='Diagnostic: unlock head/wheels to verify the entire URDF; does not alter stored planning config')
    args=parser.parse_args()
    if args.samples<1:parser.error('samples must be positive')
    result=validate(args.root.resolve(),args.samples,args.all_joints)
    data=json.dumps(result,indent=2)+'\n'
    print(data)
    if args.output:args.output.write_text(data)
