#!/usr/bin/env python3
"""Compose the reviewed analytic upper body with mechanical 958c49d base/wheels."""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = 'mechanical_958c49d'

def build():
    model = ET.parse(ROOT / 'urdf/robot_analytic_proxy.urdf.xacro').getroot()
    source = ET.parse(ROOT / f'model_sources/{SOURCE}/robot.urdf').getroot()
    names = {'base_link', *(f'{kind}{i}' for kind in ('steer', 'wheel') for i in range(1, 5))}
    model.remove(model.find("link[@name='base_link']"))
    for link in source.findall('link'):
        if link.get('name') in names:
            link = copy.deepcopy(link)
            for mesh in link.findall('.//mesh'):
                mesh.set('filename', f'package://robot_description/model_sources/{SOURCE}/' + mesh.get('filename'))
            if link.get('name') == 'base_link':
                for mesh in link.findall('visual/geometry/mesh'):
                    if mesh.get('filename').endswith('/part_001_solid_001.stl'):
                        mesh.set('filename', 'package://robot_description/model_sources/robot_v3_2_2/meshes/part_001_solid_001.stl')
            model.append(link)
    wheel_joints = []
    for joint in source.findall('joint'):
        if joint.find('child').get('link') in names:
            joint = copy.deepcopy(joint)
            # User-confirmed mechanical semantics; retain source axis and pivot position.
            if joint.get('name').startswith('steer'):
                # Parent base_link +Z: clockwise from above is negative yaw.
                rpy = joint.find('origin').get('rpy').split()
                rpy[2] = str(float(rpy[2]) - math.pi / 2)
                joint.find('origin').set('rpy', ' '.join(rpy))
                joint.find('limit').set('lower', str(-3 * math.pi / 4))
                joint.find('limit').set('upper', str(3 * math.pi / 4))
            elif joint.get('name').startswith('wheel'):
                joint.set('type', 'continuous')
                joint.find('limit').attrib.pop('lower', None)
                joint.find('limit').attrib.pop('upper', None)
                joint.find('limit').set('velocity', '31.4')
            model.append(joint)
            wheel_joints.append(joint)
    # Source wheel centers are z=-0.22 m with nominal tire radius 0.10 m.
    model.find("joint[@name='base_footprint_to_base_link']/origin").set('xyz', '0 0 0.32')
    model.find("joint[@name='base_footprint_to_base_link']/origin").set('rpy', f'0 0 {math.pi / 2}')
    ET.SubElement(model, 'link', name='imu_link')
    imu = ET.SubElement(model, 'joint', name='base_link_to_imu', type='fixed')
    ET.SubElement(imu, 'parent', link='base_link')
    ET.SubElement(imu, 'child', link='imu_link')
    ET.SubElement(imu, 'origin', xyz='0.403 0 0', rpy='0 0 0')
    ET.indent(model)
    ET.ElementTree(model).write(ROOT / 'urdf/robot_analytic_proxy_mobile.urdf', encoding='utf-8', xml_declaration=True)
    initial = yaml.safe_load((ROOT / 'config/initial_positions.yaml').read_text())
    initial['metadata'].update(model_revision='analytic-proxy-mobile', chassis_source=SOURCE,
                               integration_policy='analytic_upper_body_with_mechanical_958c49d_base')
    initial['initial_positions'].update({j.get('name'): 0.0 for j in wheel_joints})
    limits = yaml.safe_load((ROOT / 'config/joint_limits.yaml').read_text())
    limits['metadata'].update(model_revision='analytic-proxy-mobile', chassis_source=SOURCE)
    for joint in wheel_joints:
        limit = joint.find('limit')
        limits['joints'][joint.get('name')] = {
            'type': joint.get('type'), 'max_effort': float(limit.get('effort')),
            'max_velocity_rad_s': float(limit.get('velocity'))}
        if joint.get('type') != 'continuous':
            limits['joints'][joint.get('name')].update(lower_position_rad=float(limit.get('lower')), upper_position_rad=float(limit.get('upper')))
    (ROOT / 'config/joint_limits_analytic_proxy_mobile.yaml').write_text(yaml.safe_dump(limits, sort_keys=False))
    (ROOT / 'config/initial_positions_analytic_proxy_mobile.yaml').write_text(yaml.safe_dump(initial, sort_keys=False))
    manifest = {
        'entry': 'urdf/robot_analytic_proxy_mobile.urdf',
        'experimental': True, 'hardware_authoritative': False,
        'upper_body_source': 'urdf/robot_analytic_proxy.urdf.xacro',
        'upper_body_baseline_commit': 'cdbe27a8ab2ce01b06444661522b1713eeb3357e',
        'base_and_wheel_source': f'model_sources/{SOURCE}/robot.urdf',
        'source_sha256': '958c49d7889827c9f520ac858783da62ca5ad249eaac765d7d761bd51e543b09',
        'base_height_m': 0.32, 'nominal_wheel_radius_m': 0.1,
        'wheel_max_velocity_rad_s': 31.4,
        'steering_zero_heading_base_rad': -math.pi / 2,
        'control_frame': 'base_footprint',
        'base_link_yaw_in_control_frame_rad': math.pi / 2,
        'steering_zero_heading_evidence': 'User confirmed mechanical zero toward original vehicle right; clockwise90deg about base_link +Z viewed from above.',
        'imu_parent': 'base_link', 'imu_xyz_m': [0.403, 0, 0], 'imu_rpy_rad': [0, 0, 0],
        'imu_evidence': f'model_sources/{SOURCE}/imu-frame-confirmation.md',
        'base_visual_shell': 'model_sources/robot_v3_2_2/meshes/part_001_solid_001.stl',
        'composition': 'Use mechanical base inertia/collision and wheel links/joints; restore the previous enclosed base visual mesh only. Retain every analytic upper-body link and joint relative to base_link.',
        'limitations': ['Hybrid visualization/planning model; upper-body to new base assembly needs physical review.',
                         'No independent active suspension in the selected mechanical source.',
                         'No IMU shape/mass invented; driver ENU/NED convention is not specified here.',
                         'User-confirmed wheel continuous joints and steering q limits +/-135 deg; steering zero rotated clockwise90deg about base_link Z. Source axes and pivot positions retained. Controller heading offsets, calibration and operating limits require alignment to this zero reference.',
                         'Wheel velocity limit is user-confirmed 31.4 rad/s; source effort and non-wheel velocity limits remain unverified. Model changes do not constitute hardware commissioning.']
    }
    (ROOT / 'config/analytic_proxy_mobile_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

if __name__ == '__main__':
    build()
