#!/usr/bin/env python3
"""Rebuild the approved carriage assembly using checked-in source and calibration.

No network or live robot access. Hardware travel remains in description; Motion's
[-1,0] envelope is a separate consumer constraint. Refuse an unknown lift datum.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from model_geometry import frames, transform, set_origin

PREFIX = 'package://robot_description/'


def write_xml(path, model):
    ET.indent(model, space='  ')
    ET.ElementTree(model).write(path, encoding='utf-8', xml_declaration=True)
    with path.open('ab') as stream:
        stream.write(b'\n')


def collapse_assembly_frames(model):
    """Remove CAD-only frames while preserving geometry, inertia and joint motion."""
    fk = frames(model)
    carriage = model.find("link[@name='arm_carriage']")
    body = model.find("link[@name='up_and_down']")
    if len(carriage):
        raise ValueError('Expected empty upper-body frame before merging CAD geometry')
    carriage_from_body = np.linalg.inv(fk['arm_carriage']) @ fk['up_and_down']
    for element in body:
        if element.tag not in ('visual', 'collision', 'inertial'):
            raise ValueError('Unexpected carriage element: '+element.tag)
        moved = deepcopy(element)
        # Inertia tensor remains in its inertial frame; transform its origin,
        # including orientation, rather than rotating the tensor a second time.
        set_origin(moved, carriage_from_body @ transform(element.find('origin')))
        carriage.append(moved)
    lift = model.find("joint[@name='updown']")
    old_origin = transform(lift.find('origin'))
    axis_in_base = old_origin[:3,:3] @ np.fromstring(lift.find('axis').get('xyz'), sep=' ')
    base_from_carriage = np.linalg.inv(fk['base_link']) @ fk['arm_carriage']
    set_origin(lift, base_from_carriage)
    lift.find('child').set('link', 'arm_carriage')
    axis = base_from_carriage[:3,:3].T @ axis_in_base
    # The source uses Rx(3.1415927), a rounded pi. Its transformed +Z
    # has a ~4.6e-8 Y component; cuRobo requires cardinal prismatic axes.
    # Retain the original main's exact local -Z axis (world drift <50 nm/m).
    if not np.allclose(axis, [0,0,-1], atol=1e-7, rtol=0):
        raise ValueError('Lift axis is not compatible with original -Z contract')
    lift.find('axis').set('xyz', '0 0 -1')
    for name in ('arm_carriage_fixed', 'mount_to_arm_carriage'):
        model.remove(model.find(f"joint[@name='{name}']"))
    for name in ('up_and_down', 'point_arm_carriage'):
        model.remove(model.find(f"link[@name='{name}']"))


def build(root):
    root = Path(root)
    inputs = root / 'config/carriage_assembly'
    policy = json.loads((inputs / 'approval.json').read_text())
    reading = policy['lift_reference_reading_m']
    if reading is None or not np.isfinite(reading):
        raise ValueError('Assembly lift reference reading must be confirmed before publishing')
    source = (inputs / 'assembled_reference.urdf').read_bytes()
    if hashlib.sha256(source).hexdigest() != policy['source_urdf_sha256']:
        raise ValueError('Assembly source checksum mismatch')
    model = ET.fromstring(source)
    model.find("joint[@name='base_footprint_to_base_link']/origin").set('xyz', ' '.join(map(str, policy['base_xyz_m'])))
    for i in range(1, 5):
        origin = model.find(f"joint[@name='steer{i}_joint']/origin")
        xyz = np.fromstring(origin.get('xyz'), sep=' ')
        # Fixed mounting correction: zero steering and positive wheel rate roll along footprint +X.
        # Steering remains clockwise about footprint Z (heading=-q); no encoder zero is changed.
        roll, pitch, yaw = origin.get('rpy').split()
        origin.set('rpy', f'{roll} {pitch} {float(yaw) + np.pi/2:.17g}')
        xyz[0] += policy['wheel_mount_x_compensation_m']
        origin.set('xyz', ' '.join(format(v, '.17g') for v in xyz))
    fk = frames(model)
    left = np.linalg.inv(fk['arm_carriage']) @ fk['left_link1']
    right = np.linalg.inv(fk['arm_carriage']) @ fk['right_link1']
    parameters = np.linalg.lstsq(np.column_stack((left[:3,2], -right[:3,2])), right[:3,3]-left[:3,3], rcond=None)[0]
    first = left[:3,3] + parameters[0] * left[:3,2]
    second = right[:3,3] + parameters[1] * right[:3,2]
    if not np.all(parameters < 0) or np.linalg.norm(first-second) > 1e-8:
        raise ValueError('Left/right negative Z axes do not intersect')
    mount = model.find("joint[@name='preview_mount_to_arm_carriage']")
    mount.set('name', 'mount_to_arm_carriage')
    offset = -transform(mount.find('origin'))[:3,:3] @ ((first+second)/2)
    mount.find('origin').set('xyz', ' '.join(format(v, '.17g') for v in offset))
    lift = model.find("joint[@name='arm_carriage']")
    lift.set('name', 'updown')
    lift.set('type', 'prismatic')
    # This origin has zero rotation. Positive q moves the entire upper body +Z,
    # matching the old Rx(pi) * local -Z convention without rotating new geometry.
    xyz = np.fromstring(lift.find('origin').get('xyz'), sep=' ')
    xyz -= np.asarray(policy['lift_direction_in_base']) * reading
    lift.find('origin').set('xyz', ' '.join(format(v, '.17g') for v in xyz))
    ET.SubElement(lift, 'axis', xyz=' '.join(map(str, policy['lift_direction_in_base'])))
    limits = yaml.safe_load((root/'config/joint_limits_alfav3_mobile.yaml').read_text())['joints']['updown']
    ET.SubElement(lift, 'limit', lower=str(limits['lower_position_m']), upper=str(limits['upper_position_m']),
                  velocity=str(limits['max_velocity_m_s']), effort=str(limits['max_effort']))
    collapse_assembly_frames(model)
    for mesh in model.findall('.//mesh'):
        name = mesh.get('filename')
        mesh.set('filename', name.replace('preview/carriage/meshes/', 'mesh/carriage_assembly/'))
    canonical = root/'urdf/robot_alfav3_mobile.urdf'
    write_xml(canonical, model)
    for variant, remove in [('visual', 'collision'), ('collision', 'visual')]:
        projection = deepcopy(model)
        for link in projection.findall('link'):
            for element in link.findall(remove):
                link.remove(element)
        write_xml(root/f'urdf/variants/robot_{variant}.urdf', projection)
    (root/'config/curobo/fixed_body_boxes.yaml').write_bytes((inputs/policy['fixed_boxes']).read_bytes())
    manifest_path = root/'config/alfav3_mobile_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest.update(revision=policy['revision'], link_count=len(model.findall('link')),
                    movable_joint_count=sum(j.get('type')!='fixed' for j in model.findall('joint')),
                    carriage_assembly=policy,
                    collision_scope='239 unchanged link-local spheres + user-approved 4 base_link boxes; no payload. Mesh collision projections retained. Not a coverage or real-motion certificate.')
    manifest['composition']['base_lift_carriage'] = 'approved CAD carriage assembly; q=reference reading matches visual approval'
    manifest['composition']['base_collision'] = 'approved four base_link boxes plus CAD mesh projection'
    manifest['composition']['carriage_collision'] = '30 original arm_carriage-local spheres, rigidly following approved mount'
    for name in ('lidar_main','lidar_rear'):
        radar=manifest['composition'].get('radars',{}).get(name)
        if radar is not None:
            radar['xyz_m']=frames(model)[name][:3,3].tolist()
            radar['note']='Local base_link mount preserved; root translation updated, not independently recalibrated'
    manifest['mesh_sources']={key:value for key,value in manifest['mesh_sources'].items()
                              if key in {m.get('filename').removeprefix(PREFIX) for m in model.findall('.//mesh')}}
    for name in ('base_link.STL','up_and_down.STL'):
        manifest['mesh_sources']['mesh/carriage_assembly/'+name]={'commit':policy['source_commit'],
            'source_path':'preview/carriage/meshes/'+name,'method':'byte-identical copy from approved assembly source'}
    manifest['joint_reparents'].pop('updown', None)
    manifest['joint_overrides']['updown'] = 'Calibrated prismatic base_link -> arm_carriage; CAD-only frames collapsed, geometry/inertia transformed into arm_carriage'
    manifest['joint_overrides']['base_footprint_to_base_link'] = 'User-approved xyz=0.07478297093,0,0.370 m'
    for i in range(1,5):
        manifest['joint_overrides'][f'steer{i}_joint'] = 'Align positive rolling at zero steering with footprint +X; compensate base_link X shift by -74.78297093 mm'
    manifest['assembly_mount_offset_m'] = offset.tolist()
    manifest['assembly_axis_reverse_distances_m'] = (-parameters).tolist()
    for metadata in manifest['artifacts'].values():
        metadata['sha256'] = hashlib.sha256((root/metadata['path']).read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    for file in ['config/model_variants.yaml','config/motion_scope.yaml','config/poses/home.yaml','config/poses/stow.yaml']:
        path=root/file
        document=yaml.safe_load(path.read_text())
        field='revision' if file.endswith('model_variants.yaml') else 'model_revision'
        document[field]=policy['revision']
        path.write_text(yaml.safe_dump(document,sort_keys=False,allow_unicode=True))
    # Only retire files explicitly owned by the previous manifest and now unreferenced.
    mesh_manifest_path=root/'config/mesh_assets.json'
    old=json.loads(mesh_manifest_path.read_text())
    paths={mesh.get('filename').removeprefix(PREFIX) for mesh in model.findall('.//mesh')}
    for name in set(old['files'])-paths:
        path=(root/name).resolve()
        if not path.is_relative_to((root/'mesh').resolve()):
            raise ValueError('Invalid old mesh manifest path: '+name)
        path.unlink(missing_ok=True)
    mesh_manifest_path.write_text(json.dumps(dict(entry='urdf/robot_alfav3_mobile.urdf',files=sorted(paths),
        sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in sorted(paths)}),indent=2)+'\n')
    print(json.dumps(dict(revision=policy['revision'],lift_reference_reading_m=reading,links=len(model.findall('link')),
        dof=manifest['movable_joint_count'],mount_offset_m=offset.tolist()),indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    build(parser.parse_args().root.resolve())
