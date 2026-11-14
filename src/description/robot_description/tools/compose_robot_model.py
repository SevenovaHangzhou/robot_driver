#!/usr/bin/env python3
"""Rebuild the agreed robot composition from two pinned Git sources.

No network, no branch switching and no hardware access. Output contains portable
package URIs. The canonical URDF is the sole kinematic source of its projections.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from model_geometry import frames, rotation, set_origin, transform

DESCRIPTION_REV = '4f9069a86765385d3a2773f1bded3b5cf310218a'
ALGORITHM_REV = '92bd8cdb4d8b79793c428727db429288c17dd4f2'
RESEARCH = 'research/curobo_v3/'
OLD_URDF = RESEARCH+'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf'
OLD_CONFIG = RESEARCH+'generated/zero_model_candidate_20261018/fixed.yaml'
BOX_CONFIG = RESEARCH+'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json'
PREFIX = 'package://robot_description/'
MODEL_REV = 'unified-20261112-r2'
WHEEL_LINKS = {f'{kind}{i}' for kind in ('steer', 'wheel') for i in range(1, 5)}
HEAD_COLLISION_LINKS = ('H1_Link', 'H2_Link', 'mid360_Link')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def structure(element):
    return [element.tag, dict(sorted(element.attrib.items())), [structure(child) for child in element]]


def blob(repo, ref, path):
    return subprocess.check_output(['git', '-C', str(repo), 'show', ref+':'+path])


def write_json(root, path, value):
    target = root/path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def write_yaml(root, path, value):
    target = root/path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(value, sort_keys=False, allow_unicode=True))


def write_urdf(root, path, model):
    target = root/path
    target.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(model, space='  ')
    ET.ElementTree(model).write(target, encoding='utf-8', xml_declaration=True)
    with target.open('ab') as stream:
        stream.write(b'\n')


def enclose_head_meshes(model, root, cell_size=.06):
    """Cover every source head-mesh triangle with a deterministic local sphere.

    Triangles are grouped by centroid grid cell. Each cell sphere encloses all
    vertices of its assigned triangles, hence those triangles themselves. This
    is a conservative surface approximation, not a calibration or volume proof.
    Old head spheres cannot be reused for a different head shape/kinematic chain.
    """
    import trimesh
    from model_geometry import resolve_mesh
    result, evidence = {}, {}
    for name in HEAD_COLLISION_LINKS:
        triangles = []
        link = model.find(f"link[@name='{name}']")
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                raise ValueError('head fitting requires explicit collision mesh: '+name)
            shape = trimesh.load_mesh(resolve_mesh(root, mesh.get('filename')), process=False)
            local = transform(collision.find('origin'))
            scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            points = np.asarray(shape.triangles)*scale
            triangles.append(points@local[:3,:3].T+local[:3,3])
        triangles = np.concatenate(triangles)
        if not np.isfinite(triangles).all():
            raise ValueError('nonfinite head collision mesh: '+name)
        cells = np.floor(triangles.mean(axis=1)/cell_size).astype(np.int64)
        _, assignment = np.unique(cells, axis=0, return_inverse=True)
        count = int(assignment.max())+1
        lower, upper = np.full((count,3), np.inf), np.full((count,3), -np.inf)
        np.minimum.at(lower, assignment, triangles.min(axis=1))
        np.maximum.at(upper, assignment, triangles.max(axis=1))
        centers = (lower+upper)/2
        vertex_radius = np.linalg.norm(triangles-centers[assignment,None,:], axis=2)
        radii = np.zeros(count)
        np.maximum.at(radii, assignment, vertex_radius.max(axis=1))
        radii += .0005  # Explicit 0.5mm numerical coverage margin.
        result[name] = [{'center': center.tolist(), 'radius': float(radius)}
                        for center,radius in zip(centers,radii)]
        evidence[name] = {'triangle_count': len(triangles), 'sphere_count': count,
            'grid_cell_m': cell_size, 'surface_margin_m': .0005,
            'maximum_assigned_triangle_vertex_gap_m': float((vertex_radius-radii[assignment,None]).max())}
    return result, evidence


def build(description_repo, algorithm_repo, output, description_ref=DESCRIPTION_REV, algorithm_ref=ALGORITHM_REV):
    # Resolve symbolic input once; generated manifests always store commit IDs.
    description_ref = subprocess.check_output(['git','-C',str(description_repo),'rev-parse',description_ref+'^{commit}'],text=True).strip()
    algorithm_ref = subprocess.check_output(['git','-C',str(algorithm_repo),'rev-parse',algorithm_ref+'^{commit}'],text=True).strip()
    previous_manifest = output/'config/mesh_assets.json'
    previous_files = set(json.loads(previous_manifest.read_text()).get('files', [])) if previous_manifest.is_file() else set()
    src = blob(description_repo, description_ref, 'urdf/robot_alfav3_mobile.urdf')
    old_bytes = blob(algorithm_repo, algorithm_ref, OLD_URDF)
    old_config_bytes = blob(algorithm_repo, algorithm_ref, OLD_CONFIG)
    boxes_bytes = blob(algorithm_repo, algorithm_ref, BOX_CONFIG)
    upstream = ET.fromstring(src)
    old = ET.fromstring(old_bytes)
    old_kin = yaml.safe_load(old_config_bytes)['kinematics']
    source_files = {}
    source_meshes = {}
    model = ET.Element('robot', name='alfa_robot_v3')

    # Start with the complete upstream tree so no future upstream joint or
    # sensor silently disappears. Only explicitly selected body assets and
    # the root/lift transform are replaced by the PR82 source.
    core = {'base_footprint', 'base_link', 'arm_carriage'}
    old_links = {e.get('name'): e for e in old.findall('link')}
    old_joints = {e.get('name'): e for e in old.findall('joint')}
    for element in upstream:
        if element.tag == 'link' and element.get('name') in core:
            element = old_links[element.get('name')]
        elif element.tag == 'joint' and element.get('name') in ('base_footprint_to_base_link', 'updown'):
            element = old_joints[element.get('name')]
        copied = deepcopy(element)
        if copied.tag == 'link' and copied.get('name') in WHEEL_LINKS:
            # Wheel joints, visuals and inertials stay. The approved planning
            # collision model uses the chassis shell rather than wheel contact.
            for collision in list(copied.findall('collision')):
                copied.remove(collision)
        model.append(copied)

    # Preserve the enclosed base shell approved in the visual review. The two
    # other fixed-body meshes remain from the cuRobo model. Wheel DOFs are kept.
    base = model.find("link[@name='base_link']")
    for element in base.findall('visual') + base.findall('collision'):
        mesh = element.find('geometry/mesh')
        if 'part_001_solid_001-' in mesh.get('filename'):
            mesh.set('filename', PREFIX+'mesh/chassis/base_link.stl')
            mesh.set('scale', '0.001 0.001 0.001')
    # Simplified/base collision follows the same shell as the visual view.
    for element in list(base.findall('collision')):
        base.remove(element)
    for element in base.findall('visual'):
        collision = deepcopy(element)
        collision.tag = 'collision'
        for material in list(collision.findall('material')):
            collision.remove(material)
        base.append(collision)

    old_frames, new_frames = frames(old), frames(model)
    transforms = {}
    for side in ('left', 'right'):
        for index in range(1, 8):
            link = f'{side}_link{index}'
            # Anchor to carriage: never fold the old/new root discrepancy into
            # an arm-local collision shape.
            before = np.linalg.inv(old_frames['arm_carriage']) @ old_frames[link]
            after = np.linalg.inv(new_frames['arm_carriage']) @ new_frames[link]
            transforms[link] = np.linalg.inv(after) @ before

    # Tool TCP physical distance uses the agreed PR82 J6-axis distance. The
    # old link7 frame is displaced 99.2mm from the wrist intersection, whereas
    # the new J6/J7 origins coincide; copying +135mm would be incorrect.
    tool_data = {}
    for side in ('left', 'right'):
        local = np.eye(4)
        # Snap only the known frame-convention quarter-turn; source RPY
        # rounding leaves microradian terms, not a calibrated tool tilt.
        local[:3, :3] = rotation((0, 0, -np.pi/2))
        local[:3, 3] = (0, 0, .2342)
        set_origin(model.find(f"joint[@name='{side}_tool0_fixed']"), local)
        tool_data[side] = {'parent': side+'_link7', 'xyz_m': [0,0,.2342],
                           'rpy_rad': [0,0,-float(np.pi/2)],
                           'j6_axis_distance_m': .2342}

    # Accepted radar coordinates are in the new, unrotated footprint. Attach
    # to base_link so both stay fixed to the physical chassis (not the lift).
    radar_data = {}
    for name, xyz, rpy in [('lidar_main', [.453313447977869,0,.398099404], [0,15,0]),
                           ('lidar_rear', [-.047,0,1.959], [0,-90,0])]:
        ET.SubElement(model, 'link', name=name)
        joint = ET.SubElement(model, 'joint', name=name+'_fixed', type='fixed')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link=name)
        global_pose = np.eye(4)
        global_pose[:3,:3] = rotation(np.deg2rad(rpy))
        global_pose[:3,3] = xyz
        set_origin(joint, np.linalg.inv(new_frames['base_link'])@global_pose)
        radar_data[name] = {'reference_frame': 'base_footprint', 'xyz_m': xyz,
                           'rpy_deg': rpy, 'parent': 'base_link',
                           'physical_mesh': None, 'note': 'measurement frame only; no invented geometry/inertia'}

    # Ultrasonic transducer measuring surfaces (user 2026-11-13, given in a chassis frame with +X to
    # the left and -Y forward): origin at the transducer face, +X along the beam. No geometry or
    # inertia is invented; the links only carry the frame the driver publishes.
    ultrasonic_data = {}
    for channel, x, y, yaw in [
            (1, .3145, .485, 90), (2, .485, .3145, 0), (3, .485, -.3145, 0), (4, .3145, -.485, -90),
            (5, -.3145, -.485, -90), (6, -.485, -.3145, 180), (7, -.485, .3145, 180),
            (8, -.3145, .485, 90)]:
        name = f'ultrasonic_channel_{channel}_link'
        ET.SubElement(model, 'link', name=name)
        joint = ET.SubElement(model, 'joint', name=name.replace('_link', '_joint'), type='fixed')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link=name)
        set_origin(joint, transform([x, y, .019], [0, 0, np.deg2rad(yaw)]))
        ultrasonic_data[name] = {'parent': 'base_link', 'xyz_m': [x, y, .019], 'yaw_deg': yaw,
                                 'note': 'transducer measuring surface; +X is the beam direction'}

    # Vendor only meshes actually used by the composite. Source hashes are
    # taken from pinned Git blobs, not a dirty research checkout or /tmp copy.
    for mesh in model.findall('.//mesh'):
        filename = mesh.get('filename')
        if filename.startswith(PREFIX):
            path = filename[len(PREFIX):]
            data = blob(description_repo, description_ref, path)
            source_files[path] = {'repo': 'SevenovaHangzhou/robot_description', 'commit': description_ref,
                                  'path': path, 'sha256': sha(data)}
        else:
            if RESEARCH not in filename:
                raise ValueError('unknown source mesh: '+filename)
            original = RESEARCH+filename.split(RESEARCH, 1)[1]
            data = blob(algorithm_repo, algorithm_ref, original)
            path = 'mesh/curobo_body/'+Path(filename).name
            mesh.set('filename', PREFIX+path)
            source_files[path] = {'repo': 'kkozia188/alfa_robot', 'commit': algorithm_ref,
                                  'path': original, 'sha256': sha(data)}
        target = output/path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        source_meshes[path] = data

    entry = 'urdf/robot_alfav3_mobile.urdf'
    write_urdf(output, entry, model)
    for variant in ('visual', 'collision'):
        projected = deepcopy(model)
        removed = 'collision' if variant=='visual' else 'visual'
        for link in projected.findall('link'):
            for element in list(link.findall(removed)):
                link.remove(element)
        write_urdf(output, f'urdf/variants/robot_{variant}.urdf', projected)

    # Preserve radii and selected collision policy. Only re-express arm sphere
    # centres. The 71 disabled base spheres are absent from the asset entirely;
    # the required four fixed-body boxes are an explicit companion artifact.
    sphere_map = {}
    for link, spheres in old_kin['collision_spheres'].items():
        if link=='base_link' or link not in {e.get('name') for e in model.findall('link')}:
            continue
        t = transforms.get(link, np.eye(4))
        sphere_map[link] = [{'center': (t[:3,:3]@np.array(s['center'])+t[:3,3]).tolist(),
                             'radius': s['radius']} for s in spheres if s['radius']>0]
    head_spheres, head_fit = enclose_head_meshes(model, output)
    sphere_map.update(head_spheres)
    sphere_path = 'config/curobo/collision_spheres.yaml'
    write_yaml(output, sphere_path, {'collision_spheres': sphere_map})
    boxes = json.loads(boxes_bytes)
    box_path = 'config/curobo/fixed_body_boxes.yaml'
    write_yaml(output, box_path, {'schema_version': 1, 'frame': 'base_link', 'units': 'm',
        'required': True, 'relations': {'robot': 'sphere_box', 'environment': 'box_box_SAT'},
        'cuboids': [{'name': b['name'], 'center': b['center'], 'dimensions': b['dimensions'],
                    'rpy_rad': np.deg2rad(b['rotation_deg']).tolist()} for b in boxes['cuboids']]})
    k = deepcopy(old_kin)
    k['urdf_path'] = entry
    k['asset_root_path'] = '.'
    k['collision_link_names'] = list(sphere_map)
    k['collision_spheres'] = 'collision_spheres.yaml'
    k['mesh_link_names'] = list(sphere_map)
    k['tool_frames'] = ['left_tool0','right_tool0','lidar_main','lidar_rear',
        'H1_Link','H2_Link','mid360_Link','Leye_link','Reye_Link',
        'left_ft_sensor_link','right_ft_sensor_link'] + sorted(WHEEL_LINKS)
    planning_joints = set(k['cspace']['joint_names'])
    k['lock_joints'] = {j.get('name'): 0.0 for j in model.findall('joint')
        if j.get('type')!='fixed' and j.get('name') not in planning_joints}
    k['self_collision_buffer'] = {n: v for n,v in k.get('self_collision_buffer',{}).items() if n in sphere_map}
    k['self_collision_ignore'] = {n: [other for other in others if other in sphere_map]
        for n,others in k['self_collision_ignore'].items() if n in sphere_map}
    for name in HEAD_COLLISION_LINKS:
        k['self_collision_buffer'][name] = 0.0
        k['self_collision_ignore'][name] = []
    for joint in model.findall('joint'):
        parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
        if parent in sphere_map and child in sphere_map and (parent in HEAD_COLLISION_LINKS or child in HEAD_COLLISION_LINKS):
            k['self_collision_ignore'][parent].append(child)
            k['self_collision_ignore'][child].append(parent)
    k['cspace']['default_joint_position'] = [0.0]*len(k['cspace']['joint_names'])
    k['cspace']['max_acceleration'] = [10.0 if n=='updown' else float(np.pi/3)
                                       for n in k['cspace']['joint_names']]
    write_yaml(output, 'config/curobo/robot.yaml', {'kinematics': k})

    # Current limits: actual URDF values; arm acceleration from the upstream
    # control limit contract. Remaining research limits are not calibrated.
    limits = {'metadata': {'model_revision': MODEL_REV, 'status': 'software_model_for_review'}, 'joints': {}}
    for joint in model.findall('joint'):
        if joint.get('type')=='fixed':
            continue
        n = joint.get('name')
        lim = joint.find('limit')
        unit = 'm' if joint.get('type')=='prismatic' else 'rad'
        limits['joints'][n] = {'type': joint.get('type'),
            'max_velocity_'+unit+'_s': float(lim.get('velocity')),
            'max_effort': float(lim.get('effort'))}
        if joint.get('type')!='continuous':
            limits['joints'][n].update({'lower_position_'+unit: float(lim.get('lower')),
                                      'upper_position_'+unit: float(lim.get('upper'))})
        if n.startswith(('left_joint','right_joint')):
            limits['joints'][n]['max_acceleration_rad_s2'] = float(np.pi/3)
    write_yaml(output, 'config/joint_limits_alfav3_mobile.yaml', limits)

    # Preserve the upstream head joints in the semantic whole-body group.
    srdf = ET.Element('robot', name=model.get('name'))
    for side in ('left','right'):
        group = ET.SubElement(srdf,'group',name=side+'_arm')
        ET.SubElement(group,'chain',base_link='arm_carriage',tip_link=side+'_tool0')
    group = ET.SubElement(srdf,'group',name='dual_arm')
    for side in ('left','right'):ET.SubElement(group,'group',name=side+'_arm')
    for name, joints in [('dual_arm_with_updown',['updown']), ('whole',['updown','H1_joint','H2_joint'])]:
        group=ET.SubElement(srdf,'group',name=name)
        for j in joints:ET.SubElement(group,'joint',name=j)
        ET.SubElement(group,'group',name='dual_arm')
    ET.SubElement(srdf,'virtual_joint',name='map_to_base_footprint',type='planar',parent_frame='map',child_link='base_footprint')
    for joint in model.findall('joint'):
        ET.SubElement(srdf,'disable_collisions',link1=joint.find('parent').get('link'),
            link2=joint.find('child').get('link'),reason='Adjacent')
    write_urdf(output,'srdf/robot_alfav3_mobile.srdf',srdf)

    variant_paths = {'canonical_urdf': entry, 'visual_urdf': 'urdf/variants/robot_visual.urdf',
        'collision_urdf': 'urdf/variants/robot_collision.urdf', 'curobo_config': 'config/curobo/robot.yaml',
        'curobo_spheres': sphere_path, 'fixed_body_boxes': box_path}
    manifest = {
        'revision': MODEL_REV, 'status': 'review_candidate', 'sources': {
            'github_description': {'commit': description_ref, 'urdf_sha256': sha(src)},
            'curobo_pr82_assets': {'unchanged_since_pr80_commit': algorithm_ref,
                'urdf_sha256': sha(old_bytes), 'fixed_yaml_sha256': sha(old_config_bytes),
                'boxes_sha256': sha(boxes_bytes)}},
        'composition': {'arm_joint1_to_7_and_geometry': 'github_description',
            'base_lift_carriage': 'curobo_pr82', 'head_and_sensors': 'github_description, original parents and local joint transforms',
            'steering_wheels': 'github_description joints/visuals/inertials; collision represented by chassis shell',
            'base_collision': 'approved enclosed visual shell',
            'tcp': tool_data, 'tool_link_geometry_inertia_and_ft_sensors': 'github_description',
            'radars': radar_data, 'ultrasonics': ultrasonic_data},
        'link_count': len(model.findall('link')), 'movable_joint_count': len(limits['joints']),
        'sphere_count': sum(map(len,sphere_map.values())), 'fixed_box_count': len(boxes['cuboids']),
        'arm_new_link_from_old_link_zero': {n:t.tolist() for n,t in transforms.items()},
        'github_arm_joint_contract': {j.get('name'): structure(j) for j in upstream.findall('joint')
            if j.get('name') in {f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)}},
        'github_full_joint_contract': {j.get('name'): structure(j) for j in upstream.findall('joint')},
        'github_link_names': [l.get('name') for l in upstream.findall('link')],
        'joint_overrides': {'base_footprint_to_base_link': 'PR82 root: z=.335m, yaw=0',
            'updown': 'PR82 lift (same local transform as upstream)',
            'left_tool0_fixed': 'agreed own TCP distance and axes',
            'right_tool0_fixed': 'agreed own TCP distance and axes'},
        'head_collision_fit': head_fit,
        'collision_scope': 'Imported arm/body radii are approximations, not certified conservative envelopes. New head spheres enclose assigned mesh triangles with a 0.5mm surface margin. No payload. Four boxes require an explicit backend; native sphere-only queries are incomplete.',
        'mesh_sources': source_files,
        'artifacts': {n: {'path': path, 'sha256': sha((output/path).read_bytes())} for n,path in variant_paths.items()}}
    write_json(output, 'config/alfav3_mobile_manifest.json', manifest)
    write_yaml(output, 'config/model_variants.yaml', {'schema_version':1, 'revision':manifest['revision'], **variant_paths})
    write_json(output,'config/mesh_assets.json',{'entry':entry,'files':sorted(source_meshes),
        'sha256':{p:sha(data) for p,data in sorted(source_meshes.items())}})
    for path in previous_files - set(source_meshes):
        # Only retire files explicitly owned by the previous asset manifest.
        target = (output/path).resolve()
        if not path.startswith('mesh/') or not target.is_relative_to((output/'mesh').resolve()):
            raise ValueError('unsafe prior mesh manifest path: '+path)
        if target.is_file():
            target.unlink()
    print(json.dumps({k:manifest[k] for k in ('revision','link_count','movable_joint_count','sphere_count','fixed_box_count')},indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description-repo', type=Path, required=True)
    parser.add_argument('--algorithm-repo', type=Path, required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--description-ref', default=DESCRIPTION_REV)
    parser.add_argument('--algorithm-ref', default=ALGORITHM_REV)
    args = parser.parse_args()
    build(args.description_repo, args.algorithm_repo, args.output_root, args.description_ref, args.algorithm_ref)
