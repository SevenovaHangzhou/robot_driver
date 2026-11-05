"""Validate the single supported model without depending on archived generators/models."""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / 'urdf/robot_alfav3_mobile.urdf'


def test_single_entry_and_tree():
    assert list((ROOT / 'urdf').iterdir()) == [ENTRY]
    assert len(list(ROOT.rglob('*.urdf'))) == 1
    assert not list(ROOT.rglob('*.xacro'))
    result = subprocess.run(['check_urdf', str(ENTRY)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'root Link: base_footprint' in result.stdout
    model = ET.parse(ENTRY).getroot()
    assert len(model.findall('link')) == 35
    assert sum(j.get('type') != 'fixed' for j in model.findall('joint')) == 25


def test_all_and_only_referenced_meshes_present():
    manifest = json.loads((ROOT / 'config/mesh_assets.json').read_text())
    prefix = 'package://robot_description/'
    meshes = ET.parse(ENTRY).findall('.//mesh')
    assert all(m.get('filename').startswith(prefix) for m in meshes)
    refs = {m.get('filename').removeprefix(prefix) for m in meshes}
    assert refs == set(manifest['files'])
    actual = {str(p.relative_to(ROOT)) for p in (ROOT / 'mesh').rglob('*') if p.is_file()}
    assert refs == actual
    for path, digest in manifest['files'].items():
        assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest, path


def test_joint_limits_and_actuator_caps_match():
    model = ET.parse(ENTRY).getroot()
    limits = yaml.safe_load((ROOT / 'config/joint_limits_alfav3_mobile.yaml').read_text())['joints']
    specs = json.loads((ROOT / 'config/alfav3_arm_module_specs.json').read_text())['joints']
    manifest = json.loads((ROOT / 'config/alfav3_mobile_manifest.json').read_text())
    manifest_limits = manifest['arm_limits_deg']
    motion = manifest['arm_motion_limits']
    mjcf = ET.parse(ROOT / 'mujoco/robot.xml').getroot()
    assert set(limits) == {j.get('name') for j in model.findall('joint') if j.get('type') != 'fixed'}
    for j in model.findall('joint'):
        name = j.get('name')
        if name not in limits:
            continue
        data = limits[name]
        bound = j.find('limit')
        for field, value in data.items():
            key = {'lower_position_rad': 'lower', 'upper_position_rad': 'upper',
                   'lower_position_m': 'lower', 'upper_position_m': 'upper',
                   'max_velocity_rad_s': 'velocity', 'max_effort': 'effort'}.get(field)
            if key:
                assert math.isclose(float(bound.get(key)), value), (name, field)
        if name.startswith(('left_joint', 'right_joint')):
            entry = manifest_limits[name[-1]]
            if isinstance(entry, dict):
                entry = entry[name.split('_')[0]]
                degrees = [entry['lower'], entry['upper']]
            else:
                degrees = [-entry, entry]
            expected = np.deg2rad(degrees)
            assert np.allclose([float(bound.get('lower')), float(bound.get('upper'))], expected, atol=1e-12), name
            assert np.allclose(np.fromstring(mjcf.find(f".//joint[@name='{name}']").get('range'), sep=' '), expected, atol=1e-12), name
            spec = specs['joint' + name[-1]]
            assert float(bound.get('effort')) == spec['urdf_effort_nm']
            # Motion limits sit below the actuator peak speed; acceleration lives only in the YAML.
            assert math.isclose(float(bound.get('velocity')), math.radians(motion['velocity_deg_s']))
            assert float(bound.get('velocity')) <= spec['peak_velocity_rad_s']
            assert math.isclose(data['max_acceleration_rad_s2'], math.radians(motion['acceleration_deg_s2']))
            assert j.find('dynamics').attrib == {'friction': '0.0', 'damping': '0.0'}


def rotation(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def test_confirmed_zero_conventions():
    model = ET.parse(ENTRY).getroot()
    mjcf = ET.parse(ROOT / 'mujoco/robot.xml').getroot()
    for side in ('left', 'right'):
        name = side + '_joint2'
        bounds = model.find(f"joint[@name='{name}']/limit")
        expected = np.deg2rad((-92, 122) if side == 'left' else (-122, 92))
        assert np.allclose([float(bounds.get('lower')), float(bounds.get('upper'))], expected, atol=1e-12)
        assert np.allclose(np.fromstring(mjcf.find(f".//joint[@name='{name}']").get('range'), sep=' '), expected, atol=1e-12)
    left = model.find("joint[@name='left_joint5']")
    current = rotation(*map(float, left.find('origin').get('rpy').split()))
    assert np.allclose(current, rotation(-math.pi/2, 0, 0) @ rotation(0, 0, math.pi), atol=1e-11)
    assert left.find('axis').get('xyz') == '0 0 1'
    right = model.find("joint[@name='right_joint5']")
    assert np.allclose(rotation(*map(float, right.find('origin').get('rpy').split())), rotation(math.pi/2, 0, 0), atol=1e-11)
    assert right.find('axis').get('xyz') == '0 0 1'
    assert model.find("joint[@name='left_joint3']/axis").get('xyz') == '0 0 1'
    assert model.find("joint[@name='left_joint6']/axis").get('xyz') == '0 0 -1'
    assert model.find("link[@name='imu_link']") is not None
    for side in ('left', 'right'):
        xyz = model.find(f"joint[@name='{side}_tool0_fixed']/origin").get('xyz')
        assert np.allclose(list(map(float, xyz.split())), [0, 0, .2497])


def test_inertials_and_srdf_references():
    model = ET.parse(ENTRY).getroot()
    links = {link.get('name') for link in model.findall('link')}
    joints = {joint.get('name') for joint in model.findall('joint')}
    for link in model.findall('link'):
        block = link.find('inertial')
        if block is None:
            continue
        assert float(block.find('mass').get('value')) > 0
        i = {k: float(v) for k, v in block.find('inertia').attrib.items()}
        matrix = np.array([[i['ixx'], i['ixy'], i['ixz']], [i['ixy'], i['iyy'], i['iyz']], [i['ixz'], i['iyz'], i['izz']]])
        assert np.isfinite(matrix).all()
        assert np.linalg.eigvalsh(matrix).min() > 0, link.get('name')
    srdf = ET.parse(ROOT / 'srdf/robot_alfav3_mobile.srdf').getroot()
    assert srdf.get('name') == model.get('name')
    assert srdf.find('group_state') is None
    for element in srdf.iter():
        for key in ('link1', 'link2', 'base_link', 'tip_link', 'link', 'child_link'):
            if key in element.attrib:
                assert element.get(key) in links
        if element.tag == 'joint':
            assert element.get('name') in joints


def test_manifest_paths_resolve():
    imported = json.loads((ROOT / 'config/arm_collision_import.json').read_text())
    assert set(imported['links']) == {f'{side}_link{i}' for side in ('left', 'right') for i in range(1, 8)}
    assert imported['target_urdf_sha256'] == hashlib.sha256(ENTRY.read_bytes()).hexdigest()
    conversion = json.loads((ROOT / 'mujoco/conversion_report.json').read_text())
    assert conversion['source_sha256'] == hashlib.sha256(ENTRY.read_bytes()).hexdigest()
    for item in imported['links'].values():
        assert hashlib.sha256((ROOT / item['target_mesh']).read_bytes()).hexdigest() == item['target_sha256']
        transform = np.array(item['target_link_from_source_link_q0'])
        assert np.allclose(transform[3], [0, 0, 0, 1])
        assert np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-12)
        assert math.isclose(np.linalg.det(transform[:3, :3]), 1, abs_tol=1e-12)
        assert item['zero_pose_max_vertex_error_m'] < 1e-6
    manifest = json.loads((ROOT / 'config/alfav3_mobile_manifest.json').read_text())
    for key in ('entry', 'srdf', 'joint_limits', 'arm_actuator_specs'):
        assert (ROOT / manifest[key]).is_file()
    assert manifest['archived_sources']['tag'] == 'backup/before-single-model-20261020'


def test_head_motion_and_sight_ranges():
    model = ET.parse(ENTRY).getroot()
    for name in ('H1_joint', 'H2_joint'):
        limit = model.find(f"joint[@name='{name}']/limit")
        assert math.isclose(float(limit.get('lower')), -math.pi)
        assert math.isclose(float(limit.get('upper')), math.pi)
    for name in ('Leye_link', 'Reye_Link'):
        link = model.find(f"link[@name='{name}']")
        assert link.find('inertial') is None
        assert link.find('collision') is None
        assert link.find('visual/geometry/mesh') is not None
    sensor = model.find("link[@name='mid360_Link']")
    assert float(sensor.find('inertial/mass').get('value')) == .375
    # The corrected COM must lie inside the sensor's physical mesh bounds.
    manifest = json.loads((ROOT / 'config/alfav3_mobile_manifest.json').read_text())
    low, high = np.array(manifest['head_and_upper_base']['mid360']['bounds_m'])
    com = np.fromstring(sensor.find('inertial/origin').get('xyz'), sep=' ')
    assert np.all(low <= com) and np.all(com <= high)


def _joint_frames(model):
    """Zero-pose base_link frame (4x4) of every joint origin."""
    joints = {j.find('child').get('link'): j for j in model.findall('joint')}

    def origin(joint):
        element = joint.find('origin')
        frame = np.eye(4)
        frame[:3, :3] = rotation(*map(float, element.get('rpy').split()))
        frame[:3, 3] = list(map(float, element.get('xyz').split()))
        return frame

    def link(name):
        frame = np.eye(4)
        while name != 'base_link':
            joint = joints[name]
            frame = origin(joint) @ frame
            name = joint.find('parent').get('link')
        return frame

    return {j.get('name'): link(j.find('parent').get('link')) @ origin(j) for j in model.findall('joint')
            if j.find('parent').get('link') != 'base_footprint'}


def test_head_pitch_origin_relative_to_arm_base_point():
    # User 2026-11-03: arm-base-point = intersection of the left/right J1 axes; at the zero pose
    # the H2 origin is [-19, 0, 246.250093] mm from it along base_link. H1 stays at +1 mm in X.
    model = ET.parse(ENTRY).getroot()
    frames = _joint_frames(model)

    def axis(name):
        direction = np.array(list(map(float, model.find(f"joint[@name='{name}']/axis").get('xyz').split())))
        return frames[name][:3, 3], frames[name][:3, :3] @ direction

    (pl, al), (pr, ar) = axis('left_joint1'), axis('right_joint1')
    w = pl - pr
    a, b, c, d, e = al @ al, al @ ar, ar @ ar, al @ w, ar @ w
    left = pl + (b * e - c * d) / (a * c - b * b) * al
    right = pr + (a * e - b * d) / (a * c - b * b) * ar
    assert np.linalg.norm(left - right) < 1e-9
    point = (left + right) / 2
    # The J1 origins differ by micrometres left/right, so the intersection sits 0.005 mm off Y=0.
    assert np.allclose((frames['H2_joint'][:3, 3] - point) * 1000, [-19.0, 0.0, 246.250093], atol=0.01)
    assert np.allclose((frames['H1_joint'][:3, 3] - point) * 1000, [1.0, 0.0, 0.0], atol=0.01)
    assert np.allclose(axis('H2_joint')[1], [0, -1, 0], atol=1e-6)  # rpy is pi/2 to 13 digits
