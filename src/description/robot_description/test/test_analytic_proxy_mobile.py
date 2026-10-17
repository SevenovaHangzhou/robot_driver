"""Verify mobile composition against independent source models and tire geometry."""
import copy
import hashlib
import math
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from test_v3_analytic_proxy_profile import origin

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'model_sources/mechanical_958c49d'
MOBILE = ROOT / 'urdf/robot_analytic_proxy_mobile.urdf'


def canonical(element):
    return (element.tag, sorted(element.attrib.items()), [canonical(e) for e in element])


def test_sources_and_unchanged_upper_body():
    for line in (SOURCE / 'SHA256SUMS').read_text().splitlines():
        digest, name = line.split('  ', 1)
        assert hashlib.sha256((SOURCE / name).read_bytes()).hexdigest() == digest
    proxy = ET.parse(ROOT / 'urdf/robot_analytic_proxy.urdf.xacro').getroot()
    mobile = ET.parse(MOBILE).getroot()
    for element in proxy:
        if element.get('name') in ('base_link', 'base_footprint_to_base_link'):
            continue
        counterpart = mobile.find(f"{element.tag}[@name='{element.get('name')}']")
        assert counterpart is not None
        assert canonical(element) == canonical(counterpart)
    mechanical = ET.parse(SOURCE / 'robot.urdf').getroot()
    for name in ['base_link', *(f'{kind}{i}' for kind in ('steer', 'wheel') for i in range(1, 5))]:
        expected = copy.deepcopy(mechanical.find(f"link[@name='{name}']"))
        for mesh in expected.findall('.//mesh'):
            mesh.set('filename', 'package://robot_description/model_sources/mechanical_958c49d/' + mesh.get('filename'))
        if name == 'base_link':
            for mesh in expected.findall('visual/geometry/mesh'):
                if mesh.get('filename').endswith('/part_001_solid_001.stl'):
                    mesh.set('filename', 'package://robot_description/model_sources/robot_v3_2_2/meshes/part_001_solid_001.stl')
        assert canonical(expected) == canonical(mobile.find(f"link[@name='{name}']"))
    for joint in mechanical.findall('joint'):
        if joint.get('name').startswith(('steer', 'wheel')):
            actual = mobile.find(f"joint[@name='{joint.get('name')}']")
            assert joint.find('origin').get('xyz') == actual.find('origin').get('xyz')
            assert canonical(joint.find('axis')) == canonical(actual.find('axis'))
            expected = copy.deepcopy(joint)
            if joint.get('name').startswith('steer'):
                rpy = expected.find('origin').get('rpy').split()
                rpy[2] = str(float(rpy[2]) - math.pi / 2)
                expected.find('origin').set('rpy', ' '.join(rpy))
                expected.find('limit').set('lower', str(-3 * math.pi / 4))
                expected.find('limit').set('upper', str(3 * math.pi / 4))
            else:
                expected.set('type', 'continuous')
                expected.find('limit').attrib.pop('lower')
                expected.find('limit').attrib.pop('upper')
                expected.find('limit').set('velocity', '31.4')
            assert canonical(expected) == canonical(actual)


def test_tree_meshes_imu_and_joint_configs():
    checked = subprocess.run(['check_urdf', str(MOBILE)], capture_output=True, text=True)
    assert checked.returncode == 0, checked.stderr
    assert 'root Link: base_footprint' in checked.stdout
    model = ET.parse(MOBILE).getroot()
    assert len(model.findall('link')) == 30
    moving = {j.get('name') for j in model.findall('joint') if j.get('type') != 'fixed'}
    assert len(moving) == 25
    for mesh in model.findall('.//mesh'):
        assert (ROOT / mesh.get('filename').removeprefix('package://robot_description/')).is_file()
    imu = model.find("joint[@name='base_link_to_imu']")
    assert imu.find('parent').get('link') == 'base_link'
    assert imu.find('origin').attrib == {'xyz': '0.403 0 0', 'rpy': '0 0 0'}
    assert len(model.find("link[@name='imu_link']")) == 0
    for filename, key in [('initial_positions_analytic_proxy_mobile.yaml', 'initial_positions'),
                           ('joint_limits_analytic_proxy_mobile.yaml', 'joints')]:
        assert set(yaml.safe_load((ROOT / 'config' / filename).read_text())[key]) == moving
    limits = yaml.safe_load((ROOT / 'config/joint_limits_analytic_proxy_mobile.yaml').read_text())['joints']
    for i in range(1, 5):
        name = f'wheel{i}_joint'
        assert float(model.find(f"joint[@name='{name}']/limit").get('velocity')) == 31.4
        assert limits[name]['max_velocity_rad_s'] == 31.4


def test_wheel_ground_clearance_and_symmetry():
    model = ET.parse(MOBILE).getroot()
    transforms = {'base_footprint': np.eye(4)}
    pending = list(model.findall('joint'))
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in transforms]
        assert ready
        for joint in ready:
            transforms[joint.find('child').get('link')] = transforms[joint.find('parent').get('link')] @ origin(joint.find('origin'))
            pending.remove(joint)
    centers = []
    for i in range(1, 5):
        name = f'wheel{i}'
        # Positive rolling direction at q=0 must point right in base_link (-Y).
        wheel_axis = transforms[name][:3, :3] @ np.array([0.0, 0.0, 1.0])
        rolling_direction = np.cross(wheel_axis, [0.0, 0.0, 1.0])
        assert np.allclose(rolling_direction, [1.0, 0.0, 0.0], atol=1e-6)
        steer_axis = transforms[f'steer{i}'][:3, :3] @ np.array([0.0, 0.0, 1.0])
        assert np.allclose(steer_axis, [0.0, 0.0, -1.0], atol=1e-6)
        visual = model.find(f"link[@name='{name}']/visual")
        mesh = visual.find('geometry/mesh')
        raw = (ROOT / mesh.get('filename').removeprefix('package://robot_description/')).read_bytes()
        count = struct.unpack_from('<I', raw, 80)[0]
        assert len(raw) == 84 + count * 50
        triangles = np.frombuffer(raw, dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]), offset=84)
        vertices = triangles['vertices'].reshape(-1,3) * np.fromstring(mesh.get('scale'), sep=' ')
        tf = transforms[name] @ origin(visual.find('origin'))
        world = vertices @ tf[:3,:3].T + tf[:3,3]
        # Faceted source tires sit within 0.5 mm of the nominal ground plane.
        assert abs(world[:,2].min()) < 0.0005
        centers.append((world.min(0) + world.max(0)) / 2)
    assert np.allclose(np.mean(centers, axis=0), [0,0,0.1], atol=1e-6)
    assert np.allclose(transforms['imu_link'][:3,3], [0,0.403,0.32])
