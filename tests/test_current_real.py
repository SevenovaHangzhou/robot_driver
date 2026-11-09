from copy import deepcopy
from hashlib import sha256
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.interpolate import CubicHermiteSpline, CubicSpline

from gripper_jog.artifacts import inspect_artifact
from gripper_jog.current_rolling import trajectory_window
from gripper_jog.errors import JogError
from gripper_jog.real_model import ArtifactMink
from test_real_observe import observer  # noqa: F401 - shared analytic dual-arm fixture


@pytest.fixture
def artifact_model(observer):
    jog, sample, _ = observer
    root = ET.fromstring(jog.chains['left'].source_xml)
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        body = root.find("link[@name='" + joint.find('child').get('link') + "']")
        inertia = ET.SubElement(body, 'inertial')
        ET.SubElement(inertia, 'mass', value='1')
        ET.SubElement(inertia, 'inertia', ixx='1', iyy='1', izz='1', ixy='0', ixz='0', iyz='0')
    xml = ET.tostring(root, encoding='unicode')
    names = [n for side in ('right', 'left') for n in jog.chains[side].names]
    parameters = {'joints': names, 'joint_position_lower': [-2.] * 14,
                  'joint_position_upper': [2.] * 14, 'joint_velocity_limits': [1.] * 14,
                  'joint_acceleration_limits': [2.] * 14, 'position_margin': .05}
    return ArtifactMink(xml, parameters), sample['observed']['positions'] | {'head_joint': .37}, parameters


@pytest.mark.parametrize('manifest', ['current', 'checksums'])
def test_actual_and_previous_artifact_formats(manifest):
    import yaml
    files = {'robot_description.xml': '<robot/>', 'controllers.yaml': 'controller_manager: {}'}
    declared = list(files) if manifest == 'current' else {n: sha256(t.encode()).hexdigest() for n, t in files.items()}
    files['artifact.yaml'] = yaml.safe_dump({'selection': {'use_mock_hardware': False}, 'files': declared})
    artifact, hashes = inspect_artifact(files)
    assert artifact['selection']['use_mock_hardware'] is False
    assert hashes['robot_description.xml'] == sha256(b'<robot/>').hexdigest()
    if manifest == 'checksums':
        files['robot_description.xml'] = '<changed/>'
        with pytest.raises(JogError, match='校验'):
            inspect_artifact(files)


@pytest.mark.parametrize('data', [
    'selection: {use_mock_hardware: true}\nfiles: [robot_description.xml, controllers.yaml]',
    'selection: {use_mock_hardware: false}\nfiles: [robot_description.xml]',
])
def test_artifact_requires_real_selection_and_declared_inputs(data):
    with pytest.raises(JogError):
        inspect_artifact({'artifact.yaml': data, 'robot_description.xml': '<robot/>', 'controllers.yaml': ''})


def test_conversion_matches_independent_urdf_fk_and_jacobian(artifact_model):
    model, positions, _ = artifact_model
    rng = np.random.default_rng(7)
    for _ in range(12):
        sample = dict(positions)
        sample.update(zip(model.names, rng.uniform(-.8, .8, 14)))
        for result in model.verify_fk(sample).values():
            assert max(result.values()) < 1e-12


def test_limits_and_feedback_are_bound_by_joint_name(artifact_model):
    model, positions, parameters = artifact_model
    bad = deepcopy(parameters)
    bad['joint_velocity_limits'][3] = 2.
    with pytest.raises(JogError, match='URDF'):
        ArtifactMink(model.xml, bad)
    with pytest.raises(JogError, match='完整'):
        model.configuration({n: q for n, q in positions.items() if n != model.names[0]})
    with pytest.raises(JogError, match='非有限'):
        model.configuration(positions | {model.names[0]: float('nan')})


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direction', ['x+', 'x-', 'y+', 'y-', 'z+', 'z-'])
def test_mink_six_directions_freeze_other_arm_and_stop_at_rest(artifact_model, side, direction):
    model, positions, _ = artifact_model
    curve, result = model.shadow_path(positions, side, direction)
    assert result['kinematic_path_ok'] and result['cubic_limits_ok']
    assert not result['collision_checked'] and not result['hardware_motion_ready']
    assert np.max(np.abs(curve([0., curve.x[-1]], 1))) < 1e-12
    frozen = [i for i, n in enumerate(model.names) if n not in model.chains[side].names]
    assert np.max(np.abs(curve(curve.x)[:, frozen] - curve(0.)[frozen])) < 1e-12
    chain = model.chains[side]
    arm_indices = [model.names.index(n) for n in chain.names]
    delta = chain.fk(curve(curve.x[-1])[arm_indices])[:3, 3] - chain.fk(curve(0.)[arm_indices])[:3, 3]
    assert delta['xyz'.index(direction[0])] == pytest.approx(.001 if direction[1] == '+' else -.001, abs=1e-7)


def decode_time(value):
    return value['sec'] * 1_000_000_000 + value['nanosec']


def test_future_replacement_reproduces_cubic_between_all_original_knots():
    times = np.arange(0, 1.01, .02)
    curve = CubicSpline(times, np.sin(times[:, None] * np.arange(1, 15) / 3) * .001,
                        bc_type=((1, np.zeros(14)), (1, np.zeros(14))))
    origin = 1_793_765_000_123_456_789
    for start in (0., .133333333, .246666667, .6):
        window = trajectory_window(curve, start, origin, .4)
        assert decode_time(window['replace_from']) == origin + round(start * 1e9)
        pts = window['points']
        relative = np.array([decode_time(p['time_from_start']) / 1e9 for p in pts])
        reconstructed = CubicHermiteSpline(relative, [p['positions'] for p in pts], [p['velocities'] for p in pts])
        sample = np.linspace(0., relative[-1], 99)
        assert reconstructed(sample) == pytest.approx(curve(start + sample), abs=1e-10)
        assert reconstructed(sample, 1) == pytest.approx(curve(start + sample, 1), abs=1e-8)
        assert window['final'] == (start == .6)
        if window['final']:
            assert pts[-1]['velocities'] == [0.] * 14


def test_final_requires_rest_and_transport_point_bound():
    moving = CubicSpline([0., 1.], np.array([[0.] * 14, [.001] * 14]))
    with pytest.raises(JogError, match='零速度'):
        trajectory_window(moving, .8, 0, .4)
    dense = CubicSpline(np.arange(300) / 1000, np.zeros((300, 14)))
    with pytest.raises(JogError, match='点数上限'):
        trajectory_window(dense, 0., 0, .4)
