import struct
import xml.etree.ElementTree as ET

import mink
import mujoco
import numpy as np
import pytest
from scipy.interpolate import CubicHermiteSpline

from gripper_jog.errors import JogError
from gripper_jog.real_collision import CollisionPolicy
from gripper_jog.real_model import ArtifactMink
from test_current_real import artifact_model  # noqa: F401 - analytic model fixture
from test_real_observe import observer  # noqa: F401 - fixture dependency


POLICY = CollisionPolicy(.03, .12, .05)


@pytest.fixture
def collision_model(artifact_model):
    original, _, parameters = artifact_model
    root = ET.fromstring(original.xml)
    for side, y in (('left', -.07), ('right', .07)):
        first = root.find(f"joint[@name='{side}_joint1']")
        ET.SubElement(first, 'origin', xyz=f'.12 {y} 0')
        for i in range(1, 8):
            link = root.find(f"link[@name='{side}_link{i}']")
            shape = ET.SubElement(ET.SubElement(link, 'collision'), 'geometry')
            ET.SubElement(shape, 'sphere', radius='.04')
            if i > 1:
                joint = root.find(f"joint[@name='{side}_joint{i}']")
                ET.SubElement(joint, 'origin', xyz='.12 0 0')
    model = ArtifactMink(ET.tostring(root, encoding='unicode'), parameters, collision_policy=POLICY)
    return model, {n: 0. for n in model.names}


def pair_links(model):
    return {frozenset(model.model.body(int(model.model.geom_bodyid[g])).name for g in pair)
            for pair in model.collision.pairs}


def test_both_arms_and_nonadjacent_self_pairs_remain_monitored(collision_model):
    model, _ = collision_model
    pairs = pair_links(model)
    for side in ('left', 'right'):
        assert frozenset((side + '_link1', side + '_link2')) not in pairs
        assert frozenset((side + '_link1', side + '_link3')) in pairs
        assert frozenset((side + '_link5', side + '_link7')) in pairs
    for left in range(1, 8):
        for right in range(1, 8):
            assert frozenset((f'left_link{left}', f'right_link{right}')) in pairs
    assert model.collision.counts == {'between_arms': 49, 'left_self': 15, 'right_self': 15}
    assert model.collision.info()['hardware_validated'] is False


@pytest.mark.parametrize('dt', [.01, .1])
def test_qp_stops_approach_and_keeps_the_other_arm_frozen(collision_model, dt):
    model, positions = collision_model
    configuration = model.configuration(positions)
    task = mink.FrameTask('left_tool0', 'body', position_cost=1., orientation_cost=1.)
    target = configuration.get_transform_frame_to_world('left_tool0', 'body')
    task.set_target(mink.SE3.from_rotation_and_translation(target.rotation(), target.translation() + [0., .2, 0.]))
    selected = model.dofs['left_joint2']
    freeze = mink.DofFreezingTask(model.model, [d for d in range(model.model.nv) if d != selected])
    initial = configuration.q.copy()
    # Verify the pinned Mink bound is converted to displacement, not left with
    # a 1/dt factor that would allow the first step to cross the obstacle.
    native = mink.CollisionAvoidanceLimit(
        model.model, [([a], [b]) for a, b in model.collision.pairs], gain=POLICY.gain,
        minimum_distance_from_collisions=.03, collision_detection_distance=.12)
    actual = model.collision.limit.compute_qp_inequalities(configuration, dt)
    expected = native.compute_qp_inequalities(configuration, dt)
    finite = np.isfinite(actual.h)
    assert actual.h[finite] == pytest.approx(expected.h[finite] * dt)
    for _ in range(160):
        velocity = mink.solve_ik(configuration, [task], dt, solver='daqp',
                                 limits=[mink.ConfigurationLimit(model.model), model.collision.limit],
                                 constraints=[freeze])
        configuration.integrate_inplace(velocity, dt)
        assert model.collision.check_configuration(configuration) >= .03
    assert .029 < configuration.q[model.qpos['left_joint2']] < .030001
    frozen = [i for i in range(model.model.nq) if i != model.qpos['left_joint2']]
    assert configuration.q[frozen] == pytest.approx(initial[frozen], abs=1e-12)


def test_initial_overlap_is_rejected_before_solving(collision_model):
    model, positions = collision_model
    positions['left_joint2'] = .1
    with pytest.raises(JogError, match='碰撞余量') as error:
        model.shadow_path(positions, 'left', 'x+')
    assert error.value.code == 'real_collision'


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direction', ['x+', 'x-', 'y+', 'y-', 'z+', 'z-'])
def test_shadow_generation_checks_collisions_and_preserves_other_arm(collision_model, side, direction):
    model, positions = collision_model
    curve, result = model.shadow_path(positions, side, direction)
    assert result['kinematic_path_ok'] and result['collision_checked']
    assert not result['hardware_motion_ready']
    assert result['collision']['minimum_sampled_distance_m'] >= .03
    frozen = [i for i, n in enumerate(model.names) if not n.startswith(side + '_')]
    assert np.max(np.abs(curve(curve.x)[:, frozen])) < 1e-12


def excursion_curve(model, coefficient):
    q = np.zeros((2, 14))
    v = np.zeros_like(q)
    v[:, model.names.index('left_joint2')] = coefficient / 2
    # y(t)=coefficient*t*(t-.5)*(t-1): endpoints AND midpoint are clear,
    # but an interior extremum can penetrate the opposite arm.
    return CubicHermiteSpline([0., 1.], q, v, axis=0)


def test_interval_check_catches_collision_missed_by_three_samples(collision_model):
    model, positions = collision_model
    configuration = model.configuration(positions)
    curve = excursion_curve(model, 1.)
    for at in (0., .5, 1.):
        q = configuration.q.copy()
        q[model.collision.qadr] = curve(at)
        configuration.update(q)
        assert model.collision.check_configuration(configuration) >= .03
    with pytest.raises(JogError, match='碰撞余量'):
        model.collision.check_curve(curve, configuration)


def test_clear_curve_passes_without_changing_input_configuration(collision_model):
    model, positions = collision_model
    configuration = model.configuration(positions)
    original = configuration.q.copy()
    result = model.collision.check_curve(excursion_curve(model, .2), configuration)
    assert result['method'] == 'cubic_interval_motion_bounds'
    assert result['interval_checks'] >= 1
    assert result['minimum_sampled_distance_m'] >= .03
    assert np.array_equal(configuration.q, original)


def test_uncertain_interval_is_rejected_when_subdivision_budget_exhausted(collision_model):
    model, positions = collision_model
    model.collision.policy = CollisionPolicy(.03, .12, .05, max_subdivision_depth=0)
    with pytest.raises(JogError, match='无法证明') as error:
        model.collision.check_curve(excursion_curve(model, 1.), model.configuration(positions))
    assert error.value.code == 'collision_uncertain'


def test_interval_budget_exhaustion_rejects_instead_of_skipping_remaining_curve(collision_model):
    model, positions = collision_model
    model.collision.policy = CollisionPolicy(.03, .12, .05, max_interval_checks=1)
    q = np.zeros((3, 14))
    curve = CubicHermiteSpline([0., .5, 1.], q, q, axis=0)
    with pytest.raises(JogError, match='预算耗尽'):
        model.collision.check_curve(curve, model.configuration(positions))


def test_self_collision_is_not_removed_due_to_reference_pose(collision_model):
    model, _ = collision_model
    root = ET.fromstring(model.xml)
    origin = root.find("joint[@name='left_joint3']/origin")
    origin.set('xyz', '-.1 0 0')
    overlapping = ArtifactMink(ET.tostring(root, encoding='unicode'), model.parameters, collision_policy=POLICY)
    assert frozenset(('left_link1', 'left_link3')) in pair_links(overlapping)
    with pytest.raises(JogError, match='碰撞余量'):
        overlapping.collision.check_configuration(overlapping.configuration({n: 0. for n in model.names}))


def tetrahedron_stl():
    vertices = np.array([[0., 0., 0.], [.01, 0., 0.], [0., .01, 0.], [0., 0., .01]], dtype=np.float32)
    faces = [(0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)]
    data = b'test tetrahedron'.ljust(80, b' ') + struct.pack('<I', len(faces))
    for face in faces:
        data += struct.pack('<12fH', *([0.] * 3), *vertices[list(face)].ravel(), 0)
    return data, vertices


def test_artifact_collision_mesh_preserves_origin_scale_and_source_hash(collision_model, tmp_path):
    from hashlib import sha256
    from scipy.spatial.transform import Rotation
    model, positions = collision_model
    raw, vertices = tetrahedron_stl()
    path = tmp_path / 'mesh/collision.STL'
    path.parent.mkdir()
    path.write_bytes(raw)
    root = ET.fromstring(model.xml)
    collision = root.find("link[@name='left_link7']/collision")
    collision.clear()
    ET.SubElement(collision, 'origin', xyz='.03 .02 .01', rpy='.2 -.3 .4')
    geometry = ET.SubElement(collision, 'geometry')
    ET.SubElement(geometry, 'mesh', filename='package://robot_description/mesh/collision.STL', scale='2 1 3')
    imported = ArtifactMink(ET.tostring(root, encoding='unicode'), model.parameters,
                            model_share=tmp_path, collision_policy=POLICY)
    assert imported.collision.sources[0]['sha256'] == sha256(raw).hexdigest()
    configuration = imported.configuration(positions)
    g = imported.model.geom('jog_collision_left_link7_0').id
    mesh = int(imported.model.geom_dataid[g])
    start = imported.model.mesh_vertadr[mesh]
    actual = imported.model.mesh_vert[start:start + imported.model.mesh_vertnum[mesh]]
    actual = configuration.data.geom_xpos[g] + actual @ configuration.data.geom_xmat[g].reshape(3, 3).T
    body = configuration.data.body('left_link7')
    expected = (vertices * [2., 1., 3.]) @ Rotation.from_euler('xyz', [.2, -.3, .4]).as_matrix().T + [.03, .02, .01]
    expected = body.xpos + expected @ body.xmat.reshape(3, 3).T
    assert np.max(np.min(np.linalg.norm(actual[:, None, :] - expected[None, :, :], axis=-1), axis=0)) < 1e-7


@pytest.mark.parametrize('uri', ['package://robot_description/../outside.STL',
                                  'package://different/mesh.STL', 'file:///tmp/mesh.STL',
                                  'package://robot_description/missing.STL'])
def test_missing_or_external_mesh_never_falls_back_to_visual_or_no_collision(collision_model, tmp_path, uri):
    model, _ = collision_model
    root = ET.fromstring(model.xml)
    geometry = root.find("link[@name='left_link1']/collision/geometry")
    geometry.clear()
    ET.SubElement(geometry, 'mesh', filename=uri)
    with pytest.raises(JogError):
        ArtifactMink(ET.tostring(root, encoding='unicode'), model.parameters,
                     model_share=tmp_path, collision_policy=POLICY)


def test_missing_controlled_link_collision_is_rejected(collision_model):
    model, _ = collision_model
    root = ET.fromstring(model.xml)
    link = root.find("link[@name='right_link6']")
    link.remove(link.find('collision'))
    with pytest.raises(JogError, match='缺少碰撞几何'):
        ArtifactMink(ET.tostring(root, encoding='unicode'), model.parameters, collision_policy=POLICY)


@pytest.mark.parametrize('change', [{'minimum_distance_m': 0.}, {'detection_distance_m': .01},
                                   {'gain': 1.1}, {'max_subdivision_depth': -1},
                                   {'gain': True}, {'gain': '0.05'}, {'max_interval_checks': 0}])
def test_collision_policy_requires_valid_explicit_values(change):
    config = dict(minimum_distance_m=.03, detection_distance_m=.12, gain=.05)
    with pytest.raises(JogError):
        CollisionPolicy(**(config | change))
