import numpy as np
import pytest
from numpy.polynomial import Polynomial
from scipy.spatial.transform import Rotation

from gripper_jog.errors import JogError
from gripper_jog.model import Chain
from gripper_jog.planner import DIRECTIONS, Limits, Planner


def test_fk_independent_cartesian_reference(chain, q):
    pose = chain.fk(q)
    np.testing.assert_allclose(pose[:3, 3], q[:3], atol=1e-12)
    expected = Rotation.from_rotvec([q[3], 0, 0]) * Rotation.from_rotvec([0, q[4], 0]) \
        * Rotation.from_rotvec([0, 0, q[5] + q[6]])
    np.testing.assert_allclose(pose[:3, :3], expected.as_matrix(), atol=1e-12)


def test_jacobian_matches_finite_difference(chain, q):
    pose, jac = chain.fk_jacobian(q)
    numerical = np.zeros_like(jac)
    for i in range(7):
        qp = q.copy(); qp[i] += 1e-7
        p = chain.fk(qp)
        numerical[:3, i] = (p[:3, 3] - pose[:3, 3]) / 1e-7
        numerical[3:, i] = Rotation.from_matrix(p[:3, :3] @ pose[:3, :3].T).as_rotvec() / 1e-7
    np.testing.assert_allclose(jac, numerical, atol=1e-8)


def test_tcp_offset_rotates_with_tool(xml, q):
    a = Chain(xml, 'base', 'l7')
    b = Chain(xml, 'base', 'l7', (.1, .2, .3))
    np.testing.assert_allclose(b.fk(q)[:3, 3], a.fk(q)[:3, 3] + a.fk(q)[:3, :3] @ [.1, .2, .3])
    assert a.identity != b.identity


@pytest.mark.parametrize('direction', DIRECTIONS)
def test_six_directions_preserve_straight_line_pose_and_dynamics(planner, q, direction):
    plan = planner.step(q, direction, .0005, require_collision=False)
    start = planner.chain.fk(q)
    for t in np.linspace(0, plan.duration, 37):
        actual = planner.chain.fk(plan.state(t)[0])
        desired = start[:3, 3] + plan.parameter(t) * np.array(DIRECTIONS[direction]) * .0005
        np.testing.assert_allclose(actual[:3, 3], desired, atol=1e-6)
        np.testing.assert_allclose(actual[:3, :3], start[:3, :3], atol=1e-6)
    assert max(plan.ratios()) <= 1.0
    for t in (0, plan.duration):
        for derivative in plan.state(t)[1:]:
            np.testing.assert_allclose(derivative, 0, atol=1e-10)
    assert plan.collision_verified is False


def test_fail_closed_without_collision_checker(planner, q):
    with pytest.raises(JogError, match='碰撞'):
        planner.step(q, 'x+', .0005)


def test_entire_path_collision_and_workspace_rejected(chain, q):
    with pytest.raises(JogError) as error:
        Planner(chain, collision_check=lambda qi: qi[0] < q[0] + .0002).step(q, 'x+', .0005)
    assert error.value.code == 'collision'
    bounds = [[-1, -1, -1], [q[0] + .0002, 1, 1]]
    with pytest.raises(JogError) as error:
        Planner(chain, workspace=bounds).step(q, 'x+', .0005, require_collision=False)
    assert error.value.code == 'workspace_limit'


@pytest.mark.parametrize('direction,distance', [('wrong', .001), ('x+', float('nan')), ('x+', 0), ('x+', .1), ('x+', True)])
def test_invalid_step(planner, q, direction, distance):
    with pytest.raises(JogError):
        planner.step(q, direction, distance, require_collision=False)


def test_limit_singularity_and_duration_rejection(chain, q):
    q[0] = chain.upper[0] - .0001
    with pytest.raises(JogError) as error:
        Planner(chain).step(q, 'x+', .0005, require_collision=False)
    assert error.value.code == 'joint_limit'
    q[0] = .2; q[4] = np.pi / 2
    with pytest.raises(JogError) as error:
        Planner(chain).step(q, 'x+', .0005, require_collision=False)
    assert error.value.code == 'singularity'
    q[4] = .2
    with pytest.raises(JogError) as error:
        Planner(chain, Limits(tcp_speed=1e-8)).step(q, 'x+', .0005, require_collision=False)
    assert error.value.code == 'duration_limit'


@pytest.mark.parametrize('fraction', [.05, .3, .5, .8, .98])
def test_stop_stays_on_path_and_preserves_derivatives(planner, q, fraction):
    plan = planner.step(q, 'x+', .0005, require_collision=False)
    at = plan.duration * fraction
    brake = plan.brake(at)
    assert brake is not None
    for before, after in zip(plan.state(at), brake.state(0)):
        np.testing.assert_allclose(after, before, atol=2e-9)
    assert brake.duration <= plan.duration - at + 1e-9
    assert max(brake.ratios()) <= 1.00001
    for t in np.linspace(0, brake.duration, 51):
        assert plan.parameter(at) - 1e-9 <= brake.parameter(t) <= 1 + 1e-9
        assert brake.parameter(t, 1) >= -1e-9
    for derivative in brake.state(brake.duration)[1:]:
        np.testing.assert_allclose(derivative, 0, atol=1e-9)


@pytest.mark.parametrize('direction', DIRECTIONS)
def test_authoritative_alfa_model_six_axes(actual_chain, direction):
    plan = Planner(actual_chain).step([0, .4, 0, -1, 0, .5, 0], direction, .0005, require_collision=False)
    assert plan.metrics['max_path_error_mm'] < .01
    assert plan.metrics['max_orientation_error_deg'] < .01
    assert max(plan.ratios()) <= 1


def test_braking_reuses_validated_path_rate_without_fk_recalculation(planner, q, monkeypatch):
    plan = planner.step(q, 'x+', .0005, require_collision=False)
    def forbidden(*args):
        raise AssertionError('Stop must not repeat FK/Jacobian while holding execution lock')
    monkeypatch.setattr(planner.chain, 'fk_jacobian', forbidden)
    brake = plan.brake(plan.duration * .2)
    assert brake is not None
    assert max(brake.ratios()) <= 1.00001


@pytest.mark.parametrize('speed', [.0005, .001, .002, .005, .01])
def test_speed_profile_cruise_and_braking_are_bounded(planner, q, speed):
    plan = planner.step(q, 'x+', .005, require_collision=False, tcp_speed=speed, speed_profile=True)
    # A long smooth ramp must not consume the entire low-speed path.
    assert plan.state(.5)[0][0] - q[0] > speed * .2
    assert max(plan.ratios(samples=2001)) <= 1
    for t in (0, plan.duration):
        for derivative in plan.state(t)[1:]:
            np.testing.assert_allclose(derivative, 0, atol=1e-9)
    curve = plan.clock.curve
    for i, knot in enumerate(curve.x[1:-1]):
        before = Polynomial(curve.c[:, i][::-1])
        for n in range(4):
            np.testing.assert_allclose(before.deriv(n)(knot - curve.x[i]), plan.parameter(knot, n), atol=1e-9)
    for fraction in (.01, .2, .5, .85, .99):
        at = plan.duration * fraction
        brake = plan.brake(at)
        for before, after in zip(plan.state(at), brake.state(0)):
            np.testing.assert_allclose(after, before, atol=2e-9)
        assert max(brake.ratios(samples=2001)) <= 1.00001
        for derivative in brake.state(brake.duration)[1:]:
            np.testing.assert_allclose(derivative, 0, atol=1e-9)


def test_speed_clock_short_path_preserves_endpoint(planner, q):
    plan = planner.step(q, 'x+', .00001, require_collision=False, tcp_speed=.01, speed_profile=True)
    assert plan.parameter(plan.duration) == pytest.approx(1, abs=1e-12)
    assert max(plan.ratios(samples=2001)) <= 1
