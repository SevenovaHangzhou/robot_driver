"""Whole-curve and halt regressions for acceleration-constrained Mink output."""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.interpolate import CubicHermiteSpline

from gripper_jog.real_source import RealRollingSource, PLANNING_SCALE, acceleration_bounds
from gripper_jog.rolling_stream import Knot, validate_window
from test_real_collision import collision_model, artifact_model, observer  # noqa: F401


def test_endpoint_constraints_keep_interior_speed_and_next_anchor_viable():
    rng = np.random.default_rng(771107)
    dt, speed, accel = .02, np.full(14, math.pi / 6), np.full(14, math.pi / 3)
    knot = Knot(0, np.zeros(14), speed - accel * dt / 2, accel.copy())
    # Stress both ends of the QP acceleration interval, rather than sampling
    # only slow commands that never reach a dynamic constraint.
    for i in range(100):
        lo, hi = acceleration_bounds(knot, dt, speed, accel)
        assert np.all(lo <= hi + 1e-10)
        a = np.where(rng.random(14) > .5, lo, hi)
        q = knot.q + knot.v * dt + (knot.a / 3 + a / 6) * dt**2
        v = knot.v + .5 * (knot.a + a) * dt
        curve = CubicHermiteSpline([0, dt], [knot.q, q], [knot.v, v])
        assert np.max(np.abs(curve(np.linspace(0, dt, 101), 1)) - speed) < 1e-10
        assert np.max(np.abs(v + .5 * dt * a) - speed) < 1e-10
        knot = Knot((i + 1) * 20_000_000, q, v, a)


@pytest.mark.parametrize('sign', [-1, 1])
def test_halt_from_high_speed_with_outward_acceleration_stays_inside_hardware_limits(sign):
    source = RealRollingSource.__new__(RealRollingSource)
    speed, accel = math.pi / 6, math.pi / 3
    source.acceleration_limits = np.full(14, accel * PLANNING_SCALE)
    source.stop_knots = source.stop_until_ns = None
    a = np.full(14, sign * accel * PLANNING_SCALE)
    v = np.full(14, sign * speed * PLANNING_SCALE) - .01 * a
    anchor = Knot(0, np.full(14, 2.), v, a)
    source.stop_rollout(anchor, [20_000_000])
    knots = list(source.stop_knots.values())
    curve = validate_window(knots, -100., 100., speed, accel, np.inf)
    assert np.max(np.abs(curve(np.linspace(0, knots[-1].time_ns / 1e9, 2001), 1))) <= speed
    assert np.array_equal(knots[-1].v, np.zeros(14))
    assert np.array_equal(knots[-1].a, np.zeros(14))
    assert np.allclose(curve(0., 2), anchor.a, atol=2e-7)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('velocity', [[.02, 0., 0.], [0., .02, 0.], [0., 0., .02]])
def test_real_qp_reserves_acceleration_and_full_curve_speed(collision_model, side, velocity):
    model, positions = collision_model
    before = model.bounds.copy()
    source = RealRollingSource(model, positions, side)
    knots = [source.anchor, *source.rollout(source.anchor, range(20_000_000, 420_000_000, 20_000_000), velocity)]
    curve = source.validate(knots)
    assert np.array_equal(model.bounds, before)
    assert np.max(np.abs(curve(np.linspace(0., .4, 1001), 2)) - model.bounds[3] * PLANNING_SCALE) < 2e-7
    for k in knots:
        assert np.max(np.abs(k.v + .01 * k.a) - source.speed_limits) < 1e-8
