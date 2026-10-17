"""S-curve limits, synchronisation and smooth stop. Pure Python."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rt_control_arm_console.trajectory import Limits, plan_move, plan_stop, scurve  # noqa: E402

DEG = math.radians
LEISURE = Limits(DEG(10), DEG(20), DEG(80))
SPORT = Limits(DEG(30), DEG(60), DEG(240))


def _peaks(points, index=0, period=0.01):
    velocity = max(abs(p.velocities[index]) for p in points)
    acceleration = max(abs(p.accelerations[index]) for p in points)
    jerk = max(
        abs(b.accelerations[index] - a.accelerations[index]) / (b.time - a.time)
        for a, b in zip(points, points[1:])
        if b.time - a.time > period / 2
    )
    return velocity, acceleration, jerk


@pytest.mark.parametrize("limits", [LEISURE, SPORT])
@pytest.mark.parametrize("distance_deg", [0.2, 3.0, 20.0, 170.0])
def test_profile_respects_limits_and_ends_at_rest(limits, distance_deg):
    plan = plan_move([0.0, 0.1], [DEG(distance_deg), 0.1 - DEG(distance_deg) / 2], limits)
    points = plan.points(0.01)
    velocity, acceleration, jerk = _peaks(points)
    assert velocity <= limits.velocity * (1 + 1e-9)
    assert acceleration <= limits.acceleration * (1 + 1e-9)
    assert jerk <= limits.jerk * (1 + 1e-6)
    end = points[-1]
    assert end.positions == pytest.approx(
        (DEG(distance_deg), 0.1 - DEG(distance_deg) / 2), abs=1e-9
    )
    assert end.velocities == (0.0, 0.0) and end.accelerations == (0.0, 0.0)
    assert points[0].positions == pytest.approx((0.0, 0.1)) and points[0].time == 0.0
    # Smooth start: zero velocity and acceleration at t = 0.
    assert points[0].velocities == (0.0, 0.0) and points[0].accelerations == (0.0, 0.0)


def test_long_move_reaches_the_gear_velocity_and_known_duration():
    plan = plan_move([0.0], [DEG(90)], LEISURE)
    # 90 deg at 10 deg/s plus one acceleration phase (v/a + a/j = 0.5 + 0.25 s).
    assert plan.duration == pytest.approx(9.75)
    assert _peaks(plan.points(0.01))[0] == pytest.approx(LEISURE.velocity, rel=1e-6)


def test_joints_are_synchronised_and_proportional():
    plan = plan_move([0.0, 0.0, 0.0], [DEG(60), DEG(-15), 0.0], SPORT)
    for point in plan.points(0.05):
        assert point.positions[1] == pytest.approx(-point.positions[0] / 4)
        assert point.positions[2] == 0.0


def test_zero_distance_is_an_empty_profile():
    assert plan_move([0.3], [0.3], SPORT).duration == 0.0
    assert scurve(0.0, SPORT).duration == 0.0


def test_stop_continues_the_running_state_and_brakes_within_limits():
    plan = plan_move([0.0], [DEG(90)], SPORT)
    elapsed = 1.5  # cruising at 30 deg/s
    stop = plan_stop(plan, elapsed, SPORT)
    running = plan.state(elapsed)
    assert stop.start == pytest.approx(running.positions)
    assert stop.state(0.0).velocities == pytest.approx(running.velocities)
    points = stop.points(0.01)
    assert _peaks(points)[1] <= SPORT.acceleration * (1 + 1e-9)
    assert points[-1].velocities == (0.0,)
    # 30 deg/s with a = 60, j = 240: ramp 0.25 s, hold 0.25 s -> 0.75 s and 11.25 deg.
    assert stop.duration == pytest.approx(0.75)
    assert math.degrees(stop.target[0] - stop.start[0]) == pytest.approx(11.25)


def test_stop_near_the_end_never_passes_the_target():
    plan = plan_move([0.0], [DEG(30)], SPORT)
    stop = plan_stop(plan, plan.duration - 0.1, SPORT)
    assert stop.target[0] <= DEG(30) + 1e-12
    assert stop.target[0] == pytest.approx(DEG(30))
    assert plan_stop(plan, plan.duration + 1.0, SPORT).duration == 0.0
