import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rt_control_operator_web.teleop import (  # noqa: E402
    COMMAND_TIMEOUT_S,
    MAX_TICK_DT_S,
    STICK_DEADZONE,
    TRAILING_ZERO_COUNT,
    ZERO,
    AxisLimits,
    DriveArbiter,
    Gear,
    JerkLimitedFollower,
    MotionLimits,
    TeleopError,
    shape_stick,
    validate_gears,
    validate_limits,
)

DT = 0.02
GEARS = {"leisure": Gear(0.05, 0.1), "sport": Gear(0.15, 0.3)}
LIMITS = MotionLimits(AxisLimits(0.3, 0.5, 1.0, 2.0), AxisLimits(0.6, 1.0, 2.0, 4.0))


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def arbiter(clock: FakeClock) -> DriveArbiter:
    arbiter = DriveArbiter(GEARS, LIMITS, clock)
    arbiter.tick()  # establish the tick time base
    return arbiter


def run(arbiter, clock, steps, before=None):
    out = []
    for _ in range(steps):
        clock.now += DT
        if before:
            before()
        out.append(arbiter.tick())
    return out


def derivatives(values):
    first = [(values[i] - values[i - 1]) / DT for i in range(1, len(values))]
    second = [(first[i] - first[i - 1]) / DT for i in range(1, len(first))]
    return first, second


# --- stick shaping ---------------------------------------------------------
def test_deadzone() -> None:
    assert shape_stick(STICK_DEADZONE * 0.99, 0.0) == (0.0, 0.0)
    assert shape_stick(0.05, -0.05) == (0.0, 0.0)


def test_full_deflection_and_quadratic_curve() -> None:
    assert shape_stick(1.0, 0.0) == pytest.approx((1.0, 0.0))
    half = STICK_DEADZONE + (1 - STICK_DEADZONE) * 0.5
    assert shape_stick(half, 0.0)[0] == pytest.approx(0.25)


def test_magnitude_clamped_to_unit() -> None:
    assert math.hypot(*shape_stick(3.0, 4.0)) == pytest.approx(1.0)


@pytest.mark.parametrize(
    "angle_deg, expected",
    [(5.0, (1.0, 0.0)), (95.0, (0.0, 1.0)), (-176.0, (-1.0, 0.0)), (-84.0, (0.0, -1.0))],
)
def test_axis_snap(angle_deg, expected) -> None:
    a = math.radians(angle_deg)
    assert shape_stick(math.cos(a), math.sin(a)) == pytest.approx(expected)


def test_diagonal_not_snapped() -> None:
    x, y = shape_stick(math.cos(math.radians(30)), math.sin(math.radians(30)))
    assert math.degrees(math.atan2(y, x)) == pytest.approx(30.0)


# --- S-curve follower -------------------------------------------------------
@pytest.mark.parametrize("start, target", [(0.0, 0.15), (0.15, 0.0), (0.05, -0.15)])
def test_follower_respects_accel_and_jerk(start, target) -> None:
    limits = LIMITS.linear
    follower = JerkLimitedFollower(1)
    follower.value = [start]
    values = [start]
    for _ in range(200):
        follower.step([target], DT, limits, stopping=False)
        values.append(follower.value[0])
    accel, jerk = derivatives(values)
    assert values[-1] == target
    assert max(abs(a) for a in accel) <= max(limits.accel, limits.decel) + 1e-9
    assert max(abs(j) for j in jerk) <= limits.jerk + 1e-6


def test_follower_is_monotonic_without_overshoot() -> None:
    follower = JerkLimitedFollower(1)
    values = []
    for _ in range(100):
        follower.step([0.15], DT, LIMITS.linear, stopping=False)
        values.append(follower.value[0])
    assert all(b >= a for a, b in zip(values, values[1:]))
    assert max(values) == 0.15


# --- lease ------------------------------------------------------------------
def test_nothing_published_without_lease(arbiter: DriveArbiter) -> None:
    assert arbiter.tick() is None


def test_single_lease_is_exclusive(arbiter: DriveArbiter) -> None:
    arbiter.acquire("a")
    arbiter.acquire("a")
    with pytest.raises(TeleopError) as err:
        arbiter.acquire("b")
    assert err.value.code == "lease_busy"


def test_drive_and_gear_require_lease(arbiter: DriveArbiter) -> None:
    with pytest.raises(TeleopError) as err:
        arbiter.drive("a", 1.0, 0.0, 0)
    assert err.value.code == "lease_required"
    with pytest.raises(TeleopError):
        arbiter.set_gear("a", "sport")


def test_holder_publishes_zero_while_idle(arbiter, clock) -> None:
    arbiter.acquire("a")
    assert run(arbiter, clock, 3) == [ZERO] * 3


# --- motion -----------------------------------------------------------------
@pytest.mark.parametrize(
    "x, y, yaw, expected",
    [
        (1.0, 0.0, 0, (0.05, 0.0, 0.0)),
        (-1.0, 0.0, 0, (-0.05, 0.0, 0.0)),
        (0.0, 1.0, 0, (0.0, 0.05, 0.0)),
        (0.0, -1.0, 0, (0.0, -0.05, 0.0)),
        (0.0, 0.0, 1, (0.0, 0.0, 0.1)),
        (0.0, 0.0, -1, (0.0, 0.0, -0.1)),
    ],
)
def test_rep103_mapping_reaches_gear_target(arbiter, clock, x, y, yaw, expected) -> None:
    arbiter.acquire("a")
    out = run(arbiter, clock, 150, lambda: arbiter.drive("a", x, y, yaw))
    assert out[-1] == pytest.approx(expected)


def test_ramp_up_is_gradual(arbiter, clock) -> None:
    arbiter.acquire("a")
    arbiter.set_gear("a", "sport")
    out = run(arbiter, clock, 5, lambda: arbiter.drive("a", 1.0, 0.0, 0))
    assert 0.0 < out[0][0] < 0.002
    assert all(b[0] > a[0] for a, b in zip(out, out[1:]))


def test_release_ramps_down_smoothly_not_to_zero(arbiter, clock) -> None:
    arbiter.acquire("a")
    arbiter.set_gear("a", "sport")
    run(arbiter, clock, 150, lambda: arbiter.drive("a", 1.0, 0.0, 0))
    arbiter.drive("a", 0.0, 0.0, 0)
    out = run(arbiter, clock, 60, lambda: arbiter.drive("a", 0.0, 0.0, 0))
    speeds = [0.15] + [c[0] for c in out]
    assert speeds[1] > 0.14  # no step to zero on release
    accel, jerk = derivatives(speeds)
    assert all(a <= 1e-12 for a in accel)  # monotonic slowdown
    assert max(abs(a) for a in accel) <= LIMITS.linear.decel + 1e-9
    assert max(abs(j) for j in jerk[1:]) <= LIMITS.linear.jerk + 1e-6
    assert speeds[-1] == 0.0
    assert arbiter.snapshot().last_stop_reason == "stick_released"


def test_diagonal_release_keeps_direction(arbiter, clock) -> None:
    arbiter.acquire("a")
    arbiter.set_gear("a", "sport")
    s = math.sqrt(0.5)
    run(arbiter, clock, 150, lambda: arbiter.drive("a", s, s, 0))
    out = run(arbiter, clock, 60, lambda: arbiter.drive("a", 0.0, 0.0, 0))
    assert all(c[0] == pytest.approx(c[1]) for c in out)


def test_rounding_slack_above_unit_disc_is_accepted(arbiter, clock) -> None:
    arbiter.acquire("a")
    arbiter.drive("a", 0.316, 0.949, 0)  # |v| = 1.0003 after client rounding
    assert math.hypot(*arbiter.snapshot().target[:2]) == pytest.approx(0.05)


def test_mixed_translation_and_yaw_rejected(arbiter, clock) -> None:
    arbiter.acquire("a")
    run(arbiter, clock, 30, lambda: arbiter.drive("a", 1.0, 0.0, 0))
    with pytest.raises(TeleopError) as err:
        arbiter.drive("a", 1.0, 0.0, 1)
    assert err.value.code == "mixed_input_rejected"
    assert arbiter.snapshot().target == ZERO
    # Yaw with a stick inside the deadzone is not mixed input.
    arbiter.drive("a", 0.05, 0.0, 1)


@pytest.mark.parametrize(
    "x, y, yaw", [(1.0, 1.0, 0), (1.02, 0.0, 0), (float("nan"), 0.0, 0), (0.0, 0.0, 2)]
)
def test_invalid_input_rejected_and_stops(arbiter, x, y, yaw) -> None:
    arbiter.acquire("a")
    arbiter.drive("a", 1.0, 0.0, 0)
    with pytest.raises(TeleopError) as err:
        arbiter.drive("a", x, y, yaw)
    assert err.value.code == "invalid_input"
    assert arbiter.snapshot().target == ZERO


def test_watchdog_ramps_down_and_revokes_lease(arbiter, clock) -> None:
    arbiter.acquire("a")
    arbiter.set_gear("a", "sport")
    run(arbiter, clock, 150, lambda: arbiter.drive("a", 1.0, 0.0, 0))
    steps = int(COMMAND_TIMEOUT_S / DT) + 2
    out = run(arbiter, clock, steps + 60)
    snap = arbiter.snapshot()
    assert snap.lease_holder is None
    assert snap.last_stop_reason == "command_timeout"
    speeds = [c[0] for c in out if c is not None]
    assert speeds[steps] > 0.1  # still ramping, not cut
    assert speeds[-1] == 0.0
    assert out[-1] == ZERO or out[-1] is None


@pytest.mark.parametrize("end", ["release", "disconnect"])
def test_lease_end_ramps_then_trailing_zeros_then_silence(arbiter, clock, end) -> None:
    arbiter.acquire("a")
    run(arbiter, clock, 150, lambda: arbiter.drive("a", 1.0, 0.0, 0))
    getattr(arbiter, end)("a")
    out = run(arbiter, clock, 100)
    moving = [c for c in out if c is not None and c != ZERO]
    assert moving and moving[0][0] > 0.04
    assert out[-1] is None
    zeros_after = out[len(moving):]
    assert zeros_after[:TRAILING_ZERO_COUNT] == [ZERO] * TRAILING_ZERO_COUNT


def test_soft_stop_uses_faster_ramp(clock) -> None:
    def stop_time(fast: bool) -> float:
        arb = DriveArbiter(GEARS, LIMITS, clock)
        arb.tick()
        arb.acquire("a")
        arb.set_gear("a", "sport")
        run(arb, clock, 150, lambda: arb.drive("a", 1.0, 0.0, 0))
        if fast:
            arb.soft_stop()
        else:
            arb.release("a")
        out = run(arb, clock, 100)
        return next(i for i, c in enumerate(out) if c == ZERO or c is None) * DT

    fast, normal = stop_time(True), stop_time(False)
    assert fast < 0.8 * normal


def test_soft_stop_revokes_and_frees_lease(arbiter, clock) -> None:
    arbiter.acquire("a")
    arbiter.soft_stop()
    assert arbiter.snapshot().lease_holder is None
    assert arbiter.snapshot().last_stop_reason == "soft_stop"
    arbiter.acquire("b")


def test_soft_stop_without_lease_still_emits_zeros(arbiter, clock) -> None:
    arbiter.soft_stop()
    assert run(arbiter, clock, 1) == [ZERO]


def test_gear_change_only_when_stopped(arbiter, clock) -> None:
    arbiter.acquire("a")
    arbiter.set_gear("a", "sport")
    arbiter.drive("a", 1.0, 0.0, 0)
    run(arbiter, clock, 5, lambda: arbiter.drive("a", 1.0, 0.0, 0))
    with pytest.raises(TeleopError) as err:
        arbiter.set_gear("a", "leisure")
    assert err.value.code == "gear_change_while_moving"
    arbiter.drive("a", 0.0, 0.0, 0)
    run(arbiter, clock, 100, lambda: arbiter.drive("a", 0.0, 0.0, 0))
    arbiter.set_gear("a", "leisure")
    assert arbiter.snapshot().gear == "leisure"
    with pytest.raises(TeleopError):
        arbiter.set_gear("a", "turbo")


def test_halt_and_non_holder_calls(arbiter, clock) -> None:
    arbiter.acquire("a")
    arbiter.drive("a", 1.0, 0.0, 0)
    arbiter.halt("b")
    arbiter.release("b")
    arbiter.disconnect("b")
    assert arbiter.snapshot().target != ZERO
    arbiter.halt("a")
    assert arbiter.snapshot().target == ZERO
    assert arbiter.snapshot().lease_holder == "a"


def test_large_tick_gap_is_bounded(arbiter, clock) -> None:
    arbiter.acquire("a")
    arbiter.drive("a", 1.0, 0.0, 0)
    clock.now += 0.15  # still inside the input timeout
    command = arbiter.tick()
    assert 0.0 < command[0] <= LIMITS.linear.accel * MAX_TICK_DT_S


@pytest.mark.parametrize(
    "gears",
    [{}, {"a": Gear(0.0, 0.1)}, {"a": Gear(0.1, float("nan"))}, {"bad name": Gear(0.1, 0.1)}],
)
def test_invalid_gears_rejected(gears) -> None:
    with pytest.raises(ValueError):
        validate_gears(gears)


@pytest.mark.parametrize(
    "limits",
    [
        MotionLimits(AxisLimits(0.0, 0.5, 1.0, 2.0), LIMITS.angular),
        MotionLimits(LIMITS.linear, AxisLimits(0.6, 1.0, 0.5, 4.0)),
        MotionLimits(AxisLimits(0.3, 0.5, 1.0, float("inf")), LIMITS.angular),
    ],
)
def test_invalid_limits_rejected(limits) -> None:
    with pytest.raises(ValueError):
        validate_limits(limits)


def test_invalid_timeout_rejected(clock) -> None:
    with pytest.raises(ValueError):
        DriveArbiter(GEARS, LIMITS, clock, command_timeout=0.0)
