"""Jerk-limited (S-curve) joint trajectories on a straight joint-space line. No ROS.

All joints share one path parameter u (the distance travelled by the joint with the
largest move): q_i(t) = origin_i + direction_i * u(t), with |direction_i| <= 1. The
leading joint therefore uses the full velocity, acceleration and jerk limits and
every other joint stays proportionally below them. Moves start and end at rest
with zero acceleration; jerk is piecewise constant (seven-segment S-curve).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

EPSILON = 1e-12


@dataclass(frozen=True)
class Limits:
    """Path limits in joint units (rad, rad/s, rad/s^2, rad/s^3)."""

    velocity: float
    acceleration: float
    jerk: float

    def __post_init__(self) -> None:
        for name in ("velocity", "acceleration", "jerk"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} limit must be positive and finite")


@dataclass(frozen=True)
class Profile:
    """Piecewise-constant jerk on the path parameter from the given initial state."""

    segments: tuple[tuple[float, float], ...] = ()  # (duration s, jerk)
    initial_velocity: float = 0.0
    initial_acceleration: float = 0.0

    @property
    def duration(self) -> float:
        return sum(duration for duration, _ in self.segments)

    def sample(self, time: float) -> tuple[float, float, float]:
        """(u, du/dt, d2u/dt2) at ``time``; exact piecewise cubic, clamped to the profile."""
        position, velocity, acceleration = 0.0, self.initial_velocity, self.initial_acceleration
        remaining = min(max(time, 0.0), self.duration)
        for duration, jerk in self.segments:
            step = min(duration, remaining)
            position += velocity * step + acceleration * step**2 / 2 + jerk * step**3 / 6
            velocity += acceleration * step + jerk * step**2 / 2
            acceleration += jerk * step
            remaining -= step
            if remaining <= 0.0:
                break
        if time >= self.duration:
            velocity = acceleration = 0.0
        return position, velocity, acceleration


def _segments(*items: tuple[float, float]) -> tuple[tuple[float, float], ...]:
    return tuple(item for item in items if item[0] > EPSILON)


def scurve(distance: float, limits: Limits) -> Profile:
    """Time-optimal symmetric S-curve from rest to rest over ``distance`` >= 0."""
    if not math.isfinite(distance) or distance < 0.0:
        raise ValueError("distance must be finite and non-negative")
    v, a, j = limits.velocity, limits.acceleration, limits.jerk
    if distance <= EPSILON:
        return Profile()
    if v * j >= a * a:  # the acceleration limit is reached before the velocity limit
        ramp = a / j
        accel_time = ramp + v / a
    else:
        ramp = math.sqrt(v / j)
        accel_time = 2.0 * ramp
    cruise = distance / v - accel_time
    if cruise < 0.0:  # the velocity limit is not reached
        cruise = 0.0
        ramp = a / j
        accel_time = (a * a / j + math.sqrt(a**4 / j**2 + 4.0 * a * distance)) / (2.0 * a)
        if accel_time < 2.0 * ramp:  # the acceleration limit is not reached either
            ramp = (distance / (2.0 * j)) ** (1.0 / 3.0)
            accel_time = 2.0 * ramp
    hold = max(accel_time - 2.0 * ramp, 0.0)
    return Profile(
        _segments(
            (ramp, j), (hold, 0.0), (ramp, -j), (cruise, 0.0), (ramp, -j), (hold, 0.0), (ramp, j)
        )
    )


def brake(velocity: float, limits: Limits) -> Profile:
    """Jerk-limited deceleration from path ``velocity`` (zero acceleration) to rest."""
    velocity = max(velocity, 0.0)
    a, j = limits.acceleration, limits.jerk
    if velocity <= EPSILON:
        return Profile()
    if velocity * j >= a * a:
        ramp = a / j
        hold = velocity / a - ramp
    else:
        ramp = math.sqrt(velocity / j)
        hold = 0.0
    return Profile(_segments((ramp, -j), (hold, 0.0), (ramp, j)), velocity)


@dataclass(frozen=True)
class TrajectoryPoint:
    time: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    accelerations: tuple[float, ...]


@dataclass(frozen=True)
class Plan:
    """A path profile applied to the line ``origin + direction * u``."""

    origin: tuple[float, ...]
    direction: tuple[float, ...]
    profile: Profile

    @property
    def duration(self) -> float:
        return self.profile.duration

    @property
    def start(self) -> tuple[float, ...]:
        return self.state(0.0).positions

    @property
    def target(self) -> tuple[float, ...]:
        return self.state(self.duration).positions

    def state(self, time: float) -> TrajectoryPoint:
        u, du, ddu = self.profile.sample(time)
        return TrajectoryPoint(
            time,
            tuple(o + d * u for o, d in zip(self.origin, self.direction)),
            tuple(d * du for d in self.direction),
            tuple(d * ddu for d in self.direction),
        )

    def points(self, period: float) -> list[TrajectoryPoint]:
        """Samples every ``period`` from t = 0; the last point is the exact end at rest."""
        if period <= 0.0:
            raise ValueError("sample period must be positive")
        count = max(int(math.ceil(self.duration / period - 1e-9)), 1)
        result = [self.state(index * period) for index in range(count)]
        result.append(self.state(self.duration))
        if result[-1].time <= result[-2].time:
            last = result.pop()
            result.append(TrajectoryPoint(result[-1].time + period, *_rest(last)))
        return result


    def bounded_points(self, period: float, max_points: int) -> list[TrajectoryPoint]:
        """Bound FJT size while retaining each constant-jerk phase boundary.

        q/v/a at both endpoints let JTC spline interpolation reproduce each cubic
        phase. Short trajectories retain their original sampling.
        """
        if not math.isfinite(period) or period <= 0:
            raise ValueError("sample period must be positive and finite")
        if isinstance(max_points, bool) or not isinstance(max_points, int) or max_points < 2:
            raise ValueError("max_points must be an integer >= 2")
        count = max(int(math.ceil(self.duration / period - 1e-9)), 1) + 1
        if count <= max_points:
            return self.points(period)
        phases = self.profile.segments
        if max_points <= len(phases) + 1:
            raise ValueError("point budget cannot preserve phase boundaries")
        # Reserve one interval for rounding up each phase's interval count.
        step = max(period, self.duration / (max_points - 1 - len(phases)))
        result = [self.state(0.0)]
        start = 0.0
        for duration, _ in phases:
            intervals = max(1, int(math.ceil(duration / step)))
            for i in range(1, intervals + 1):
                result.append(self.state(start + duration * i / intervals))
            start += duration
        result[-1] = self.state(self.duration)
        if len(result) > max_points:
            raise ValueError("trajectory exceeds point budget")
        return result


def _rest(point: TrajectoryPoint):
    zeros = tuple(0.0 for _ in point.positions)
    return point.positions, zeros, zeros


def plan_move(start: Sequence[float], target: Sequence[float], limits: Limits) -> Plan:
    """Synchronised S-curve from ``start`` to ``target``; all joints arrive together."""
    if len(start) != len(target) or not start:
        raise ValueError("start and target must have the same non-zero length")
    if not all(math.isfinite(value) for value in (*start, *target)):
        raise ValueError("joint values must be finite")
    delta = [goal - origin for origin, goal in zip(start, target)]
    distance = max(abs(value) for value in delta)
    if distance <= EPSILON:
        return Plan(tuple(start), tuple(0.0 for _ in delta), Profile())
    return Plan(tuple(start), tuple(d / distance for d in delta), scurve(distance, limits))


def plan_stop(plan: Plan, elapsed: float, limits: Limits) -> Plan:
    """Smooth stop on the line of ``plan``, starting from its state at ``elapsed``.

    Braking starts from the path velocity at ``elapsed`` with zero acceleration (the
    acceleration step is bounded by the profile limit). When the original remainder
    stops sooner than a new brake would, the remainder is kept unchanged, so a stop
    never travels past the original target.
    """
    u, du, ddu = plan.profile.sample(elapsed)
    origin = tuple(o + d * u for o, d in zip(plan.origin, plan.direction))
    if elapsed >= plan.duration or du <= EPSILON:
        return Plan(origin, tuple(0.0 for _ in origin), Profile())
    profile = brake(du, limits)
    remaining = max(plan.profile.sample(plan.duration)[0] - u, 0.0)
    travelled = profile.sample(profile.duration)[0]
    if travelled > remaining > EPSILON:
        # The original deceleration is already gentler: keep the original remainder.
        return Plan(
            origin,
            plan.direction,
            Profile(_trim(plan.profile, elapsed), du, ddu),
        )
    return Plan(origin, plan.direction, profile)


def _trim(profile: Profile, elapsed: float) -> tuple[tuple[float, float], ...]:
    """Segments of ``profile`` after ``elapsed``; the caller supplies the initial state."""
    result, clock = [], 0.0
    for duration, jerk in profile.segments:
        end = clock + duration
        if end > elapsed:
            result.append((end - max(clock, elapsed), jerk))
        clock = end
    return _segments(*result)
