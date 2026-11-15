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

    def integrate(self, time: float) -> tuple[float, float, float]:
        """Integrate the segments without replacing terminal velocity/acceleration."""
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
        return position, velocity, acceleration

    def sample(self, time: float) -> tuple[float, float, float]:
        """Sample the profile, holding its terminal position after completion."""
        position, velocity, acceleration = self.integrate(time)
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


def brake(velocity: float, limits: Limits, acceleration: float = 0.0) -> Profile:
    """Brake a forward path from its full (velocity, acceleration) state.

    Apply -jerk down to the required negative peak acceleration, optionally hold,
    then +jerk back to zero. Reject states that cannot stop without reversing or
    exceeding the velocity limit while their positive acceleration is removed.
    """
    if not math.isfinite(velocity) or not math.isfinite(acceleration) or velocity < 0.0:
        raise ValueError("braking state must be finite with non-negative velocity")
    a, j = limits.acceleration, limits.jerk
    if abs(acceleration) > a + EPSILON:
        raise ValueError("initial acceleration exceeds the braking limit")
    if acceleration < 0.0 and velocity < acceleration**2 / (2.0 * j) - EPSILON:
        raise ValueError("braking state cannot reach rest without reversing")
    peak_velocity = velocity + max(acceleration, 0.0)**2 / (2.0 * j)
    if peak_velocity > limits.velocity + EPSILON:
        raise ValueError("braking state cannot respect the velocity limit")
    if velocity <= EPSILON and abs(acceleration) <= EPSILON:
        return Profile()
    peak = min(a, math.sqrt(j * velocity + acceleration**2 / 2.0))
    ramp_down = max((acceleration + peak) / j, 0.0)
    ramp_up = peak / j
    hold = max((velocity + (acceleration**2 - 2.0 * peak**2) / (2.0 * j)) / peak, 0.0)
    profile = Profile(
        _segments((ramp_down, -j), (hold, 0.0), (ramp_up, j)), velocity, acceleration
    )
    _, final_velocity, final_acceleration = profile.integrate(profile.duration)
    if abs(final_velocity) > 1e-10 or abs(final_acceleration) > 1e-10:
        raise ValueError("braking integration does not finish at rest")
    return profile


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

    def window_points(
        self, start: float, end: float, period: float, max_points: int | None = None
    ) -> list[TrajectoryPoint]:
        """Sample a time window without crossing an unrepresented jerk boundary."""
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("sample period must be positive and finite")
        if not all(math.isfinite(t) for t in (start, end)) or start < 0.0 or end < start:
            raise ValueError("sample window must be finite, ordered and non-negative")
        if max_points is not None and (
            isinstance(max_points, bool) or not isinstance(max_points, int) or max_points < 2
        ):
            raise ValueError("max_points must be an integer >= 2")
        boundaries = [start]
        phase_end = 0.0
        for duration, _ in self.profile.segments:
            phase_end += duration
            endpoint_roundoff = (
                abs(phase_end - start) <= 4.0 * max(math.ulp(phase_end), math.ulp(start))
                or round(phase_end * 1e9) == round(end * 1e9)
            )
            if start < phase_end < end and not endpoint_roundoff:
                boundaries.append(phase_end)
        if end > start:
            boundaries.append(end)
        if max_points is not None and len(boundaries) > max_points:
            raise ValueError("point budget cannot preserve phase boundaries")
        step = period
        if max_points is not None:
            spare = max_points - len(boundaries)
            step = max(period, (end - start) / spare) if spare else math.inf
        times = [start]
        for lo, hi in zip(boundaries, boundaries[1:]):
            intervals = max(1, int(math.ceil((hi - lo) / step - 1e-9)))
            times.extend(lo + (hi - lo) * i / intervals for i in range(1, intervals))
            times.append(hi)
        ticks = [round(t * 1e9) for t in times]
        if any(b <= a for a, b in zip(ticks, ticks[1:])):
            raise ValueError("phase boundaries cannot be represented as distinct nanoseconds")
        if max_points is not None and len(times) > max_points:
            raise ValueError("trajectory exceeds point budget")
        # Match the bridge's nanosecond timestamps before evaluating q/v/a. This
        # prevents a rounded timestamp from being paired with an unrounded state.
        points = []
        terminal_tick = round(self.duration * 1e9)
        for tick in ticks:
            time = tick / 1e9
            state = self.state(self.duration if tick == terminal_tick else time)
            points.append(TrajectoryPoint(time, state.positions, state.velocities, state.accelerations))
        for p0, p1 in zip(points, points[1:]):
            h = p1.time - p0.time
            # Hermite jerk amplifies endpoint-position roundoff by at most 60/h^3.
            # Reject unresolvable short phases instead of widening the frozen
            # 1e-4 rad/s^3 encoding-error budget or emitting an unchecked stop.
            for axis, (q0, q1) in enumerate(zip(p0.positions, p1.positions)):
                if (q0 == q1 and p0.velocities[axis] == p1.velocities[axis] == 0.0
                        and p0.accelerations[axis] == p1.accelerations[axis] == 0.0):
                    continue
                if 60.0 * (math.ulp(q0) + math.ulp(q1)) / h**3 > 1e-4:
                    raise ValueError("phase precision cannot satisfy the jerk encoding budget")
        return points

    def points(self, period: float) -> list[TrajectoryPoint]:
        """Preserve every jerk boundary for both short and long trajectories."""
        end = self.duration if self.duration > 0.0 else period
        return self.window_points(0.0, end, period)

    def bounded_points(self, period: float, max_points: int) -> list[TrajectoryPoint]:
        """Bound point count while always retaining the exact cubic phase boundaries."""
        end = self.duration if self.duration > 0.0 else period
        return self.window_points(0.0, end, period, max_points)


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

    Preserve position, velocity and acceleration at the cut. If the original
    remainder stops within less distance, keep that already feasible remainder.
    """
    if not math.isfinite(elapsed) or elapsed < 0.0:
        raise ValueError("stop time must be finite and non-negative")
    u, du, ddu = plan.profile.sample(elapsed)
    origin = tuple(o + d * u for o, d in zip(plan.origin, plan.direction))
    if elapsed >= plan.duration or (du <= EPSILON and abs(ddu) <= EPSILON):
        return Plan(origin, tuple(0.0 for _ in origin), Profile())
    profile = brake(du, limits, ddu)
    remaining = max(plan.profile.sample(plan.duration)[0] - u, 0.0)
    travelled = profile.sample(profile.duration)[0]
    if travelled > remaining + EPSILON:
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


def stopping_points(plan: Plan, now: float, lead: float, period: float, limits: Limits):
    """Return a C2 stop and its timestamped prefix on the original timeline.

    The cut normally uses ``lead``. If rounding makes a phase unrepresentable,
    retry at most one sampling period later (or at the original endpoint).
    No motion limit or encoding tolerance is relaxed by that retry.
    """
    start = max(now - period, 0.0)
    phase_end = 0.0
    for duration, _ in plan.profile.segments:
        phase_end += duration
        if start < phase_end <= now:
            # This phase boundary is already in the past. Starting here avoids a
            # tiny initial interval without moving the first point into the future.
            start = phase_end
    nominal = now + lead
    cuts = (nominal, min(nominal + period, plan.duration)) if nominal < plan.duration else (nominal,)
    error = None
    for candidate in cuts:
        tick = round(candidate * 1e9)
        if candidate >= plan.duration:
            # Never round a completed move back into a sub-nanosecond brake.
            tick = max(tick, math.ceil(plan.duration * 1e9))
        cut = tick / 1e9
        try:
            stop = plan_stop(plan, cut, limits)
            prefix = plan.window_points(start, cut, period)
            points = prefix[:-1] + [
                TrajectoryPoint(round((p.time + cut) * 1e9) / 1e9,
                                p.positions, p.velocities, p.accelerations)
                for p in stop.points(period)
            ]
            if any(b.time <= a.time for a, b in zip(points, points[1:])):
                raise ValueError("stop timestamps must increase after nanosecond rounding")
            return stop, cut, points
        except ValueError as exc:
            error = exc
    raise ValueError("stop phases cannot be represented within one sample period") from error
