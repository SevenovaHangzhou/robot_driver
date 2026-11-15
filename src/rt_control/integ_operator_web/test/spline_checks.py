"""Independent Hermite q/v/a extrema oracle. No ROS, no device access."""

import numpy as np
from numpy.polynomial import polynomial as poly

# Floating-point / nanosecond encoding tolerances; physical limits are unchanged.
TOL = {
    "position_rad": 1e-10,
    "velocity_rad_s": 1e-9,
    "acceleration_rad_s2": 1e-7,
    "jerk_rad_s3": 1e-4,
    "raw_terminal_velocity_rad_s": 1e-10,
    "raw_terminal_acceleration_rad_s2": 1e-10,
}


def coefficients(p0, p1, axis):
    h = p1.time - p0.time
    if h <= 0:
        raise AssertionError("non-positive interval")
    v0, a0 = p0.velocities[axis], p0.accelerations[axis]
    c1, c2 = h * v0, h * h * a0 / 2
    x = (p1.positions[axis] - p0.positions[axis]) - c1 - c2
    y = h * (p1.velocities[axis] - v0) - 2 * c2
    z = h * h * (p1.accelerations[axis] - a0)
    return np.array(
        [0.0, c1, c2, 10 * x - 4 * y + z / 2, -15 * x + 7 * y - z, 6 * x - 3 * y + z / 2]
    ), h


def extrema(co, h, order):
    derivative = poly.polyder(co, m=order) / h**order
    slope = poly.polyder(derivative)
    roots = poly.polyroots(slope) if np.any(slope) and len(slope) > 1 else []
    samples = [0.0, 1.0] + [float(r.real) for r in roots if abs(r.imag) < 1e-7 and 0 < r.real < 1]
    values = poly.polyval(samples, derivative)
    return float(min(values)), float(max(values))


def check_points(points, limits, axes=None, bounds=None):
    assert len(points) >= 2
    assert all(b.time > a.time for a, b in zip(points, points[1:]))
    axes = list(range(len(points[0].positions))) if axes is None else list(axes)
    peaks = np.zeros(3)
    for p0, p1 in zip(points, points[1:]):
        for axis in axes:
            if (
                p0.positions[axis] == p1.positions[axis]
                and p0.velocities[axis] == p1.velocities[axis] == 0.0
                and p0.accelerations[axis] == p1.accelerations[axis] == 0.0
            ):
                if bounds is not None:
                    assert (
                        bounds[axis][0] - TOL["position_rad"]
                        <= p0.positions[axis]
                        <= bounds[axis][1] + TOL["position_rad"]
                    )
                continue
            co, h = coefficients(p0, p1, axis)
            for order, limit, key in (
                (1, limits.velocity, "velocity_rad_s"),
                (2, limits.acceleration, "acceleration_rad_s2"),
                (3, limits.jerk, "jerk_rad_s3"),
            ):
                low, high = extrema(co, h, order)
                value = max(abs(low), abs(high))
                peaks[order - 1] = max(peaks[order - 1], value)
                assert value <= limit + TOL[key], (axis, order, p0.time, p1.time, value, limit)
            if bounds is not None:
                low, high = extrema(co, h, 0)
                low += p0.positions[axis]
                high += p0.positions[axis]
                assert low >= bounds[axis][0] - TOL["position_rad"]
                assert high <= bounds[axis][1] + TOL["position_rad"]
    return peaks.tolist()


def raw_terminal(profile):
    # Do not use Profile.sample or its forced terminal zeros.
    q, v, a = 0.0, profile.initial_velocity, profile.initial_acceleration
    for duration, jerk in profile.segments:
        q += v * duration + a * duration**2 / 2 + jerk * duration**3 / 6
        v += a * duration + jerk * duration**2 / 2
        a += jerk * duration
    assert abs(v) <= TOL["raw_terminal_velocity_rad_s"], v
    assert abs(a) <= TOL["raw_terminal_acceleration_rad_s2"], a
    return q, v, a
