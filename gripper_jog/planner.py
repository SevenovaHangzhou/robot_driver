"""Bounded Cartesian steps with continuous joint interpolation and path-time braking.

All checks here are sampled offline checks, not a certified hardware safety layer.
"""
from dataclasses import dataclass, replace
from typing import Callable

import numpy as np
from numpy.polynomial import Polynomial
from scipy.interpolate import CubicSpline, PPoly
from scipy.spatial.transform import Rotation

from .errors import JogError

DIRECTIONS = {"x+": (1, 0, 0), "x-": (-1, 0, 0), "y+": (0, 1, 0),
              "y-": (0, -1, 0), "z+": (0, 0, 1), "z-": (0, 0, -1)}


class SpeedClock:
    """Smooth acceleration, constant speed, smooth deceleration along a path.

    Match the Polynomial operations used by Plan, including time shifts for
    braking. Position, velocity, acceleration and jerk are continuous.
    """
    def __init__(self, curve):
        self.curve = curve

    def __call__(self, at):
        if isinstance(at, Polynomial):
            if len(at.coef) != 2 or at.coef[1] <= 0:
                raise ValueError('Clock composition requires an increasing affine time')
            offset, scale = at.coef
            powers = np.arange(self.curve.c.shape[0] - 1, -1, -1)
            return SpeedClock(PPoly(self.curve.c * scale**powers[:, None],
                                    (self.curve.x - offset) / scale))
        return self.curve(at)

    def deriv(self, order=1):
        return SpeedClock(self.curve.derivative(order))

    @classmethod
    def create(cls, rate, ramp):
        cruise = max(0., 1 / rate - ramp)
        integral = Polynomial([0, 0, 0, 0, 2.5, -3, 1])
        acceleration = rate * ramp * integral(Polynomial([0, 1 / ramp]))
        parts, knots = [acceleration], [0., ramp]
        if cruise > 1e-12:
            parts.append(Polynomial([rate * ramp / 2, rate]))
            knots.append(ramp + cruise)
        parts.append(Polynomial([rate * (ramp / 2 + cruise), rate]) - acceleration)
        knots.append(2 * ramp + cruise)
        coefficients = np.column_stack([np.pad(p.coef, (0, 7 - len(p.coef)))[::-1] for p in parts])
        return cls(PPoly(coefficients, knots)), knots[-1]


@dataclass(frozen=True)
class Limits:
    tcp_speed: float = .001
    joint_speed: float = .035
    joint_acceleration: float = .07
    joint_jerk: float = .28
    position_tolerance: float = 1e-5
    orientation_tolerance: float = 1e-4
    min_sigma: float = .015
    max_joint_travel: float = .08
    max_duration: float = 30.

    def __post_init__(self):
        if not all(np.isfinite(v) and v > 0 for v in vars(self).values()):
            raise JogError("config_invalid", "规划限制必须是有限正数")


@dataclass
class Plan:
    chain: object
    spline: CubicSpline
    clock: Polynomial | SpeedClock
    duration: float
    limits: Limits
    start_pose: np.ndarray
    delta: np.ndarray
    collision_verified: bool = False
    metrics: dict = None
    tcp_path_rate: float = None

    def parameter(self, at, derivative=0):
        return self.clock.deriv(derivative)(np.clip(at, 0., self.duration))

    def state(self, at):
        s = np.clip(self.parameter(at), 0., 1.)
        v, a, j = (self.parameter(at, n) for n in (1, 2, 3))
        p1, p2, p3 = (self.spline(s, n) for n in (1, 2, 3))
        # Both scalar and vector times are supported.
        if np.ndim(s):
            v, a, j = v[:, None], a[:, None], j[:, None]
        return (self.spline(s), p1 * v, p2 * v**2 + p1 * a,
                p3 * v**3 + 3 * p2 * v * a + p1 * j)

    def ratios(self, samples=401):
        q, v, a, j = self.state(np.linspace(0, self.duration, samples))
        joint_v = np.minimum(self.chain.velocity, self.limits.joint_speed)
        if self.tcp_path_rate is None:
            self.tcp_path_rate = 1.03 * max(
                np.linalg.norm(self.chain.fk_jacobian(self.spline(s))[1][:3] @ self.spline(s, 1))
                for s in np.linspace(0, 1, 201))
        # Conservative sampled path-rate bound, reusable for all time curves.
        tcp_v = self.tcp_path_rate * float(np.max(np.abs(
            self.parameter(np.linspace(0, self.duration, samples), 1))))
        return max(float(np.max(np.abs(v) / joint_v)), tcp_v / self.limits.tcp_speed), \
            float(np.max(np.abs(a)) / self.limits.joint_acceleration), \
            float(np.max(np.abs(j)) / self.limits.joint_jerk)

    def brake(self, at):
        """Continue on the validated path, preserving q/v/a and zeroing final v/a/j.

        A short remaining natural tail is also a valid bounded stop. No new IK or
        shortcut through joint space is used. Candidate limits are checked before use.
        """
        at = float(np.clip(at, 0., self.duration))
        remaining = self.duration - at
        if remaining < 1e-6:
            return None
        natural = Plan(self.chain, self.spline, self.clock(Polynomial([at, 1.])),
                       remaining, self.limits, self.start_pose, self.delta, self.collision_verified, tcp_path_rate=self.tcp_path_rate)
        s, v, a, j = (float(self.parameter(at, n)) for n in range(4))
        if v <= 1e-10 or remaining < .1:
            return natural
        for duration in np.linspace(.08, remaining, 30):
            end = s + v * duration / 2 + a * duration**2 / 12 + j * duration**3 / 120
            if not s < end < 1.:
                continue
            # Seventh-degree Hermite polynomial, solving the final four derivatives.
            c = np.zeros(8)
            c[:4] = [s, v * duration, a * duration**2 / 2, j * duration**3 / 6]
            matrix = np.array([[Polynomial.basis(k).deriv(n)(1.) for k in range(4, 8)]
                               for n in range(4)])
            fixed = Polynomial(c)
            c[4:] = np.linalg.solve(matrix, [end - fixed(1.), -fixed.deriv()(1.),
                                             -fixed.deriv(2)(1.), -fixed.deriv(3)(1.)])
            clock = Polynomial(c)(Polynomial([0, 1 / duration]))
            candidate = Plan(self.chain, self.spline, clock, duration, self.limits,
                             self.start_pose, self.delta, self.collision_verified, tcp_path_rate=self.tcp_path_rate)
            times = np.linspace(0, duration, 401)
            if np.min(candidate.parameter(times, 1)) < -1e-10:
                continue
            if max(candidate.ratios()) <= 1.00001:
                return candidate
        return natural

    def document(self):
        return {"duration_s": self.duration, "collision_verified": self.collision_verified,
                "distance_mm": float(np.linalg.norm(self.delta) * 1000),
                "speed_mm_s": self.limits.tcp_speed * 1000,
                "metrics": self.metrics, "target_q": self.state(self.duration)[0].tolist()}


class Planner:
    def __init__(self, chain, limits=Limits(), collision_check: Callable | None = None,
                 workspace=((-2., -2., -2.), (2., 2., 2.))):
        self.chain, self.limits, self.collision_check = chain, limits, collision_check
        self.workspace = np.asarray(workspace, dtype=float)
        if self.workspace.shape != (2, 3) or not np.isfinite(self.workspace).all() \
                or np.any(self.workspace[0] >= self.workspace[1]):
            raise JogError("config_invalid", "工作区无效")

    def step(self, start, direction, distance_m, require_collision=True, orientation=None, tcp_speed=None,
             speed_profile=False):
        if tcp_speed is not None:
            if type(tcp_speed) not in (int, float) or not np.isfinite(tcp_speed) or not 0 < tcp_speed <= .05:
                raise JogError("bad_speed", "速度必须在 (0, 50] mm/s 内")
            # Each plan owns its limits; changing speed must not alter an active
            # plan or the shared planner used by another arm/request.
            return Planner(self.chain, replace(self.limits, tcp_speed=tcp_speed),
                           self.collision_check, self.workspace).step(
                               start, direction, distance_m, require_collision, orientation,
                               speed_profile=speed_profile)
        if direction not in DIRECTIONS or isinstance(distance_m, bool) \
                or not np.isfinite(distance_m) or not 0 < distance_m <= .01:
            raise JogError("bad_step", "方向无效或步长不在 (0, 10] mm")
        if require_collision and self.collision_check is None:
            raise JogError("collision_unavailable", "未接入全模型碰撞检查，禁止实机规划")
        start = self.chain.validate(start)
        pose = self.chain.fk(start)
        if orientation is not None:
            orientation = np.asarray(orientation, dtype=float)
            if orientation.shape != (3, 3) or not np.isfinite(orientation).all() or not np.allclose(
                    orientation.T @ orientation, np.eye(3), atol=1e-8) or np.linalg.det(orientation) < .999999:
                raise JogError("orientation_invalid", "锁定姿态无效")
            pose[:3, :3] = orientation
        delta = np.array(DIRECTIONS[direction], dtype=float) * distance_m
        # At least eight intervals; bound all work independently of requested duration.
        grid = np.linspace(0., 1., max(9, int(np.ceil(distance_m / .0002)) + 1))
        values = [start]
        for s in grid[1:]:
            target = pose.copy()
            target[:3, 3] += delta * s
            values.append(self.chain.ik(target, values[-1], self.limits.min_sigma))
        spline = CubicSpline(grid, values, axis=0)
        if np.max(np.abs(np.asarray(values) - start)) > self.limits.max_joint_travel:
            raise JogError("joint_jump", "本次点动累计关节变化过大")
        pos_error = orientation_error = tcp_path_rate = 0.
        # Validate the actual interpolant, not just the IK knots.
        for s in np.linspace(0, 1, 201):
            q = spline(s)
            if np.max(np.abs(q - start)) > self.limits.max_joint_travel:
                raise JogError("joint_jump", "插值路径累计关节变化过大")
            actual, jac = self.chain.fk_jacobian(q)
            xyz = actual[:3, 3]
            tcp_path_rate = max(tcp_path_rate, np.linalg.norm(jac[:3] @ spline(s, 1)))
            if np.any(xyz < self.workspace[0]) or np.any(xyz > self.workspace[1]):
                raise JogError("workspace_limit", "TCP 路径超出允许工作区")
            sigma = np.linalg.svd(jac * np.array([1, 1, 1, .25, .25, .25])[:, None],
                                  compute_uv=False)[-1]
            if sigma < self.limits.min_sigma:
                raise JogError("singularity", "插值路径接近奇异位形")
            if self.collision_check is not None and not self.collision_check(q):
                raise JogError("collision", "路径碰撞检查未通过")
            pos_error = max(pos_error, np.linalg.norm(xyz - (pose[:3, 3] + s * delta)))
            orientation_error = max(orientation_error, Rotation.from_matrix(
                actual[:3, :3] @ pose[:3, :3].T).magnitude())
        if pos_error > self.limits.position_tolerance or orientation_error > self.limits.orientation_tolerance:
            raise JogError("path_error", "实际关节插值超过直线或姿态误差限制")
        duration = max(.2, distance_m / self.limits.tcp_speed * 2.2)
        phase = Polynomial([0, 0, 0, 0, 35, -84, 70, -20])
        plan = Plan(self.chain, spline, phase(Polynomial([0, 1 / duration])), duration,
                    self.limits, pose, delta, self.collision_check is not None,
                    tcp_path_rate=float(tcp_path_rate * 1.03))
        if speed_profile:
            path_rates = np.max(np.abs(spline(np.linspace(0, 1, 201), 1)), axis=0)
            rate = min(self.limits.tcp_speed / plan.tcp_path_rate,
                       float(np.min(np.minimum(self.chain.velocity, self.limits.joint_speed)
                                    / np.maximum(path_rates, 1e-15))))
            def ramp_time(value):
                return max(.05, float(np.max(1.875 * path_rates * value / self.limits.joint_acceleration)),
                           float(np.sqrt(np.max(10 / np.sqrt(3) * path_rates * value / self.limits.joint_jerk))))
            ramp = ramp_time(rate)
            if rate * ramp > 1:
                rate /= rate * ramp
                ramp = ramp_time(rate)
            plan.clock, plan.duration = SpeedClock.create(rate, ramp)
        ratios = plan.ratios()
        factor = max(1., ratios[0], np.sqrt(ratios[1]), np.cbrt(ratios[2])) * 1.03
        plan.duration *= factor
        plan.clock = plan.clock(Polynomial([0, 1 / factor]))
        if plan.duration > self.limits.max_duration:
            raise JogError("duration_limit", "轨迹时长超过点动上限")
        final_ratios = plan.ratios()
        if max(final_ratios) > 1.00001:
            raise JogError("dynamic_limit", "轨迹动态限制复核失败")
        plan.metrics = {"max_path_error_mm": float(pos_error * 1000),
                        "max_orientation_error_deg": float(np.degrees(orientation_error)),
                        "max_joint_delta_deg": float(np.degrees(np.max(np.abs(np.asarray(values) - start)))),
                        "velocity_ratio": final_ratios[0], "acceleration_ratio": final_ratios[1],
                        "jerk_ratio": final_ratios[2], "validation_samples": 201}
        return plan
