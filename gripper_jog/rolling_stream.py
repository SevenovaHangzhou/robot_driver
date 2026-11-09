"""Continuous, acknowledged future windows on the public Rolling 1.0 contract.

The source supplies constant-jerk knots. Accepted knots are immutable until a
replacement is acknowledged. Neither wall time nor browser time is a trajectory
timestamp. Lifecycle and uncertain service recovery are shared with RollingMotion.
"""
from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.interpolate import CubicHermiteSpline

from .errors import JogError
from .failure import violation
from .rolling import RollingMotion, value


@dataclass
class Knot:
    time_ns: int
    q: np.ndarray
    v: np.ndarray
    a: np.ndarray


def validate_window(knots, lower, upper, speed, acceleration, jerk, *, joint_names=None):
    """Check exact extrema of every controller cubic, including acceleration seams."""
    times = np.array([k.time_ns for k in knots], dtype=float) / 1e9
    if len(knots) < 2 or np.any(np.diff(times) <= 0):
        raise JogError('rolling_interpolation', 'Rolling 时间点无效')
    q, v = np.array([k.q for k in knots]), np.array([k.v for k in knots])
    if q.shape != (len(knots), 14) or v.shape != q.shape or not np.isfinite([q, v]).all():
        raise JogError('rolling_interpolation', 'Rolling 双臂轨迹数据无效')
    lower, upper, speed, acceleration, jerk = [np.broadcast_to(x, (14,))
                                                for x in (lower, upper, speed, acceleration, jerk)]
    a = np.array([k.a for k in knots])
    if a.shape != q.shape or not np.isfinite(a).all():
        raise JogError('rolling_seam', 'Rolling 接缝加速度数据无效')
    curve = CubicHermiteSpline(times, q, v, axis=0)
    for i, (begin, end) in enumerate(zip(times, times[1:])):
        dt = end - begin
        c3, c2, c1, c0 = curve.c[:, i]
        a0, a1 = 2 * c2, 2 * c2 + 6 * c3 * dt
        seam = np.maximum(np.abs(a0 - knots[i].a), np.abs(a1 - knots[i + 1].a))
        for quantity, values, limits, tolerance, code in (
                ('seam_acceleration', seam, np.zeros(14), 2e-7, 'rolling_seam'),
                ('acceleration', np.maximum(np.abs(a0), np.abs(a1)), acceleration, 2e-7, 'rolling_dynamics'),
                ('jerk', np.abs(6 * c3), jerk, 2e-6, 'rolling_dynamics')):
            axes = np.flatnonzero(values > limits + tolerance)
            if len(axes):
                axis = int(axes[0])
                violation(code, quantity, axis, i, knots, values[axis],
                          limits[axis] + tolerance if quantity == 'seam_acceleration' else limits[axis], joint_names)
        minimum, maximum, positive, negative = (np.zeros(14) for _ in range(4))
        for axis in range(14):
            positions = [c0[axis], q[i + 1, axis]]
            velocities = [c1[axis], v[i + 1, axis]]
            # Position extrema occur at v=0; velocity extrema occur at a=0.
            roots = (np.roots([3 * c3[axis], 2 * c2[axis], c1[axis]])
                     if c3[axis] != 0 or c2[axis] != 0 else ())
            for root in roots:
                if abs(root.imag) < 1e-10 and 0 < root.real < dt:
                    t = root.real
                    positions.append(((c3[axis] * t + c2[axis]) * t + c1[axis]) * t + c0[axis])
            if abs(c3[axis]) > 1e-15:
                t = -c2[axis] / (3 * c3[axis])
                if 0 < t < dt:
                    velocities.append((3 * c3[axis] * t + 2 * c2[axis]) * t + c1[axis])
            for quantity, value, limit, exceeded in (
                    ('position_lower', min(positions), lower[axis], min(positions) < lower[axis] - 1e-10),
                    ('position_upper', max(positions), upper[axis], max(positions) > upper[axis] + 1e-10),
                    ('speed', max(map(abs, velocities)), speed[axis], max(map(abs, velocities)) > speed[axis] + 1e-8)):
                if exceeded:
                    violation('rolling_dynamics', quantity, axis, i, knots, value, limit, joint_names)
            minimum[axis], maximum[axis] = min(positions), max(positions)
            positive[axis], negative[axis] = max(0., max(velocities)), max(0., -min(velocities))
        stop_time = float(np.max(np.maximum(positive, negative) / acceleration))
        for quantity, values, limits, exceeded in (
                ('stopping_lower', minimum - .5 * negative * stop_time, lower,
                 minimum - .5 * negative * stop_time < lower - 1e-10),
                ('stopping_upper', maximum + .5 * positive * stop_time, upper,
                 maximum + .5 * positive * stop_time > upper + 1e-10)):
            axes = np.flatnonzero(exceeded)
            if len(axes):
                axis = int(axes[0])
                violation('rolling_stopping_envelope', quantity, axis, i, knots, values[axis], limits[axis], joint_names)
    return curve


class RollingStream(RollingMotion):
    """One session for an arbitrarily long velocity command; bounded memory."""
    def __init__(self, source, transport, velocity, clock):
        self.source = source
        candidate = np.asarray(velocity, dtype=float)
        if candidate.shape != (3,) or not np.isfinite(candidate).all() or np.linalg.norm(candidate) > .05 + 1e-12:
            raise JogError('bad_speed', '连续速度必须为有限三维向量，最高 50 mm/s')
        self.velocity = candidate.copy()
        self.revision, self.sent_revision = 1, 0
        self.committed, self.proposed = [], None
        self.replacements = 0
        self.execution_ns = 0
        self.last_generation_s = 0.
        self.finishing = False
        super().__init__(source, transport, clock)

    def _negotiate(self, response):
        super()._negotiate(response)
        if self.window // self.spacing * self.spacing < self.initial_horizon:
            raise JogError('rolling_negotiation', '连续轨迹采样无法覆盖协商首批窗口')

    def set_velocity(self, velocity):
        if self.finishing or self.stopping or self.complete:
            raise JogError('rolling_stopping', '停止中的 Rolling 会话不接受新的速度')
        candidate = np.asarray(velocity, dtype=float)
        if candidate.shape != (3,) or not np.isfinite(candidate).all() or np.linalg.norm(candidate) > .05 + 1e-12:
            raise JogError('bad_speed', '连续速度必须为有限三维向量，最高 50 mm/s')
        if not np.array_equal(candidate, self.velocity):
            self.velocity = candidate.copy()
            self.revision += 1

    def request_halt(self):
        """Plan a C2 stop in the future tail; faults still call stop() directly."""
        if not self.finishing:
            self.finishing = True
            self.velocity[:] = 0.
            self.revision += 1

    def _publish(self, start, end):
        end = int(end) // self.spacing * self.spacing
        if end <= start:
            return
        if self.proposed is not None:
            raise JogError('rolling_pending', '上一未来窗口尚未确认')
        generated_at = time.perf_counter()
        if not self.committed:
            anchor = Knot(0, self.hold.copy(), np.zeros(14), np.zeros(14))
        else:
            anchor = next((k for k in self.committed if k.time_ns == start), None)
            if anchor is None:
                raise JogError('rolling_seam', '未来接缝不在已确认轨迹上')
        times = list(range(start + self.spacing, end + 1, self.spacing))
        if not times:
            return
        initial_hold = not self.committed
        if initial_hold:
            knots = [anchor, *[Knot(t, anchor.q.copy(), anchor.v.copy(), anchor.a.copy()) for t in times]]
        elif self.finishing:
            knots = [anchor, *self.source.stop_rollout(anchor, times)]
        else:
            knots = [anchor, *self.source.rollout(anchor, times, self.velocity)]
        self.source.validate(knots)
        snapshot = self.transport.state()
        self._protocol(snapshot)
        if any(snapshot.get(k) != self.identity[k] for k in ('controller_boot_id', 'session_id', 'client_instance_id')):
            raise JogError('rolling_identity', '生成轨迹期间 Rolling 会话身份变化')
        self.execution_ns = int(snapshot['execution_time_ns'])
        if value(snapshot['session_state']) not in (1, 2) or (start and start < snapshot['replaceable_from_ns']):
            raise JogError('rolling_late', '轨迹生成完成时接缝已过期，已停止')
        if knots[-1].time_ns - self.execution_ns > self.horizon:
            raise JogError('rolling_late', 'Rolling 轨迹超过协商前瞻窗口')
        # Keep only the cubic containing execution time and its future prefix.
        history = max((i for i, k in enumerate(self.committed) if k.time_ns <= self.execution_ns), default=0)
        prefix = [k for k in self.committed[history:] if k.time_ns < start]
        if len(prefix) + len(knots) + bool(prefix) > self.capacity or len(knots) > self.capacity // 2:
            raise JogError('rolling_capacity', '连续轨迹超过协商缓冲预算')
        points = [{'time_from_session_start_ns': k.time_ns, 'positions': k.q.tolist(), 'velocities': k.v.tolist()}
                  for k in knots]
        self.sequence += 1
        # Retain the proposal before dispatch: a lost publish/ACK must never
        # cause another update to be based on an unconfirmed future.
        self.proposed = prefix + knots
        self.sent_revision = 0 if initial_hold else self.revision
        self.transport.publish({**self.identity, 'sequence': self.sequence, 'replace_from_ns': start, 'points': points})
        if start < self.until:
            self.replacements += 1
        self.pending, self.until = self.sequence, knots[-1].time_ns
        self.sent_at = self.last_send = self.clock()
        self.last_generation_s = time.perf_counter() - generated_at

    def _accepted(self):
        self.committed, self.proposed = self.proposed, None

    def _maintain(self, snapshot):
        self.execution_ns = int(snapshot['execution_time_ns'])
        if self.finishing and self.source.stop_until_ns is not None and self.execution_ns >= self.source.stop_until_ns:
            self.stop()  # Desired v=a=0 before entering controller HOLDING.
            return
        if self.until < snapshot['replaceable_from_ns']:
            raise JogError('rolling_late', 'Rolling 未来轨迹已过期')
        if self.revision != self.sent_revision:
            # Reserve generation/transport time in addition to the RT lead.
            budget = max(3 * self.spacing, 2 * self.interval, math.ceil(2 * self.last_generation_s * 1e9))
            boundary = int(snapshot['replaceable_from_ns']) + budget
            anchor = next((k for k in self.committed if k.time_ns >= boundary), None)
            if anchor is None:
                raise JogError('rolling_late', '没有可安全替换的未来接缝')
            self._publish(anchor.time_ns, self.execution_ns + self.window)
        elif (self.clock() - self.last_send) * 1e9 >= self.interval:
            self._publish(self.until, self.execution_ns + self.window)

    def publication(self):
        return {**super().publication(), 'continuous': True, 'replacements': self.replacements,
                'stopping': self.stopping or self.finishing,
                'command_revision': self.revision, 'accepted_revision': self.sent_revision if not self.pending else None,
                'execution_time_ns': self.execution_ns, 'generation_s': self.last_generation_s,
                'buffered_points': len(self.proposed or self.committed)}
