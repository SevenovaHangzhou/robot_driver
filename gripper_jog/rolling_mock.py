"""Local protocol simulator, not the RT controller or its safety validation."""
from copy import deepcopy
import time

import numpy as np
from scipy.interpolate import CubicHermiteSpline

from .errors import JogError
from .rolling import AXIS_SET_HASH, JOINT_NAMES, RollingMotion, identifier


class MockRollingTransport:
    def __init__(self, positions, stop_acceleration, clock=time.monotonic):
        self.clock, self.stop_acceleration = clock, stop_acceleration
        self.q, self.v = np.asarray(positions, dtype=float).copy(), np.zeros(14)
        self.boot = identifier()
        self.identity, self.status = {}, 0
        self.sequence, self.rejected_sequence, self.reject = 0, 0, 0
        self.execution, self.until = 0, 0
        self.points, self.curve = [], None
        self.started, self.updated = 0., 0.
        self.batches, self.close_operations = [], []
        self.stop_started, self.stop_duration = 0., 0.
        self.close_cache = {}
        # Same defaults as the referenced C++ controller; simulation only.
        self.negotiated = {'transport_max_points': 256, 'buffer_capacity': 64,
                           'max_horizon_ns': 600_000_000, 'required_initial_horizon_ns': 500_000_000,
                           'replace_lead_ns': 16_000_000, 'update_timeout_ns': 200_000_000,
                           'nominal_controller_period_ns': 4_000_000}

    def _advance(self):
        now = self.clock()
        if self.status == 1 and now - self.updated > .1:
            self._begin_stop(self.updated + .1)
        if self.status == 2:
            deadline = min(self.updated + self.negotiated['update_timeout_ns'] / 1e9,
                           self.started + self.until / 1e9)
            at = min(now, deadline)
            self.execution = max(0, round((at - self.started) * 1e9))
            self.q, self.v = self.curve(self.execution / 1e9), self.curve(self.execution / 1e9, 1)
            if now >= deadline:
                self._begin_stop(deadline)
        if self.status == 3:
            elapsed = min(max(0., now - self.stop_started), self.stop_duration)
            self.q = self.stop_q + self.stop_v * elapsed + .5 * self.stop_a * elapsed**2
            self.v = self.stop_v + self.stop_a * elapsed
            if elapsed >= self.stop_duration:
                self.v[:] = 0
                self.status = 4

    def _begin_stop(self, at):
        self.stop_started = at
        self.stop_q, self.stop_v = self.q.copy(), self.v.copy()
        self.stop_duration = float(np.max(np.abs(self.v)) / self.stop_acceleration)
        self.stop_a = -self.v / self.stop_duration if self.stop_duration else np.zeros(14)
        self.status = 3 if self.stop_duration else 4

    def state(self):
        self._advance()
        return {'protocol_major': 1, 'protocol_minor': 0, 'axis_set_hash': AXIS_SET_HASH,
                'controller_boot_id': self.boot, **self.identity, 'has_session': self.status != 0,
                'session_state': self.status, 'execution_time_ns': self.execution,
                'replaceable_from_ns': self.execution + self.negotiated['replace_lead_ns'],
                'buffered_until_ns': self.until, 'last_accepted_sequence': self.sequence,
                'last_rejected_sequence': self.rejected_sequence, 'last_reject': self.reject,
                'desired_positions': self.q.tolist(), 'desired_velocities': self.v.tolist()}

    def open(self, request):
        self._advance()
        if self.status or request['expected_controller_boot_id'] != self.boot or \
                request['axis_set_hash'] != AXIS_SET_HASH:
            return {'accepted': False}
        self.identity = {'session_id': identifier(), 'client_instance_id': request['client_instance_id']}
        self.status, self.sequence, self.rejected_sequence, self.reject = 1, 0, 0, 0
        self.execution, self.until, self.points, self.curve = 0, 0, [], None
        self.close_cache = {}
        self.updated = self.clock()
        return {'accepted': True, 'protocol_major': 1, 'protocol_minor': 0,
                'controller_boot_id': self.boot, **self.identity, 'axis_set_hash': AXIS_SET_HASH,
                'hold_positions': self.q.tolist(), 'hold_velocities': self.v.tolist(), **self.negotiated}

    def publish(self, batch):
        self._advance()
        points = batch['points']
        identity_ok = batch['controller_boot_id'] == self.boot and all(batch[k] == v for k, v in self.identity.items())
        if self.status not in (1, 2) or not identity_ok or batch['sequence'] <= self.sequence:
            raise JogError('rolling_rejected', 'Mock Rolling 会话身份或序号无效')
        times = np.array([p['time_from_session_start_ns'] for p in points], dtype=np.int64)
        positions, velocities = (np.asarray([p[key] for p in points]) for key in ('positions', 'velocities'))
        start = batch['replace_from_ns']
        if not (2 <= len(points) <= self.negotiated['buffer_capacity']) or \
                positions.shape != (len(points), 14) or velocities.shape != positions.shape or \
                not np.isfinite(positions).all() or not np.isfinite(velocities).all() or \
                np.any(np.diff(times) <= 0) or times[0] != start:
            raise JogError('rolling_rejected', 'Mock Rolling 点数组无效')
        if times[-1] - self.execution > self.negotiated['max_horizon_ns']:
            raise JogError('rolling_rejected', 'Mock Rolling 超过最大前瞻窗口')
        if self.status == 1:
            if start or not np.array_equal(positions[0], self.q) or np.any(velocities[0]) or \
                    times[-1] < self.negotiated['required_initial_horizon_ns']:
                raise JogError('rolling_rejected', 'Mock Rolling 首批 hold 锚点或窗口无效')
            prefix = []
            self.started = self.clock()
        else:
            if start < self.execution + self.negotiated['replace_lead_ns'] or start > self.until or \
                    not np.allclose(self.curve(start / 1e9), positions[0], atol=1e-12, rtol=0) or \
                    not np.allclose(self.curve(start / 1e9, 1), velocities[0], atol=1e-12, rtol=0):
                raise JogError('rolling_rejected', 'Mock Rolling 接缝过期、不连续或存在时间间隙')
            history = max(i for i, p in enumerate(self.points) if p['time_from_session_start_ns'] <= self.execution)
            prefix = [p for p in self.points[history:] if p['time_from_session_start_ns'] < start]
        if len(prefix) + len(points) + bool(prefix) > self.negotiated['buffer_capacity']:
            raise JogError('rolling_rejected', 'Mock Rolling 缓冲容量不足')
        self.points = deepcopy(prefix + points)
        self.curve = CubicHermiteSpline(
            np.array([p['time_from_session_start_ns'] for p in self.points]) / 1e9,
            [p['positions'] for p in self.points], [p['velocities'] for p in self.points], axis=0)
        self.until, self.sequence, self.status, self.updated = int(times[-1]), batch['sequence'], 2, self.clock()
        self.batches = (self.batches + [deepcopy(batch)])[-32:]

    def close(self, request):
        self._advance()
        key = bytes(request['request_id']['uuid'])
        if key in self.close_cache:
            old_request, response = self.close_cache[key]
            return deepcopy(response) if old_request == request else {'accepted': False}
        if request['controller_boot_id'] != self.boot or \
                any(request[k] != v for k, v in self.identity.items()) or self.status == 0:
            return {'accepted': False}
        operation = request['operation']
        if operation == 'REQUEST_STOP' and self.status in (1, 2):
            self._begin_stop(self.clock())
        elif operation == 'FINALIZE':
            if self.status != 4:
                return {'accepted': False}
            self.status = 0
        elif operation != 'REQUEST_STOP':
            return {'accepted': False}
        response = {'accepted': True, 'completed': self.status in (0, 4)}
        self.close_operations = (self.close_operations + [operation])[-32:]
        self.close_cache[key] = deepcopy(request), response
        return response


class RollingExecutor:
    def __init__(self, clock=time.monotonic):
        self.clock, self.motion, self.transport = clock, None, None

    def start(self, plan, positions):
        if set(positions) != set(JOINT_NAMES):
            raise JogError('rolling_axes', 'Rolling 必须提供双臂全部 14 轴的保持位置')
        if self.motion is not None and not self.motion.complete:
            raise JogError('rolling_busy', '上一 Rolling 会话尚未完成 FINALIZE')
        hold = np.asarray([positions[n] for n in JOINT_NAMES])
        if self.transport is None:
            self.transport = MockRollingTransport(hold, plan.limits.joint_acceleration, self.clock)
        elif self.transport.status != 0 or not np.allclose(self.transport.q, hold, atol=1e-12, rtol=0):
            raise JogError('rolling_anchor', 'Rolling 控制器保持状态与双臂规划起点不一致')
        self.motion = RollingMotion(plan, self.transport, self.clock)
        return self.motion

    def publication(self):
        return self.motion.publication() if self.motion is not None else {'mode': 'rolling', 'sequence': 0}
