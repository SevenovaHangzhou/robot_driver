"""Rolling protocol 1.0 client. No ROS imports or hardware side effects on import."""
from hashlib import sha256
import math
import time
import uuid

import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation

from .errors import JogError

JOINT_NAMES = tuple(f'{side}_joint{i}' for side in ('right', 'left') for i in range(1, 8))
AXIS_SET_HASH = list(sha256(''.join(f'{n}:rad:rad/s\n' for n in JOINT_NAMES).encode()).digest())
PREFIX = '/rt/rolling_joint_control'


def identifier():
    return {'uuid': list(uuid.uuid4().bytes)}


def value(field):
    return field['value'] if isinstance(field, dict) else field


class RollingMotion:
    """One bounded Cartesian plan delivered as acknowledged rolling windows.

    The transport supplies controller-session time; browser and wall-clock time
    are never placed in trajectory timestamps. Only one update is unacknowledged.
    """
    def __init__(self, plan, transport, clock=time.monotonic):
        self.plan, self.transport, self.clock = plan, transport, clock
        self.duration = plan.duration
        self.indices = [JOINT_NAMES.index(n) for n in plan.chain.names]
        self.complete, self.stopping, self.fault = False, False, ''
        self.pending, self.sequence, self.until = 0, 0, 0
        self.sent_at, self.last_send = clock(), clock()
        self.stop_request, self.finalize_request = None, None
        self.stop_confirmed = False
        self.open_pending = False
        self.last_q = plan.state(0)[0].copy()
        self.last_v = np.zeros(7)
        initial = transport.state()
        self._protocol(initial)
        if initial['has_session']:
            raise JogError('rolling_busy', 'Rolling 控制器已有会话')
        self.identity = {'protocol_major': 1, 'protocol_minor': 0,
                         'controller_boot_id': initial['controller_boot_id'],
                         'client_instance_id': identifier()}
        # Keep the request and client identity before dispatch. A timed-out open
        # may have succeeded; only this request can resolve that outcome.
        self.open_request = {
            'protocol_major': 1, 'protocol_minor': 0, 'request_id': identifier(),
            'expected_controller_boot_id': initial['controller_boot_id'],
            'client_instance_id': self.identity['client_instance_id'], 'axis_set_hash': AXIS_SET_HASH,
        }
        self.open_pending = True
        try:
            response = transport.open(self.open_request)
        except (JogError, TimeoutError) as error:
            self.fault, self.stopping = str(error), True
            return  # Retain ownership; state() resolves and closes, never resumes.
        if not response['accepted']:
            self.open_pending = False
            raise JogError('rolling_open_rejected', 'Rolling open 被拒绝')
        try:
            self._accept_open(response)
            self.hold = np.asarray(response['hold_positions'], dtype=float)
            if self.hold.shape != (14,) or not np.isfinite(self.hold).all() or \
                    not np.array_equal(self.hold[self.indices], self.last_q) or \
                    np.any(response['hold_velocities']):
                raise JogError('rolling_anchor', 'Rolling hold 与规划起点不一致')
            self._negotiate(response)
            self._publish(0, self.window)
        except Exception as error:
            self.fault, self.stopping = str(error), True
            # Keep the motion visible to its executor until cleanup is confirmed.
            # Do not discard an open/stop/finalize whose response is unknown.
            if not self.open_pending:
                self._abort_open()
            if self.complete:
                raise

    def _negotiate(self, response):
        self.capacity = min(response['buffer_capacity'], response['transport_max_points'], 256)
        self.horizon = int(response['max_horizon_ns'])
        self.initial_horizon = int(response['required_initial_horizon_ns'])
        self.timeout = int(response['update_timeout_ns'])
        self.lead = int(response['replace_lead_ns'])
        period = self.period = int(response['nominal_controller_period_ns'])
        if not (16 <= self.capacity <= 256 and 0 < period <= self.lead < self.initial_horizon <= self.horizon
                and 0 < self.timeout < self.horizon):
            raise JogError('rolling_negotiation', 'Rolling 协商窗口不适用于点动客户端')
        self.window = max(math.ceil(self.initial_horizon / period) * period,
                          int(self.horizon * .9) // period * period)
        if self.window > self.horizon:
            raise JogError('rolling_negotiation', 'Rolling 初始窗口无法对齐控制周期')
        self.spacing = math.ceil(self.window / (self.capacity // 2 - 4) / period) * period
        self.interval = min(self.timeout // 3, (self.window - self.lead) // 3)

    def _accept_open(self, response):
        self._protocol(response)
        if response['controller_boot_id'] != self.identity['controller_boot_id'] or \
                response['client_instance_id'] != self.identity['client_instance_id']:
            raise JogError('rolling_identity', 'Rolling open 返回的会话身份不一致')
        self.identity['session_id'] = response['session_id']
        self.open_pending = False

    def _abort_open(self):
        # A single nonblocking lifecycle step. Later ticks own any remaining
        # cleanup, including retrying close requests with their original IDs.
        try:
            self.stop()
        except (JogError, TimeoutError):
            pass
        self.state(0)

    @staticmethod
    def _protocol(message):
        if (message['protocol_major'], message['protocol_minor']) != (1, 0) or \
                list(message['axis_set_hash']) != AXIS_SET_HASH:
            raise JogError('rolling_contract', 'Rolling 协议或 V3 双七轴身份不匹配')

    def _publish(self, start, end):
        # Real controller time need not be an exact number of periods. Keep
        # trailing intervals at least one period long; tiny Hermite intervals
        # magnify floating-point cancellation in acceleration and jerk.
        end = int(end) // self.period * self.period
        if end <= start:
            return
        times = list(range(start, end, self.spacing)) + [end]
        terminal = math.ceil(self.duration * 1e9)
        if start < terminal < end and min(abs(t - terminal) for t in times) >= self.period:
            times = sorted(set(times + [terminal]))
        if len(times) > self.capacity // 2:
            raise JogError('rolling_capacity', 'Rolling 批次超过协商容量预算')
        seconds = np.asarray(times, dtype=float) / 1e9
        q, v, _, _ = self.plan.state(np.minimum(seconds, self.duration))
        v[seconds >= self.duration] = 0.
        # Validate the controller's cubic Hermite interpolation, not just the
        # planner spline. This remains sampled, offline validation.
        curve = CubicHermiteSpline(seconds, q, v, axis=0)
        check_times = np.unique(np.concatenate([np.linspace(a, b, 5) for a, b in zip(seconds, seconds[1:])]))
        qs, vs, acc, jerk = (curve(check_times, d) for d in range(4))
        limits = self.plan.limits
        if np.any(qs < self.plan.chain.lower) or np.any(qs > self.plan.chain.upper) or \
                np.max(np.abs(vs) / np.minimum(self.plan.chain.velocity, limits.joint_speed)) > 1.00001 or \
                np.max(np.abs(acc)) > limits.joint_acceleration * 1.00001 or \
                np.max(np.abs(jerk)) > limits.joint_jerk * 1.00001:
            raise JogError('rolling_interpolation', 'Rolling Hermite 插值超过关节动态限制')
        for t, joints, velocity in zip(check_times, qs, vs):
            pose, jacobian = self.plan.chain.fk_jacobian(joints)
            target = self.plan.start_pose[:3, 3] + self.plan.delta * self.plan.parameter(min(t, self.duration))
            if np.linalg.norm(pose[:3, 3] - target) > limits.position_tolerance or \
                    Rotation.from_matrix(pose[:3, :3] @ self.plan.start_pose[:3, :3].T).magnitude() > limits.orientation_tolerance or \
                    np.linalg.norm(jacobian[:3] @ velocity) > limits.tcp_speed * 1.00001:
                raise JogError('rolling_interpolation', 'Rolling Hermite 插值超过 TCP 路径或速度限制')
        points = []
        for timestamp, position, velocity in zip(times, q, v):
            full_q, full_v = self.hold.copy(), np.zeros(14)
            full_q[self.indices], full_v[self.indices] = position, velocity
            points.append({'time_from_session_start_ns': timestamp,
                           'positions': full_q.tolist(), 'velocities': full_v.tolist()})
        self.sequence += 1
        self.transport.publish({**self.identity, 'sequence': self.sequence,
                                'replace_from_ns': start, 'points': points})
        self.pending, self.until = self.sequence, end
        self.sent_at = self.last_send = self.clock()

    def stop(self):
        if self.open_pending:
            self.stopping = True
            return
        if self.stop_request is None and not self.complete:
            self.stopping = True
            self.stop_request = {**self.identity, 'request_id': identifier(), 'operation': 'REQUEST_STOP'}
        if not self.complete and not self.stop_confirmed:
            response = self.transport.close(self.stop_request)
            if not response['accepted']:
                raise JogError('rolling_stop_unconfirmed', 'Rolling REQUEST_STOP 未确认')
            self.stop_confirmed = True

    def state(self, _elapsed):
        if self.complete:
            return self.last_q.copy(), self.last_v.copy(), np.zeros(7), np.zeros(7)
        try:
            if self.open_pending:
                response = self.transport.open(self.open_request)
                if not response['accepted']:
                    self.open_pending, self.complete = False, True
                    return self.last_q.copy(), self.last_v.copy(), np.zeros(7), np.zeros(7)
                self._accept_open(response)
                # Any uncertain open is abandoned even if its response arrives.
                self.stopping = True
                self.stop()
            if self.finalize_request is not None:
                # FINALIZE can already have removed the server-side session.
                # Resolve its original response before requiring active identity.
                response = self.transport.close(self.finalize_request)
                if not response['accepted'] or not response['completed']:
                    raise JogError('rolling_finalize_unconfirmed', 'Rolling FINALIZE 未确认')
                self._finished()
                return self.last_q.copy(), self.last_v.copy(), np.zeros(7), np.zeros(7)
            snapshot = self.transport.state()
            self._protocol(snapshot)
            if any(snapshot[k] != self.identity[k] for k in ('controller_boot_id', 'session_id', 'client_instance_id')):
                raise JogError('rolling_identity', 'Rolling 控制器重启或会话身份变化')
            full_q, full_v = (np.asarray(snapshot[key]) for key in ('desired_positions', 'desired_velocities'))
            if full_q.shape != (14,) or full_v.shape != (14,) or \
                    not np.isfinite(full_q).all() or not np.isfinite(full_v).all():
                raise JogError('rolling_feedback', 'Rolling 期望关节状态无效')
            self.last_q, self.last_v = full_q[self.indices].copy(), full_v[self.indices].copy()
            status = value(snapshot['session_state'])
            if status == 4:  # HOLDING, and only HOLDING, permits FINALIZE.
                if not self.stopping:
                    self.fault = 'Rolling 控制器提前停止'
                if self.stop_request is None:
                    self.stop()
                if self.finalize_request is None:
                    self.finalize_request = {**self.identity, 'request_id': identifier(), 'operation': 'FINALIZE'}
                response = self.transport.close(self.finalize_request)
                if not response['accepted'] or not response['completed']:
                    raise JogError('rolling_finalize_unconfirmed', 'Rolling FINALIZE 未确认')
                self._finished()
            elif status == 3:
                if not self.stopping:
                    self.fault = 'Rolling 控制器超时、低水位或故障停止'
                self.stop()
            elif status not in (1, 2):
                raise JogError('rolling_terminated', 'Rolling 会话已异常结束')
            elif self.stopping:
                self.stop()
            else:
                if snapshot['last_rejected_sequence'] == self.pending and self.pending:
                    raise JogError('rolling_rejected', f"Rolling 批次被拒绝：{value(snapshot['last_reject'])}")
                if snapshot['last_accepted_sequence'] >= self.pending:
                    if self.pending:
                        self._accepted()
                    self.pending = 0
                if self.pending:
                    if (self.clock() - self.sent_at) * 1e9 > self.timeout:
                        raise JogError('rolling_ack_timeout', 'Rolling 批次确认超时')
                else:
                    self._maintain(snapshot)
        except (JogError, TimeoutError) as error:
            self.fault = str(error)
            try:
                self.stop()
            except (JogError, TimeoutError):
                pass
        return self.last_q.copy(), self.last_v.copy(), np.zeros(7), np.zeros(7)

    def _accepted(self):
        """Commit a staged window only after the controller acknowledges it."""

    def _finished(self):
        self.complete = True
        finalized = getattr(self.transport, 'session_finalized', None)
        if finalized:
            finalized(self.identity)

    def _maintain(self, snapshot):
        if snapshot['execution_time_ns'] >= math.ceil(self.duration * 1e9):
            self.stop()
        elif (self.clock() - self.last_send) * 1e9 >= self.interval:
            if self.until < snapshot['replaceable_from_ns']:
                raise JogError('rolling_late', 'Rolling 补充窗口已过期')
            self._publish(self.until, int(snapshot['execution_time_ns']) + self.window)

    def brake(self, _elapsed):
        try:
            self.stop()
        except (JogError, TimeoutError) as error:
            self.fault = str(error)
        return None if self.complete else self

    def publication(self):
        return {'mode': 'rolling', 'sequence': self.sequence, 'pending_sequence': self.pending,
                'buffered_until_ns': self.until, 'session_id': self.identity.get('session_id'), 'open_pending': self.open_pending,
                'stopping': self.stopping, 'complete': self.complete, 'fault': self.fault}
