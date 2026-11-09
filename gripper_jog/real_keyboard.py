"""Keyboard input and Mink planning; publication belongs to the primary console."""
import asyncio
import json
import logging
import math
import secrets
import time

import numpy as np

from .cartesian_input import velocity_command, ANGULAR_SPEED_OPTIONS_DEG_S
from .completed_hold import CompletedHold
from .current_stream import CurrentStream
from .errors import JogError
from .failure import describe, matching_detail
from .latency import Latency, PROFILES
from .real_source import RealRollingSource


class RealKeyboard:
    def __init__(self, model, jog, *, timing_profile="baseline"):
        if timing_profile not in PROFILES:
            raise ValueError("Unknown Rolling timing profile")
        self.timing_profile = timing_profile
        self.latency = Latency()
        self.ack_started = None
        self.press_started = None
        self.first_ack = False
        self.model, self.jog = model, jog
        self.entry, self.epoch = None, ''
        self.seq = self.heartbeat_seq = -1
        self.input_at = -math.inf
        self.task = None
        self.starting = False
        self.finishing = False
        self.velocity = np.zeros(6)
        self.stream = None
        self.result = '等待共享双臂控制权和 Rolling 就绪'
        self.fault = ''
        self.last_exit = None
        self.last_input = None
        self.timings = {}
        self.stage = 'idle'
        self.completed_hold = CompletedHold(model.names)
        self.anchor_source = 'measured_feedback'

    def status(self, entry):
        # Diagnostics and public ACKs use independent ROS subscriptions. A late
        # diagnostic may enrich a stopped run, but only with that exact epoch/seq.
        if (self.entry is entry and not self.epoch and self.task is None and not self.starting
                and self.last_exit and self.last_exit.get('source') == 'planner'
                and not self.last_exit.get('details')):
            snapshot = entry.status.get('rolling') or {}
            detail = matching_detail(snapshot)
            previous = self.last_exit.get('rolling', {}).get('controller') or {}
            if detail and (detail['epoch'], detail['sequence']) == (previous.get('epoch'), previous.get('last_sequence')):
                self.last_exit['details'] = detail
                self.last_exit['message'] += '；' + describe(detail) if describe(detail) else ''
                self.fault = self.result = self.last_exit['message']
        state = self.jog.state('', '')
        fresh = entry.compatible and time.monotonic() - entry.received_at <= .75
        mine = fresh and entry.status.get('lease') == 'you'
        owned = self.entry is entry and bool(self.epoch)
        authority = entry.status.get('control_authority', {})
        busy = self.starting or self.task is not None or authority.get('phase') in ('moving', 'stopping')
        state.update(mode='real-rolling', readonly=False, input_mode='continuous',
            rotation_supported=True, angular_speed_options_deg_s=list(ANGULAR_SPEED_OPTIONS_DEG_S),
            armed=owned, epoch=self.epoch if owned else '', busy=busy, stopping=self.finishing,
            can_arm=bool(mine and fresh and not busy and state['feedback_fresh'] and
                         entry.status.get('motion_ready') and entry.status.get('motion_enabled')),
            result=self.result, fault=self.fault,
            diagnostics={'last_exit': self.last_exit, 'last_input': self.last_input,
                         'stage': self.stage, 'timings_ms': dict(self.timings),
                         'anchor_source': self.anchor_source,
                         'latency_ms': self.latency.snapshot(),
                         'timing': self.latency.assessment(self.model.parameters, self.timing_profile)},
            blockers=['碰撞余量为候选配置；躯干、头部及环境障碍物未纳入约束',
                      'TCP 和现场方向尚需实机低速核验'],
            publication={'mode': 'rolling', 'backend': 'real_mink_rolling', 'continuous': True,
                         'feedback_source': 'hardware_joint_states',
                         'sequence': self.stream.sequence if self.stream else 0,
                         'replacements': self.stream.replacements if self.stream else 0,
                         'motion_enabled': bool(entry.status.get('motion_enabled'))})
        return state

    def require(self, entry, body):
        if time.monotonic() - self.input_at > .4:
            self.epoch = ''
        if self.entry is not entry or not self.epoch or body.get('epoch') != self.epoch:
            raise JogError('epoch_expired', '键盘接管已失效')
        if not entry.compatible or time.monotonic() - entry.received_at > .75 or entry.status.get('lease') != 'you':
            raise JogError('authority_lost', '主操作台未确认本页持有双臂控制权')
        key = 'heartbeat_seq' if body.get('_action') == 'heartbeat' else 'seq'
        seq = body.get('seq')
        if type(seq) is not int or seq <= getattr(self, key):
            raise JogError('stale_command', '输入序号已过期')
        setattr(self, key, seq)

    async def command(self, action, entry, body, rpc):
        if action == 'select_arm':
            if self.epoch or self.task is not None:
                raise JogError('busy', '切换机械臂前请停止并退出键盘接管')
            return self.jog.dispatch(action, '', body)
        if action == 'stop':
            if self.entry is entry:
                self.record_input(action, body)
                reason = body.get('reason')
                await self.stop(rpc, reason[:200] if isinstance(reason, str) and reason
                                else '已退出键盘接管；等待控制器停车确认')
            return {'stopping': self.task is not None}
        if action == 'arm':
            if not self.status(entry)['can_arm'] or self.task is not None:
                raise JogError('not_ready', '请先获取共享控制权，并确认真机 Rolling 已就绪')
            self.record_input(action, body)
            self.entry, self.epoch = entry, secrets.token_urlsafe(24)
            self.seq = self.heartbeat_seq = -1
            self.input_at, self.fault = time.monotonic(), ''
            return {'epoch': self.epoch}
        self.require(entry, {**body, '_action': action})
        if action in ('velocity', 'halt'):
            self.record_input(action, body)
        self.input_at = time.monotonic()
        if action == 'heartbeat':
            return {}
        if action == 'halt':
            self.finishing = self.task is not None
            self.epoch = secrets.token_urlsafe(24)
            self.seq = self.heartbeat_seq = -1
            return {'epoch': self.epoch, 'stopping': self.finishing}
        if action != 'velocity':
            raise JogError('bad_request', '真机键盘仅支持连续速度控制')
        velocity, command = velocity_command(body, self.model.direction_maps[self.jog.selected_arm])
        if self.finishing:
            raise JogError('stopping', '请等待当前减速尾段完成')
        if self.starting:
            raise JogError('busy', '上一条连续移动启动尚未确认')
        if self.task is not None and not np.allclose(
                velocity / np.linalg.norm(velocity), self.velocity / np.linalg.norm(self.velocity),
                atol=1e-12, rtol=0):
            raise JogError('busy', '切换平移或旋转方向前，请先松键并等待减速停止')
        self.velocity = velocity
        if self.task is None:
            # Reserve the primary resource before planning; all windows use this
            # same WebSocket. No client-side lease grants command admission.
            generation = self.epoch
            self.starting = True
            requested_at = self.press_started = time.monotonic()
            self.first_ack = False
            try:
                begin = await rpc('begin_motion', {'model_sha256': self.jog.planner.chain.model_hash,
                                                 'joint_names': entry.status['runtime_identity']['joint_names']})
                begin_at = time.monotonic()
            finally:
                self.starting = False
                self.latency.add('begin_rpc', (time.monotonic() - requested_at) * 1000)
            if self.epoch != generation:
                await rpc('stop', {})
                raise JogError('input_cancelled', '松键或退出接管已取消这次移动')
            for name, value in self.model.parameters.items():
                if name in begin['parameters'] and value != begin['parameters'][name]:
                    await rpc('stop', {})
                    raise JogError('runtime_limits_changed', '运行 Rolling 参数与采集的 artifact 不一致')
            self.task = asyncio.create_task(self.run(entry, begin, rpc, begin_at))
        self.result = (f"Mink → Rolling 连续旋转 · {command['speed_deg_s']:g} °/s"
                       if 'speed_deg_s' in command else
                       f"Mink → Rolling 连续移动 · {command['speed_mm_s']:g} mm/s")
        return command

    async def run(self, entry, begin, rpc, begin_at):
        snapshot = begin['rolling']
        try:
            self.timings = {}
            self.ack_started = None
            self.stage = 'source'
            source_at = time.monotonic()
            cached = self.completed_hold.take(entry, snapshot)
            reference = begin.get('hold_reference')
            held = reference['positions'] if reference is not None else cached
            self.anchor_source = ('controller_hold' if reference is not None else
                                  'completed_final' if held is not None else 'measured_feedback')
            source = await self.plan(RealRollingSource, self.model, begin['positions'],
                                     self.jog.selected_arm, held_positions=held)
            self.latency.add('source', (time.monotonic() - source_at) * 1000)
            clock_at = time.monotonic()
            initial_ms, lead_ms = PROFILES[self.timing_profile]
            self.stream = CurrentStream(source, begin['parameters'],
                begin['rolling']['ros_time_ns'] + initial_ms * 1_000_000, lead_ns=lead_ms * 1_000_000)
            snapshot, snapshot_at = begin['rolling'], begin_at
            while True:
                self.stage = 'feedback'
                if not self.epoch or time.monotonic() - self.input_at > .4:
                    raise JogError('input_timeout', '键盘输入保活超时')
                if not entry.compatible or time.monotonic() - max(entry.received_at, begin_at) > .2:
                    raise JogError('authority_stale', '共享控制权状态已过期')
                # begin_motion resets the sender and can switch the controller epoch.
                # A status cached before its reply still describes the previous run.
                if entry.received_at > snapshot_at and entry.status.get('rolling'):
                    snapshot, snapshot_at = entry.status['rolling'], entry.received_at
                pending = self.stream.pending
                self.stream.acknowledge(snapshot)
                if pending is not None and self.stream.pending is None and self.ack_started is not None:
                    self.latency.add('ack', (time.monotonic() - self.ack_started) * 1000)
                    if not self.first_ack and self.press_started is not None:
                        self.latency.add('key_to_ack', (time.monotonic() - self.press_started) * 1000)
                        self.first_ack = True
                    self.ack_started = None
                if self.stream.finished:
                    self.completed_hold.capture(entry, self.stream, snapshot)
                    self.result = '减速尾段执行完成；等待停稳确认'
                    break
                now_ros = snapshot['ros_time_ns'] + round((time.monotonic() - snapshot_at) * 1e9)
                self.stage = 'planning'
                planned_at = time.monotonic()
                window = await self.plan(self.stream.propose, now_ros, self.velocity.copy(), finishing=self.finishing)
                if window is not None:
                    self.latency.add('window_plan', (time.monotonic() - planned_at) * 1000)
                    start = window['replace_from']['sec'] * 10**9 + window['replace_from']['nanosec']
                    remaining = (start - now_ros) / 1e6 - (time.monotonic() - planned_at) * 1000
                    self.latency.add('replacement_lead' if self.stream.committed else 'initial_lead', remaining)
                    if remaining <= begin['parameters']['replace_lead_ms']:
                        raise JogError('rolling_late', f'规划完成后接缝仅余 {remaining:.3f} ms，未发布',
                                       details={'source': 'client_timing', 'remaining_ms': round(remaining, 3),
                                                'required_ms': begin['parameters']['replace_lead_ms']})
                    self.stage = 'window_rpc'
                    sent_at = time.monotonic()
                    try:
                        reply = await rpc('window', {'window': window})
                    finally:
                        self.timings['window_rpc'] = round((time.monotonic() - sent_at) * 1000, 3)
                        self.latency.add('window_rpc', self.timings['window_rpc'])
                    if not self.stream.committed and self.press_started is not None:
                        self.latency.add('key_to_publish', (sent_at - self.press_started) * 1000)
                    self.stream.sent(reply)
                    self.ack_started = sent_at
                    snapshot, snapshot_at = reply['rolling'], time.monotonic()
                self.stage = 'waiting'
                next_at = clock_at + 1 / 30
                await asyncio.sleep(max(0., next_at - time.monotonic()))
                self.latency.add('loop_interval', (time.monotonic() - clock_at) * 1000)
                clock_at = time.monotonic()
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.fault = self.result = str(exc) or f'{self.stage}: {type(exc).__name__}'
            self.last_exit = {'ts': time.time(), 'source': 'planner', 'stage': self.stage,
                'code': getattr(exc, 'code', type(exc).__name__), 'message': self.fault,
                'details': getattr(exc, 'details', None), 'latency_ms': self.latency.snapshot(),
                'input_age_ms': round((time.monotonic() - self.input_at) * 1000, 3),
                'primary_age_ms': round((time.monotonic() - entry.received_at) * 1000, 3),
                'timings_ms': dict(self.timings), 'rolling': snapshot,
                'sequence': self.stream.sequence if self.stream else 0,
                'replacements': self.stream.replacements if self.stream else 0}
            logging.getLogger(__name__).warning('jog_exit %s', json.dumps(self.last_exit, ensure_ascii=False))
            self.epoch = ''
            try:
                await rpc('stop', {})
            except Exception:
                pass  # Primary disconnect fencing and RT timeout retain responsibility.
        finally:
            self.task = None
            self.finishing = False
            self.stage = 'idle'

    async def plan(self, function, *args, **kwargs):
        started = time.monotonic()
        task = asyncio.create_task(asyncio.to_thread(function, *args, **kwargs))
        try:
            return await asyncio.shield(task)
        except asyncio.CancelledError:
            # A running QP cannot be cancelled. Drain it before another command
            # can reuse the model, and never publish its abandoned result.
            try:
                await task
            except Exception:
                pass
            raise
        finally:
            elapsed = round((time.monotonic() - started) * 1000, 3)
            self.timings['planning'] = elapsed
            self.timings['max_planning'] = max(elapsed, self.timings.get('max_planning', 0))

    def record_input(self, action, body):
        events = body.get('input_events', [])
        if not isinstance(events, list):
            events = []
        clean = []
        for event in events[-24:]:
            if not isinstance(event, dict):
                continue
            values = {k: v for k, v in event.items() if k in
                      ('type', 'code', 'repeat', 'trusted', 'at_ms', 'focused', 'hidden')
                      and (type(v) is bool or type(v) in (int, float) and math.isfinite(v)
                           or isinstance(v, str) and len(v) <= 32)}
            clean.append(values)
        reason = body.get('reason')
        self.last_input = {'ts': time.time(), 'action': action,
                           'reason': reason[:200] if isinstance(reason, str) else '', 'events': clean}
        logging.getLogger(__name__).warning('jog_input %s', json.dumps(self.last_input, ensure_ascii=False))

    async def stop(self, rpc, reason):
        if self.epoch or self.task is not None or self.starting:
            self.last_exit = {'ts': time.time(), 'source': 'browser_or_disconnect',
                              'message': reason, 'stage': self.stage,
                              'timings_ms': dict(self.timings), 'input': self.last_input}
            logging.getLogger(__name__).warning('jog_exit %s', json.dumps(self.last_exit, ensure_ascii=False))
        self.epoch, self.result = '', reason
        task = self.task
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        self.finishing = False
        await rpc('stop', {})
