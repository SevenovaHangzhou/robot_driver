"""Bounded, acknowledged suffix replacements for the current Rolling contract."""
import math
import numpy as np

from .current_rolling import ros_time
from .errors import JogError
from .failure import controller_error


class CurrentStream:
    def __init__(self, source, parameters, origin_ros_ns, *, spacing_ns=20_000_000,
                 lead_ns=100_000_000, horizon_ns=400_000_000):
        self.source, self.parameters, self.origin = source, parameters, origin_ros_ns
        self.spacing, self.lead, self.horizon = spacing_ns, lead_ns, horizon_ns
        if lead_ns <= parameters['replace_lead_ms'] * 10**6 or \
                lead_ns + horizon_ns + spacing_ns > parameters['max_horizon_ms'] * 10**6:
            raise JogError('rolling_timing', '客户端规划时间与运行 Rolling 包络不相容')
        self.committed, self.proposed, self.pending = [], None, None
        self.epoch, self.finished, self.final_sent = None, False, False
        self.replacements, self.sequence = 0, 0

    def acknowledge(self, snapshot):
        state = snapshot.get('controller')
        if state is None or snapshot.get('state_age_s', math.inf) > .2:
            raise JogError('rolling_stale', 'Rolling 状态已失效')
        if self.epoch is None:
            self.epoch = state['epoch']
        if state['epoch'] != self.epoch or snapshot['phase'] in (5, 6, 7):
            raise controller_error('rolling_interrupted',
                f"Rolling 执行已中断或首批失败：phase={snapshot['phase']}，"
                f"epoch={state['epoch']}（预期 {self.epoch}），"
                f"拒绝码={snapshot.get('last_rejection', state['last_result'])}", snapshot)
        if self.pending is not None:
            if state['last_sequence'] > self.pending:
                raise JogError('rolling_foreign_writer', 'Rolling 出现其他发布者的序列')
            if state['last_sequence'] == self.pending:
                if state['last_result'] != 0:
                    raise controller_error('rolling_rejected', f"Rolling 拒绝未来窗口：{state['last_result']}", snapshot)
                self.committed, self.proposed, self.pending = self.proposed, None, None
        if snapshot['phase'] == 4:
            raise JogError('rolling_stopped', f"Rolling 控制器已停车：{snapshot.get('stop_reason')}")
        if snapshot['phase'] == 3:
            if not self.final_sent or self.pending is not None:
                raise JogError('rolling_unexpected_end', 'Rolling 在最终窗口确认前结束')
            self.finished = True

    def propose(self, now_ros_ns, velocity, *, finishing=False):
        if self.pending is not None or self.proposed is not None or self.final_sent:
            return None
        elapsed = now_ros_ns - self.origin
        if not self.committed:
            anchor = self.source.anchor
        else:
            minimum = max(self.spacing, elapsed + self.lead)
            anchor = next((k for k in self.committed if k.time_ns >= minimum), None)
            if anchor is None:
                raise JogError('rolling_late', '已确认轨迹中没有可替换的未来接缝')
        end = anchor.time_ns + self.horizon
        times = list(range(anchor.time_ns + self.spacing, end + 1, self.spacing))
        rollout = self.source.stop_rollout if finishing else self.source.rollout
        tail = rollout(anchor, times) if finishing else rollout(anchor, times, velocity)
        knots = [anchor, *tail]
        self.source.validate(knots)
        prefix = [k for k in self.committed if elapsed - self.spacing <= k.time_ns < anchor.time_ns]
        if len(prefix) + len(knots) + bool(prefix) > self.parameters['buffer_capacity']:
            raise JogError('rolling_capacity', '未来窗口与保留前缀超过 Rolling 缓冲容量')
        final = finishing and self.source.stop_until_ns <= knots[-1].time_ns
        self.proposed = prefix + knots
        self.proposal_final = final
        return {'replace_from': ros_time(self.origin + anchor.time_ns), 'final': bool(final),
                'points': [{'time_from_start': ros_time(k.time_ns - anchor.time_ns),
                            'positions': k.q.tolist(), 'velocities': k.v.tolist()} for k in knots]}

    def sent(self, result):
        if result['send_result'] != 0:
            self.proposed = None
            # Sending while the first ACK is pending is not a retry of a plan.
            raise JogError('rolling_not_sent', f"RollingSender 未发布窗口：{result['send_result']}")
        self.pending = result['rolling']['last_sent_sequence']
        self.sequence = self.pending
        self.replacements += bool(self.committed)
        self.final_sent = self.proposal_final
