"""Human-readable evidence without guessing details absent from the public ACK."""
from .errors import JogError

LABELS = {'position_lower': ('位置下限', 'rad'), 'position_upper': ('位置上限', 'rad'),
          'velocity_positive': ('正向速度', 'rad/s'), 'velocity_negative': ('反向速度', 'rad/s'),
          'acceleration_positive': ('正向加速度', 'rad/s²'), 'acceleration_negative': ('反向加速度', 'rad/s²'),
          'stopping_lower': ('制动位置下界', 'rad'), 'stopping_upper': ('制动位置上界', 'rad'),
          'acceleration': ('加速度绝对值', 'rad/s²'), 'jerk': ('加加速度绝对值', 'rad/s³'),
          'speed': ('速度绝对值', 'rad/s'), 'seam_acceleration': ('接缝加速度差', 'rad/s²')}


ADMISSION_REASONS = {'snapshot_busy': '实时状态快照在读取期限内持续忙碌',
    'state_not_running': '控制器已进入停车或非运行状态', 'epoch_changed': '控制会话已改变',
    'invalid_desired': '控制指令快照无效', 'time_overflow': '制动前瞻时间无效',
    'snapshot_publish_failed': '未来轨迹交接失败', 'feedback_stale': '硬件反馈已过期',
    'takeover_mismatch': '实测位置与保持指令不一致', 'sequence_overflow': '会话序号不可再用'}


def describe(detail):
    if not detail:
        return ''
    if 'quantity' not in detail:
        return ADMISSION_REASONS.get(detail.get('reason'), '')
    label, unit = LABELS.get(detail['quantity'], (detail['quantity'], ''))
    return (f"{detail['joint']} · 段 {detail['segment']} "
            f"[{detail['start_ns'] / 1e6:g}, {detail['end_ns'] / 1e6:g}] ms · "
            f"{label} {detail['value']:.8g} {unit}，限值 {detail['limit']:.8g} {unit}")


def matching_detail(snapshot):
    detail, state = snapshot.get('rejection_detail'), snapshot.get('controller') or {}
    if detail and (detail.get('epoch'), detail.get('sequence'), detail.get('result')) == (
            state.get('epoch'), state.get('last_sequence'), state.get('last_result')):
        return detail
    return None


def controller_error(code, message, snapshot):
    detail = matching_detail(snapshot)
    explanation = describe(detail)
    if not explanation and (snapshot.get('controller') or {}).get('last_result') == 6:
        explanation = '控制器报告位置、速度或加速度超限，尚无匹配批次的轴级诊断'
    return JogError(code, message + ('；' + explanation if explanation else ''), details=detail)


def violation(code, quantity, axis, segment, knots, value, limit, names):
    detail = {'source': 'client_cubic', 'joint': names[axis] if names else f'axis[{axis}]',
              'axis': axis, 'segment': segment, 'start_ns': knots[segment].time_ns,
              'end_ns': knots[segment + 1].time_ns, 'quantity': quantity,
              'value': float(value), 'limit': float(limit)}
    raise JogError(code, describe(detail), details=detail)
