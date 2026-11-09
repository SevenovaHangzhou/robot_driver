"""Future windows for the current RollingSender API; no ROS or session services."""
import numpy as np

from .errors import JogError


def ros_time(nanoseconds):
    seconds, nanos = divmod(int(nanoseconds), 1_000_000_000)
    return {'sec': seconds, 'nanosec': nanos}


def trajectory_window(curve, start_s, origin_ros_ns, horizon_s):
    """Sample one immutable plan, so every replacement lies on the same cubic.

    The public RollingSender supplies epoch/sequence and follows ACKs. This
    function only builds its Window input. It neither sends nor assumes an ACK.
    """
    end = float(curve.x[-1])
    if (type(origin_ros_ns) is not int or origin_ros_ns < 0 or
            not np.isfinite([start_s, horizon_s]).all() or start_s < 0 or start_s >= end or horizon_s <= 0):
        raise JogError('rolling_window', 'Rolling 窗口起点或 ROS 时间无效')
    finish = min(start_s + horizon_s, end)
    # Keep every original cubic boundary. Dropping one changes the controller's
    # interpolating polynomial and breaks replacements between original knots.
    times = [start_s, *curve.x[(curve.x > start_s) & (curve.x < finish)], finish]
    if len(times) > 256:
        raise JogError('rolling_window', 'Rolling 窗口超过公共接口的点数上限')
    final = finish == end
    points = []
    for i, t in enumerate(times):
        positions, velocities = curve(t).tolist(), curve(t, 1).tolist()
        if not np.isfinite([positions, velocities]).all() or len(positions) != 14:
            raise JogError('rolling_window', 'Rolling 窗口必须包含有限的 14 轴 q/v')
        if final and i == len(times) - 1:
            if np.max(np.abs(velocities)) > 1e-10:
                raise JogError('rolling_final', 'final 窗口必须以零速度收尾')
            velocities = [0.] * 14
        points.append({'time_from_start': ros_time(round(t * 1e9) - round(start_s * 1e9)),
                       'positions': positions, 'velocities': velocities})
    return {'replace_from': ros_time(origin_ros_ns + round(start_s * 1e9)),
            'points': points, 'final': final}
