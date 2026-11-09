"""Body-frame translation and moving J7-frame rotation. SI twist units."""
import math

import numpy as np

from .control import SPEED_OPTIONS_MM_S, SPEED_ERROR
from .errors import JogError
from .planner import DIRECTIONS

ANGULAR_SPEED_OPTIONS_DEG_S = (.5, 1, 2, 5)
ROTATION_AXES = {'roll': 'x', 'pitch': 'y', 'yaw': 'z'}


def velocity_command(body, mapping):
    rotation = body.get('robot_rotation')
    if rotation is not None:
        if (not isinstance(rotation, str) or rotation[:-1] not in ROTATION_AXES
                or rotation[-1:] not in ('+', '-') or body.get('rotation_frame', 'j7') != 'j7'
                or any(k in body for k in
                ('robot_direction', 'direction', 'speed_mm_s', 'step_mm'))):
            raise JogError('bad_rotation', '姿态控制只接受单个 J7 局部旋转轴 yaw/pitch/roll 的正反方向')
        speed = body.get('speed_deg_s')
        if type(speed) not in (int, float) or speed not in ANGULAR_SPEED_OPTIONS_DEG_S:
            raise JogError('bad_speed', '旋转速度必须为 0.5、1、2 或 5 °/s')
        axis = ROTATION_AXES[rotation[:-1]] + rotation[-1]
        twist = np.r_[np.zeros(3), np.asarray(DIRECTIONS[axis]) * math.radians(speed)]
        return twist, {'direction': rotation, 'rotation_axis': axis, 'rotation_frame': 'j7',
                       'speed_deg_s': speed, 'continuous': True}
    direction = body.get('robot_direction')
    if (not isinstance(direction, str) or direction not in mapping
            or any(k in body for k in ('direction', 'speed_deg_s', 'rotation_frame', 'robot_rotation'))):
        raise JogError('bad_direction', '请提供单个机器人本体平移方向 robot_direction')
    speed = body.get('speed_mm_s')
    if type(speed) not in (int, float) or speed not in SPEED_OPTIONS_MM_S or 'step_mm' in body:
        raise JogError('bad_speed', SPEED_ERROR)
    axis = mapping[direction]
    return np.r_[np.asarray(DIRECTIONS[axis]) * speed / 1000, np.zeros(3)], {
        'direction': axis, 'speed_mm_s': speed, 'continuous': True}
