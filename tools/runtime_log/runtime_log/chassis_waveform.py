"""Archived measured wheel twist and displacement from two steering-angle sources.

All channels use measured wheel feedback and the archived run geometry; none
consults targets or today's live deployment.
"""

import json
import math
from pathlib import Path

CHANNEL = "__chassis_actual__"
ENCODER_CHANNEL = "__chassis_encoder_actual__"
DISPLACEMENT_CHANNEL = "__chassis_displacement__"
ENCODER_DISPLACEMENT_CHANNEL = "__chassis_encoder_displacement__"
CHANNELS = frozenset((CHANNEL, ENCODER_CHANNEL, DISPLACEMENT_CHANNEL, ENCODER_DISPLACEMENT_CHANNEL))
DISPLACEMENT_CHANNELS = frozenset((DISPLACEMENT_CHANNEL, ENCODER_DISPLACEMENT_CHANNEL))
ENCODER_CHANNELS = frozenset((ENCODER_CHANNEL, ENCODER_DISPLACEMENT_CHANNEL))
DESCRIPTIONS = {
    CHANNEL: (
        "轮系估计（转向电机角）：实际轮速 × 轮半径，配合转向电机实际角度和方向标定，"
        "按 swerve_driver 正运动学最小二乘及异常轮剔除求 vx、vy、w；"
        "不是 /cmd_vel 指令，也不是独立定位测得的地面速度。无有效解时断线。"
        "历史记录不含控制器内部保持锁存状态，不能逐周期重放该状态过滤。"
    ),
    ENCODER_CHANNEL: (
        "轮系估计（外部转角编码器）：实际轮速 × 轮半径，配合外部转角编码器的"
        "实际轮组方向，按 swerve_driver 正运动学最小二乘及异常轮剔除求 vx、vy、w；"
        "不是 /cmd_vel 指令，也不是独立定位测得的地面速度。编码器未就绪、反馈过期"
        "或无有效解时断线。CANopen 编码器与 EtherCAT 轮速并非同周期采样。"
    ),
    DISPLACEMENT_CHANNEL: (
        "轮系里程计位移（转向电机角）：用实际轮位置增量 × 轮半径，配合转向"
        "电机实际角度，从所选时间区间起点开始累计 x、y、yaw 和路程；"
        "不是外部定位测得的真实位移，轮胎打滑和轮径误差会累积。"
    ),
    ENCODER_DISPLACEMENT_CHANNEL: (
        "轮系里程计位移（外部转角编码器）：用实际轮位置增量 × 轮半径，"
        "配合外部转角编码器方向，从所选时间区间起点开始累计 x、y、yaw"
        "和路程；不是外部定位测得的真实位移。编码器未就绪或过期时断线。"
    ),
}


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


class ChassisFeedback:
    def __init__(self, parameters):
        self.p = parameters
        names = ("drive_joints", "steering_joints")
        arrays = (
            "module_x",
            "module_y",
            "wheel_radius",
            "steering_to_yaw_sign",
            "steering_min",
            "steering_max",
            "steering_limit_margin",
        )
        for key in names + arrays:
            if not isinstance(parameters.get(key), list) or len(parameters[key]) != 4:
                raise ValueError("归档缺少四轮运动学参数：" + key)
        for key in names:
            if (
                any(not isinstance(n, str) or not n for n in parameters[key])
                or len(set(parameters[key])) != 4
            ):
                raise ValueError("归档的关节名称无效")
        for key in arrays:
            if not all(finite(n) for n in parameters[key]):
                raise ValueError("归档的运动学参数非有限：" + key)
        self.timeout = parameters.get("feedback_timeout")
        self.threshold = parameters.get("slip_residual_threshold")
        if not all(finite(n) and n > 0 for n in (self.timeout, self.threshold)):
            raise ValueError("归档缺少有效的反馈超时或异常轮阈值")
        self.modules = []
        for i in range(4):
            radius = parameters["wheel_radius"][i]
            sign = parameters["steering_to_yaw_sign"][i]
            margin = parameters["steering_limit_margin"][i]
            low = parameters["steering_min"][i] + margin
            high = parameters["steering_max"][i] - margin
            if radius <= 0 or sign not in (-1, 1) or margin < 0 or low >= high:
                raise ValueError("归档的轮半径、方向或转向范围无效")
            self.modules.append(
                (
                    parameters["drive_joints"][i],
                    parameters["steering_joints"][i],
                    parameters["module_x"][i],
                    parameters["module_y"][i],
                    radius,
                    sign,
                    low,
                    high,
                )
            )
        if len({(m[2], m[3]) for m in self.modules}) < 2:
            raise ValueError("归档的轮子坐标不可求解")
        self.feedback_keys = [
            (
                drive + "/velocity",
                drive + "/position",
                steer + "/position",
                drive + "/status_word",
                steer + "/status_word",
                steer + "/mode_of_operation_display",
                drive + "/mode_of_operation_display",
            )
            for drive, steer, *_ in self.modules
        ]
        encoders = parameters.get("steering_encoders")
        self.encoder_names = (
            encoders
            if isinstance(encoders, list)
            and len(encoders) == 4
            and all(isinstance(n, str) and n for n in encoders)
            and len(set(encoders)) == 4
            else None
        )
        self.center_x = sum(m[2] for m in self.modules) / 4
        self.center_y = sum(m[3] for m in self.modules) / 4
        self.fit_denominator = sum(
            (m[2] - self.center_x) ** 2 + (m[3] - self.center_y) ** 2 for m in self.modules
        )

    def fields_available(self, fields, channel=CHANNEL):
        required = {"ethercat_domain/process_data_age_ms"}
        for index, (drive, steer, *_) in enumerate(self.modules):
            required.update(
                f"{drive}/{key}"
                for key in ("position", "velocity", "status_word", "mode_of_operation_display")
            )
            if channel not in ENCODER_CHANNELS:
                required.update(
                    f"{steer}/{key}"
                    for key in ("position", "status_word", "mode_of_operation_display")
                )
            elif self.encoder_names:
                required.update(
                    f"{self.encoder_names[index]}/{key}"
                    for key in ("position", "feedback_age_ms", "ready")
                )
            else:
                return False
        return required <= set(fields)

    def module_feedback(self, row, channel=CHANNEL):
        # Same bus freshness gate as SwerveController::read_feedback().
        age = row.get("ethercat_domain/process_data_age_ms")
        if not finite(age) or not 0 <= age <= self.timeout * 1000:
            return [None] * 4
        modules = []
        for index, ((_, _, x, y, radius, sign, low, high), keys) in enumerate(
            zip(self.modules, self.feedback_keys)
        ):
            velocity, position = row.get(keys[0]), row.get(keys[1])
            drive_word = row.get(keys[3])
            if not all(finite(v) for v in (velocity, position, drive_word)):
                modules.append(None)
                continue
            if channel not in ENCODER_CHANNELS:
                angle = row.get(keys[2])
                steer_word = row.get(keys[4])
                if not all(finite(v) for v in (angle, steer_word)):
                    modules.append(None)
                    continue
                angle *= sign
                angle_valid = int(steer_word) & 0x6F == 0x27 and row.get(keys[5]) == 8
            elif self.encoder_names:
                encoder = self.encoder_names[index]
                angle = row.get(encoder + "/position")
                encoder_age = row.get(encoder + "/feedback_age_ms")
                ready = row.get(encoder + "/ready")
                angle_valid = (
                    all(finite(v) for v in (angle, encoder_age, ready))
                    and ready == 1
                    and 0 <= encoder_age <= self.timeout * 1000
                )
            else:
                return [None] * 4
            if (
                not angle_valid
                or not low <= angle <= high
                or int(drive_word) & 0x6F != 0x27
                or row.get(keys[6]) not in (8, 9)
            ):
                modules.append(None)
                continue
            speed = velocity * radius
            if not finite(speed) or not finite(position * radius):
                modules.append(None)
                continue
            modules.append((x, y, speed, position * radius, angle))
        return modules

    def fit(self, row, channel=CHANNEL):
        modules = self.module_feedback(row, channel)
        observations = [
            (i, x, y, speed * math.cos(angle), speed * math.sin(angle))
            for i, item in enumerate(modules)
            if item is not None
            for x, y, speed, _, angle in (item,)
        ]
        # Equivalent normal-equation solve to fit_module_observations(), including
        # removal of the largest residual and the same minimum of two modules.
        while len(observations) >= 2:
            n = len(observations)
            if n == 4:
                x, y, denominator = self.center_x, self.center_y, self.fit_denominator
                total_u = total_v = moment = 0.0
                for _, a, b, c, d in observations:
                    total_u += c
                    total_v += d
                    moment += -(b - y) * c + (a - x) * d
                u, v = total_u / n, total_v / n
            else:
                x, y, u, v = [sum(o[k] for o in observations) / n for k in range(1, 5)]
                denominator = sum((a - x) ** 2 + (b - y) ** 2 for _, a, b, _, _ in observations)
                moment = sum(
                    -(b - y) * (c - u) + (a - x) * (d - v) for _, a, b, c, d in observations
                )
            if denominator < 1e-12:
                return None
            w = moment / denominator
            vx, vy = u + w * y, v - w * x
            if not all(finite(z) for z in (vx, vy, w)):
                return None
            residuals = [
                math.hypot(c - (vx - w * b), d - (vy + w * a)) for _, a, b, c, d in observations
            ]
            worst = max(range(n), key=residuals.__getitem__)
            if residuals[worst] <= self.threshold:
                return (vx, vy, w), {item[0] for item in observations}, modules
            if n == 2:
                return None
            observations.pop(worst)
        return None

    def sample(self, row, channel=CHANNEL):
        fit = self.fit(row, channel)
        return fit[0] if fit else None


def _solve_three_by_three(matrix):
    matrix = [row[:] for row in matrix]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(matrix[row][column]))
        if abs(matrix[pivot][column]) < 1e-12:
            return None
        matrix[column], matrix[pivot] = matrix[pivot], matrix[column]
        divisor = matrix[column][column]
        matrix[column] = [value / divisor for value in matrix[column]]
        for row in range(3):
            if row == column:
                continue
            scale = matrix[row][column]
            matrix[row] = [
                value - scale * other for value, other in zip(matrix[row], matrix[column])
            ]
    return tuple(row[3] for row in matrix)


def _position_delta(previous, current, used):
    normal = [[0.0] * 4 for _ in range(3)]
    count = 0
    for index in used:
        old, new = previous[index], current[index]
        if old is None or new is None:
            continue
        x, y, _, distance, angle = new
        delta = distance - old[3]
        midpoint = old[4] + math.atan2(math.sin(angle - old[4]), math.cos(angle - old[4])) / 2
        for coefficients, value in (
            ((1.0, 0.0, -y), delta * math.cos(midpoint)),
            ((0.0, 1.0, x), delta * math.sin(midpoint)),
        ):
            for outer in range(3):
                for inner in range(3):
                    normal[outer][inner] += coefficients[outer] * coefficients[inner]
                normal[outer][3] += coefficients[outer] * value
        count += 1
    return _solve_three_by_three(normal) if count >= 2 else None


class ChassisDisplacement:
    """Interval-relative wheel odometry using the archived per-cycle positions."""

    def __init__(self, feedback, channel):
        self.feedback = feedback
        self.channel = channel
        self.previous = [None] * 4
        self.x = self.y = self.yaw = self.distance = 0.0

    def sample(self, row):
        current = self.feedback.module_feedback(row, self.channel)
        fit = self.feedback.fit(row, self.channel)
        if not fit:
            for index, module in enumerate(current):
                if module is not None:
                    self.previous[index] = module
            return None
        _, used, current = fit
        delta = _position_delta(self.previous, current, used)
        for index, module in enumerate(current):
            if module is not None:
                self.previous[index] = module
        if delta is None:
            if sum(item is not None for item in current) < 2:
                return None
            return self.x, self.y, self.yaw, self.distance
        dx, dy, dyaw = delta
        if not all(finite(value) for value in delta):
            return None
        if abs(dyaw) < 1e-9:
            local_x = dx * (1.0 - dyaw * dyaw / 6.0) - dy * dyaw / 2.0
            local_y = dx * dyaw / 2.0 + dy * (1.0 - dyaw * dyaw / 6.0)
        else:
            local_x = dx * math.sin(dyaw) / dyaw - dy * (1.0 - math.cos(dyaw)) / dyaw
            local_y = dx * (1.0 - math.cos(dyaw)) / dyaw + dy * math.sin(dyaw) / dyaw
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)
        self.x += local_x * cosine - local_y * sine
        self.y += local_x * sine + local_y * cosine
        self.yaw += dyaw
        self.distance += math.hypot(local_x, local_y)
        return self.x, self.y, self.yaw, self.distance


def from_run(run_file: Path):
    try:
        run = json.loads(run_file.read_text())
        artifact = run["artifact"]
        for name, plugin in artifact.get("controllers", {}).items():
            if plugin == "swerve_driver/SwerveController":
                return ChassisFeedback(artifact["controller_parameters"][name])
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        pass
    return None


def for_source(source: Path):
    return from_run(source.parents[2] / "run.json")
