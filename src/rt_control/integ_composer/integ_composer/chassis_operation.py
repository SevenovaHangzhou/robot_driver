"""Compose the opt-in chassis operation path from the existing NAV calibration."""

from copy import deepcopy
import math
from pathlib import Path

import yaml

from .schema.primitives import resolve_share_path


def configure_operation(controllers, deployment, robot, package_share, *, mock):
    option = deployment.get("chassis_operation", {})
    if not option.get("enabled", False):
        return
    name = option["controller"]
    if name not in controllers:
        # Other scopes of the same unit do not acquire chassis resources.
        return
    params = controllers[name]["ros__parameters"]
    if params.get("calibration_verified") is not True:
        raise ValueError("Operation requires the verified navigation calibration")
    imu = deployment.get("chassis_imu", {})
    if imu.get("driver") != "lpms_nav3_can" or not imu.get("frame_id"):
        raise ValueError("Operation requires the deployed chassis IMU")
    drives = params.get("drive_joints", [])
    steering = params.get("steering_joints", [])
    if len(drives) != 4 or len(steering) != 4 or len(set(drives + steering)) != 8:
        raise ValueError("Operation requires four drive and four steering joints")
    joints = {joint.attrib["name"]: joint for joint in robot.findall(".//ros2_control/joint")}
    for joint in drives:
        if joint not in joints:
            raise ValueError("Operation drive joint missing from hardware: " + joint)
        if not mock:
            element = joints[joint]
            commands = {item.attrib["name"] for item in element.findall("command_interface")}
            states = {item.attrib["name"] for item in element.findall("state_interface")}
            if not {
                "position",
                "velocity",
                "mode_of_operation",
                "write_sequence",
                "write_mask",
            }.issubset(commands) or not {
                "mode_switch_ready",
                "mode_request_ack",
                "feedback_sequence",
                "sent_sequence",
                "feedback_sequence_at_send",
                "sent_velocity",
            }.issubset(states):
                raise ValueError("Operation requires acknowledged mode-switch interfaces: " + joint)
    reference = option["parameters"]
    path = resolve_share_path(Path(package_share(reference["package"])), reference["file"])
    source = yaml.safe_load(path.read_text())[name]["ros__parameters"]
    relative = deepcopy(source["relative"])
    radius = params["wheel_radius"]
    if len(radius) != 4 or any(not math.isfinite(r) or r <= 0 for r in radius):
        raise ValueError("Operation requires four verified positive wheel radii")
    # NAV wheel speed and acceleration are at the rim; runtime limits are wheel rad/s.
    relative.update(
        max_wheel_velocity=[params["max_wheel_speed"] / r for r in radius],
        max_wheel_acceleration=[params["max_wheel_acceleration"] / r for r in radius],
        max_steering_velocity=params["max_steering_slew"],
        max_steering_acceleration=params["max_steering_acceleration"],
        feedback_timeout=params["feedback_timeout"],
        max_update_period=params["max_update_period"],
        imu_timeout=params["imu_timeout"],
        imu_max_increment=params["max_imu_yaw_step"],
        encoder_difference=params["max_encoder_difference"],
        stationary_wheel_velocity=params["fault_stop_speed"] / max(radius),
        stationary_dwell=params["fault_stop_hold"],
        stop_timeout=params["fault_stop_timeout"],
    )
    for field, ceiling in (
        ("max_translation_velocity", "max_linear_speed"),
        ("max_translation_acceleration", "max_body_acceleration"),
        ("max_yaw_velocity", "max_angular_speed"),
    ):
        relative[field] = min(relative[field], params[ceiling])
    for field, value in relative.items():
        if field in ("enabled", "profile_verified"):
            continue
        values = value if isinstance(value, list) else [value]
        if isinstance(value, list) and len(value) != 4:
            raise ValueError("Operation requires four entries: " + field)
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError("Operation requires positive finite parameter: " + field)
    for velocity, acceleration in zip(
        relative["max_wheel_velocity"], relative["max_wheel_acceleration"]
    ):
        if (
            velocity / acceleration
            + relative["stationary_dwell"]
            + 2 * relative["max_update_period"]
            >= relative["stop_timeout"]
        ):
            raise ValueError("Operation cannot stop the navigation wheel limit before timeout")
    # This flag describes the reused hardware/calibration profile. Proposed motion
    # tolerances remain estimates and do not constitute a physical acceptance test.
    relative.update(enabled=True, profile_verified=True)
    params.update(
        relative=relative, imu_enabled=True, imu_topic="/imu/data", imu_frame_id=imu["frame_id"]
    )
