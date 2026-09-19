"""Validate the explicit V3 DaMiao head runtime configuration."""

import math
from pathlib import Path

import yaml


_FIELDS = {
    "can_interface", "configure_timeout_ms", "feedback_timeout_ms",
    "transition_timeout_ms", "command_rate_hz", "disabled_poll_interval_ms",
    "max_rx_frames_per_cycle", "joints",
}
_JOINT_FIELDS = {
    "name", "can_id", "master_id", "min", "max", "velocity_limit",
    "acceleration_krad_s2", "deceleration_krad_s2", "maximum_speed_rad_s",
}
_JOINT_NAMES = ("head_joint", "head_pitch_joint")


def _integer(value, field, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"head CAN {field} must be an integer in [{minimum}, {maximum}]")
    return value


def _number(value, field):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"head CAN {field} must be a finite number")
    return float(value)


def load_head_can_config(path: str | Path):
    with Path(path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if type(config) is not dict or set(config) != _FIELDS:
        raise ValueError(f"head CAN configuration requires exactly {sorted(_FIELDS)}")
    if config["can_interface"] != "can2":
        raise ValueError("V3 DaMiao head requires PCIe L2 as can2")
    for field in (
        "configure_timeout_ms", "feedback_timeout_ms", "transition_timeout_ms",
        "disabled_poll_interval_ms",
    ):
        _integer(config[field], field, 1, 2147483647)
    rate = _number(config["command_rate_hz"], "command_rate_hz")
    if rate <= 0.0 or rate > 1000.0:
        raise ValueError("head CAN command_rate_hz must be in (0, 1000]")
    _integer(config["max_rx_frames_per_cycle"], "max_rx_frames_per_cycle", 1, 256)
    joints = config["joints"]
    if type(joints) is not list or len(joints) != 2:
        raise ValueError("head CAN joints must contain exactly two motors")
    if tuple(joint.get("name") for joint in joints if type(joint) is dict) != _JOINT_NAMES:
        raise ValueError("V3 head joints must be head_joint and head_pitch_joint in order")
    ids, master_ids = set(), set()
    for joint in joints:
        if type(joint) is not dict or set(joint) != _JOINT_FIELDS:
            raise ValueError(f"head CAN joint requires exactly {sorted(_JOINT_FIELDS)}")
        can_id = _integer(joint["can_id"], "can_id", 1, 15)
        master_id = _integer(joint["master_id"], "master_id", 0, 2047)
        if can_id in ids or master_id in master_ids:
            raise ValueError("head CAN motor IDs and feedback Master IDs must be unique")
        ids.add(can_id)
        master_ids.add(master_id)
        lower = _number(joint["min"], "min")
        upper = _number(joint["max"], "max")
        speed = _number(joint["velocity_limit"], "velocity_limit")
        acceleration = _number(joint["acceleration_krad_s2"], "acceleration_krad_s2")
        deceleration = _number(joint["deceleration_krad_s2"], "deceleration_krad_s2")
        maximum_speed = _number(joint["maximum_speed_rad_s"], "maximum_speed_rad_s")
        if lower >= upper or speed <= 0.0:
            raise ValueError("head CAN joint limits require min < max and velocity_limit > 0")
        if acceleration <= 0.0 or deceleration >= 0.0 or maximum_speed <= 0.0:
            raise ValueError("head CAN trapezoidal profile requires ACC>0, DEC<0 and MAX_SPD>0")
    return config
