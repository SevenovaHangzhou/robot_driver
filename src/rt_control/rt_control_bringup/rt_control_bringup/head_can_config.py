"""Validate the opt-in native CAN head configuration before starting nodes."""

import math
from pathlib import Path
import re

import yaml


_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
_FIELDS = {
    "can_interface", "configure_timeout_ms", "feedback_timeout_ms",
    "disabled_poll_interval_ms", "max_rx_frames_per_cycle", "joints",
}
_JOINT_FIELDS = {"name", "can_id", "master_id", "min", "max", "velocity_limit"}


def _integer(value, field, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"head CAN {field} must be an integer in [{minimum}, {maximum}]")
    return value


def _number(value, field):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"head CAN {field} must be a finite number")
    return float(value)


def load_head_can_config(path: str | Path, occupied_names=()):
    with Path(path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if type(config) is not dict or set(config) != _FIELDS:
        raise ValueError(f"head CAN configuration requires exactly {sorted(_FIELDS)}")
    if config["can_interface"] != "can2":
        raise ValueError("head CAN requires can2 (PCIe physical port L2)")
    for field in ("configure_timeout_ms", "feedback_timeout_ms", "disabled_poll_interval_ms"):
        _integer(config[field], field, 1, 2147483647)
    _integer(config["max_rx_frames_per_cycle"], "max_rx_frames_per_cycle", 1, 256)
    joints = config["joints"]
    if type(joints) is not list or len(joints) != 2:
        raise ValueError("head CAN joints must contain exactly two motors")
    names = set(occupied_names)
    ids, master_ids = set(), set()
    for joint in joints:
        if type(joint) is not dict or set(joint) != _JOINT_FIELDS:
            raise ValueError(f"head CAN joint requires exactly {sorted(_JOINT_FIELDS)}")
        name = joint["name"]
        if type(name) is not str or not _NAME.fullmatch(name) or name in names:
            raise ValueError(f"head CAN joint name {name!r} is invalid or already owned")
        names.add(name)
        can_id = _integer(joint["can_id"], "can_id", 1, 15)
        master_id = _integer(joint["master_id"], "master_id", 0, 2047)
        if can_id in ids or master_id in master_ids:
            raise ValueError("head CAN motor IDs and feedback Master IDs must be unique")
        ids.add(can_id)
        master_ids.add(master_id)
        lower = _number(joint["min"], "min")
        upper = _number(joint["max"], "max")
        speed = _number(joint["velocity_limit"], "velocity_limit")
        if lower >= upper or speed <= 0:
            raise ValueError("head CAN limits must satisfy min < max and velocity_limit > 0")
    return config
