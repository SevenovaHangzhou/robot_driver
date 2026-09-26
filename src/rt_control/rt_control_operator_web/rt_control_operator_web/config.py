"""Explicit, fail-closed operator web configuration."""

from __future__ import annotations

import ipaddress
import os
import stat
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from .teleop import AxisLimits, Gear, MotionLimits, validate_gears, validate_limits

TOKEN_FILE_ENV = "RT_OPERATOR_WEB_TOKEN_FILE"
# Operator decision (2026-09-25): any non-empty token is accepted, including a
# single character. Brute-force protection is therefore minimal; see README.
MIN_TOKEN_LENGTH = 1
# BQ-153 (ELECTRI-109/ELECTRI-142): N-04 names Motion as the sole producer of
# this topic. V1 must never publish it until the public contract is amended.
FORBIDDEN_COMMAND_TOPICS = frozenset({"/cmd_vel_safe", "cmd_vel_safe"})
CORNERS = {
    "front_left": {"front", "left"},
    "front_right": {"front", "right"},
    "rear_left": {"rear", "left"},
    "rear_right": {"rear", "right"},
}


class ConfigError(Exception):
    pass


@dataclass(frozen=True)
class UltrasonicChannel:
    topic: str
    corner: str
    facing: str


@dataclass(frozen=True)
class OperatorWebConfig:
    bind_host: str
    port: int
    command_topic: str
    controller_node: str
    joint_states_topic: str
    battery_topic: str
    gears: Mapping[str, Gear]
    limits: MotionLimits
    ultrasonic: tuple[UltrasonicChannel, ...]
    ultrasonic_layout_confirmed: bool
    robot_description_topic: str
    chassis_root_link: str


def load_token(environ: Mapping[str, str] = os.environ) -> str:
    path_text = environ.get(TOKEN_FILE_ENV, "").strip()
    if not path_text:
        raise ConfigError(f"{TOKEN_FILE_ENV} must name the access-token file")
    path = Path(path_text)
    try:
        info = path.stat()
    except OSError as exc:
        raise ConfigError(f"cannot read token file: {exc.strerror}") from exc
    if not stat.S_ISREG(info.st_mode):
        raise ConfigError("token file must be a regular file")
    if info.st_mode & (stat.S_IRWXG | stat.S_IRWXO):
        raise ConfigError("token file must not be accessible by group or others (chmod 600)")
    token = path.read_text(encoding="utf-8").strip()
    if len(token) < MIN_TOKEN_LENGTH or any(ch.isspace() for ch in token):
        raise ConfigError(
            f"token must be at least {MIN_TOKEN_LENGTH} characters without whitespace"
        )
    return token


def _required_name(value: str, label: str) -> str:
    text = value.strip()
    if not text:
        raise ConfigError(f"{label} must be set explicitly")
    return text


def _ultrasonic(topics: Sequence[str], layout: Sequence[str]) -> tuple[UltrasonicChannel, ...]:
    if len(topics) != len(layout):
        raise ConfigError("ultrasonic_topics and ultrasonic_layout must have equal length")
    channels = []
    for topic, entry in zip(topics, layout):
        corner, _, facing = entry.partition(":")
        if corner not in CORNERS or facing not in CORNERS[corner]:
            raise ConfigError(
                f"ultrasonic layout entry {entry!r} must be <corner>:<outward facing>"
            )
        channels.append(
            UltrasonicChannel(_required_name(topic, "ultrasonic topic"), corner, facing)
        )
    if len({c.topic for c in channels}) != len(channels):
        raise ConfigError("ultrasonic topics must be unique")
    return tuple(channels)


def build_config(
    *,
    bind_host: str,
    port: int,
    command_topic: str,
    controller_node: str,
    joint_states_topic: str,
    battery_topic: str,
    gear_names: Sequence[str],
    gear_linear_speeds: Sequence[float],
    gear_angular_speeds: Sequence[float],
    linear_limits: Sequence[float],
    angular_limits: Sequence[float],
    ultrasonic_topics: Sequence[str],
    ultrasonic_layout: Sequence[str],
    ultrasonic_layout_confirmed: bool,
    robot_description_topic: str,
    chassis_root_link: str,
) -> OperatorWebConfig:
    host = bind_host.strip()
    if not host:
        raise ConfigError("bind_host must be set explicitly")
    try:
        address = ipaddress.ip_address(host)
    except ValueError as exc:
        raise ConfigError("bind_host must be a literal IP address") from exc
    if address.is_unspecified:
        raise ConfigError("bind_host must not be a wildcard address")
    if not isinstance(port, int) or not 1 <= port <= 65535:
        raise ConfigError("port must be in 1..65535")
    topic = _required_name(command_topic, "command_topic")
    if topic in FORBIDDEN_COMMAND_TOPICS:
        raise ConfigError(
            "/cmd_vel_safe is Motion-owned (N-04); V1 may only target a Mock controller topic"
        )
    if not (len(gear_names) == len(gear_linear_speeds) == len(gear_angular_speeds)):
        raise ConfigError("gear parameter lists must have equal length")
    if len(set(gear_names)) != len(gear_names):
        raise ConfigError("gear names must be unique")
    if len(linear_limits) != 4 or len(angular_limits) != 4:
        raise ConfigError("motion limits must be [accel, decel, stop_decel, jerk]")
    try:
        gears = validate_gears(
            {
                name: Gear(linear, angular)
                for name, linear, angular in zip(
                    gear_names, gear_linear_speeds, gear_angular_speeds
                )
            }
        )
        limits = validate_limits(
            MotionLimits(AxisLimits(*linear_limits), AxisLimits(*angular_limits))
        )
    except (TypeError, ValueError) as exc:
        raise ConfigError(str(exc)) from exc
    return OperatorWebConfig(
        bind_host=host,
        port=port,
        command_topic=topic,
        controller_node=_required_name(controller_node, "controller_node"),
        joint_states_topic=_required_name(joint_states_topic, "joint_states_topic"),
        battery_topic=_required_name(battery_topic, "battery_topic"),
        gears=gears,
        limits=limits,
        ultrasonic=_ultrasonic(ultrasonic_topics, ultrasonic_layout),
        ultrasonic_layout_confirmed=bool(ultrasonic_layout_confirmed),
        robot_description_topic=_required_name(robot_description_topic, "robot_description_topic"),
        chassis_root_link=_required_name(chassis_root_link, "chassis_root_link"),
    )
