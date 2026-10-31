"""Chassis console connection and command-profile configuration."""

import ipaddress
import os
import stat
from dataclasses import dataclass
from pathlib import Path
from .teleop import Gear, AxisLimits, MotionLimits, validate_gears, validate_limits

TOKEN_FILE_ENV = "RT_OPERATOR_WEB_TOKEN_FILE"


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class OperatorWebConfig:
    bind_host: str
    port: int
    command_topic: str
    controller_node: str
    gears: dict
    limits: MotionLimits


def load_token(environ=os.environ):
    name = environ.get(TOKEN_FILE_ENV, "")
    if not name:
        raise ConfigError(f"{TOKEN_FILE_ENV} is required")
    path = Path(name)
    info = path.stat()
    if not stat.S_ISREG(info.st_mode) or info.st_mode & 0o077:
        raise ConfigError("access token must be a regular mode600 file")
    token = path.read_text().strip()
    if not token or any(x.isspace() for x in token):
        raise ConfigError("invalid access token")
    return token


def build_config(
    *,
    bind_host,
    port,
    command_topic,
    controller_node,
    gear_names,
    gear_linear_speeds,
    gear_angular_speeds,
    linear_limits,
    angular_limits,
):
    if ipaddress.ip_address(bind_host).is_unspecified:
        raise ConfigError("use an explicit bind IP")
    if not 1 <= port <= 65535:
        raise ConfigError("invalid port")
    if command_topic in {"/cmd_vel_safe", "cmd_vel_safe"}:
        raise ConfigError("/cmd_vel_safe is Motion-owned")
    for name in [command_topic, controller_node]:
        if not name.startswith("/") or name.endswith("/") or " " in name:
            raise ConfigError("use absolute ROS names")
    if (
        list(gear_names) != ["leisure", "sport"]
        or len(gear_linear_speeds) != 2
        or len(gear_angular_speeds) != 2
    ):
        raise ConfigError("configure leisure and sport profiles")
    gears = validate_gears(
        {n: Gear(v, w) for n, v, w in zip(gear_names, gear_linear_speeds, gear_angular_speeds)}
    )
    if len(linear_limits) != 4 or len(angular_limits) != 4:
        raise ConfigError("limits are accel,decel,stop_decel,jerk")
    limits = validate_limits(MotionLimits(AxisLimits(*linear_limits), AxisLimits(*angular_limits)))
    return OperatorWebConfig(bind_host, port, command_topic, controller_node, gears, limits)
