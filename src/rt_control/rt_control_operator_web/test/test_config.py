import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rt_control_operator_web.config import (  # noqa: E402
    TOKEN_FILE_ENV,
    ConfigError,
    build_config,
    load_token,
)

LAYOUT = [
    "front_left:front", "front_left:left", "front_right:front", "front_right:right",
    "rear_left:rear", "rear_left:left", "rear_right:rear", "rear_right:right",
]
VALID = dict(
    bind_host="127.0.0.1",
    port=18109,
    command_topic="/swerve_controller/cmd_vel",
    controller_node="/swerve_controller",
    joint_states_topic="/joint_states",
    battery_topic="/battery_state",
    gear_names=["leisure", "sport"],
    gear_linear_speeds=[0.05, 0.15],
    gear_angular_speeds=[0.1, 0.3],
    linear_limits=[0.3, 0.5, 1.0, 2.0],
    angular_limits=[0.6, 1.0, 2.0, 4.0],
    ultrasonic_topics=[f"/ultrasonic/channel{i}/range" for i in range(1, 9)],
    ultrasonic_layout=LAYOUT,
    ultrasonic_layout_confirmed=False,
    robot_description_topic="/robot_description",
    chassis_root_link="chassis_base",
)


def config(**overrides):
    return build_config(**{**VALID, **overrides})


def test_valid_config() -> None:
    result = config()
    assert result.command_topic == "/swerve_controller/cmd_vel"
    assert list(result.gears) == ["leisure", "sport"]
    assert result.limits.linear.stop_decel == 1.0
    assert [c.corner for c in result.ultrasonic][:2] == ["front_left", "front_left"]
    assert result.ultrasonic_layout_confirmed is False


@pytest.mark.parametrize("topic", ["/cmd_vel_safe", "cmd_vel_safe", " /cmd_vel_safe "])
def test_motion_owned_topic_is_forbidden(topic: str) -> None:
    with pytest.raises(ConfigError, match="N-04"):
        config(command_topic=topic)


@pytest.mark.parametrize(
    "overrides",
    [
        {"bind_host": ""},
        {"bind_host": "0.0.0.0"},
        {"bind_host": "::"},
        {"bind_host": "robot.local"},
        {"port": 0},
        {"port": 70000},
        {"command_topic": ""},
        {"controller_node": " "},
        {"joint_states_topic": ""},
        {"battery_topic": " "},
        {"gear_names": ["leisure"]},
        {"gear_names": ["sport", "sport"]},
        {"gear_linear_speeds": [0.05, 0.0]},
        {"linear_limits": [0.3, 0.5, 1.0]},
        {"angular_limits": [0.6, 1.0, 0.5, 4.0]},
        {"linear_limits": [0.3, 0.5, 1.0, "x"]},
        {"ultrasonic_layout": LAYOUT[:7]},
        {"ultrasonic_layout": ["front_left:rear"] + LAYOUT[1:]},
        {"ultrasonic_layout": ["middle:front"] + LAYOUT[1:]},
        {"ultrasonic_topics": ["/a"] * 8},
        {"robot_description_topic": ""},
        {"chassis_root_link": " "},
    ],
)
def test_invalid_config_rejected(overrides) -> None:
    with pytest.raises(ConfigError):
        config(**overrides)


def write_token(tmp_path: Path, text: str, mode: int = 0o600) -> Path:
    path = tmp_path / "token"
    path.write_text(text, encoding="utf-8")
    os.chmod(path, mode)
    return path


def test_token_loaded_from_private_file(tmp_path: Path) -> None:
    path = write_token(tmp_path, "0123456789abcdef-token\n")
    assert load_token({TOKEN_FILE_ENV: str(path)}) == "0123456789abcdef-token"


def test_token_env_required() -> None:
    with pytest.raises(ConfigError, match=TOKEN_FILE_ENV):
        load_token({})


def test_missing_token_file(tmp_path: Path) -> None:
    with pytest.raises(ConfigError):
        load_token({TOKEN_FILE_ENV: str(tmp_path / "absent")})


def test_token_file_must_be_regular(tmp_path: Path) -> None:
    with pytest.raises(ConfigError):
        load_token({TOKEN_FILE_ENV: str(tmp_path)})


def test_group_readable_token_rejected(tmp_path: Path) -> None:
    path = write_token(tmp_path, "0123456789abcdef-token", 0o640)
    with pytest.raises(ConfigError, match="chmod 600"):
        load_token({TOKEN_FILE_ENV: str(path)})


def test_single_character_token_accepted(tmp_path: Path) -> None:
    path = write_token(tmp_path, "1\n")
    assert load_token({TOKEN_FILE_ENV: str(path)}) == "1"


@pytest.mark.parametrize("text", ["", "  \n", "12 34"])
def test_empty_or_whitespace_token_rejected(tmp_path: Path, text: str) -> None:
    path = write_token(tmp_path, text)
    with pytest.raises(ConfigError):
        load_token({TOKEN_FILE_ENV: str(path)})
