import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rt_control_operator_web.config import (  # noqa: E402
    TOKEN_FILE_ENV,
    ConfigError,
    build_config,
    load_token,
)

VALID = dict(
    bind_host="127.0.0.1",
    port=18109,
    command_topic="/cmd_vel",
    controller_node="/swerve_controller",
    gear_names=["leisure", "sport"],
    gear_linear_speeds=[0.05, 0.15],
    gear_angular_speeds=[0.1, 0.3],
    linear_limits=[0.3, 0.5, 1.0, 2.0],
    angular_limits=[0.6, 1.0, 2.0, 4.0],
)


def test_valid_config():
    config = build_config(**VALID)
    assert config.command_topic == "/cmd_vel"
    assert set(config.gears) == {"leisure", "sport"}


@pytest.mark.parametrize(
    "overrides",
    [
        {"bind_host": "0.0.0.0"},
        {"port": 0},
        {"port": 70000},
        {"command_topic": "cmd_vel"},
        {"controller_node": "/swerve_controller/"},
        {"gear_names": ["sport", "leisure"]},
        {"gear_linear_speeds": [0.05]},
        {"linear_limits": [0.3, 0.5, 1.0]},
        {"angular_limits": [0.6, 1.0, 2.0]},
    ],
)
def test_invalid_config_rejected(overrides):
    with pytest.raises(ConfigError):
        build_config(**{**VALID, **overrides})


@pytest.mark.parametrize("topic", ["/cmd_vel_safe", "cmd_vel_safe"])
def test_motion_owned_topic_is_forbidden(topic):
    with pytest.raises(ConfigError, match="Motion-owned"):
        build_config(**{**VALID, "command_topic": topic})


def write_token(tmp_path, mode=0o600, text="secret-token"):
    path = tmp_path / "token"
    path.write_text(text)
    os.chmod(path, mode)
    return path


def test_token_is_read_from_a_private_file(tmp_path):
    path = write_token(tmp_path)
    assert load_token({TOKEN_FILE_ENV: str(path)}) == "secret-token"


def test_missing_token_variable_is_rejected():
    with pytest.raises(ConfigError, match=TOKEN_FILE_ENV):
        load_token({})


@pytest.mark.parametrize("mode,text", [(0o640, "secret"), (0o600, ""), (0o600, "two words")])
def test_unsafe_or_invalid_token_rejected(tmp_path, mode, text):
    path = write_token(tmp_path, mode, text)
    with pytest.raises(ConfigError):
        load_token({TOKEN_FILE_ENV: str(path)})
