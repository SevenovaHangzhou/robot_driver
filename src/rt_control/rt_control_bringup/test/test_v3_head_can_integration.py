from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from rt_control_bringup.head_can_config import load_head_can_config
from rt_control_bringup.head_runtime import build_head_runtime


BRINGUP = Path(__file__).resolve().parents[1]
ROOT = BRINGUP.parents[2]


def _config():
    return {
        "can_interface": "can2",
        "configure_timeout_ms": 100,
        "feedback_timeout_ms": 50,
        "transition_timeout_ms": 250,
        "command_rate_hz": 250.0,
        "disabled_poll_interval_ms": 20,
        "max_rx_frames_per_cycle": 32,
        "joints": [
            {
                "name": "head_joint", "can_id": 1, "master_id": 17,
                "min": -1.57, "max": 1.57, "velocity_limit": 0.5,
                "acceleration_krad_s2": 0.025,
                "deceleration_krad_s2": -0.05,
                "maximum_speed_rad_s": 20.0,
            },
            {
                "name": "head_pitch_joint", "can_id": 2, "master_id": 18,
                "min": -1.0, "max": 1.0, "velocity_limit": 0.4,
                "acceleration_krad_s2": 0.02,
                "deceleration_krad_s2": -0.04,
                "maximum_speed_rad_s": 18.0,
            },
        ],
    }


def _write(tmp_path, value):
    path = tmp_path / "head.yaml"
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    return path


def test_v3_head_config_requires_motor_trapezoid_and_official_joint_names(tmp_path):
    config = load_head_can_config(_write(tmp_path, _config()))
    assert [joint["name"] for joint in config["joints"]] == [
        "head_joint", "head_pitch_joint"
    ]
    assert config["command_rate_hz"] == 250.0

    invalid = _config()
    invalid["joints"][0]["name"] = "head_motor_1_joint"
    with pytest.raises(ValueError, match="V3 head joints"):
        load_head_can_config(_write(tmp_path, invalid))
    invalid = _config()
    invalid["joints"][0]["deceleration_krad_s2"] = 0.05
    with pytest.raises(ValueError, match="trapezoidal"):
        load_head_can_config(_write(tmp_path, invalid))


def test_head_runtime_uses_v3_model_forward_position_and_no_jtc(tmp_path):
    build = build_head_runtime(
        config=load_head_can_config(_write(tmp_path, _config())),
        description_share=ROOT / "src/description/robot_description",
        hardware_share=ROOT / "src/rt_control/robot_hw_can",
        controller_share=ROOT / "src/rt_control/damiao_head_controller",
        use_mock_hardware=True,
    )
    robot = ET.fromstring(build.robot_description)
    physical = {joint.attrib["name"] for joint in robot.findall("joint")}
    assert {"head_joint", "head_pitch_joint"}.issubset(physical)
    system = robot.find("ros2_control[@name='damiao_head']")
    assert system is not None
    assert [joint.attrib["name"] for joint in system.findall("joint")] == [
        "head_joint", "head_pitch_joint"
    ]
    manager = build.controllers["controller_manager"]["ros__parameters"]
    assert manager["head_position_controller"]["type"] == (
        "forward_command_controller/ForwardCommandController"
    )
    assert manager["damiao_head_manager"]["type"] == (
        "damiao_head_controller/HeadManagerController"
    )
    assert not any("trajectory" in str(value).lower()
                   for value in build.controllers.values())


def test_v3_install_contract_adds_only_explicit_head_runtime():
    cmake = (BRINGUP / "CMakeLists.txt").read_text(encoding="utf-8")
    package = (BRINGUP / "package.xml").read_text(encoding="utf-8")
    start = (BRINGUP / "scripts/rt_control_start").read_text(encoding="utf-8")
    assert "launch/rt_control_head_runtime.launch.py" in cmake
    assert "launch/rt_control.launch.py" not in cmake
    assert "<exec_depend>robot_hw_can</exec_depend>" in package
    assert "<exec_depend>damiao_head_controller</exec_depend>" in package
    assert "<exec_depend>forward_command_controller</exec_depend>" in package
    assert '"--head-runtime"' in start
