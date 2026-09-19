"""Optional head hardware wiring must stay dormant without explicit configuration."""

import importlib.util
from pathlib import Path
import sys

from launch import LaunchContext
from launch_ros.actions import Node
import pytest
import yaml


BRINGUP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRINGUP))
sys.path.insert(0, str(BRINGUP.parent / "x503_force_sensor"))
from rt_control_bringup.head_can_config import load_head_can_config  # noqa: E402


def _config():
    return {
        "can_interface": "can2",
        "configure_timeout_ms": 100,
        "feedback_timeout_ms": 100,
        "disabled_poll_interval_ms": 100,
        "max_rx_frames_per_cycle": 16,
        "joints": [
            {"name": "head_motor_1_joint", "can_id": 1, "master_id": 17,
             "min": -0.1, "max": 0.1, "velocity_limit": 0.1},
            {"name": "head_motor_2_joint", "can_id": 2, "master_id": 18,
             "min": -0.2, "max": 0.2, "velocity_limit": 0.2},
        ],
    }


def _write(tmp_path, config):
    path = tmp_path / "head.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    return path


def test_missing_limits_cannot_be_selected(tmp_path):
    config = _config()
    config["joints"][0].pop("min")
    with pytest.raises(ValueError, match="requires exactly"):
        load_head_can_config(_write(tmp_path, config))


@pytest.mark.parametrize("change", [
    lambda c: c["joints"][1].update(name="head_motor_1_joint"),
    lambda c: c["joints"][1].update(master_id=17),
    lambda c: c["joints"][0].update(min=0.2),
    lambda c: c["joints"][0].update(velocity_limit=0),
    lambda c: c["joints"][0].update(name="turn"),
    lambda c: c.update(can_interface="pciecan2"),
])
def test_invalid_head_configuration_rejected(tmp_path, change):
    config = _config()
    change(config)
    with pytest.raises(ValueError):
        load_head_can_config(_write(tmp_path, config), occupied_names=["turn"])


def test_valid_head_configuration_builds_matching_xacro_arguments(tmp_path):
    config = load_head_can_config(_write(tmp_path, _config()), occupied_names=["turn"])
    spec = importlib.util.spec_from_file_location("head_bringup", BRINGUP / "launch/rt_control.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    arguments = "".join(module._head_xacro_arguments(config))
    assert "head_joint_1_name:=head_motor_1_joint" in arguments
    assert "head_joint_2_name:=head_motor_2_joint" in arguments
    assert "head_motor_2_master_id:=18" in arguments
    declared = {entity.name for entity in module.generate_launch_description().entities
                if hasattr(entity, "name")}
    assert "head_can_config" in declared


def test_head_controller_yaml_matches_hardware_joint_names():
    config = yaml.safe_load((BRINGUP / "config/controllers.yaml").read_text(encoding="utf-8"))
    expected = ["head_motor_1_joint", "head_motor_2_joint"]
    registered = config["controller_manager"]["ros__parameters"]
    assert registered["damiao_head_controller"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert registered["damiao_joint_state_broadcaster"]["type"] == (
        "joint_state_broadcaster/JointStateBroadcaster"
    )
    broadcaster = config["damiao_joint_state_broadcaster"]["ros__parameters"]
    trajectory = config["damiao_head_controller"]["ros__parameters"]
    assert broadcaster["joints"] == trajectory["joints"] == expected
    assert broadcaster["interfaces"] == ["position", "velocity", "effort"]
    assert trajectory["command_interfaces"] == ["position"]
    assert trajectory["state_interfaces"] == ["position", "velocity"]
    assert trajectory["allow_partial_joints_goal"] is False
    assert "damiao_motor_1_controller" not in config


def test_launch_rejects_head_names_that_disagree_with_controller_yaml(tmp_path, monkeypatch):
    config = _config()
    config["joints"][0]["name"] = "head_different_joint"
    spec = importlib.util.spec_from_file_location("head_bringup_validation", BRINGUP / "launch/rt_control.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(
        BRINGUP.parent / name if name != "rt_control_bringup" else BRINGUP))
    context = LaunchContext()
    context.launch_configurations.update({
        "use_mock_hardware": "true", "ethercat_variant": "alfa_v1",
        "canopen_variant": "alfa_v1", "head_can_config": str(_write(tmp_path, config)),
    })
    with pytest.raises(ValueError, match="must match head CAN configuration"):
        module._launch_setup(context)


def test_head_controller_is_loaded_inactive_by_existing_manager(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("head_bringup_mock", BRINGUP / "launch/rt_control.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(
        BRINGUP.parent / name if name != "rt_control_bringup" else BRINGUP))
    nodes = []

    class RecordingNode(Node):
        def __init__(self, **kwargs):
            nodes.append(kwargs)
            super().__init__(**kwargs)

    monkeypatch.setattr(module, "Node", RecordingNode)
    context = LaunchContext()
    context.launch_configurations.update({
        "use_sim_time": "false", "use_mock_hardware": "true",
        "ethercat_variant": "alfa_v1", "canopen_variant": "alfa_v1",
        "head_can_config": str(_write(tmp_path, _config())),
        "start_x503_force_sensor": "false", "start_plc": "false",
        "start_bms": "false",
    })
    module._launch_setup(context)
    spawners = {node["arguments"][0]: node["arguments"]
                for node in nodes if node.get("executable") == "spawner"}
    assert len([node for node in nodes if node.get("executable") == "ros2_control_node"]) == 1
    assert "--inactive" in spawners["damiao_head_controller"]
    assert "--inactive" not in spawners["damiao_joint_state_broadcaster"]


def test_default_launch_has_no_head_configuration():
    from launch.utilities import perform_substitutions

    spec = importlib.util.spec_from_file_location("head_bringup_default", BRINGUP / "launch/rt_control.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    argument = next(entity for entity in module.generate_launch_description().entities
                    if getattr(entity, "name", None) == "head_can_config")
    assert perform_substitutions(LaunchContext(), argument.default_value) == ""
