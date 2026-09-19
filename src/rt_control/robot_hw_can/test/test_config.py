from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_example_controller_matches_xacro_joint_names():
    control = yaml.safe_load((PACKAGE / "config/controllers.example.yaml").read_text())
    broadcaster = control["damiao_joint_state_broadcaster"]["ros__parameters"]
    trajectory = control["damiao_head_controller"]["ros__parameters"]
    assert broadcaster["joints"] == trajectory["joints"] == [
        "head_motor_1_joint", "head_motor_2_joint"]
    assert trajectory["command_interfaces"] == ["position"]
    assert trajectory["state_interfaces"] == ["position", "velocity"]


def test_plugin_manifest_has_native_can_only():
    plugin = ET.parse(PACKAGE / "robot_hw_can_plugins.xml").getroot()
    assert plugin.find("class").attrib["name"] == "robot_hw_can/DamiaoSystem"
    package = ET.parse(PACKAGE / "package.xml").getroot()
    deps = {element.text for element in package if element.tag.endswith("depend")}
    assert "canopen_ros2_control" not in deps
    assert "robot_hw_canopen" not in deps


def test_xacro_expands_real_and_mock_systems(tmp_path):
    xacro_path = PACKAGE / "urdf/damiao.ros2_control.xacro"
    source = tmp_path / "head.xacro"
    source.write_text(
        '<robot name="head" xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:arg name="mock" default="false"/>'
        f'<xacro:include filename="{xacro_path}"/>'
        '<xacro:damiao_head_system '
        'joint_1_name="head_motor_1_joint" joint_2_name="head_motor_2_joint" '
        'joint_1_min="-0.1" joint_1_max="0.1" '
        'joint_2_min="-0.2" joint_2_max="0.2" '
        'joint_1_velocity_limit="0.1" joint_2_velocity_limit="0.2" '
        'configure_timeout_ms="100" feedback_timeout_ms="100" '
        'disabled_poll_interval_ms="50" max_rx_frames_per_cycle="32" '
        'use_mock_hardware="$(arg mock)"/>'
        '</robot>'
    )
    for mock, expected_plugin in (
        ("false", "robot_hw_can/DamiaoSystem"),
        ("true", "mock_components/GenericSystem"),
    ):
        output = subprocess.run(
            ["xacro", str(source), f"mock:={mock}"],
            check=True, capture_output=True, text=True,
        )
        root = ET.fromstring(output.stdout)
        assert root.find("./ros2_control/hardware/plugin").text == expected_plugin
        joints = root.findall("./ros2_control/joint")
        assert len(joints) == 2
        assert [joint.find("./param[@name='can_id']").text for joint in joints] == ["1", "2"]
        assert [joint.find("./param[@name='master_id']").text for joint in joints] == ["17", "18"]
        assert all(joint.find("./command_interface[@name='position']") is not None
                   for joint in joints)
        if mock == "false":
            assert root.find("./ros2_control/hardware/param[@name='can_interface']").text == "can2"
