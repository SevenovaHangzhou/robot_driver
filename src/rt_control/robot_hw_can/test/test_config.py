from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]


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
        'joint_1_name="head_joint" joint_2_name="head_pitch_joint" '
        'joint_1_min="-0.1" joint_1_max="0.1" '
        'joint_2_min="-0.2" joint_2_max="0.2" '
        'joint_1_velocity_limit="0.1" joint_2_velocity_limit="0.2" '
        'joint_1_acceleration_krad_s2="0.025" '
        'joint_1_deceleration_krad_s2="-0.05" joint_1_maximum_speed_rad_s="20" '
        'joint_2_acceleration_krad_s2="0.02" '
        'joint_2_deceleration_krad_s2="-0.04" joint_2_maximum_speed_rad_s="18" '
        'configure_timeout_ms="100" feedback_timeout_ms="100" '
        'transition_timeout_ms="250" command_rate_hz="250" '
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
        control = root.find("./ros2_control/gpio[@name='damiao_head_control']")
        assert control is not None
        assert {item.attrib["name"] for item in control.findall("command_interface")} == {
            "enable_request", "reset_generation"}
        if mock == "false":
            assert root.find("./ros2_control/hardware/param[@name='can_interface']").text == "can2"
