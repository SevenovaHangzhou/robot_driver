"""Operation admission and conversions; no devices or ROS nodes are created."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from rt_control_composer.chassis_operation import configure_operation

RT = Path(__file__).resolve().parents[2]


def inputs():
    params = yaml.safe_load((RT / "swerve_driver/config/controller.draft.yaml").read_text())[
        "swerve_controller"
    ]["ros__parameters"]
    params.update(
        calibration_verified=True,
        wheel_radius=[0.1, 0.2, 0.1, 0.2],
        drive_joints=["d0", "d1", "d2", "d3"],
        steering_joints=["s0", "s1", "s2", "s3"],
    )
    robot = ET.Element("robot")
    control = ET.SubElement(robot, "ros2_control")
    for name in params["drive_joints"]:
        joint = ET.SubElement(control, "joint", name=name)
        for interface in (
            "position",
            "velocity",
            "mode_of_operation",
            "write_sequence",
            "write_mask",
        ):
            ET.SubElement(joint, "command_interface", name=interface)
        for interface in (
            "mode_switch_ready",
            "mode_request_ack",
            "feedback_sequence",
            "sent_sequence",
            "feedback_sequence_at_send",
            "sent_velocity",
        ):
            ET.SubElement(joint, "state_interface", name=interface)
    deployment = {
        "chassis_operation": {
            "enabled": True,
            "controller": "swerve_controller",
            "parameters": {
                "package": "swerve_driver",
                "file": "config/relative.commissioning.yaml",
            },
        },
        "chassis_imu": {"driver": "lpms_nav3_can", "frame_id": "imu_link"},
    }
    return {"swerve_controller": {"ros__parameters": params}}, deployment, robot


def apply(controllers, deployment, robot):
    configure_operation(controllers, deployment, robot, lambda name: RT / name, mock=False)


def test_nonuniform_radii_preserve_rim_limits_and_common_stationary_bound():
    controllers, deployment, robot = inputs()
    apply(controllers, deployment, robot)
    params = controllers["swerve_controller"]["ros__parameters"]
    relative = params["relative"]
    assert relative["max_wheel_velocity"] == pytest.approx([20, 10, 20, 10])
    for radius in params["wheel_radius"]:
        assert relative["stationary_wheel_velocity"] * radius <= params["fault_stop_speed"]


@pytest.mark.parametrize("failure", ["calibration", "imu", "switching", "stop_deadline"])
def test_missing_execution_prerequisites_are_rejected(failure):
    controllers, deployment, robot = inputs()
    params = controllers["swerve_controller"]["ros__parameters"]
    if failure == "calibration":
        params["calibration_verified"] = False
    elif failure == "imu":
        deployment.pop("chassis_imu")
    elif failure == "switching":
        joint = robot.find(".//joint")
        joint.remove(joint.find("state_interface[@name='sent_sequence']"))
    elif failure == "stop_deadline":
        params["max_wheel_acceleration"] = 0.01
    with pytest.raises(ValueError):
        apply(controllers, deployment, robot)
    assert "relative" not in params


def test_tight_navigation_speed_limits_cannot_be_relaxed_by_operation_template():
    controllers, deployment, robot = inputs()
    params = controllers["swerve_controller"]["ros__parameters"]
    params.update(max_linear_speed=0.02, max_angular_speed=0.03, max_body_acceleration=0.04)
    apply(controllers, deployment, robot)
    assert params["relative"]["max_translation_velocity"] == 0.02
    assert params["relative"]["max_yaw_velocity"] == 0.03
    assert params["relative"]["max_translation_acceleration"] == 0.04
