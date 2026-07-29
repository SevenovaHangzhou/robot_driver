#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
ROBOT_XACRO = ROOT / "src/description/robot_description/urdf/robot.urdf.xacro"
CONTROLLERS = ROOT / "src/rt_control/rt_control_bringup/config/controllers.yaml"
BRINGUP_LAUNCH = ROOT / "src/rt_control/rt_control_bringup/launch/rt_control.launch.py"
BRINGUP_MANIFEST = ROOT / "src/rt_control/rt_control_bringup/package.xml"
DOCKERFILE = ROOT / "docker/rt-control/Dockerfile"


class TfContractTest(unittest.TestCase):
    def assert_fixed_joint(self, joints, name, parent, child, xyz, rpy):
        joint = joints[name]
        self.assertEqual(joint.attrib["type"], "fixed")
        self.assertEqual(joint.find("parent").attrib["link"], parent)
        self.assertEqual(joint.find("child").attrib["link"], child)
        self.assertEqual(joint.find("origin").attrib["xyz"], xyz)
        self.assertEqual(joint.find("origin").attrib["rpy"], rpy)

    def test_robot_model_roots_at_base_footprint(self):
        robot = ET.parse(ROBOT_XACRO).getroot()
        link_names = {link.attrib["name"] for link in robot.findall("link")}
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

        self.assertIn("base_footprint", link_names)
        self.assertNotIn("world", link_names)
        self.assertNotIn("world_to_base", joints)

        base_joint = joints["base_footprint_to_base"]
        self.assertEqual(base_joint.attrib["type"], "fixed")
        self.assertEqual(base_joint.find("parent").attrib["link"], "base_footprint")
        self.assertEqual(base_joint.find("child").attrib["link"], "base_link")
        self.assertEqual(base_joint.find("origin").attrib["xyz"], "0 0 0.202094")
        self.assertEqual(base_joint.find("origin").attrib["rpy"], "0 0 0")

        base_parents = [
            joint.find("parent").attrib["link"]
            for joint in joints.values()
            if joint.find("child").attrib["link"] == "base_link"
        ]
        self.assertEqual(base_parents, ["base_footprint"])

    def test_diff_drive_publishes_odom_to_base_footprint_at_50_hz(self):
        config = yaml.safe_load(CONTROLLERS.read_text(encoding="utf-8"))
        params = config["diff_drive_controller"]["ros__parameters"]

        self.assertEqual(params["odom_frame_id"], "odom")
        self.assertEqual(params["base_frame_id"], "base_footprint")
        self.assertIs(params["enable_odom_tf"], True)
        self.assertEqual(params["publish_rate"], 50.0)

    def test_bringup_owns_rsp_and_image_asserts_it_is_installed(self):
        launch_text = BRINGUP_LAUNCH.read_text(encoding="utf-8")
        self.assertIn('package="robot_state_publisher"', launch_text)
        self.assertIn('executable="robot_state_publisher"', launch_text)
        self.assertIn("parameters=[robot_description", launch_text)
        self.assertIn('"publish_frequency": 50.0', launch_text)

        manifest = ET.parse(BRINGUP_MANIFEST).getroot()
        exec_dependencies = {dep.text for dep in manifest.findall("exec_depend")}
        self.assertIn("robot_state_publisher", exec_dependencies)

        dockerfile = DOCKERFILE.read_text(encoding="utf-8")
        self.assertIn(
            "test -x /opt/ros/humble/lib/robot_state_publisher/robot_state_publisher",
            dockerfile,
        )

    def test_perception_sensor_frames_match_the_onsite_reference(self):
        robot = ET.parse(ROBOT_XACRO).getroot()
        link_names = {link.attrib["name"] for link in robot.findall("link")}
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

        self.assertTrue(
            {
                "lidar_front_mount",
                "lidar_front",
                "livox_fused_frame",
                "top_sensor",
                "lidar_rear_mount",
                "lidar_rear",
                "camera_top_color_optical_frame",
            }.issubset(link_names)
        )

        self.assert_fixed_joint(
            joints,
            "lidar_front_mount_joint",
            "turn",
            "lidar_front_mount",
            "0.16084 0 1.2155",
            "0 0 0",
        )
        self.assert_fixed_joint(
            joints,
            "lidar_front_frame_joint",
            "lidar_front_mount",
            "lidar_front",
            "0 0 0",
            "3.141592653589793 0 -1.570796326794897",
        )
        self.assert_fixed_joint(
            joints,
            "lidar_front_to_livox_fused_frame",
            "lidar_front",
            "livox_fused_frame",
            "0 0.175 0",
            "3.141592653589793 0 0",
        )
        self.assert_fixed_joint(
            joints,
            "lidar_front_to_top_sensor",
            "lidar_front",
            "top_sensor",
            "0 0 0",
            "3.141592653589793 0 -1.570796326794897",
        )
        self.assert_fixed_joint(
            joints,
            "lidar_rear_mount_joint",
            "turn",
            "lidar_rear_mount",
            "-0.19316 0 1.2155",
            "0 0 0",
        )
        self.assert_fixed_joint(
            joints,
            "lidar_rear_frame_joint",
            "lidar_rear_mount",
            "lidar_rear",
            "0 0 0",
            "3.141592653589793 0 -1.570796326794897",
        )
        self.assert_fixed_joint(
            joints,
            "cam_front",
            "turn",
            "cam_front",
            "0.23667 0 1.2725",
            "0 0.7854 0",
        )
        self.assert_fixed_joint(
            joints,
            "cam_front_to_camera_top_color_optical_frame",
            "cam_front",
            "camera_top_color_optical_frame",
            "0 0 0",
            "-1.57079632679 0 -1.57079632679",
        )
        self.assert_fixed_joint(
            joints,
            "cam_rear",
            "turn",
            "cam_rear",
            "-0.25649 0 1.2854",
            "0 0.2618 3.1416",
        )

    def test_sensor_frame_overlay_preserves_control_joint_contract(self):
        robot = ET.parse(ROBOT_XACRO).getroot()
        links = {link.attrib["name"] for link in robot.findall("link")}
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

        self.assertIn("left_joint1", links)
        self.assertIn("right_joint1", links)
        self.assertNotIn("leftjoint1", links)
        self.assertNotIn("rightjoint1", links)
        self.assertEqual(joints["updown"].find("limit").attrib["upper"], "0.8")
        self.assertEqual(joints["updown"].find("limit").attrib["velocity"], "0.3")


if __name__ == "__main__":
    unittest.main()
