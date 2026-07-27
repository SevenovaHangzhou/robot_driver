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


if __name__ == "__main__":
    unittest.main()
