#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / "src/description/robot_description"
ROBOT_XACRO = DESCRIPTION / "urdf/robot.urdf.xacro"
ARM_XACRO = DESCRIPTION / "urdf/robot_v3_1_1.xacro"
CHASSIS_XACRO = DESCRIPTION / "urdf/robot_v3_chassis.xacro"
SOURCE_LOCK = ROOT / "src/description/source-lock.yaml"
BRINGUP_LAUNCH = ROOT / "src/rt_control/rt_control_bringup/launch/rt_control.launch.py"
BRINGUP_MANIFEST = ROOT / "src/rt_control/rt_control_bringup/package.xml"
DOCKERFILE = ROOT / "docker/rt-control/Dockerfile"


class TfContractTest(unittest.TestCase):
    def test_v3_description_source_is_identity_locked(self):
        source = yaml.safe_load(SOURCE_LOCK.read_text(encoding="utf-8"))
        self.assertEqual(source["schema_version"], 1)
        self.assertEqual(source["package"], "robot_description")
        self.assertEqual(
            source["repository"],
            "https://github.com/SevenovaHangzhou/robot_description.git",
        )
        self.assertEqual(source["source_branch"], "feature/electri-136-inertial-import")
        self.assertEqual(
            source["source_revision"],
            "11f6d906dcb4cd5abba7cd4693afc9bb34c6a81e",
        )
        self.assertEqual(
            source["source_tree"],
            "ff4bddfc25e56b88b85b124c7825038664922021",
        )
        self.assertEqual(source["model_family"], "alfa_v3")
        self.assertEqual(source["default_end_effector"], "suction")

    def test_v3_robot_model_roots_at_base_footprint(self):
        robot = ET.parse(ROBOT_XACRO).getroot()
        links = {link.attrib["name"] for link in robot.findall("link")}
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

        self.assertEqual(robot.attrib["name"], "alfa_robot_v3")
        self.assertIn("base_footprint", links)
        self.assertNotIn("world", links)
        base_joint = joints["base_footprint_to_base_link"]
        self.assertEqual(base_joint.attrib["type"], "fixed")
        self.assertEqual(base_joint.find("parent").attrib["link"], "base_footprint")
        self.assertEqual(base_joint.find("child").attrib["link"], "base_link")
        self.assertEqual(
            base_joint.find("origin").attrib["xyz"],
            "0.190000002779484 -0.0000442724271391554 0.40000250599116",
        )

    def test_v3_control_joint_names_are_present_in_the_model_sources(self):
        arms = ET.parse(ARM_XACRO).getroot()
        joints = {joint.attrib["name"]: joint for joint in arms.findall("joint")}
        expected_arms = {
            *(f"right_joint{index}" for index in range(1, 8)),
            *(f"left_joint{index}" for index in range(1, 8)),
        }
        self.assertTrue(expected_arms.issubset(joints))
        self.assertEqual(joints["updown"].attrib["type"], "prismatic")
        self.assertEqual(joints["updown"].find("limit").attrib["lower"], "-1")
        self.assertEqual(joints["updown"].find("limit").attrib["upper"], "0")

        chassis = ET.parse(CHASSIS_XACRO).getroot()
        chassis_joints = {
            joint.attrib["name"]: joint for joint in chassis.findall("joint")
        }
        suspension = chassis_joints["active_suspension_joint"]
        self.assertEqual(suspension.attrib["type"], "prismatic")
        self.assertEqual(suspension.find("parent").attrib["link"], "chassis_base")
        self.assertEqual(
            suspension.find("child").attrib["link"],
            "active_suspension_carriage",
        )
        self.assertEqual(suspension.find("axis").attrib["xyz"], "0 0 1")
        self.assertEqual(suspension.find("limit").attrib["lower"], "-0.15")
        self.assertEqual(suspension.find("limit").attrib["upper"], "0")

    def test_v3_model_assets_and_variants_are_installed_by_the_package(self):
        for relative_path in (
            "urdf/robot_dual_suction.urdf.xacro",
            "urdf/robot_dual_gripper.urdf.xacro",
            "srdf/robot.srdf",
            "config/joint_limits_suction.yaml",
            "config/joint_limits_gripper.yaml",
            "meshes/active_suspension/active_suspension_carriage.stl",
            "meshes/chassis/part_001_solid_001.stl",
            "meshes/head/part_001_part.stl",
            "meshes/robot_v3/suction_visual.stl",
        ):
            self.assertTrue((DESCRIPTION / relative_path).is_file(), relative_path)

        cmake = (DESCRIPTION / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn(
            "install(DIRECTORY urdf srdf meshes config DESTINATION share/${PROJECT_NAME})",
            cmake,
        )

    def test_bringup_owns_rsp_and_image_asserts_it_is_installed(self):
        launch_text = BRINGUP_LAUNCH.read_text(encoding="utf-8")
        self.assertIn('package="robot_state_publisher"', launch_text)
        self.assertIn('executable="robot_state_publisher"', launch_text)
        self.assertIn("parameters=[robot_description", launch_text)

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
