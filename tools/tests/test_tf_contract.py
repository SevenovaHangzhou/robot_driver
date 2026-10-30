#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / "src/description/robot_description"
MOBILE_URDF = DESCRIPTION / "urdf/robot_alfav3_mobile.urdf"
SOURCE_LOCK = ROOT / "src/description/source-lock.yaml"
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
        self.assertEqual(source["source_branch"], "main")
        self.assertEqual(
            source["source_revision"],
            "12a83dd0a008b2e1046b86053cd3a47592307acb",
        )
        self.assertEqual(
            source["source_tree"],
            "adc951282e0c6746ca21d55014d649481f3a000d",
        )
        self.assertEqual(source["model_family"], "alfa_v3")
        self.assertEqual(source["default_end_effector"], "mobile_proxy")

    def test_v3_robot_model_roots_at_base_footprint(self):
        robot = ET.parse(MOBILE_URDF).getroot()
        links = {link.attrib["name"] for link in robot.findall("link")}
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

        self.assertEqual(robot.attrib["name"], "alfa_robot_v3")
        self.assertIn("base_footprint", links)
        self.assertNotIn("world", links)
        base_joint = joints["base_footprint_to_base_link"]
        self.assertEqual(base_joint.attrib["type"], "fixed")
        self.assertEqual(base_joint.find("parent").attrib["link"], "base_footprint")
        self.assertEqual(base_joint.find("child").attrib["link"], "base_link")
        # Deployed chassis frame: new forward +X is the old base_link -Y.
        self.assertEqual(base_joint.find("origin").attrib["xyz"], "0 0 0.32")
        self.assertEqual(base_joint.find("origin").attrib["rpy"], "0 0 1.5707963267948966")

    def test_v3_control_joint_names_are_present_in_the_model_sources(self):
        robot = ET.parse(MOBILE_URDF).getroot()
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
        expected = {
            "updown",
            "H1_joint",
            "H2_joint",
            *(f"right_joint{index}" for index in range(1, 8)),
            *(f"left_joint{index}" for index in range(1, 8)),
            *(f"steer{index}_joint" for index in range(1, 5)),
            *(f"wheel{index}_joint" for index in range(1, 5)),
        }
        moving = {name for name, joint in joints.items() if joint.attrib["type"] != "fixed"}
        self.assertEqual(moving, expected)
        self.assertEqual(joints["updown"].attrib["type"], "prismatic")
        for index in range(1, 5):
            self.assertEqual(joints[f"wheel{index}_joint"].attrib["type"], "continuous")
            limit = joints[f"steer{index}_joint"].find("limit").attrib
            self.assertEqual(limit["lower"], "-2.356194490192345")
            self.assertEqual(limit["upper"], "2.356194490192345")

    def test_v3_model_assets_and_variants_are_installed_by_the_package(self):
        for relative_path in (
            "urdf/robot_alfav3_mobile.urdf",
            "srdf/robot_alfav3_mobile.srdf",
            "config/joint_limits_alfav3_mobile.yaml",
        ):
            self.assertTrue((DESCRIPTION / relative_path).is_file(), relative_path)
        robot = ET.parse(MOBILE_URDF).getroot()
        for mesh in robot.iter("mesh"):
            path = mesh.attrib["filename"].replace("package://robot_description/", "")
            self.assertTrue((DESCRIPTION / path).is_file(), path)

        cmake = (DESCRIPTION / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("install(DIRECTORY urdf srdf config mesh DESTINATION", cmake)

    def test_image_and_bringup_require_robot_state_publisher(self):
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
