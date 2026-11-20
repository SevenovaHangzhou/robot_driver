#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path



ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / "src/description/robot_description"
MOBILE_URDF = DESCRIPTION / "urdf/robot_alfav3_mobile.urdf"


class TfContractTest(unittest.TestCase):

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
        # Model main 98d498e: base_link sits 0.335 m above base_footprint with no yaw.
        # The previous model used z=0.32 with a 90 deg yaw, so this is a deliberate
        # frame change that consumers must be checked against.
        self.assertEqual(base_joint.find("origin").attrib["xyz"], "0 0 0.335")
        self.assertEqual(base_joint.find("origin").attrib["rpy"], "0 0 0")

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


if __name__ == "__main__":
    unittest.main()
