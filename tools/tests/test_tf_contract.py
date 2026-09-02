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
        self.assertEqual(base_joint.find("origin").attrib["xyz"], "0.15 0 0.202094")
        self.assertEqual(base_joint.find("origin").attrib["rpy"], "0 0 0")

        base_parents = [
            joint.find("parent").attrib["link"]
            for joint in joints.values()
            if joint.find("child").attrib["link"] == "base_link"
        ]
        self.assertEqual(base_parents, ["base_footprint"])

    def test_diff_drive_publishes_raw_wheel_odom_without_owning_odom_tf(self):
        config = yaml.safe_load(CONTROLLERS.read_text(encoding="utf-8"))
        params = config["diff_drive_controller"]["ros__parameters"]
        launch_text = BRINGUP_LAUNCH.read_text(encoding="utf-8")

        self.assertEqual(params["odom_frame_id"], "odom")
        self.assertEqual(params["base_frame_id"], "base_footprint")
        self.assertIs(params["enable_odom_tf"], False)
        self.assertEqual(params["publish_rate"], 50.0)
        self.assertIn(
            '("/diff_drive_controller/odom", "/wheel/odom")', launch_text
        )
        self.assertNotIn(
            '("/diff_drive_controller/odom", "/odom")', launch_text
        )

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
        self.assertIn(
            "install -d -m 0750 -o 1000 -g 1000 /home/user",
            dockerfile,
        )

    def test_lidar_main_frame_and_mesh_contract(self):
        robot = ET.parse(ROBOT_XACRO).getroot()
        links = {link.attrib["name"]: link for link in robot.findall("link")}
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

        self.assertIn("lidar_main", links)
        self.assert_fixed_joint(
            joints,
            "lidar_main_joint",
            "base_link",
            "lidar_main",
            "0.382364228640 0.133500000000 0.121820508080",
            "0 0.523598775598 0",
        )

        lidar = links["lidar_main"]
        expected_scale = "0.001 0.001 0.001"
        expected_origin_xyz = "-0.270226881461 -0.133500000000 -0.296681769019"
        expected_origin_rpy = "0 -0.523598775598 0"
        for geometry_name, mesh_path in (
            ("visual", "package://robot_description/meshes/current_robot/visual/lidar_main.STL"),
            ("collision", "package://robot_description/meshes/current_robot/collision/lidar_main.STL"),
        ):
            origin = lidar.find(f"{geometry_name}/origin")
            mesh = lidar.find(f"{geometry_name}/geometry/mesh")
            self.assertIsNotNone(origin)
            self.assertIsNotNone(mesh)
            self.assertEqual(origin.attrib["xyz"], expected_origin_xyz)
            self.assertEqual(origin.attrib["rpy"], expected_origin_rpy)
            self.assertEqual(mesh.attrib["filename"], mesh_path)
            self.assertEqual(mesh.attrib["scale"], expected_scale)
            asset_path = (
                ROOT
                / "src/description/robot_description"
                / mesh_path.split("package://robot_description/", 1)[1]
            )
            self.assertTrue(asset_path.is_file())
            asset = asset_path.read_bytes()
            self.assertIn(b"frame=base_link", asset[:80])
            self.assertIn(b"unit=millimeter", asset[:80])
            triangle_count = int.from_bytes(asset[80:84], "little")
            self.assertEqual(len(asset), 84 + 50 * triangle_count)

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
