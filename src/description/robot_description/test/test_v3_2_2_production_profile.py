import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf" / "robot.urdf"
SOURCE_PATH = PACKAGE_ROOT / "model_sources" / "robot_v3_2_2" / "robot.urdf"
MOVABLE = {
    "updown", "head_joint", "head_pitch_joint",
    *(f"{side}_joint{index}" for side in ("left", "right") for index in range(1, 8)),
}


def values(text):
    return np.asarray([float(value) for value in text.split()], dtype=float)


def rpy_matrix(text):
    roll, pitch, yaw = values(text)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    return rz @ ry @ rx


def origin(element):
    result = np.eye(4)
    result[:3, :3] = rpy_matrix(element.attrib.get("rpy", "0 0 0"))
    result[:3, 3] = values(element.attrib.get("xyz", "0 0 0"))
    return result


def axis_angle(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, d = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array((
        (c + x*x*d, x*y*d - z*s, x*z*d + y*s),
        (y*x*d + z*s, c + y*y*d, y*z*d - x*s),
        (z*x*d - y*s, z*y*d + x*s, c + z*z*d),
    ))


def fk(robot, positions):
    joints = robot.findall("joint")
    links = {link.attrib["name"] for link in robot.findall("link")}
    children = {joint.find("child").attrib["link"] for joint in joints}
    root = next(iter(links - children))
    transforms = {root: np.eye(4)}
    pending = joints[:]
    while pending:
        ready = [joint for joint in pending if joint.find("parent").attrib["link"] in transforms]
        assert ready
        for joint in ready:
            motion = np.eye(4)
            value = positions.get(joint.attrib["name"], 0.0)
            axis_element = joint.find("axis")
            axis = values(axis_element.attrib["xyz"]) if axis_element is not None else np.array([1., 0., 0.])
            if joint.attrib["type"] in ("revolute", "continuous"):
                motion[:3, :3] = axis_angle(axis, value)
            elif joint.attrib["type"] == "prismatic":
                motion[:3, 3] = axis * value
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            transforms[child] = transforms[parent] @ origin(joint.find("origin")) @ motion
            pending.remove(joint)
    return transforms


def test_production_tree_and_joint_inventory():
    robot = ET.parse(URDF_PATH).getroot()
    joints = robot.findall("joint")
    links = {link.attrib["name"] for link in robot.findall("link")}
    children = {joint.find("child").attrib["link"] for joint in joints}
    assert robot.attrib["name"] == "alfa_robot_v3"
    assert links - children == {"base_footprint"}
    assert {joint.attrib["name"] for joint in joints if joint.attrib["type"] != "fixed"} == MOVABLE
    assert not any(name.startswith(("caster", "wheel", "active_suspension")) for name in links | MOVABLE)
    assert {"left_tool0", "right_tool0"} <= links
    assert not {"left_suction", "right_suction"} & links
    for side in ("left", "right"):
        tool_joint = robot.find(f"joint[@name='{side}_tool0_fixed']")
        assert tool_joint.find("parent").attrib["link"] == f"{side}_link7"
        assert tool_joint.find("origin").attrib == {"xyz": "0 0 0.151", "rpy": "0 0 0"}


def test_v311_limits_and_configuration_cover_exactly_the_model():
    robot = ET.parse(URDF_PATH).getroot()
    limits = yaml.safe_load((PACKAGE_ROOT / "config" / "joint_limits.yaml").read_text())["joints"]
    initial = yaml.safe_load((PACKAGE_ROOT / "config" / "initial_positions.yaml").read_text())["initial_positions"]
    assert set(limits) == MOVABLE
    assert set(initial) == MOVABLE
    expected = {
        1: (-3.14159265, 3.14159265), 2: (-1.8325957145940461, 1.8325957145940461),
        3: (-3.14159265, 3.14159265), 4: (-2.53072742, 2.53072742),
        5: (-3.14159265, 3.14159265), 6: (-1.91986218, 1.91986218),
        7: (-3.14159265, 3.14159265),
    }
    for side in ("left", "right"):
        for index, bounds in expected.items():
            limit = robot.find(f"joint[@name='{side}_joint{index}']/limit")
            assert (float(limit.attrib["lower"]), float(limit.attrib["upper"])) == bounds
    updown = robot.find("joint[@name='updown']")
    assert updown.find("axis").attrib["xyz"] == "0 0 -1"
    assert (float(updown.find("limit").attrib["lower"]), float(updown.find("limit").attrib["upper"])) == (-1., 0.)


def test_named_poses_are_valid_and_preserve_the_confirmed_source_geometry():
    production = ET.parse(URDF_PATH).getroot()
    source = ET.parse(SOURCE_PATH).getroot()
    poses = yaml.safe_load((PACKAGE_ROOT / "config" / "named_poses.yaml").read_text())["named_poses"]
    link_map = {
        "base_footprint": "base_footprint", "base_link": "base_link",
        "arm_carriage": "up and down", "head_yaw": "yaw", "head": "pitch",
        **{f"{side}_link{index}": f"{side}_link{index}" for side in ("left", "right") for index in range(1, 8)},
    }
    for pose in poses.values():
        raw = {
            "up and down_joint": -pose["updown"],
            "yaw_joint": pose["head_joint"],
            "pitch_joint": pose["head_pitch_joint"],
        }
        for side in ("left", "right"):
            sign = 1 if side == "left" else -1
            for index in range(1, 8):
                raw[f"{side}_link{index}_joint"] = pose[f"{side}_joint{index}"]
            raw[f"{side}_link1_joint"] -= sign * math.pi / 2
            raw[f"{side}_link5_joint"] += sign * math.pi / 2
        production_fk = fk(production, pose)
        source_fk = fk(source, raw)
        for production_link, source_link in link_map.items():
            assert np.max(np.abs(production_fk[production_link] - source_fk[source_link])) < 2e-8


def test_updated_unloading_poses_match_reviewed_values():
    poses = yaml.safe_load((PACKAGE_ROOT / "config" / "named_poses.yaml").read_text())["named_poses"]
    expected = {
        "unloading": ([140, -100, -180, 20, -90, -20, 0], [-140, 100, 180, -20, 90, 20, 0], -0.3),
        "second_unloading": ([0, 100, 0, -30, 100, 30, 0], [0, -100, 0, 30, -100, -30, 0], -0.6),
    }
    for name, (left, right, updown) in expected.items():
        pose = poses[name]
        assert pose["updown"] == updown
        for side, values_deg in (("left", left), ("right", right)):
            actual = [pose[f"{side}_joint{index}"] for index in range(1, 8)]
            assert np.allclose(actual, np.radians(values_deg), atol=1e-12)


def test_srdf_references_only_existing_links_and_joints():
    robot = ET.parse(URDF_PATH).getroot()
    links = {link.attrib["name"] for link in robot.findall("link")}
    joints = {joint.attrib["name"] for joint in robot.findall("joint")}
    srdf = ET.parse(PACKAGE_ROOT / "srdf" / "robot.srdf").getroot()
    assert {group.attrib["name"] for group in srdf.findall("group")} == {
        "left_arm", "right_arm", "dual_arm", "dual_arm_with_updown", "whole"
    }
    for chain in srdf.findall(".//chain"):
        assert chain.attrib["base_link"] in links
        assert chain.attrib["tip_link"] in links
    for joint in srdf.findall(".//joint"):
        assert joint.attrib["name"] in joints

    disabled = {
        frozenset((item.attrib["link1"], item.attrib["link2"]))
        for item in srdf.findall("disable_collisions")
    }
    expected_adjacent = {
        frozenset((joint.find("parent").attrib["link"], joint.find("child").attrib["link"]))
        for joint in robot.findall("joint")
        if joint.find("parent").attrib["link"] in links
        and joint.find("child").attrib["link"] in links
        and joint.find("parent").attrib["link"] != "base_footprint"
        and joint.find("child").attrib["link"] not in {"left_tool0", "right_tool0"}
    }
    assert expected_adjacent <= disabled

    limits = yaml.safe_load((PACKAGE_ROOT / "config" / "joint_limits.yaml").read_text())["joints"]
    poses = yaml.safe_load((PACKAGE_ROOT / "config" / "named_poses.yaml").read_text())["named_poses"]
    for pose in poses.values():
        for name, value in pose.items():
            assert limits[name]["lower_position_rad" if limits[name]["type"] == "revolute" else "lower_position_m"] <= value
            assert value <= limits[name]["upper_position_rad" if limits[name]["type"] == "revolute" else "upper_position_m"]


def test_default_xacro_and_direct_urdf_are_valid():
    rendered = subprocess.run(
        ["xacro", str(PACKAGE_ROOT / "urdf" / "robot.urdf.xacro"), "end_effector:=suction"],
        capture_output=True, text=True, check=True,
    ).stdout
    rendered_robot = ET.fromstring(rendered)
    direct_robot = ET.parse(URDF_PATH).getroot()
    assert {link.attrib["name"] for link in rendered_robot.findall("link")} == {
        link.attrib["name"] for link in direct_robot.findall("link")
    }
    with subprocess.Popen(["check_urdf", str(URDF_PATH)], stdout=subprocess.PIPE, text=True) as process:
        stdout, _ = process.communicate()
    assert process.returncode == 0
    assert "root Link: base_footprint" in stdout
