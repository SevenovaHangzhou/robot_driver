import hashlib
import json
import math
import xml.etree.ElementTree as ET

import pytest
import yaml

from test_v3_description_semantics import PACKAGE_ROOT, render_urdf


def vector(text):
    return tuple(float(value) for value in text.split())


def matrix_multiply(lhs, rhs):
    return tuple(
        tuple(sum(lhs[row][inner] * rhs[inner][column] for inner in range(4)) for column in range(4))
        for row in range(4)
    )


def transform(origin):
    x, y, z = vector(origin.attrib.get("xyz", "0 0 0"))
    roll, pitch, yaw = vector(origin.attrib.get("rpy", "0 0 0"))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y),
        (-sp, cp * sr, cp * cr, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def motion(joint, position):
    result = [list(row) for row in ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
                                    (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))]
    if joint.attrib["type"] == "prismatic":
        axis = vector(joint.find("axis").attrib["xyz"])
        for index in range(3):
            result[index][3] = axis[index] * position
    elif joint.attrib["type"] in ("revolute", "continuous"):
        assert vector(joint.find("axis").attrib["xyz"]) == (0.0, 0.0, 1.0)
        cosine, sine = math.cos(position), math.sin(position)
        result[0][0], result[0][1] = cosine, -sine
        result[1][0], result[1][1] = sine, cosine
    return tuple(tuple(row) for row in result)


def fk(robot, positions):
    joints = list(robot.findall("joint"))
    children = {joint.find("child").attrib["link"] for joint in joints}
    root = next(link.attrib["name"] for link in robot.findall("link") if link.attrib["name"] not in children)
    transforms = {root: ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
                         (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))}
    pending = joints[:]
    while pending:
        ready = [joint for joint in pending if joint.find("parent").attrib["link"] in transforms]
        assert ready
        for joint in ready:
            parent = transforms[joint.find("parent").attrib["link"]]
            child = joint.find("child").attrib["link"]
            transforms[child] = matrix_multiply(
                matrix_multiply(parent, transform(joint.find("origin"))),
                motion(joint, positions.get(joint.attrib["name"], 0.0)),
            )
            pending.remove(joint)
    return transforms


@pytest.mark.parametrize("variant", ("gripper", "suction"))
def test_original_zero_and_named_poses_are_mirrored(variant):
    robot = ET.fromstring(render_urdf(variant))
    reflection = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
    poses = [{}, *(
        {
            joint: value
            for joint, value in yaml.safe_load(
                (PACKAGE_ROOT / f"config/named_poses_{variant}.yaml").read_text()
            )["named_poses"][name].items()
            if joint.startswith(("left_joint", "right_joint"))
        }
        for name in ("home", "second_home", "unloading", "second_unloading")
    )]
    for positions in poses:
        transforms = fk(robot, positions)
        left = transforms["left_tool0"]
        right = transforms["right_tool0"]
        mirror_y = 0.5 * (left[1][3] + right[1][3])
        expected_position = (
            left[0][3],
            2.0 * mirror_y - left[1][3],
            left[2][3],
        )
        for axis in range(3):
            assert abs(right[axis][3] - expected_position[axis]) < 1e-6
        for row in range(3):
            for column in range(3):
                expected = reflection[row][row] * left[row][column] * reflection[column][column]
                assert abs(right[row][column] - expected) < 1e-6


def test_v3_1_1_source_snapshots_are_immutable():
    manifest = json.loads((PACKAGE_ROOT / "config/v3_1_1_integration_manifest.json").read_text())
    for relative_path, expected_hash in manifest["source_files"].items():
        payload = (PACKAGE_ROOT / relative_path).read_bytes()
        assert hashlib.sha256(payload).hexdigest() == expected_hash


@pytest.mark.parametrize("variant", ("gripper", "suction"))
def test_arm_geometry_and_kinematic_landmarks_come_from_v3_1_1(variant):
    robot = ET.fromstring(render_urdf(variant))
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    for side, mesh_numbers in (("left", range(22, 28)), ("right", range(31, 37))):
        for index, mesh_number in enumerate(mesh_numbers, start=1):
            link = robot.find(f"link[@name='{side}_link{index}']")
            expected = f"package://robot_description/meshes/robot_v3_1_1/part_{mesh_number:03d}_solid_{mesh_number:03d}.stl"
            assert link.find("visual/geometry/mesh").attrib["filename"] == expected
            assert link.find("collision/geometry/mesh").attrib["filename"] == expected

    assert joints["left_joint2"].find("origin").attrib["xyz"] == "0.039999999 0 0.1512"
    assert joints["right_joint3"].find("origin").attrib["xyz"] == "-0.1795 0 -0.040000001"
    assert joints["left_joint6"].find("origin").attrib["xyz"] == "0.061499999 0 0.2415"
    assert joints["right_joint7"].find("origin").attrib["xyz"] == "-0.0993 0 -0.0615"
    for side in ("left", "right"):
        mount = joints[f"{side}_arm_mount"]
        assert mount.find("origin").attrib == {
            "xyz": "0 0 -0.0142",
            "rpy": "0 0 1.57079632679",
        }


@pytest.mark.parametrize("variant", ("gripper", "suction"))
def test_updown_rebase_preserves_one_meter_vertical_travel(variant):
    robot = ET.fromstring(render_urdf(variant))
    joint = robot.find("joint[@name='updown']")
    assert float(joint.find("limit").attrib["lower"]) == -1.0
    assert float(joint.find("limit").attrib["upper"]) == 0.0
    upper = fk(robot, {"updown": 0.0})["arm_carriage"]
    lower = fk(robot, {"updown": -1.0})["arm_carriage"]
    delta = tuple(lower[index][3] - upper[index][3] for index in range(3))
    assert abs(delta[0]) < 1e-7
    assert abs(delta[1]) < 1e-7
    assert abs(delta[2] + 1.0) < 1e-7


def test_profile_specific_end_effectors_and_kkozia_modules_remain_selected():
    gripper = ET.fromstring(render_urdf("gripper"))
    suction = ET.fromstring(render_urdf("suction"))
    assert gripper.find("joint[@name='left_moving_jaw_joint']") is not None
    assert suction.find("joint[@name='left_moving_jaw_joint']") is None
    for robot in (gripper, suction):
        assert robot.find("joint[@name='head_pitch_joint']") is not None
        assert robot.find("joint[@name='active_suspension_joint']") is not None
        assert robot.find("joint[@name='left_tool0_fixed']") is not None
