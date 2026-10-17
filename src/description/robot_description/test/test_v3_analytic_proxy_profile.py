import copy
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PRODUCTION_PATH = PACKAGE_ROOT / "urdf" / "robot.urdf.xacro"
PROXY_PATH = PACKAGE_ROOT / "urdf" / "robot_analytic_proxy.urdf.xacro"
POSES_PATH = PACKAGE_ROOT / "config" / "named_poses_analytic_proxy.yaml"
OVERRIDES = {
    "left_joint3": "0.23761775 0.063669485 -0.04949",
    "left_joint4": "0 0.085000001 0.26",
    "left_joint5": "0 -0.241 -0.085",
    "left_joint7": "0 0.0992 -0.0905",
    "right_joint3": "-0.23761775 0.063669485 -0.065",
    "right_joint4": "0 -0.085000001 0.26",
    "right_joint5": "0 0.241 -0.085",
    "right_joint7": "0 -0.0992 -0.0905",
}


def values(text):
    return np.asarray([float(value) for value in text.split()], dtype=float)


def rpy_matrix(text):
    roll, pitch, yaw = values(text)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1))) @ np.array(
        ((cp, 0, sp), (0, 1, 0), (-sp, 0, cp))) @ np.array(
        ((1, 0, 0), (0, cr, -sr), (0, sr, cr)))


def origin(element):
    transform = np.eye(4)
    transform[:3, :3] = rpy_matrix(element.attrib.get("rpy", "0 0 0"))
    transform[:3, 3] = values(element.attrib.get("xyz", "0 0 0"))
    return transform


def without_origin(joint):
    clone = copy.deepcopy(joint)
    clone.remove(clone.find("origin"))
    return ET.tostring(clone)


def zero_joint_axes(robot):
    joints = robot.findall("joint")
    links = {link.attrib["name"] for link in robot.findall("link")}
    children = {joint.find("child").attrib["link"] for joint in joints}
    transforms = {next(iter(links - children)): np.eye(4)}
    axes = {}
    pending = joints[:]
    while pending:
        ready = [joint for joint in pending if joint.find("parent").attrib["link"] in transforms]
        assert ready
        for joint in ready:
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            frame = transforms[parent] @ origin(joint.find("origin"))
            transforms[child] = frame
            axis_element = joint.find("axis")
            if axis_element is not None:
                axis = frame[:3, :3] @ values(axis_element.attrib["xyz"])
                axes[joint.attrib["name"]] = (frame[:3, 3], axis / np.linalg.norm(axis))
            pending.remove(joint)
    return axes


def closest_midpoint(first, second):
    first_point, first_axis = first
    second_point, second_axis = second
    parameters = np.linalg.lstsq(
        np.column_stack((first_axis, -second_axis)),
        second_point - first_point,
        rcond=None,
    )[0]
    return 0.5 * (
        first_point + parameters[0] * first_axis +
        second_point + parameters[1] * second_axis
    )


def test_proxy_changes_only_reviewed_joint_origins():
    production = ET.parse(PRODUCTION_PATH).getroot()
    proxy = ET.parse(PROXY_PATH).getroot()
    production_links = {link.attrib["name"]: ET.tostring(link) for link in production.findall("link")}
    proxy_links = {link.attrib["name"]: ET.tostring(link) for link in proxy.findall("link")}
    assert production_links == proxy_links

    production_joints = {joint.attrib["name"]: joint for joint in production.findall("joint")}
    proxy_joints = {joint.attrib["name"]: joint for joint in proxy.findall("joint")}
    assert production_joints.keys() == proxy_joints.keys()
    for name, production_joint in production_joints.items():
        proxy_joint = proxy_joints[name]
        assert without_origin(production_joint) == without_origin(proxy_joint)
        if name in OVERRIDES:
            assert proxy_joint.find("origin").attrib["xyz"] == OVERRIDES[name]
            assert proxy_joint.find("origin").attrib["rpy"] == production_joint.find("origin").attrib["rpy"]
        else:
            assert proxy_joint.find("origin").attrib == production_joint.find("origin").attrib


def test_proxy_restores_intersecting_axis_groups():
    axes = zero_joint_axes(ET.parse(PROXY_PATH).getroot())
    for side in ("left", "right"):
        for indices in ((1, 2, 3), (3, 4, 5), (5, 6, 7)):
            first = closest_midpoint(
                axes[f"{side}_joint{indices[0]}"], axes[f"{side}_joint{indices[1]}"])
            second = closest_midpoint(
                axes[f"{side}_joint{indices[1]}"], axes[f"{side}_joint{indices[2]}"])
            assert np.linalg.norm(first - second) < 1e-7


def test_proxy_named_poses_and_third_unloading_contract():
    document = yaml.safe_load(POSES_PATH.read_text())
    assert document["metadata"]["model_revision"] == "robot_v3.2.2-analytic-proxy"
    assert document["metadata"]["experimental"] is True
    pose = document["named_poses"]["third_unloading"]
    assert set(pose) == {
        "updown",
        *(f"{side}_joint{index}" for side in ("left", "right") for index in range(1, 8)),
    }
    assert pose["updown"] == 0.0
    assert np.allclose(
        [pose[f"left_joint{index}"] for index in range(1, 8)],
        np.radians([130, -105, 10, 90, -90, -40, 0]),
        atol=1e-12,
    )
    assert np.allclose(
        [pose[f"right_joint{index}"] for index in range(1, 8)],
        np.radians([-130, 105, -10, -90, 90, 40, 0]),
        atol=1e-12,
    )


def test_proxy_renders_as_valid_urdf():
    rendered = subprocess.run(
        ["xacro", str(PROXY_PATH)], capture_output=True, text=True, check=True
    ).stdout
    robot = ET.fromstring(rendered)
    assert robot.attrib["name"] == "alfa_robot_v3"
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as temporary:
        temporary.write(rendered)
        temporary.flush()
        checked = subprocess.run(
            ["check_urdf", temporary.name], capture_output=True, text=True
        )
    assert checked.returncode == 0, checked.stderr
    assert "root Link: base_footprint" in checked.stdout
