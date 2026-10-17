#!/usr/bin/env python3
"""Record five V3 planar-root poses while keeping the map environment fixed."""

from __future__ import annotations

import argparse
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rerun as rr
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BASE_PATH = "map/odom/base_footprint"
POSES = (
    (0.0, 0.0, 0.0, "origin"),
    (0.50, 0.0, 0.0, "forward 0.50m"),
    (0.50, 0.35, math.radians(30.0), "left 0.35m, yaw +30deg"),
    (0.10, -0.40, math.radians(-60.0), "right 0.40m, yaw -60deg"),
    (-0.30, 0.15, math.radians(179.0), "rear 0.30m, yaw +179deg"),
)


def transform(translation=(0.0, 0.0, 0.0), rotation=None):
    result = np.eye(4)
    result[:3, 3] = np.asarray(translation, dtype=float)
    if rotation is not None:
        result[:3, :3] = rotation
    return result


def rpy_matrix(values):
    roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=float)
    return rz @ ry @ rx


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        (
            (c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s),
            (y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s),
            (z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c),
        ),
        dtype=float,
    )


def vector(element, name, default):
    if element is None or name not in element.attrib:
        return tuple(default)
    return tuple(float(value) for value in element.attrib[name].split())


def origin_transform(element):
    if element is None:
        return np.eye(4)
    return transform(vector(element, "xyz", (0, 0, 0)), rpy_matrix(vector(element, "rpy", (0, 0, 0))))


def joint_transform(joint, position):
    result = origin_transform(joint.find("origin"))
    joint_type = joint.attrib["type"]
    axis = vector(joint.find("axis"), "xyz", (1, 0, 0))
    if joint_type in ("revolute", "continuous"):
        result = result @ transform(rotation=axis_angle_matrix(axis, position))
    elif joint_type == "prismatic":
        result = result @ transform(np.asarray(axis) * position)
    return result


def render_robot():
    completed = subprocess.run(
        ["xacro", str(PACKAGE_ROOT / "urdf" / "robot.urdf.xacro"), "end_effector:=suction"],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode:
        raise RuntimeError(completed.stderr)
    return ET.fromstring(completed.stdout)


def validate_contract(robot):
    links = {link.attrib["name"] for link in robot.findall("link")}
    joints = robot.findall("joint")
    children = {joint.find("child").attrib["link"] for joint in joints}
    if links - children != {"base_footprint"}:
        raise RuntimeError(f"unexpected URDF roots: {sorted(links - children)}")
    movable_count = sum(joint.attrib["type"] != "fixed" for joint in joints)
    if movable_count != 26:
        raise RuntimeError(f"suction profile has {movable_count} movable joints, expected 26")

    srdf = ET.parse(PACKAGE_ROOT / "srdf" / "robot.srdf").getroot()
    virtual_joints = srdf.findall("virtual_joint")
    expected = {
        "name": "map_to_base_footprint",
        "type": "planar",
        "parent_frame": "map",
        "child_link": "base_footprint",
    }
    if len(virtual_joints) != 1 or virtual_joints[0].attrib != expected:
        raise RuntimeError("SRDF does not provide the expected map planar virtual joint")


def named_pose():
    data = yaml.safe_load((PACKAGE_ROOT / "config" / "named_poses_suction.yaml").read_text())
    return {name: float(value) for name, value in data["named_poses"]["home"].items()}


def link_transforms(robot, positions):
    transforms = {"base_footprint": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find("parent").attrib["link"]
            if parent not in transforms:
                continue
            child = joint.find("child").attrib["link"]
            transforms[child] = transforms[parent] @ joint_transform(
                joint, positions.get(joint.attrib["name"], 0.0)
            )
            pending.remove(joint)
            progress = True
        if not progress:
            raise RuntimeError("URDF joint graph is disconnected")
    return transforms


def mesh_path(filename):
    prefix = "package://robot_description/"
    if not filename.startswith(prefix):
        raise RuntimeError(f"unsupported mesh URI: {filename}")
    return PACKAGE_ROOT / filename.removeprefix(prefix)


def record_robot(robot, transforms):
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    for link_name, link_to_base in transforms.items():
        if link_name == "base_footprint":
            continue
        link_path = f"{BASE_PATH}/links/{link_name}"
        rr.log(
            link_path,
            rr.Transform3D(
                translation=link_to_base[:3, 3],
                mat3x3=link_to_base[:3, :3],
            ),
            static=True,
        )
        for index, visual in enumerate(links[link_name].findall("visual")):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            visual_to_link = origin_transform(visual.find("origin"))
            visual_path = f"{link_path}/visual_{index}"
            rr.log(
                visual_path,
                rr.Transform3D(
                    translation=visual_to_link[:3, 3],
                    mat3x3=visual_to_link[:3, :3],
                    scale=vector(mesh, "scale", (1, 1, 1)),
                ),
                static=True,
            )
            rr.log(
                visual_path,
                rr.Asset3D(path=mesh_path(mesh.attrib["filename"]), albedo_factor=[185, 195, 205, 215]),
                static=True,
            )


def record(output):
    robot = render_robot()
    validate_contract(robot)
    transforms = link_transforms(robot, named_pose())

    rr.init("motion_226_v3_planar_root", recording_id="motion-226-planar-root")
    rr.save(output)
    rr.log("map", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("map/odom", rr.Transform3D(translation=[0.0, 0.0, 0.0]), static=True)
    rr.log(
        "map/environment/floor",
        rr.Boxes3D(centers=[[0, 0, -0.025]], sizes=[[4.0, 4.0, 0.05]], colors=[[80, 86, 92, 255]]),
        static=True,
    )
    rr.log(
        "map/environment/fixed_wall",
        rr.Boxes3D(centers=[[1.65, 0, 1.0]], sizes=[[0.08, 2.4, 2.0]], colors=[[76, 120, 96, 210]]),
        static=True,
    )
    rr.log(
        "map/planar_samples",
        rr.Points3D(
            [[x, y, 0.0] for x, y, _, _ in POSES],
            radii=0.035,
            colors=[[40, 145, 220, 255]],
            labels=[label for _, _, _, label in POSES],
            show_labels=True,
        ),
        static=True,
    )
    rr.log(
        "map/planar_path",
        rr.LineStrips3D([[[x, y, 0.01] for x, y, _, _ in POSES]], colors=[[40, 145, 220, 180]], radii=0.008),
        static=True,
    )
    rr.log(
        BASE_PATH,
        rr.Arrows3D(
            origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            vectors=[[0.35, 0, 0], [0, 0.35, 0], [0, 0, 0.35]],
            colors=[[230, 70, 70, 255], [70, 210, 110, 255], [70, 120, 230, 255]],
            radii=0.008,
        ),
        static=True,
    )
    record_robot(robot, transforms)

    for index, (x, y, yaw, label) in enumerate(POSES):
        rr.set_time("pose_index", sequence=index)
        rr.log(
            BASE_PATH,
            rr.Transform3D(
                translation=[x, y, 0.0],
                quaternion=[0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)],
            ),
        )
        rr.log(
            "map/current_pose",
            rr.TextLog(f"pose {index}: {label}; x={x:.3f}m y={y:.3f}m yaw={math.degrees(yaw):.1f}deg"),
        )
    print(f"Validated planar-root contract and wrote {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("/tmp/motion-226-v3-planar-root.rrd"))
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    robot = render_robot()
    validate_contract(robot)
    if args.check_only:
        print("PASS: V3 suction URDF root and SRDF planar virtual joint are consistent")
        return
    args.output.parent.mkdir(parents=True, exist_ok=True)
    record(args.output)


if __name__ == "__main__":
    main()
