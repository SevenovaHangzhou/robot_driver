"""Verify the Alfav3 mobile entry against its sources and the previous deployed model geometry."""

import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "model_sources/alfav3_arms_1011_4"
ENTRY = ROOT / "urdf/robot_alfav3_mobile.urdf"
PREVIOUS = ROOT / "urdf/robot_analytic_proxy_mobile.urdf"
SRDF = ROOT / "srdf/robot_alfav3_mobile.srdf"
MANIFEST = json.loads((ROOT / "config/alfav3_mobile_manifest.json").read_text())
SIDES = ("left", "right")
PREFIX = "package://robot_description/model_sources/alfav3_arms_1011_4/"
LIMITS_DEG = {1: 180, 2: 105, 3: 180, 4: 145, 5: 180, 6: 110, 7: 180}
# +q = drive encoder increasing; left J3/J6 confirmed on the bench 2026-10-18, right arm unverified.
AXIS_FLIP = {"left": (3, 6), "right": ()}


def canonical(element):
    return (element.tag, sorted(element.attrib.items()), [canonical(e) for e in element])


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin(element):
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, :3] = rpy_matrix(*[float(v) for v in (element.get("rpy") or "0 0 0").split()])
        matrix[:3, 3] = [float(v) for v in (element.get("xyz") or "0 0 0").split()]
    return matrix


def rotation(axis, angle):
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    matrix = np.eye(4)
    matrix[:3, :3] = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
    return matrix


def forward(model, root, angles=None):
    """Frames of every link relative to `root` with optional joint angles (rad)."""
    angles = angles or {}
    frames = {root: np.eye(4)}
    joints = list(model.findall("joint"))
    while True:
        ready = [j for j in joints if j.find("parent").get("link") in frames and j.find("child").get("link") not in frames]
        if not ready:
            return frames
        for joint in ready:
            matrix = frames[joint.find("parent").get("link")] @ origin(joint.find("origin"))
            if joint.get("type") != "fixed":
                axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
                matrix = matrix @ rotation(axis, angles.get(joint.get("name"), 0.0))
            frames[joint.find("child").get("link")] = matrix


def axis_line(model, frames, joint_name):
    joint = model.find(f"joint[@name='{joint_name}']")
    frame = frames[joint.find("child").get("link")]
    direction = frame[:3, :3] @ np.array([float(v) for v in joint.find("axis").get("xyz").split()])
    return frame[:3, 3], direction / np.linalg.norm(direction)


def line_distance(a, b):
    (p1, d1), (p2, d2) = a, b
    normal = np.cross(d1, d2)
    if np.linalg.norm(normal) < 1e-9:
        return np.linalg.norm(np.cross(p2 - p1, d1))
    return abs((p2 - p1) @ normal) / np.linalg.norm(normal)


def test_source_integrity_and_naming():
    for line in (SOURCE / "SHA256SUMS").read_text().splitlines():
        digest, name = line.split("  ", 1)
        assert hashlib.sha256((SOURCE / name).read_bytes()).hexdigest() == digest, name
    text = (SOURCE / "robot.urdf").read_text(encoding="utf-8")
    assert text.isascii()
    source = ET.fromstring(text)
    assert source.get("name") == "Alfav3"
    expected_links = {"base_link"} | {f"{s}_link{i}" for s in SIDES for i in range(1, 8)} | {f"{s}_tool_link" for s in SIDES}
    assert {link.get("name") for link in source.findall("link")} == expected_links
    report = json.loads((SOURCE / "export_report.json").read_text())
    # Visual meshes keep the exported bytes.
    for old, new in report["link_rename"].items():
        visual = SOURCE / "meshes/visual" / f"{new}.STL"
        assert hashlib.sha256(visual.read_bytes()).hexdigest() == report["original_files_sha256"][f"meshes/{old}.STL"]
    # Collision envelopes contain every sampled visual vertex.
    for name, stats in report["collision"]["links"].items():
        assert stats["visual_vertices_outside"] == 0 and stats["max_penetration_mm"] == 0.0, name


def test_non_arm_elements_unchanged():
    previous = ET.parse(PREVIOUS).getroot()
    entry = ET.parse(ENTRY).getroot()
    arm = {f"{s}_link{i}" for s in SIDES for i in range(1, 8)} | {f"{s}_tool0" for s in SIDES}
    for element in previous:
        child = element.find("child")
        if element.get("name") in arm or (child is not None and child.get("link") in arm):
            continue
        counterpart = entry.find(f"{element.tag}[@name='{element.get('name')}']")
        assert counterpart is not None, element.get("name")
        assert canonical(element) == canonical(counterpart), element.get("name")
    assert entry.get("name") == previous.get("name")


def test_arm_links_and_joints_from_source():
    source = ET.parse(SOURCE / "robot.urdf").getroot()
    entry = ET.parse(ENTRY).getroot()
    previous = ET.parse(PREVIOUS).getroot()
    for side in SIDES:
        for name in [f"{side}_link{i}" for i in range(1, 8)] + [f"{side}_tool_link"]:
            expected = copy.deepcopy(source.find(f"link[@name='{name}']"))
            for mesh in expected.findall(".//mesh"):
                mesh.set("filename", PREFIX + mesh.get("filename"))
            actual = entry.find(f"link[@name='{name}']")
            assert canonical(expected) == canonical(actual)
            assert actual.find("visual/geometry/mesh").get("filename").startswith(PREFIX + "meshes/visual/")
            assert actual.find("collision/geometry/mesh").get("filename").startswith(PREFIX + "meshes/collision/")
        for index in range(1, 8):
            name = f"{side}_joint{index}"
            joint, src, old = (m.find(f"joint[@name='{name}']") for m in (entry, source, previous))
            assert joint.get("type") == "revolute"
            axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
            source_axis = np.array([float(v) for v in src.find("axis").get("xyz").split()])
            sign = -1.0 if index in AXIS_FLIP[side] else 1.0
            assert np.allclose(axis, sign * source_axis), name
            limit = joint.find("limit")
            assert math.isclose(float(limit.get("lower")), -math.radians(LIMITS_DEG[index]))
            assert math.isclose(float(limit.get("upper")), math.radians(LIMITS_DEG[index]))
            assert limit.get("effort") == old.find("limit").get("effort")
            assert limit.get("velocity") == old.find("limit").get("velocity")
            expected_parent = "arm_carriage" if index == 1 else f"{side}_link{index - 1}"
            assert joint.find("parent").get("link") == expected_parent
            if index > 1:
                assert canonical(joint.find("origin")) == canonical(src.find("origin"))


def test_geometry_matches_previous_model():
    entry = ET.parse(ENTRY).getroot()
    previous = ET.parse(PREVIOUS).getroot()
    new_frames = forward(entry, "arm_carriage")
    old_zero = forward(previous, "arm_carriage")
    for side in SIDES:
        # J1 axis lines do not depend on any zero convention.
        a, b = axis_line(previous, old_zero, f"{side}_joint1"), axis_line(entry, new_frames, f"{side}_joint1")
        assert line_distance(a, b) < 1e-5 and abs(a[1] @ b[1]) > 1 - 1e-9
        pose = MANIFEST["previous_model_pose_of_alfav3_zero_deg"][side]
        old = forward(previous, "arm_carriage", {f"{side}_joint{i + 1}": math.radians(q) for i, q in enumerate(pose)})
        signs = MANIFEST["axis_sign_vs_previous"][side]
        for index in range(1, 8):
            pa = axis_line(previous, old, f"{side}_joint{index}")
            pb = axis_line(entry, new_frames, f"{side}_joint{index}")
            assert abs(pa[1] @ pb[1]) > math.cos(math.radians(0.05))
            assert (pa[1] @ pb[1] > 0) == (signs[index - 1] == "+")
        # Flange: previous link7 origin vs Alfav3 tool_link origin.
        assert np.linalg.norm(old[f"{side}_link7"][:3, 3] - new_frames[f"{side}_tool_link"][:3, 3]) < 1e-3
        # tool0 keeps its previous physical pose.
        assert np.linalg.norm(old[f"{side}_tool0"][:3, 3] - new_frames[f"{side}_tool0"][:3, 3]) < 1e-3
        relative = old[f"{side}_tool0"][:3, :3].T @ new_frames[f"{side}_tool0"][:3, :3]
        assert math.degrees(math.acos(min(1.0, (np.trace(relative) - 1) / 2))) < 0.05


def test_tree_meshes_configs_and_srdf():
    checked = subprocess.run(["check_urdf", str(ENTRY)], capture_output=True, text=True)
    assert checked.returncode == 0, checked.stderr
    assert "root Link: base_footprint" in checked.stdout
    entry = ET.parse(ENTRY).getroot()
    links = {link.get("name") for link in entry.findall("link")}
    assert len(links) == 32
    moving = {j.get("name") for j in entry.findall("joint") if j.get("type") != "fixed"}
    assert len(moving) == 25
    for mesh in entry.findall(".//mesh"):
        assert (ROOT / mesh.get("filename").removeprefix("package://robot_description/")).is_file()
    limits = yaml.safe_load((ROOT / "config/joint_limits_alfav3_mobile.yaml").read_text())["joints"]
    assert set(limits) == moving
    for side in SIDES:
        for index, degrees in LIMITS_DEG.items():
            assert math.isclose(limits[f"{side}_joint{index}"]["upper_position_rad"], math.radians(degrees))
    srdf = ET.parse(SRDF).getroot()
    assert srdf.find("group_state") is None
    assert srdf.get("name") == entry.get("name")
    for element in srdf.iter():
        for key in ("link1", "link2", "base_link", "tip_link", "link"):
            if key in element.attrib and element.tag != "virtual_joint":
                assert element.get(key) in links, (element.tag, element.get(key))
    assert MANIFEST["axis_flip_vs_export"] == {side: list(AXIS_FLIP[side]) for side in SIDES}
    for name in ("named_poses_alfav3_mobile.yaml", "initial_positions_alfav3_mobile.yaml"):
        assert not (ROOT / "config" / name).exists()
