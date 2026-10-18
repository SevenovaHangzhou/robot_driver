#!/usr/bin/env python3
"""Compose the deployed mobile model with the Alfav3 dual arms (model_sources/alfav3_arms_1011_4).

The previous deployed entry (urdf/robot_analytic_proxy_mobile.urdf) supplies everything except the arms:
base, wheels, Updown/arm_carriage, head and IMU stay byte-for-byte equivalent. Both arms (link1..7 and
tool_link) come from the Alfav3 export; its base_link is the arm_carriage torso and only defines the
mount. Arm zero = Alfav3 export zero = drive zero (2^18 counts), user-confirmed 2026-10-17.
"""

import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = "alfav3_arms_1011_4"
BASE_ENTRY = "urdf/robot_analytic_proxy_mobile.urdf"
ENTRY = "urdf/robot_alfav3_mobile.urdf"
SIDES = ("left", "right")
# Alfav3 base_link -> arm_carriage. Fitted from both J1 axis lines of the previous model:
# residual 4.7 um / 1.5 urad (source rounding), so the exact value is used.
MOUNT_XYZ = (0.266, 0.0, 0.36515)
MOUNT_RPY = (math.pi, 0.0, 0.0)
# User-confirmed 2026-10-17: symmetric about the Alfav3 zero.
LIMITS_DEG = {1: 180.0, 2: 105.0, 3: 180.0, 4: 145.0, 5: 180.0, 6: 110.0, 7: 180.0}
# Joints whose export axis is negated so that +q = drive encoder count increasing (CiA 402 positive).
# User-confirmed 2026-10-18 on the left-arm bench (Alfa_V3-bench-01): with the export axis, left J3 and
# J6 turned the right way in the model but +q was the encoder's negative direction. Right arm: not
# verified on hardware, export axis kept.
AXIS_FLIP = {"left": (3, 6), "right": ()}
# tool0 keeps its previous physical pose: 151 mm beyond the flange along link7 +Z, same orientation.
TOOL0_BEYOND_FLANGE_M = 0.151


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(roll), math.sin(roll), math.cos(pitch),
                                               math.sin(pitch), math.cos(yaw), math.sin(yaw)))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_rpy(rotation):
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:  # gimbal lock: put everything into roll
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def transform(origin):
    xyz = [float(v) for v in (origin.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (origin.get("rpy") or "0 0 0").split()]
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_matrix(*rpy)
    matrix[:3, 3] = xyz
    return matrix


def fmt(values):
    return " ".join(repr(round(float(v), 12) + 0.0) for v in values)


def mount_matrix():
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_matrix(*MOUNT_RPY)
    matrix[:3, 3] = MOUNT_XYZ
    return matrix


def arm_names(side):
    links = [f"{side}_link{i}" for i in range(1, 8)] + [f"{side}_tool_link"]
    joints = [f"{side}_joint{i}" for i in range(1, 8)] + [f"{side}_tool_fixed"]
    return links, joints


def build():
    model = ET.parse(ROOT / BASE_ENTRY).getroot()
    previous = copy.deepcopy(model)
    source = ET.parse(ROOT / f"model_sources/{SOURCE}/robot.urdf").getroot()
    prefix = f"package://robot_description/model_sources/{SOURCE}/"

    removed = {f"{side}_link{i}" for side in SIDES for i in range(1, 8)} | {f"{side}_tool0" for side in SIDES}
    for element in list(model):
        if element.tag == "link" and element.get("name") in removed:
            model.remove(element)
        elif element.tag == "joint" and element.find("child").get("link") in removed:
            model.remove(element)

    for side in SIDES:
        links, joints = arm_names(side)
        for name in links:
            link = copy.deepcopy(source.find(f"link[@name='{name}']"))
            for mesh in link.findall(".//mesh"):
                mesh.set("filename", prefix + mesh.get("filename"))
            model.append(link)
        for name in joints:
            joint = copy.deepcopy(source.find(f"joint[@name='{name}']"))
            if name.endswith("_joint1"):
                assert joint.find("parent").get("link") == "base_link"
                joint.find("parent").set("link", "arm_carriage")
                matrix = mount_matrix() @ transform(joint.find("origin"))
                joint.find("origin").set("xyz", fmt(matrix[:3, 3]))
                joint.find("origin").set("rpy", fmt(matrix_rpy(matrix[:3, :3])))
            if joint.get("type") == "revolute":
                index = int(name[-1])
                if index in AXIS_FLIP[side]:
                    axis = joint.find("axis")
                    axis.set("xyz", " ".join(format(-float(v) + 0.0, "g") for v in axis.get("xyz").split()))
                old = previous.find(f"joint[@name='{name}']/limit")
                limit = joint.find("limit")
                limit.set("lower", repr(-math.radians(LIMITS_DEG[index])))
                limit.set("upper", repr(math.radians(LIMITS_DEG[index])))
                limit.set("effort", old.get("effort"))
                limit.set("velocity", old.get("velocity"))
            model.append(joint)
        flange_z = float(source.find(f"joint[@name='{side}_tool_fixed']/origin").get("xyz").split()[2])
        ET.SubElement(model, "link", name=f"{side}_tool0")
        tool0 = ET.SubElement(model, "joint", name=f"{side}_tool0_fixed", type="fixed")
        ET.SubElement(tool0, "origin", xyz=fmt([0.0, 0.0, flange_z + TOOL0_BEYOND_FLANGE_M]), rpy="0 0 0")
        ET.SubElement(tool0, "parent", link=f"{side}_link7")
        ET.SubElement(tool0, "child", link=f"{side}_tool0")

    ET.indent(model)
    ET.ElementTree(model).write(ROOT / ENTRY, encoding="utf-8", xml_declaration=True)

    limits = yaml.safe_load((ROOT / "config/joint_limits_analytic_proxy_mobile.yaml").read_text())
    limits["metadata"].update(
        model_revision="alfav3-mobile",
        arm_source=f"model_sources/{SOURCE}/robot.urdf",
        source="Alfav3 dual-arm export with the previous deployed base/head/chassis",
        warning="Arm zero is the Alfav3 export zero (drive zero 2^18 counts). Arm limits are user-confirmed "
                "symmetric limits; effort/velocity carried over and remain unverified.",
    )
    for side in SIDES:
        for index, degrees in LIMITS_DEG.items():
            entry = dict(limits["joints"][f"{side}_joint{index}"])
            entry.update(lower_position_rad=-math.radians(degrees), upper_position_rad=math.radians(degrees))
            limits["joints"][f"{side}_joint{index}"] = entry
    (ROOT / "config/joint_limits_alfav3_mobile.yaml").write_text(yaml.safe_dump(limits, sort_keys=False))

    srdf = ET.parse(ROOT / "srdf/robot.srdf").getroot()
    for state in srdf.findall("group_state"):
        srdf.remove(state)
    for side in SIDES:
        ET.SubElement(srdf, "disable_collisions", link1=f"{side}_link7", link2=f"{side}_tool_link", reason="Adjacent")
    ET.indent(srdf)
    ET.ElementTree(srdf).write(ROOT / "srdf/robot_alfav3_mobile.srdf", encoding="utf-8", xml_declaration=True)

    manifest = {
        "entry": ENTRY,
        "srdf": "srdf/robot_alfav3_mobile.srdf",
        "joint_limits": "config/joint_limits_alfav3_mobile.yaml",
        "base_entry": BASE_ENTRY,
        "arm_source": f"model_sources/{SOURCE}/robot.urdf",
        "arm_carriage_mount": {"xyz_m": list(MOUNT_XYZ), "rpy_rad": list(MOUNT_RPY),
                               "evidence": "fit of both J1 axis lines to the previous model; residual 4.7 um / 1.5 urad"},
        "arm_zero": "Alfav3 export zero = mechanical zero = drive zero (2^18 counts), user-confirmed 2026-10-17",
        "arm_limits_deg": {str(k): v for k, v in LIMITS_DEG.items()},
        "tool0": {"parent": "<side>_link7", "z_m": "flange (tool_fixed z) + 0.151", "orientation": "link7"},
        "previous_model_pose_of_alfav3_zero_deg": {"left": [90, -15, 180, 0, 90, 0, 0],
                                                   "right": [-90, 15, -180, 0, -90, 0, 0]},
        "axis_flip_vs_export": {side: list(AXIS_FLIP[side]) for side in SIDES},
        "axis_flip_evidence": "left J3/J6: +q = encoder count increasing, user-confirmed 2026-10-18 on "
                              "Alfa_V3-bench-01; right arm not verified on hardware",
        "axis_sign_vs_previous": {"left": ["+", "+", "+", "+", "+", "-", "+"],
                                  "right": ["+", "+", "-", "-", "+", "+", "+"]},
        "removed": ["named poses", "initial positions", "SRDF group_state entries"],
        "limitations": [
            "Arm inertials come from the SolidWorks export and are not hardware validated.",
            "Arm effort/velocity limits are carried over from the previous model and remain unverified.",
            "Joint-space values of the previous model do not apply; consumers must migrate.",
        ],
    }
    (ROOT / "config/alfav3_mobile_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    build()
