"""R-IN-16 /rt/head/move: goal checks and progress from joint samples; pure functions, no ROS.

The goal's two fields follow the head controller's joint order: head_joint_position_rad is
the first joint (H1, yaw), head_pitch_joint_position_rad the second (H2, pitch).
"""

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

CAPABILITY = "head_move"
# User 2026-11-04: both joints within 0.01 rad is arrival; the time limit is the slowest joint's
# distance at its velocity limit plus 2 s.
TOLERANCE_RAD = 0.01
TIMEOUT_MARGIN_S = 2.0


@dataclass(frozen=True)
class HeadJoint:
    name: str
    lower: float
    upper: float
    velocity_limit: float


def load_head(generated_dir):
    """Command topic and joints of the artifact's head controller (capability head_move)."""
    root = Path(generated_dir)
    lifecycle = yaml.safe_load((root / "lifecycle.yaml").read_text(encoding="utf-8"))
    [capability] = [c for c in lifecycle["capabilities"] if c["name"] == CAPABILITY]
    controller = capability["controller"]
    controllers = yaml.safe_load((root / "controllers.yaml").read_text(encoding="utf-8"))
    names = controllers[controller]["ros__parameters"]["joints"]
    model = ET.fromstring((root / "robot_description.xml").read_text(encoding="utf-8"))
    joints = []
    for name in names:
        [joint] = [
            j for j in model.iter("joint") if j.get("name") == name and j.find("param") is not None
        ]
        params = {p.get("name"): float(p.text) for p in joint.findall("param")}
        command = {
            p.get("name"): float(p.text)
            for p in joint.find("command_interface[@name='position']").findall("param")
        }
        joints.append(HeadJoint(name, command["min"], command["max"], params["velocity_limit"]))
    return f"/{controller}/commands", joints


def target_error(joints, target):
    """None when every target is finite and inside the joint's commanded position range."""
    for joint, value in zip(joints, target):
        if not math.isfinite(value) or not joint.lower <= value <= joint.upper:
            return f"{joint.name} target {value:.4f} rad outside [{joint.lower}, {joint.upper}]"
    return None


def time_limit(joints, start, target):
    return TIMEOUT_MARGIN_S + max(
        abs(goal - begin) / joint.velocity_limit
        for joint, begin, goal in zip(joints, start, target)
    )


def arrived(target, actual):
    return all(abs(goal - now) <= TOLERANCE_RAD for goal, now in zip(target, actual))
