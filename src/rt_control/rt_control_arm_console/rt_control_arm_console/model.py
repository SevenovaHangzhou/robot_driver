"""Robot Model access for the console: joint limits and a browser copy of the arm.

The console never keeps its own geometry. It reads the URDF that the running
controller_manager was started with, takes the subtree that carries the
controlled joints and rewrites ``package://`` mesh URIs to console URLs.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path, PurePosixPath
from typing import Callable, Optional, Sequence
import xml.etree.ElementTree as ET

MESH_SUFFIXES = frozenset({".stl", ".dae"})
MESH_URL_PREFIX = "/mesh/"

ShareLookup = Callable[[str], str]


class ModelError(ValueError):
    pass


@dataclass(frozen=True)
class JointLimit:
    name: str
    lower: float
    upper: float
    velocity: float


@dataclass(frozen=True)
class ArmModel:
    joints: tuple[JointLimit, ...]
    root_link: str
    browser_urdf: str


def load_arm_model(urdf: str, joints: Sequence[str]) -> ArmModel:
    try:
        robot = ET.fromstring(urdf)
    except ET.ParseError as exc:
        raise ModelError(f"robot_description is not valid XML: {exc}") from exc
    elements = {joint.get("name"): joint for joint in robot.findall("joint")}
    limits = []
    for name in joints:
        joint = elements.get(name)
        if joint is None:
            raise ModelError(f"controlled joint missing from the Robot Model: {name}")
        if joint.get("type") not in ("revolute", "prismatic"):
            raise ModelError(f"controlled joint is not bounded revolute/prismatic: {name}")
        limit = joint.find("limit")
        try:
            values = [float(limit.get(key)) for key in ("lower", "upper", "velocity")]
        except (AttributeError, TypeError, ValueError) as exc:
            raise ModelError(f"joint limit incomplete in the Robot Model: {name}") from exc
        if not all(math.isfinite(value) for value in values) or values[0] >= values[1]:
            raise ModelError(f"joint limit invalid in the Robot Model: {name}")
        limits.append(JointLimit(name, *values))
    root = elements[joints[0]].find("parent").get("link")
    return ArmModel(tuple(limits), root, _arm_view(robot, root, set(joints)))


def _arm_view(robot: ET.Element, root: str, controlled: set[str]) -> str:
    """Controlled branch below ``root`` and everything after it, placed as in the robot.

    Sibling branches of ``root`` (other arm, head) and the visuals of ``root`` itself
    are left out. A fixed joint from ``console_world`` gives ``root`` its zero-pose
    placement in the robot root frame, so the arm keeps its real orientation.
    """
    children: dict[str, list[ET.Element]] = {}
    parent_joint: dict[str, ET.Element] = {}
    for joint in robot.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(joint)
        parent_joint[joint.find("child").get("link")] = joint
    links = {link.get("name"): link for link in robot.findall("link")}

    def carries_controlled(joint: ET.Element) -> bool:
        if joint.get("name") in controlled:
            return True
        return any(carries_controlled(j) for j in children.get(joint.find("child").get("link"), []))

    result = ET.Element("robot", name=robot.get("name", "robot") + "_console_view")
    for material in robot.findall("material"):
        result.append(material)
    ET.SubElement(result, "link", name="console_world")
    placement = ET.SubElement(result, "joint", name="console_world_to_" + root, type="fixed")
    ET.SubElement(placement, "parent", link="console_world")
    ET.SubElement(placement, "child", link=root)
    xyz, rpy = _decompose(_root_pose(root, parent_joint))
    ET.SubElement(
        placement,
        "origin",
        xyz=" ".join(f"{v:.9g}" for v in xyz),
        rpy=" ".join(f"{v:.9g}" for v in rpy),
    )
    ET.SubElement(result, "link", name=root)
    pending = [joint for joint in children.get(root, []) if carries_controlled(joint)]
    while pending:
        joint = pending.pop()
        name = joint.find("child").get("link")
        link = links.get(name)
        if link is None:
            raise ModelError(f"link missing from the Robot Model: {name}")
        copy = ET.fromstring(ET.tostring(link))
        for collision in copy.findall("collision"):
            copy.remove(collision)
        result.extend((joint, copy))
        pending.extend(children.get(name, []))
    for mesh in result.iter("mesh"):
        mesh.set("filename", mesh_url(mesh.get("filename", "")))
    return ET.tostring(result, encoding="unicode")


Matrix = list[list[float]]


def _origin(joint: ET.Element) -> Matrix:
    origin = joint.find("origin")
    xyz = [
        float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()
    ]
    rpy = [
        float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()
    ]
    roll, pitch, yaw = rpy
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _multiply(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _root_pose(link: str, parent_joint: dict[str, ET.Element]) -> Matrix:
    """Pose of ``link`` in the robot root frame with every joint at zero."""
    pose = [[float(i == j) for j in range(4)] for i in range(4)]
    while link in parent_joint:
        joint = parent_joint[link]
        pose = _multiply(_origin(joint), pose)
        link = joint.find("parent").get("link")
    return pose


def _decompose(pose: Matrix) -> tuple[list[float], list[float]]:
    pitch = math.asin(max(-1.0, min(1.0, -pose[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(pose[2][1], pose[2][2])
        yaw = math.atan2(pose[1][0], pose[0][0])
    else:  # gimbal lock: put the whole rotation into yaw
        roll = 0.0
        yaw = math.atan2(-pose[0][1], pose[1][1])
    return [pose[0][3], pose[1][3], pose[2][3]], [roll, pitch, yaw]


def mesh_url(filename: str) -> str:
    if not filename.startswith("package://"):
        raise ModelError(f"only package:// mesh URIs are supported: {filename!r}")
    return MESH_URL_PREFIX + filename[len("package://") :]


def resolve_mesh(url_path: str, share_lookup: ShareLookup) -> Optional[Path]:
    """Map ``<package>/<relative path>`` to a mesh file inside that package share."""
    parts = PurePosixPath(url_path).parts
    if len(parts) < 2 or any(part in ("", ".", "..") for part in parts) or "\\" in url_path:
        return None
    if PurePosixPath(url_path).suffix.lower() not in MESH_SUFFIXES:
        return None
    try:
        share = Path(share_lookup(parts[0])).resolve()
    except Exception:  # noqa: BLE001 - unknown package means "not found"
        return None
    path = share.joinpath(*parts[1:]).resolve()
    if not path.is_relative_to(share) or not path.is_file():
        return None
    return path
