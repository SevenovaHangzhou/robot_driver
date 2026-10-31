"""Robot Model access for the console: joint limits and a browser copy of the robot.

Control limits come from the running controller_manager. A selected authoritative
description share can supply the browser geometry without changing those limits.
Both views retain every link and joint, name the controlled arm links, and rewrite
``package://`` mesh URIs to console URLs.
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
    arm_links: tuple[str, ...] = ()  # links moved by the controlled joints (target preview)


def load_arm_model(
    urdf: str, joints: Sequence[str], *, mesh_prefix: str = MESH_URL_PREFIX
) -> ArmModel:
    try:
        robot = ET.fromstring(urdf)
    except ET.ParseError as exc:
        raise ModelError(f"robot_description is not valid XML: {exc}") from exc
    if not joints:
        if not robot.findall("link"):
            raise ModelError("browser model has no links")
        return ArmModel((), "", _robot_view(robot, mesh_prefix), ())
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
    return ArmModel(
        tuple(limits), root, _robot_view(robot, mesh_prefix), _arm_links(robot, root, set(joints))
    )


def _robot_view(robot: ET.Element, mesh_prefix: str = MESH_URL_PREFIX) -> str:
    """Every link and joint of the Robot Model, visuals only, for the browser.

    Collision geometry, ros2_control and simulator tags are dropped; joints that the
    console does not command stay at their zero position in the browser.
    """
    result = ET.Element("robot", name=robot.get("name", "robot") + "_console_view")
    for element in robot:
        if element.tag in ("material", "joint"):
            result.append(element)
        elif element.tag == "link":
            copy = ET.fromstring(ET.tostring(element))
            for collision in copy.findall("collision"):
                copy.remove(collision)
            result.append(copy)
    for mesh in result.iter("mesh"):
        mesh.set("filename", mesh_url(mesh.get("filename", ""), mesh_prefix))
    return ET.tostring(result, encoding="unicode")


def _arm_links(robot: ET.Element, root: str, controlled: set[str]) -> tuple[str, ...]:
    """Links below ``root`` on branches that carry a controlled joint (the moving arm)."""
    children: dict[str, list[ET.Element]] = {}
    for joint in robot.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(joint)

    def carries_controlled(joint: ET.Element) -> bool:
        if joint.get("name") in controlled:
            return True
        return any(carries_controlled(j) for j in children.get(joint.find("child").get("link"), []))

    links, pending = [], [j for j in children.get(root, []) if carries_controlled(j)]
    while pending:
        joint = pending.pop(0)
        name = joint.find("child").get("link")
        links.append(name)
        pending.extend(children.get(name, []))
    return tuple(links)


def mesh_url(filename: str, prefix: str = MESH_URL_PREFIX) -> str:
    if not filename.startswith("package://"):
        raise ModelError(f"only package:// mesh URIs are supported: {filename!r}")
    return prefix + filename[len("package://") :]


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
    # Parts exclude "", "." and "..", so the lexical path stays inside the share. It is not
    # resolved: a symlink install points installed meshes back into the source tree.
    path = share.joinpath(*parts[1:])
    if not path.is_relative_to(share) or not path.is_file():
        return None
    return path
