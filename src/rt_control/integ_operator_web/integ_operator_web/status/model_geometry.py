"""Chassis drawing geometry read from the running Robot Model.

The status page draws the chassis from the model that the stack actually loaded, so a moved
bumper or a new wheelbase appears without editing the page. Only fixed joints are read; nothing
here decides what is installed — a missing frame simply removes its sector from the drawing.

Coordinates follow REP-103: x forward, y left. Where the model has no `ultrasonic_channel_*_link`,
`sonar` is empty and the page falls back to its placeholder layout.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Iterable, Optional

SONAR_PREFIX = "ultrasonic_channel_"
STEER_JOINTS = tuple(f"steer{index}_joint" for index in range(1, 5))


def _origin(joint: ET.Element) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    origin = joint.find("origin")
    if origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = tuple(float(value) for value in origin.get("xyz", "0 0 0").split())
    rpy = tuple(float(value) for value in origin.get("rpy", "0 0 0").split())
    return xyz, rpy


def _yaw_deg(rpy: Iterable[float]) -> float:
    return round(math.degrees(tuple(rpy)[2]), 2)


def frames(urdf_text: str) -> dict:
    """Sonar frames and the steering footprint declared in one URDF document."""
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return {"sonar": [], "wheels": [], "body": None}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    sonar = []
    for name, joint in joints.items():
        if not name or not name.startswith(SONAR_PREFIX) or joint.get("type") != "fixed":
            continue
        channel = name[len(SONAR_PREFIX) :].split("_", 1)[0]
        if not channel.isdigit():
            continue
        xyz, rpy = _origin(joint)
        sonar.append(
            {"channel": int(channel), "x_m": xyz[0], "y_m": xyz[1], "yaw_deg": _yaw_deg(rpy)}
        )
    sonar.sort(key=lambda item: item["channel"])

    wheels = [
        {"joint": name, "x_m": _origin(joints[name])[0][0], "y_m": _origin(joints[name])[0][1]}
        for name in STEER_JOINTS
        if name in joints
    ]
    body = None
    if wheels:
        half_length = max(abs(item["x_m"]) for item in wheels)
        half_width = max(abs(item["y_m"]) for item in wheels)
        body = {"half_length_m": round(half_length, 4), "half_width_m": round(half_width, 4)}
    return {"sonar": sonar, "wheels": wheels, "body": body}


def parse(urdf_path: Optional[str]) -> dict:
    """`frames` for a model file; an unreadable or missing file yields the empty drawing."""
    if not urdf_path:
        return {"sonar": [], "wheels": [], "body": None}
    try:
        from pathlib import Path

        return frames(Path(urdf_path).read_text(encoding="utf-8"))
    except OSError:
        return {"sonar": [], "wheels": [], "body": None}


__all__ = ["SONAR_PREFIX", "STEER_JOINTS", "frames", "parse"]
