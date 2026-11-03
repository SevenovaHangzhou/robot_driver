"""Named joint poses saved by the operator. JSON file, atomic replace, no ROS."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
from typing import Mapping, Sequence

MAX_NAME_LENGTH = 32
MAX_POSES = 200
SCHEMA_VERSION = 1


class PoseError(ValueError):
    pass


def check_name(name: object) -> str:
    if not isinstance(name, str):
        raise PoseError("位姿名称必须是文本")
    name = name.strip()
    if not 0 < len(name) <= MAX_NAME_LENGTH:
        raise PoseError(f"位姿名称长度必须在 1 到 {MAX_NAME_LENGTH} 个字符之间")
    if any(ord(character) < 32 for character in name):
        raise PoseError("位姿名称不能包含控制字符")
    return name


class PoseStore:
    """Poses keyed by name; each pose stores every controlled joint in radians."""

    def __init__(self, path: Path, joints: Sequence[str]) -> None:
        self._path = Path(path)
        self._joints = tuple(joints)
        self._lock = threading.Lock()
        self._poses: dict[str, dict] = {}
        if self._path.exists():
            document = json.loads(self._path.read_text(encoding="utf-8"))
            if document.get("schema_version") != SCHEMA_VERSION:
                raise PoseError(f"unsupported pose file: {self._path}")
            for name, pose in document.get("poses", {}).items():
                # Poses of another joint set (e.g. a different scope) are kept but not offered.
                self._poses[check_name(name)] = pose

    def _usable(self, pose: Mapping) -> bool:
        joints = pose.get("joints", {})
        return set(joints) == set(self._joints) and all(
            isinstance(joints[name], (int, float)) and math.isfinite(joints[name])
            for name in self._joints
        )

    def list(self) -> list[dict]:
        with self._lock:
            return [
                {"name": name, "joints": dict(pose["joints"]), "saved_at": pose.get("saved_at")}
                for name, pose in sorted(self._poses.items(), key=lambda item: item[0])
                if self._usable(pose)
            ]

    def get(self, name: str) -> tuple[float, ...]:
        name = check_name(name)
        with self._lock:
            pose = self._poses.get(name)
            if pose is None or not self._usable(pose):
                raise PoseError(f"没有可用的位姿：{name}")
            return tuple(float(pose["joints"][joint]) for joint in self._joints)

    def save(self, name: str, positions: Sequence[float], *, overwrite: bool = False) -> None:
        name = check_name(name)
        if len(positions) != len(self._joints) or not all(math.isfinite(p) for p in positions):
            raise PoseError("位姿数据无效")
        with self._lock:
            if name in self._poses and not overwrite:
                raise PoseError(f"位姿已存在：{name}")
            if name not in self._poses and len(self._poses) >= MAX_POSES:
                raise PoseError("位姿数量已达上限")
            poses = dict(self._poses)
            poses[name] = {
                "joints": {joint: float(value) for joint, value in zip(self._joints, positions)},
                "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            }
            self._write(poses)
            self._poses = poses

    def delete(self, name: str) -> None:
        name = check_name(name)
        with self._lock:
            if name not in self._poses:
                raise PoseError(f"没有该位姿：{name}")
            poses = {key: value for key, value in self._poses.items() if key != name}
            self._write(poses)
            self._poses = poses

    def _write(self, poses: Mapping[str, dict]) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        text = json.dumps(
            {"schema_version": SCHEMA_VERSION, "poses": poses}, ensure_ascii=False, indent=2
        )
        handle, temporary = tempfile.mkstemp(dir=self._path.parent, prefix=".poses-")
        try:
            with os.fdopen(handle, "w", encoding="utf-8") as stream:
                stream.write(text + "\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self._path)
        except BaseException:
            Path(temporary).unlink(missing_ok=True)
            raise
