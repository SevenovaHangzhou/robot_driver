#!/usr/bin/env python3
"""Classify changed paths for focused CI jobs."""

from __future__ import annotations

import sys
from collections.abc import Iterable


_HEAD_PREFIXES = (
    "src/rt_control/robot_hw_can/",
    "src/rt_control/damiao_head_controller/",
)
_HEAD_FILES = {
    "src/rt_control/rt_control_bringup/launch/rt_control_head_runtime.launch.py",
    "src/rt_control/rt_control_bringup/rt_control_bringup/head_can_config.py",
    "src/rt_control/rt_control_bringup/rt_control_bringup/head_runtime.py",
    "src/rt_control/rt_control_bringup/test/test_v3_head_can_integration.py",
    "src/rt_control/rt_control_bringup/urdf/alfa_v3_head.ros2_control.xacro",
}


def is_head_only(paths: Iterable[str]) -> bool:
    normalized = [path.strip() for path in paths if path.strip()]
    return bool(normalized) and all(
        path in _HEAD_FILES or path.startswith(_HEAD_PREFIXES)
        for path in normalized
    )


def main() -> int:
    print("true" if is_head_only(sys.stdin) else "false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
