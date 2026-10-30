#!/usr/bin/env python3
"""Classify changed paths for focused CI jobs."""

from __future__ import annotations

import sys
from collections.abc import Iterable


# Head control itself lives in enable_manager (shared), so only the DaMiao protocol
# package qualifies for the head-only fast path.
_HEAD_PREFIXES = ("src/rt_control/robot_hw_can/",)


def is_head_only(paths: Iterable[str]) -> bool:
    normalized = [path.strip() for path in paths if path.strip()]
    return bool(normalized) and all(
        path.startswith(_HEAD_PREFIXES)
        for path in normalized
    )


def main() -> int:
    print("true" if is_head_only(sys.stdin) else "false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
