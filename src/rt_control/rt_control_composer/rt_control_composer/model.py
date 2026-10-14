"""Immutable handoff from offline composition to launch and preflight."""

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Composed:
    robot_description: str
    controllers: dict
    controller_order: dict
    runtime_dir: Path
    systems: tuple
    use_mock_hardware: bool
    safety_policy: dict
