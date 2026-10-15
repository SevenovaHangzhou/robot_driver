"""Immutable handoff from offline composition to artifact writing, launch and preflight."""

from dataclasses import dataclass
from pathlib import Path

ARTIFACT = "artifact.yaml"


@dataclass(frozen=True)
class Composed:
    robot_description: str
    controllers: dict
    controller_order: dict
    runtime_dir: Path
    systems: tuple
    use_mock_hardware: bool
    safety_policy: dict
    # Relative path -> text for every output; the artifact step alone writes them.
    files: dict
    # Controller -> {parameter: runtime key}; values are supplied per start, not stored.
    runtime_parameters: dict
