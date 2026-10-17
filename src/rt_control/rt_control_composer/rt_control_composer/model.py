"""Immutable handoff from offline composition to launch and preflight."""

from dataclasses import dataclass, field
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
    # State-only systems first, actuator systems after; teardown runs in reverse.
    hardware_activation_order: tuple = ()
    # Opaque bus resources (e.g. "ethercat:0/ring:3", "canopen:can0/node:11") for exclusive locks.
    resource_claims: tuple = ()
    staged_hardware_activation: bool = False
    lifecycle: dict = field(default_factory=dict)
