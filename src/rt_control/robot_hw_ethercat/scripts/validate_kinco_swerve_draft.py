#!/usr/bin/env python3
"""Validate the fail-closed alfa_v3 Kinco swerve EtherCAT draft."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any

import yaml


TBD = "TBD"
MODULES = ("front_left", "front_right", "rear_left", "rear_right")
ROOT_KEYS = {
    "schema_version",
    "module",
    "status",
    "verified",
    "shared_system",
    "master_id",
    "driver_plugin",
    "requested_control_frequency_hz",
    "dc_cycle_ns",
    "pdo_watchdog_ms",
    "device_identity",
    "mechanical_reference",
    "axes",
    "pending_facts",
}
AXIS_KEYS = {
    "module",
    "role",
    "joint_name",
    "ring_position",
    "family",
    "profile",
    "mode_of_operation",
    "interface_contract",
    "command_unit",
    "feedback_units",
}


class ValidationError(ValueError):
    """The Kinco swerve draft violates its fail-closed contract."""


class UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _unique_mapping(loader, node, deep=False):
    loader.flatten_mapping(node)
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"duplicate mapping key {key!r}",
                key_node.start_mark,
            )
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping
)


def _mapping(value: Any, context: str) -> dict[str, Any]:
    if type(value) is not dict:
        raise ValidationError(f"{context} must be a mapping")
    return value


def _exact_keys(value: dict[str, Any], expected: set[str], context: str) -> None:
    if set(value) != expected:
        raise ValidationError(
            f"{context} keys mismatch: missing={sorted(expected - set(value))}, "
            f"unknown={sorted(set(value) - expected)}"
        )


def validate(path: Path) -> dict[str, Any]:
    try:
        document = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)
    except (OSError, yaml.YAMLError) as error:
        raise ValidationError(f"cannot load {path}: {error}") from error
    root = _mapping(document, path.name)
    _exact_keys(root, ROOT_KEYS, path.name)
    if root["schema_version"] != 1 or root["module"] != "swerve_chassis":
        raise ValidationError("profile identity must be schema 1 swerve_chassis")
    if root["status"] != "draft" or root["verified"] is not False:
        raise ValidationError("Kinco swerve profile must remain unverified draft")
    if root["shared_system"] != "ecat_arms" or root["master_id"] != 0:
        raise ValidationError("swerve axes must use the shared EtherCAT master 0")
    if root["driver_plugin"] != "ethercat_generic_plugins/EcCiA402Drive":
        raise ValidationError("swerve axes must use the generic CiA402 drive plugin")
    if root["requested_control_frequency_hz"] != 1000:
        raise ValidationError("swerve axes must preserve the shared 1 kHz control request")
    if root["dc_cycle_ns"] != TBD or root["pdo_watchdog_ms"] != TBD:
        raise ValidationError("DC cycle support and PDO watchdog must remain TBD")
    identity = _mapping(root["device_identity"], "device_identity")
    _exact_keys(identity, {"vendor_id", "product_code", "revision_number"}, "device_identity")
    if any(value != TBD for value in identity.values()):
        raise ValidationError("device identity must remain TBD")
    reference = _mapping(root["mechanical_reference"], "mechanical_reference")
    if reference != {
        "package": "rt_control_bringup",
        "file": "config/machines/alfa_v3_swerve_mechanics.draft.yaml",
        "verified": False,
    }:
        raise ValidationError("mechanical reference must remain the unverified vendor draft")

    axes = root["axes"]
    if type(axes) is not list or len(axes) != 8:
        raise ValidationError("Kinco swerve draft requires exactly eight ordered axes")
    expected_modules = MODULES + MODULES
    expected_roles = ("steering",) * 4 + ("drive",) * 4
    for index, raw_axis in enumerate(axes):
        axis = _mapping(raw_axis, f"axes[{index}]")
        _exact_keys(axis, AXIS_KEYS, f"axes[{index}]")
        role = expected_roles[index]
        if axis["module"] != expected_modules[index] or axis["role"] != role:
            raise ValidationError("axis module/role order must be steering then drive FL/FR/RL/RR")
        for field in ("joint_name", "ring_position", "family", "profile"):
            if axis[field] != TBD:
                raise ValidationError(f"axes[{index}].{field} must remain TBD")
        if role == "steering":
            expected = {
                "mode_of_operation": 8,
                "interface_contract": "swerve_steering_csp",
                "command_unit": "output_axis_rad",
                "feedback_units": {"position": "output_axis_rad"},
            }
        else:
            expected = {
                "mode_of_operation": 9,
                "interface_contract": "swerve_drive_csv",
                "command_unit": "wheel_rad_per_second",
                "feedback_units": {
                    "position": "wheel_rad",
                    "velocity": "wheel_rad_per_second",
                },
            }
        for field, value in expected.items():
            if axis[field] != value:
                raise ValidationError(f"axes[{index}].{field} is inconsistent with {role}")
    pending = root["pending_facts"]
    if type(pending) is not list or not pending or not all(
        type(item) is str and item.strip() == item and item for item in pending
    ):
        raise ValidationError("pending_facts must contain explicit unresolved facts")
    return root


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True, type=Path)
    arguments = parser.parse_args()
    try:
        profile = validate(arguments.profile)
    except ValidationError as error:
        print(f"Kinco swerve draft validation failed: {error}", file=sys.stderr)
        return 1
    print(
        f"validated {profile['module']} draft: {len(profile['axes'])} axes, "
        f"master_id={profile['master_id']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
