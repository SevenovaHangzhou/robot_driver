#!/usr/bin/env python3
"""Validate the fail-closed alfa_v3 swerve encoder draft profile."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys
from typing import Any

import yaml


TBD = "TBD"
RESOURCE_NAMES = (
    "front_left_steering_encoder",
    "front_right_steering_encoder",
    "rear_left_steering_encoder",
    "rear_right_steering_encoder",
)
ROOT_KEYS = {
    "schema_version",
    "module",
    "status",
    "verified",
    "plugin",
    "node_driver",
    "master_driver",
    "can_interface",
    "master_node_id",
    "sync_period_us",
    "position_pdo",
    "state_interfaces",
    "nodes",
    "pending_facts",
}
NODE_KEYS = {
    "resource_name",
    "node_id",
    "eds_file",
    "counts_per_revolution",
    "distinguishable_revolutions",
    "ring_gear_teeth",
    "pinion_gear_teeth",
    "direction",
    "installation_offset_rad",
}
CONFIRMED_GEARING = {
    "ring_gear_teeth": 108,
    "pinion_gear_teeth": 27,
}
IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


class ValidationError(ValueError):
    """The swerve encoder draft profile violates its fail-closed contract."""


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
    if root["schema_version"] != 1 or root["module"] != "swerve_encoders":
        raise ValidationError("profile identity must be schema 1 swerve_encoders")
    if root["status"] != "draft" or root["verified"] is not False:
        raise ValidationError("encoder profile must remain unverified draft")
    if root["plugin"] != "robot_hw_canopen/SwerveEncoderSystem":
        raise ValidationError("encoder profile plugin is incorrect")
    expected_node_driver = {
        "class": "ros2_canopen::ProxyDriver",
        "package": "canopen_proxy_driver",
    }
    expected_master_driver = {
        "class": "ros2_canopen::MasterDriver",
        "package": "canopen_master_driver",
    }
    if root["node_driver"] != expected_node_driver:
        raise ValidationError("encoder nodes must use the CANopen ProxyDriver")
    if root["master_driver"] != expected_master_driver:
        raise ValidationError("encoder bus must use the CANopen MasterDriver")
    if root["can_interface"] != TBD or root["master_node_id"] != TBD:
        raise ValidationError("CAN interface and master node ID must remain TBD")
    if root["sync_period_us"] != 4000:
        raise ValidationError("swerve encoder SYNC period must be 4000 us")
    pdo = _mapping(root["position_pdo"], "position_pdo")
    _exact_keys(pdo, {"index", "subindex", "bit_length", "signed"}, "position_pdo")
    if pdo != {"index": 0x6004, "subindex": 0, "bit_length": 32, "signed": False}:
        raise ValidationError("position PDO must be unsigned 0x6004:00/32")
    if root["state_interfaces"] != ["position", "feedback_age_ms"]:
        raise ValidationError("encoder states must be position and feedback_age_ms")
    nodes = root["nodes"]
    if type(nodes) is not list or len(nodes) != len(RESOURCE_NAMES):
        raise ValidationError("encoder profile requires exactly four ordered nodes")
    for index, raw_node in enumerate(nodes):
        node = _mapping(raw_node, f"nodes[{index}]")
        _exact_keys(node, NODE_KEYS, f"nodes[{index}]")
        if node["resource_name"] != RESOURCE_NAMES[index] or (
            IDENTIFIER.fullmatch(node["resource_name"]) is None
        ):
            raise ValidationError("encoder resource order or name is invalid")
        for field in NODE_KEYS - {"resource_name", *CONFIRMED_GEARING}:
            if node[field] != TBD:
                raise ValidationError(
                    f"nodes[{index}].{field} must remain TBD until hardware confirmation"
                )
        for field, expected in CONFIRMED_GEARING.items():
            if node[field] != expected:
                raise ValidationError(
                    f"nodes[{index}] must use confirmed external encoder gearing "
                    f"{field}={expected}"
                )
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
        print(f"CANopen swerve encoder draft validation failed: {error}", file=sys.stderr)
        return 1
    print(
        f"validated {profile['module']} draft: {len(profile['nodes'])} nodes, "
        f"sync_period_us={profile['sync_period_us']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
