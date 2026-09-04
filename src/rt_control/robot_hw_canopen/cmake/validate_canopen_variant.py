#!/usr/bin/env python3
"""Validate a CANopen variant descriptor against its runtime bus config."""

import argparse
from pathlib import Path
import re
import sys
from typing import Any

import yaml


ROOT_FIELDS = {"schema_version", "variant", "system", "nodes"}
NODE_FIELDS = {
    "joint_name",
    "node_id",
    "operation_mode",
    "side",
    "profile",
}
SUPPORTED_SYSTEM = "canopen_mobile_axes"
SUPPORTED_MODE = 3
SUPPORTED_PROFILE = "ld2_drive"
SUPPORTED_SIDES = {"left", "right"}
SUPPORTED_DCF = "eds/ld2_drive.eds"
SUPPORTED_DRIVER = "ros2_canopen::Cia402Driver"
SUPPORTED_DRIVER_PACKAGE = "canopen_402_driver"
FORBIDDEN_NODE_PROFILE_OVERRIDES = {"dcf", "driver", "package"}
ROS_SAFE_IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
RESERVED_NODE_BIN_STEMS = {"master"}


class ValidationError(ValueError):
    """Raised when a descriptor or bus config violates the frozen contract."""


class UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(
    loader: UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as exc:
            raise ValidationError(
                f"unhashable YAML mapping key at line {key_node.start_mark.line + 1}"
            ) from exc
        if duplicate:
            raise ValidationError(
                f"duplicate YAML key {key!r} at line {key_node.start_mark.line + 1}"
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


def _load_mapping(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.load(stream, Loader=UniqueKeyLoader)
    except (OSError, yaml.YAMLError, ValidationError) as exc:
        raise ValidationError(f"cannot load {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValidationError(f"{path} root must be a mapping")
    return document


def _require_exact_fields(
    mapping: dict[str, Any], expected: set[str], context: str
) -> None:
    actual = set(mapping)
    if actual != expected:
        missing = sorted(expected - actual)
        unknown = sorted(actual - expected)
        raise ValidationError(
            f"{context} fields must be exactly {sorted(expected)}; "
            f"missing={missing}, unknown={unknown}"
        )


def _require_exact_int(value: Any, context: str) -> int:
    if type(value) is not int:
        raise ValidationError(f"{context} must be an integer")
    return value


def _require_nonempty_string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValidationError(f"{context} must be a non-empty string")
    return value


def validate(variant_path: Path, bus_path: Path) -> dict[str, Any]:
    """Return the validated descriptor or raise ``ValidationError``."""

    descriptor = _load_mapping(variant_path)
    _require_exact_fields(descriptor, ROOT_FIELDS, "descriptor root")

    if _require_exact_int(descriptor["schema_version"], "schema_version") != 1:
        raise ValidationError("CANopen descriptor requires schema_version 1")
    expected_variant = variant_path.stem
    if descriptor["variant"] != expected_variant:
        raise ValidationError(
            f"descriptor variant must match filename {expected_variant!r}"
        )
    if descriptor["system"] != SUPPORTED_SYSTEM:
        raise ValidationError(
            f"CANopen descriptor system must be {SUPPORTED_SYSTEM}"
        )

    nodes = descriptor["nodes"]
    if not isinstance(nodes, list) or not nodes:
        raise ValidationError("CANopen descriptor nodes must be a non-empty list")

    joint_names: list[str] = []
    node_ids: list[int] = []
    for index, node in enumerate(nodes):
        context = f"descriptor node[{index}]"
        if not isinstance(node, dict):
            raise ValidationError(f"{context} must be a mapping")
        _require_exact_fields(node, NODE_FIELDS, context)
        joint_name = _require_nonempty_string(node["joint_name"], f"{context}.joint_name")
        if ROS_SAFE_IDENTIFIER.fullmatch(joint_name) is None:
            raise ValidationError(
                f"{context}.joint_name must be a ROS-safe identifier matching "
                "^[A-Za-z][A-Za-z0-9_]*$"
            )
        if joint_name in RESERVED_NODE_BIN_STEMS:
            raise ValidationError(
                f"{context}.joint_name {joint_name!r} collides with a reserved "
                "build artifact"
            )
        node_id = _require_exact_int(node["node_id"], f"{context}.node_id")
        if not 1 <= node_id <= 127:
            raise ValidationError(f"{context}.node_id must be in [1, 127]")
        operation_mode = _require_exact_int(
            node["operation_mode"], f"{context}.operation_mode"
        )
        if operation_mode != SUPPORTED_MODE:
            raise ValidationError(
                f"CANopen descriptor nodes require operation_mode {SUPPORTED_MODE}"
            )
        if node["side"] not in SUPPORTED_SIDES:
            raise ValidationError("CANopen descriptor node side must be left/right")
        if node["profile"] != SUPPORTED_PROFILE:
            raise ValidationError(
                f"CANopen descriptor nodes require profile {SUPPORTED_PROFILE}"
            )
        joint_names.append(joint_name)
        node_ids.append(node_id)

    if len(set(joint_names)) != len(joint_names):
        raise ValidationError("duplicate CANopen joint_name in descriptor")
    if len(set(node_ids)) != len(node_ids):
        raise ValidationError("duplicate CANopen node_id in descriptor")

    bus = _load_mapping(bus_path)
    defaults = bus.get("defaults")
    bus_nodes = bus.get("nodes")
    if not isinstance(defaults, dict):
        raise ValidationError("bus.yml defaults must be a mapping")
    if not isinstance(bus_nodes, dict):
        raise ValidationError("bus.yml nodes must be a mapping")

    descriptor_names = set(joint_names)
    if set(bus_nodes) != descriptor_names:
        raise ValidationError(
            "bus.yml node names must exactly match descriptor nodes; "
            f"descriptor={sorted(descriptor_names)}, bus={sorted(bus_nodes)}"
        )

    required_defaults = {
        "dcf": SUPPORTED_DCF,
        "driver": SUPPORTED_DRIVER,
        "package": SUPPORTED_DRIVER_PACKAGE,
    }
    for field, expected in required_defaults.items():
        if defaults.get(field) != expected:
            raise ValidationError(f"bus.yml defaults.{field} must be {expected}")

    for node in nodes:
        joint_name = node["joint_name"]
        bus_node = bus_nodes[joint_name]
        if not isinstance(bus_node, dict):
            raise ValidationError(f"bus.yml node {joint_name} must be a mapping")
        if bus_node.get("mandatory") is not True:
            raise ValidationError(
                f"bus.yml node {joint_name} mandatory must be true"
            )
        overrides = sorted(FORBIDDEN_NODE_PROFILE_OVERRIDES.intersection(bus_node))
        if overrides:
            raise ValidationError(
                f"bus.yml node {joint_name} must not override dcf/driver/package; "
                f"found={overrides}"
            )
        bus_node_id = _require_exact_int(
            bus_node.get("node_id"), f"bus.yml node {joint_name} node_id"
        )
        if not 1 <= bus_node_id <= 127:
            raise ValidationError(
                f"bus.yml node {joint_name} node_id must be in [1, 127]"
            )
        bus_operation_mode = _require_exact_int(
            bus_node.get("operation_mode"),
            f"bus.yml node {joint_name} operation_mode",
        )
        if bus_operation_mode != SUPPORTED_MODE:
            raise ValidationError(
                f"bus.yml node {joint_name} operation_mode must be "
                f"{SUPPORTED_MODE}"
            )
        actual = (bus_node_id, bus_operation_mode)
        expected = (node["node_id"], node["operation_mode"])
        if actual != expected:
            raise ValidationError(
                f"bus.yml node_id/operation_mode mismatch for {joint_name}: "
                f"expected={expected}, actual={actual}"
            )

    return descriptor


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", required=True, type=Path)
    parser.add_argument("--bus", required=True, type=Path)
    parser.add_argument(
        "--print-node-bin-names",
        action="store_true",
        help="Print validated dcfgen node bin filenames, one per line",
    )
    args = parser.parse_args()

    try:
        descriptor = validate(args.variant, args.bus)
    except ValidationError as exc:
        print(f"CANopen variant validation failed: {exc}", file=sys.stderr)
        return 1

    if args.print_node_bin_names:
        for node in descriptor["nodes"]:
            print(f"{node['joint_name']}.bin")
    else:
        print(
            f"validated CANopen variant {descriptor['variant']} "
            f"with {len(descriptor['nodes'])} node(s)"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
