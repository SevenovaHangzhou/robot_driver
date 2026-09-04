#!/usr/bin/env python3
"""Validate one DynamicJointState snapshot against enable-manager policy."""

from __future__ import annotations

import argparse
import math
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class AxisStateError(ValueError):
    """The snapshot does not satisfy the requested CiA 402 state contract."""


@dataclass(frozen=True)
class AxisStatePolicy:
    """Frozen managed-joint and disabled-terminal policy for one check."""

    axes: tuple[str, ...]
    ready_to_switch_on_disable_terminal_axes: frozenset[str]

    def __post_init__(self) -> None:
        if not self.axes or len(set(self.axes)) != len(self.axes):
            raise AxisStateError("managed_joints must be nonempty and unique")
        if any(not isinstance(axis, str) or not axis for axis in self.axes):
            raise AxisStateError("managed_joints must contain nonempty strings")
        unknown = self.ready_to_switch_on_disable_terminal_axes - set(self.axes)
        if unknown:
            raise AxisStateError(
                "ready-to-switch-on terminal joints are not managed: "
                + ", ".join(sorted(unknown))
            )


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe loader that rejects ambiguous duplicate mapping keys."""


def _construct_unique_mapping(
    loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as error:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                "mapping keys must be scalar values",
                key_node.start_mark,
            ) from error
        if duplicate:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"duplicate mapping key {key!r}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def load_axis_state_policy(path: str | Path) -> AxisStatePolicy:
    """Load only the policy fields owned by enable_manager."""

    config_path = Path(path)
    try:
        document = yaml.load(config_path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise AxisStateError(f"could not load controller config {config_path}: {error}") from error
    if not isinstance(document, Mapping):
        raise AxisStateError("controller config root is not a mapping")
    enable_manager = document.get("enable_manager")
    if not isinstance(enable_manager, Mapping):
        raise AxisStateError("controller config is missing enable_manager")
    parameters = enable_manager.get("ros__parameters")
    if not isinstance(parameters, Mapping):
        raise AxisStateError("enable_manager is missing ros__parameters")
    axes = parameters.get("managed_joints")
    ready_axes = parameters.get("ready_to_switch_on_disable_terminal_joints")
    if not isinstance(axes, list) or not all(isinstance(axis, str) for axis in axes):
        raise AxisStateError("enable_manager.managed_joints must be a string list")
    if not isinstance(ready_axes, list) or not all(
        isinstance(axis, str) for axis in ready_axes
    ):
        raise AxisStateError(
            "enable_manager.ready_to_switch_on_disable_terminal_joints must be a string list"
        )
    if len(set(ready_axes)) != len(ready_axes):
        raise AxisStateError(
            "ready_to_switch_on_disable_terminal_joints must be unique"
        )
    return AxisStatePolicy(tuple(axes), frozenset(ready_axes))


def _extract_status_words(document: object, axes: tuple[str, ...]) -> dict[str, int]:
    if not isinstance(document, Mapping):
        raise AxisStateError("DynamicJointState snapshot is not a mapping")

    joint_names = document.get("joint_names")
    interface_values = document.get("interface_values")
    if not isinstance(joint_names, list) or not isinstance(interface_values, list):
        raise AxisStateError("DynamicJointState is missing joint_names/interface_values lists")
    if len(joint_names) != len(interface_values):
        raise AxisStateError("DynamicJointState joint/interface array lengths differ")

    records: dict[str, object] = {}
    for joint, interfaces in zip(joint_names, interface_values):
        if not isinstance(joint, str) or joint in records:
            raise AxisStateError(f"invalid or duplicate joint name: {joint!r}")
        records[joint] = interfaces

    observed: dict[str, int] = {}
    for joint in axes:
        interfaces = records.get(joint)
        if not isinstance(interfaces, Mapping):
            raise AxisStateError(f"{joint}: missing joint interface record")
        names = interfaces.get("interface_names")
        values = interfaces.get("values")
        if not isinstance(names, list) or not isinstance(values, list):
            raise AxisStateError(f"{joint}: invalid interface_names/values")
        try:
            status_index = names.index("status_word")
        except ValueError as error:
            raise AxisStateError(f"{joint}: missing status_word") from error
        if status_index >= len(values):
            raise AxisStateError(f"{joint}: missing status_word value")

        value = values[status_index]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise AxisStateError(f"{joint}: status_word is not numeric")
        numeric = float(value)
        if not math.isfinite(numeric) or not numeric.is_integer():
            raise AxisStateError(f"{joint}: status_word is not an integer: {value!r}")
        status_word = int(numeric)
        if not 0 <= status_word <= 0xFFFF:
            raise AxisStateError(f"{joint}: status_word is outside uint16: {status_word}")
        observed[joint] = status_word
    return observed


def check_axis_states(
    document: object, expected: str, policy: AxisStatePolicy
) -> dict[str, int]:
    """Return all raw words, or raise on the first contract mismatch."""

    if expected not in {"disabled", "enabled"}:
        raise AxisStateError(f"unsupported expected state: {expected}")
    observed = _extract_status_words(document, policy.axes)

    for joint, status_word in observed.items():
        if expected == "enabled":
            if status_word & 0x006F != 0x0027:
                raise AxisStateError(
                    f"{joint}: raw=0x{status_word:04X}, expected=OperationEnabled(0x0027)"
                )
            continue

        switch_on_disabled = status_word & 0x004F == 0x0040
        ti5_ready_exception = (
            joint in policy.ready_to_switch_on_disable_terminal_axes
            and status_word & 0x006F == 0x0021
        )
        if not switch_on_disabled and not ti5_ready_exception:
            expected_state = "SwitchOnDisabled(0x0040)"
            if joint in policy.ready_to_switch_on_disable_terminal_axes:
                expected_state += " or configured ReadyToSwitchOn(0x0021)"
            raise AxisStateError(
                f"{joint}: raw=0x{status_word:04X}, expected={expected_state}"
            )
    return observed


def _load_first_document(stream: str) -> object:
    yaml_start = stream.find("---")
    if yaml_start > 0:
        stream = stream[yaml_start:]
    try:
        documents = yaml.safe_load_all(stream)
        return next(document for document in documents if document is not None)
    except (StopIteration, yaml.YAMLError) as error:
        raise AxisStateError("no valid DynamicJointState YAML document received") from error


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected", choices=("disabled", "enabled"), required=True)
    parser.add_argument("--controller-config", type=Path, required=True)
    arguments = parser.parse_args()
    try:
        policy = load_axis_state_policy(arguments.controller_config)
        observed = check_axis_states(
            _load_first_document(sys.stdin.read()), arguments.expected, policy
        )
    except AxisStateError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1

    print(f"PASS: {len(observed)} axes satisfy the {arguments.expected} CiA 402 contract")
    for joint, status_word in observed.items():
        print(f"  {joint}: 0x{status_word:04X}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
