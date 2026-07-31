#!/usr/bin/env python3
"""Validate one DynamicJointState snapshot for the current 14-axis IPC."""

from __future__ import annotations

import argparse
import math
import sys
from collections.abc import Mapping

import yaml


AXES = (
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "turn",
    "updown",
)
TI5_DISABLED_EXCEPTION_AXES = frozenset(
    {"right_joint2", "right_joint3", "left_joint2", "left_joint3"}
)


class AxisStateError(ValueError):
    """The snapshot does not satisfy the requested CiA 402 state contract."""


def _extract_status_words(document: object) -> dict[str, int]:
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
    for joint in AXES:
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


def check_axis_states(document: object, expected: str) -> dict[str, int]:
    """Return all raw words, or raise on the first contract mismatch."""

    if expected not in {"disabled", "enabled"}:
        raise AxisStateError(f"unsupported expected state: {expected}")
    observed = _extract_status_words(document)

    for joint, status_word in observed.items():
        if expected == "enabled":
            if status_word & 0x006F != 0x0027:
                raise AxisStateError(
                    f"{joint}: raw=0x{status_word:04X}, expected=OperationEnabled(0x0027)"
                )
            continue

        switch_on_disabled = status_word & 0x004F == 0x0040
        ti5_ready_exception = (
            joint in TI5_DISABLED_EXCEPTION_AXES
            and status_word & 0x006F == 0x0021
        )
        if not switch_on_disabled and not ti5_ready_exception:
            expected_state = "SwitchOnDisabled(0x0040)"
            if joint in TI5_DISABLED_EXCEPTION_AXES:
                expected_state += " or frozen Ti5 ReadyToSwitchOn(0x0021)"
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
    arguments = parser.parse_args()
    try:
        observed = check_axis_states(_load_first_document(sys.stdin.read()), arguments.expected)
    except AxisStateError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1

    print(f"PASS: {len(observed)} axes satisfy the {arguments.expected} CiA 402 contract")
    for joint, status_word in observed.items():
        print(f"  {joint}: 0x{status_word:04X}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
