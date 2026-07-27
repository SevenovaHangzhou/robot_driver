#!/usr/bin/env python3
"""Semantic migration gate for the frozen EtherCAT configuration."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import yaml
from yaml.nodes import MappingNode, Node, ScalarNode, SequenceNode


LEGACY_CONFIG_PATH = Path(
    "ethercat_driver_ros2/dual_arm_ethercat_control/config"
)
TARGET_PROFILE_PATH = Path(
    "src/rt_control/robot_hw_ethercat/config/slaves"
)
TARGET_LIMITS_PATH = Path(
    "src/rt_control/rt_control_bringup/config/joint_limits.yaml"
)

AXIS_PROFILES = {
    "right_joint1": ("joint1.yaml", "zeroerr_j1.yaml"),
    "right_joint2": ("joint2.yaml", "ti5_j2.yaml"),
    "right_joint3": ("joint3.yaml", "ti5_j3.yaml"),
    "right_joint4": ("joint4.yaml", "zeroerr_j4.yaml"),
    "right_joint5": ("joint5.yaml", "zeroerr_j5.yaml"),
    "right_joint6": ("right_joint6.yaml", "right_j6.yaml"),
    "left_joint1": ("joint1.yaml", "zeroerr_j1.yaml"),
    "left_joint2": ("joint2.yaml", "ti5_j2.yaml"),
    "left_joint3": ("joint3.yaml", "ti5_j3.yaml"),
    "left_joint4": ("joint4.yaml", "zeroerr_j4.yaml"),
    "left_joint5": ("joint5.yaml", "zeroerr_j5.yaml"),
    "left_joint6": ("left_joint6.yaml", "left_j6.yaml"),
    "turn": ("turn.yaml", "turn.yaml"),
}

TI5_CSP_OVERLAY = """\
sdo:
  - {index: 0x6060, sub_index: 0, type: int8, value: 8}
  - {index: 0x60c2, sub_index: 1, type: uint8, value: 4}
  - {index: 0x60c2, sub_index: 2, type: int8, value: -3}
rpdo:
  - index: 0x1601
    channels:
      - {index: 0x6040, sub_index: 0, type: uint16, command_interface: control_word, default: 0}
      - {index: 0x607a, sub_index: 0, type: int32, command_interface: position, factor: 41721.513402, default: .nan}
tpdo:
  - index: 0x1a01
    channels:
      - {index: 0x6041, sub_index: 0, type: uint16, state_interface: status_word}
      - {index: 0x6064, sub_index: 0, type: int32, state_interface: position, factor: 0.0000239684498}
"""
XMC_UPDOWN_SW511_PROFILE = """\
vendor_id: 0x0000034E
product_id: 0x00445566
assign_activate: 0x0300
auto_fault_reset: false
auto_state_transitions: false
sdo:
  - {index: 0x10f1, sub_index: 2, type: uint16, value: 100}
  - {index: 0x60c2, sub_index: 1, type: uint8, value: 4}
  - {index: 0x60c2, sub_index: 2, type: int8, value: -3}
  - {index: 0x6060, sub_index: 0, type: int8, value: 8}
rpdo:
  - index: 0x1600
    channels:
      - {index: 0x6040, sub_index: 0, type: uint16, command_interface: control_word, default: 0}
      - {index: 0x6071, sub_index: 0, type: int16, command_interface: fixed_target_torque, default: 0}
      - {index: 0x60ff, sub_index: 0, type: int32, command_interface: fixed_target_velocity, default: 0}
      - {index: 0x607a, sub_index: 0, type: int32, command_interface: position, factor: 6553600.0, default: .nan}
      - {index: 0x6081, sub_index: 0, type: uint32, command_interface: fixed_profile_velocity, default: 0}
      - {index: 0x6060, sub_index: 0, type: int8, command_interface: mode_of_operation, default: 8}
      - {index: 0x2302, sub_index: 0, type: uint16, command_interface: fixed_vendor_control, default: 0}
tpdo:
  - index: 0x1a00
    channels:
      - {index: 0x6041, sub_index: 0, type: uint16, state_interface: status_word}
      - {index: 0x603f, sub_index: 0, type: uint16}
      - {index: 0x6078, sub_index: 0, type: int16}
      - {index: 0x606c, sub_index: 0, type: int32}
      - {index: 0x6064, sub_index: 0, type: int32, state_interface: position, factor: 0.000000152587890625}
      - {index: 0x6061, sub_index: 0, type: int8}
      - {index: 0x6000, sub_index: 0, type: uint8}
      - {index: 0x2300, sub_index: 0, type: uint16}
      - {index: 0x2301, sub_index: 0, type: uint16}
      - {index: 0x60fd, sub_index: 0, type: uint32}
"""
XMC_UPDOWN_LIMITS = """\
joints:
  updown:
    profile: xmc_updown_sw511
    lower_position_m: 0.0
    upper_position_m: 0.8
    max_velocity_m_s: 0.3
    max_acceleration_m_s2: 0.5
    max_deceleration_m_s2: 0.5
    counts_per_rev: 65536
    screw_lead_m_per_rev: 0.01
    reduction_ratio: 1.0
    counts_per_m: 6553600.0
    m_per_count: 0.000000152587890625
    direction: 1
    zero_offset_m: 0.0
    verification:
      source: USER_AND_SUPPLIER_CONFIRMED_XMC_SW_5_11
      direction: RAW_6064_INCREASES_UPWARD
      zero_offset: RAW_0_IS_0_M
"""
SDO_KEY = re.compile(r"^sdo\[(\d+)\](.*)$")


class GateError(RuntimeError):
    """Raised when the migration gate cannot perform a trustworthy comparison."""


@dataclass(frozen=True)
class Scalar:
    """A YAML scalar retaining both its type and source spelling."""

    kind: str
    value: object
    text: str

    def display(self) -> str:
        return f"{self.text} ({self.kind})"


@dataclass(frozen=True)
class Difference:
    axis: str
    key: str
    expected: Scalar | None
    actual: Scalar | None


SemanticMap = dict[str, Scalar]


def parse_integer(text: str) -> int:
    normalized = text.replace("_", "")
    sign = -1 if normalized.startswith("-") else 1
    unsigned = normalized[1:] if normalized[:1] in {"-", "+"} else normalized
    if unsigned.lower().startswith(("0x", "0o", "0b")):
        return sign * int(unsigned, 0)
    return int(normalized, 10)


def scalar_from_node(node: ScalarNode) -> Scalar:
    tag = node.tag.rsplit(":", maxsplit=1)[-1]
    if tag == "int":
        return Scalar("int", parse_integer(node.value), node.value)
    if tag == "float":
        return Scalar("float", node.value, node.value)
    if tag == "bool":
        value = node.value.lower() in {"true", "yes", "on"}
        return Scalar("bool", value, node.value)
    if tag == "null":
        return Scalar("null", None, node.value)
    return Scalar("string", node.value, node.value)


def mapping_key(node: Node) -> str:
    if not isinstance(node, ScalarNode):
        raise GateError("Only scalar YAML mapping keys are supported")
    return node.value


def flatten_node(node: Node, prefix: str = "") -> SemanticMap:
    if isinstance(node, ScalarNode):
        if not prefix:
            raise GateError("A scalar cannot be used as the configuration root")
        return {prefix: scalar_from_node(node)}

    flattened: SemanticMap = {}
    if isinstance(node, MappingNode):
        for key_node, value_node in node.value:
            key = mapping_key(key_node)
            child_prefix = f"{prefix}.{key}" if prefix else key
            flattened.update(flatten_node(value_node, child_prefix))
        return flattened

    if isinstance(node, SequenceNode):
        for index, value_node in enumerate(node.value):
            child_prefix = f"{prefix}[{index}]"
            flattened.update(flatten_node(value_node, child_prefix))
        return flattened

    raise GateError(f"Unsupported YAML node type: {type(node).__name__}")


def compose_yaml(content: str, source: str) -> SemanticMap:
    try:
        node = yaml.compose(content)
    except yaml.YAMLError as error:
        raise GateError(f"Invalid YAML in {source}: {error}") from error
    if node is None:
        raise GateError(f"Empty YAML document: {source}")
    return flatten_node(node)


def load_yaml(path: Path) -> SemanticMap:
    try:
        content = path.read_text(encoding="utf-8")
    except OSError as error:
        raise GateError(f"Cannot read {path}: {error}") from error
    return compose_yaml(content, str(path))


def equivalent(expected: Scalar, actual: Scalar) -> bool:
    if expected.kind != actual.kind:
        return False
    if expected.kind == "float":
        return expected.text == actual.text
    return expected.value == actual.value


def compare_maps(axis: str, expected: SemanticMap, actual: SemanticMap) -> list[Difference]:
    differences: list[Difference] = []
    for key in sorted(expected.keys() | actual.keys()):
        expected_value = expected.get(key)
        actual_value = actual.get(key)
        if expected_value is None or actual_value is None:
            differences.append(Difference(axis, key, expected_value, actual_value))
        elif not equivalent(expected_value, actual_value):
            differences.append(Difference(axis, key, expected_value, actual_value))
    return differences


def prepend_sync_tolerance(profile: SemanticMap, value_type: str) -> SemanticMap:
    """Prepend the BQ-114-approved 0x10F1:02=100 startup SDO."""
    expected: SemanticMap = {}
    for key, value in profile.items():
        match = SDO_KEY.match(key)
        if match:
            shifted_index = int(match.group(1)) + 1
            expected[f"sdo[{shifted_index}]{match.group(2)}"] = value
        else:
            expected[key] = value
    expected.update(
        {
            "sdo[0].index": Scalar("int", 0x10F1, "0x10f1"),
            "sdo[0].sub_index": Scalar("int", 2, "2"),
            "sdo[0].type": Scalar("string", value_type, value_type),
            "sdo[0].value": Scalar("int", 100, "100"),
        }
    )
    return expected


def apply_frozen_overlay(legacy: SemanticMap, ti5: bool) -> SemanticMap:
    expected = dict(legacy)
    expected["auto_state_transitions"] = Scalar("bool", False, "false")
    if ti5:
        for key in list(expected):
            if key.startswith(("sdo", "rpdo", "tpdo")):
                del expected[key]
        expected.update(compose_yaml(TI5_CSP_OVERLAY, "built-in frozen Ti5 CSP overlay"))
    return prepend_sync_tolerance(expected, "uint32" if ti5 else "uint16")


def resolve_legacy_config(root: Path) -> Path:
    candidates = (root / LEGACY_CONFIG_PATH, root)
    for candidate in candidates:
        if (candidate / "icube_profiles").is_dir() and (candidate / "joint_limits.yaml").is_file():
            return candidate
    raise GateError(
        f"Legacy root {root} does not contain {LEGACY_CONFIG_PATH} or a config directory"
    )


def resolve_target_paths(root: Path) -> tuple[Path, Path]:
    profile_candidates = (root / TARGET_PROFILE_PATH, root / "config/slaves")
    profiles = next((path for path in profile_candidates if path.is_dir()), None)
    limits = root / TARGET_LIMITS_PATH
    if profiles is None:
        raise GateError(f"Target root {root} does not contain {TARGET_PROFILE_PATH}")
    if not limits.is_file():
        raise GateError(f"Target root {root} does not contain {TARGET_LIMITS_PATH}")
    return profiles, limits


def compare_configuration(legacy_root: Path, target_root: Path) -> tuple[list[Difference], int]:
    legacy_config = resolve_legacy_config(legacy_root)
    target_profiles, target_limits = resolve_target_paths(target_root)
    differences: list[Difference] = []
    compared_values = 0

    for axis, (legacy_name, target_name) in AXIS_PROFILES.items():
        legacy = load_yaml(legacy_config / "icube_profiles" / legacy_name)
        expected = apply_frozen_overlay(legacy, ti5=legacy_name in {"joint2.yaml", "joint3.yaml"})
        actual = load_yaml(target_profiles / target_name)
        differences.extend(compare_maps(axis, expected, actual))
        compared_values += len(expected)

    expected_updown = compose_yaml(
        XMC_UPDOWN_SW511_PROFILE, "built-in BQ-118 XMC SW5.11 profile"
    )
    actual_updown = load_yaml(target_profiles / "xmc_updown_sw511.yaml")
    differences.extend(compare_maps("updown", expected_updown, actual_updown))
    compared_values += len(expected_updown)

    legacy_limits = load_yaml(legacy_config / "joint_limits.yaml")
    approved_updown_limits = compose_yaml(
        XMC_UPDOWN_LIMITS, "built-in BQ-118 XMC Updown limits"
    )
    expected_limits = dict(legacy_limits)
    expected_limits.update(approved_updown_limits)
    migrated_limits = load_yaml(target_limits)
    differences.extend(compare_maps("joint_limits", expected_limits, migrated_limits))
    compared_values += len(expected_limits)
    return differences, compared_values


def render_value(value: Scalar | None) -> str:
    return "<missing>" if value is None else value.display()


def print_differences(differences: Iterable[Difference]) -> None:
    print("axis | semantic_key | expected | target")
    print("--- | --- | --- | ---")
    for difference in differences:
        print(
            f"{difference.axis} | {difference.key} | "
            f"{render_value(difference.expected)} | {render_value(difference.actual)}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare the frozen robot_driver EtherCAT YAML with the migrated configuration, "
            "including only the approved CSP/enable-manager, BQ-114 sync-tolerance, and "
            "BQ-118 XMC SW5.11 overlays."
        )
    )
    parser.add_argument(
        "--legacy-root",
        type=Path,
        required=True,
        help="Read-only robot_driver@6bc94cd checkout or its dual-arm config directory",
    )
    parser.add_argument(
        "--target-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Target robot repository root (default: repository containing this script)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        differences, compared_values = compare_configuration(
            args.legacy_root.resolve(), args.target_root.resolve()
        )
    except GateError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    if differences:
        print_differences(differences)
        print(f"FAIL: {len(differences)} semantic difference(s)", file=sys.stderr)
        return 1

    print(
        "PASS 100%: 14 logical-axis profiles and joint_limits.yaml match "
        f"({compared_values} semantic values)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
