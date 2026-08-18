#!/usr/bin/env python3
"""Validate the versioned RT-Control driver-variant manifest.

The manifest is an offline/on-configure authority.  This module never talks to
ROS, a field bus, or hardware, and it is safe to run in repository quality
gates.
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    ROOT
    / "src"
    / "rt_control"
    / "rt_control_bringup"
    / "config"
    / "driver_variant.yaml"
)

TOP_LEVEL_KEYS = {
    "schema_version",
    "variant_id",
    "buses",
    "drive_profiles",
    "transmissions",
    "actuators",
    "joints",
    "enable",
}
PROFILE_BUSES = {"ethercat", "canopen"}
RING_ROLES = {"hub", "drive"}
CONTROLLER_ROLES = {"whole_body_trajectory", "left_track", "right_track"}
CONVERSION_INTERFACES = {"position", "velocity", "effort"}
PROFILE_INTERFACES = CONVERSION_INTERFACES | {
    "control_word",
    "status_word",
    "digital_outputs",
    "digital_inputs",
}
COMMAND_PROFILE_INTERFACES = CONVERSION_INTERFACES | {
    "control_word",
    "digital_outputs",
}
STATE_PROFILE_INTERFACES = CONVERSION_INTERFACES | {
    "status_word",
    "digital_inputs",
}
DISABLE_TERMINALS = {"switch_on_disabled", "ready_to_switch_on"}
SI_UNITS = {"rad", "m"}
MAX_ENABLE_AXES = 128
MAX_ENABLE_BATCHES = 128
MAX_ENABLE_BATCH_SIZE = 3
ROLE_PROFILE_REQUIREMENTS = {
    "whole_body_trajectory": (
        "ethercat",
        frozenset({"position"}),
        frozenset({"position"}),
    ),
    "left_track": (
        "canopen",
        frozenset({"velocity"}),
        frozenset({"position", "velocity"}),
    ),
    "right_track": (
        "canopen",
        frozenset({"velocity"}),
        frozenset({"position", "velocity"}),
    ),
}


class DriverVariantError(ValueError):
    """Raised when a driver-variant manifest is malformed or inconsistent."""

    def __init__(self, findings: str | Iterable[str]):
        if isinstance(findings, str):
            self.findings = (findings,)
        else:
            self.findings = tuple(findings)
        super().__init__("invalid driver variant manifest:\n- " + "\n- ".join(self.findings))


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects silently overwritten mapping keys."""


def _construct_unique_mapping(
    loader: _UniqueKeyLoader, node: yaml.nodes.MappingNode, deep: bool = False
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    result: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in result
        except TypeError as exc:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                "YAML mapping keys must be hashable",
                key_node.start_mark,
            ) from exc
        if duplicate:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"duplicate YAML key '{key}'",
                key_node.start_mark,
            )
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


def parse_yaml_document(text: str, source: str = "YAML document") -> object:
    """Parse YAML without permitting duplicate mapping keys."""
    try:
        return yaml.load(text, Loader=_UniqueKeyLoader)
    except yaml.YAMLError as exc:
        raise DriverVariantError(f"{source}: invalid YAML: {exc}") from exc


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _mapping(value: object, path: str, findings: list[str]) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        findings.append(f"{path}: must be a mapping")
        return {}
    return value


def _sequence(value: object, path: str, findings: list[str]) -> Sequence[Any]:
    if not isinstance(value, list):
        findings.append(f"{path}: must be a list")
        return []
    return value


def _check_keys(
    document: Mapping[str, Any],
    path: str,
    required: Iterable[str],
    optional: Iterable[str],
    findings: list[str],
) -> None:
    required_keys = set(required)
    allowed_keys = required_keys | set(optional)
    document_keys = set(document)
    for key in document_keys:
        if not isinstance(key, str):
            findings.append(f"{path}: field names must be strings, got {key!r}")
    string_keys = {key for key in document_keys if isinstance(key, str)}
    for key in sorted(required_keys - string_keys):
        findings.append(f"{path}: required field '{key}' is missing")
    for key in sorted(string_keys - allowed_keys):
        findings.append(f"{path}: unexpected field '{key}'")


def _nonempty_string(value: object, path: str, findings: list[str]) -> str | None:
    if not isinstance(value, str) or not value.strip():
        findings.append(f"{path}: must be a non-empty string")
        return None
    return value


def _string_list(
    value: object,
    path: str,
    findings: list[str],
    *,
    allow_empty: bool = False,
) -> list[str]:
    values = _sequence(value, path, findings)
    result: list[str] = []
    for index, item in enumerate(values):
        text = _nonempty_string(item, f"{path}[{index}]", findings)
        if text is not None:
            result.append(text)
    if not allow_empty and not result:
        findings.append(f"{path}: must contain at least one item")
    duplicates = sorted(name for name, count in Counter(result).items() if count > 1)
    for duplicate in duplicates:
        findings.append(f"{path}: duplicate item '{duplicate}'")
    return result


def _duplicate_ids(
    entries: Sequence[Any],
    path: str,
    label: str,
    findings: list[str],
) -> tuple[list[Mapping[str, Any]], set[str]]:
    mappings: list[Mapping[str, Any]] = []
    identifiers: list[str] = []
    for index, entry in enumerate(entries):
        item = _mapping(entry, f"{path}[{index}]", findings)
        mappings.append(item)
        identifier = item.get("id")
        if isinstance(identifier, str) and identifier:
            identifiers.append(identifier)
    for identifier, count in Counter(identifiers).items():
        if count > 1:
            findings.append(f"{path}: duplicate {label} '{identifier}'")
    return mappings, set(identifiers)


def _validate_buses(
    document: Mapping[str, Any], findings: list[str]
) -> tuple[dict[str, list[str]], set[int], set[int]]:
    buses = _mapping(document.get("buses"), "buses", findings)
    _check_keys(buses, "buses", {"ethercat", "canopen"}, set(), findings)

    actuator_addresses: dict[str, list[str]] = {}
    ring_positions: list[int] = []
    canopen_node_ids: list[int] = []

    ethercat = _mapping(buses.get("ethercat"), "buses.ethercat", findings)
    _check_keys(ethercat, "buses.ethercat", {"master_id", "ring"}, set(), findings)
    master_id = ethercat.get("master_id")
    if not _is_int(master_id) or int(master_id) < 0:
        findings.append("buses.ethercat.master_id: must be a non-negative integer")
    ring = _sequence(ethercat.get("ring"), "buses.ethercat.ring", findings)
    for index, raw_entry in enumerate(ring):
        path = f"buses.ethercat.ring[{index}]"
        entry = _mapping(raw_entry, path, findings)
        _check_keys(entry, path, {"position", "role"}, {"actuator"}, findings)
        position = entry.get("position")
        if not _is_int(position) or int(position) < 0:
            findings.append(f"{path}.position: must be a non-negative integer")
        else:
            ring_positions.append(int(position))
        role = entry.get("role")
        if role not in RING_ROLES:
            findings.append(f"{path}.role: must be one of {sorted(RING_ROLES)}")
        actuator = entry.get("actuator")
        if role == "drive":
            actuator_name = _nonempty_string(actuator, f"{path}.actuator", findings)
            if actuator_name is not None and _is_int(position):
                actuator_addresses.setdefault(actuator_name, []).append(
                    f"ethercat:{position}"
                )
        elif actuator is not None:
            findings.append(f"{path}: hub entries must not name an actuator")

    for position, count in Counter(ring_positions).items():
        if count > 1:
            findings.append(
                f"buses.ethercat.ring: duplicate EtherCAT ring position {position}"
            )
    if ring_positions:
        expected_positions = set(range(max(ring_positions) + 1))
        missing_positions = sorted(expected_positions - set(ring_positions))
        if missing_positions:
            findings.append(
                "buses.ethercat.ring: full physical ring must include positions "
                + ", ".join(str(position) for position in missing_positions)
            )

    canopen = _mapping(buses.get("canopen"), "buses.canopen", findings)
    _check_keys(
        canopen,
        "buses.canopen",
        {"interface", "master_node_id", "nodes"},
        set(),
        findings,
    )
    _nonempty_string(canopen.get("interface"), "buses.canopen.interface", findings)
    master_node_id = canopen.get("master_node_id")
    if not _is_int(master_node_id) or not 1 <= int(master_node_id) <= 127:
        findings.append(
            "buses.canopen.master_node_id: must be an integer from 1 to 127"
        )
    nodes = _sequence(canopen.get("nodes"), "buses.canopen.nodes", findings)
    for index, raw_entry in enumerate(nodes):
        path = f"buses.canopen.nodes[{index}]"
        entry = _mapping(raw_entry, path, findings)
        _check_keys(
            entry,
            path,
            {"node_id", "actuator", "operation_mode"},
            set(),
            findings,
        )
        node_id = entry.get("node_id")
        if not _is_int(node_id) or not 1 <= int(node_id) <= 127:
            findings.append(f"{path}.node_id: must be an integer from 1 to 127")
        else:
            canopen_node_ids.append(int(node_id))
        operation_mode = entry.get("operation_mode")
        if not _is_int(operation_mode) or int(operation_mode) != 3:
            findings.append(
                f"{path}.operation_mode: must be 3 (CiA 402 Profiled Velocity) "
                "in phase 1"
            )
        actuator_name = _nonempty_string(entry.get("actuator"), f"{path}.actuator", findings)
        if actuator_name is not None and _is_int(node_id):
            actuator_addresses.setdefault(actuator_name, []).append(f"canopen:{node_id}")

    for node_id, count in Counter(canopen_node_ids).items():
        if count > 1:
            findings.append(f"buses.canopen.nodes: duplicate CANopen node_id {node_id}")
    if _is_int(master_node_id) and int(master_node_id) in canopen_node_ids:
        findings.append(
            "buses.canopen.master_node_id must not reuse a drive node_id "
            f"({master_node_id})"
        )
    return actuator_addresses, set(ring_positions), set(canopen_node_ids)


def _validate_profiles(
    document: Mapping[str, Any],
    findings: list[str],
    repository_root: Path | None,
) -> tuple[set[str], dict[str, str], dict[str, set[str]], dict[str, set[str]]]:
    entries = _sequence(document.get("drive_profiles"), "drive_profiles", findings)
    profiles, identifiers = _duplicate_ids(
        entries, "drive_profiles", "drive profile", findings
    )
    profile_buses: dict[str, str] = {}
    profile_command_interfaces: dict[str, set[str]] = {}
    profile_state_interfaces: dict[str, set[str]] = {}
    for index, profile in enumerate(profiles):
        path = f"drive_profiles[{index}]"
        _check_keys(
            profile,
            path,
            {
                "id",
                "family",
                "bus",
                "config",
                "command_interfaces",
                "state_interfaces",
            },
            set(),
            findings,
        )
        identifier = _nonempty_string(profile.get("id"), f"{path}.id", findings)
        _nonempty_string(profile.get("family"), f"{path}.family", findings)
        bus = profile.get("bus")
        if bus not in PROFILE_BUSES:
            findings.append(f"{path}.bus: must be one of {sorted(PROFILE_BUSES)}")
        elif identifier is not None:
            profile_buses[identifier] = str(bus)
        config = _nonempty_string(profile.get("config"), f"{path}.config", findings)
        if config is not None:
            config_path = Path(config)
            if config_path.is_absolute() or ".." in config_path.parts:
                findings.append(f"{path}.config: must be a repository-relative path")
            else:
                if bus == "ethercat" and identifier is not None:
                    expected_config = (
                        "src/rt_control/robot_hw_ethercat/config/slaves/"
                        f"{identifier}.yaml"
                    )
                    if config != expected_config:
                        findings.append(
                            f"{path}.config: must equal runtime profile path "
                            f"'{expected_config}'"
                        )
                if repository_root is not None:
                    resolved_root = repository_root.resolve()
                    resolved_config = (resolved_root / config_path).resolve()
                    try:
                        resolved_config.relative_to(resolved_root)
                    except ValueError:
                        findings.append(
                            f"{path}.config: resolves outside repository root: {config}"
                        )
                    else:
                        if not resolved_config.is_file():
                            findings.append(
                                f"{path}.config: file does not exist: {config}"
                            )
        command_interfaces = _string_list(
            profile.get("command_interfaces"), f"{path}.command_interfaces", findings
        )
        state_interfaces = _string_list(
            profile.get("state_interfaces"), f"{path}.state_interfaces", findings
        )
        for interface in command_interfaces:
            if interface not in PROFILE_INTERFACES:
                findings.append(f"{path}: unexpected interface '{interface}'")
            elif interface not in COMMAND_PROFILE_INTERFACES:
                findings.append(
                    f"{path}.command_interfaces: '{interface}' is not a command "
                    "interface"
                )
        for interface in state_interfaces:
            if interface not in PROFILE_INTERFACES:
                findings.append(f"{path}: unexpected interface '{interface}'")
            elif interface not in STATE_PROFILE_INTERFACES:
                findings.append(
                    f"{path}.state_interfaces: '{interface}' is not a state interface"
                )
        if identifier is not None:
            profile_command_interfaces[identifier] = set(command_interfaces)
            profile_state_interfaces[identifier] = set(state_interfaces)
    return (
        identifiers,
        profile_buses,
        profile_command_interfaces,
        profile_state_interfaces,
    )


def _validate_transmissions(
    document: Mapping[str, Any], findings: list[str]
) -> tuple[set[str], dict[str, set[str]], dict[str, str]]:
    entries = _sequence(document.get("transmissions"), "transmissions", findings)
    transmissions, identifiers = _duplicate_ids(
        entries, "transmissions", "transmission", findings
    )
    transmission_interfaces: dict[str, set[str]] = {}
    transmission_units: dict[str, str] = {}
    for index, transmission in enumerate(transmissions):
        path = f"transmissions[{index}]"
        _check_keys(
            transmission,
            path,
            {
                "id",
                "si_unit",
                "effective_conversion",
                "reciprocity_rel_tol",
            },
            set(),
            findings,
        )
        identifier = _nonempty_string(transmission.get("id"), f"{path}.id", findings)
        si_unit = transmission.get("si_unit")
        if si_unit not in SI_UNITS:
            findings.append(f"{path}.si_unit: must be one of {sorted(SI_UNITS)}")
        elif identifier is not None:
            transmission_units[identifier] = str(si_unit)
        tolerance = transmission.get("reciprocity_rel_tol")
        tolerance_value: float | None = None
        if not _is_number(tolerance) or not 0.0 < float(tolerance) <= 1.0e-6:
            findings.append(
                f"{path}.reciprocity_rel_tol: required value must be in (0, 1e-6]"
            )
        else:
            tolerance_value = float(tolerance)

        conversions = _mapping(
            transmission.get("effective_conversion"),
            f"{path}.effective_conversion",
            findings,
        )
        if not conversions:
            findings.append(f"{path}.effective_conversion: must not be empty")
        if identifier is not None:
            transmission_interfaces[identifier] = {
                interface
                for interface in conversions
                if isinstance(interface, str) and interface in CONVERSION_INTERFACES
            }
        for interface, raw_conversion in conversions.items():
            conversion_path = f"{path}.effective_conversion.{interface}"
            if interface not in CONVERSION_INTERFACES:
                findings.append(
                    f"{path}.effective_conversion: unexpected interface '{interface}'"
                )
            conversion = _mapping(raw_conversion, conversion_path, findings)
            _check_keys(
                conversion,
                conversion_path,
                {"to_device", "from_device"},
                set(),
                findings,
            )
            to_device = conversion.get("to_device")
            from_device = conversion.get("from_device")
            if not _is_number(to_device) or float(to_device) == 0.0:
                findings.append(f"{conversion_path}.to_device: must be finite and non-zero")
            if not _is_number(from_device) or float(from_device) == 0.0:
                findings.append(
                    f"{conversion_path}.from_device: must be finite and non-zero"
                )
            if (
                identifier is not None
                and tolerance_value is not None
                and _is_number(to_device)
                and _is_number(from_device)
                and not math.isclose(
                    float(to_device) * float(from_device),
                    1.0,
                    rel_tol=tolerance_value,
                    abs_tol=0.0,
                )
            ):
                findings.append(
                    f"transmission '{identifier}' {interface} factors are not reciprocal "
                    f"within {tolerance_value:g}"
                )
    return identifiers, transmission_interfaces, transmission_units


def _validate_actuators(
    document: Mapping[str, Any],
    findings: list[str],
    profile_ids: set[str],
    transmission_ids: set[str],
    profile_buses: Mapping[str, str],
    profile_command_interfaces: Mapping[str, set[str]],
    profile_state_interfaces: Mapping[str, set[str]],
    transmission_interfaces: Mapping[str, set[str]],
    actuator_addresses: Mapping[str, list[str]],
) -> tuple[set[str], dict[str, str], dict[str, str]]:
    entries = _sequence(document.get("actuators"), "actuators", findings)
    actuators, identifiers = _duplicate_ids(entries, "actuators", "actuator", findings)
    used_profiles: set[str] = set()
    used_transmissions: set[str] = set()
    actuator_profiles: dict[str, str] = {}
    actuator_transmissions: dict[str, str] = {}
    for index, actuator in enumerate(actuators):
        path = f"actuators[{index}]"
        _check_keys(
            actuator,
            path,
            {"id", "drive_profile", "transmission"},
            set(),
            findings,
        )
        identifier = _nonempty_string(actuator.get("id"), f"{path}.id", findings)
        drive_profile = _nonempty_string(
            actuator.get("drive_profile"), f"{path}.drive_profile", findings
        )
        transmission = _nonempty_string(
            actuator.get("transmission"), f"{path}.transmission", findings
        )
        if drive_profile is not None:
            used_profiles.add(drive_profile)
            if identifier is not None:
                actuator_profiles[identifier] = drive_profile
            if drive_profile not in profile_ids:
                findings.append(
                    f"actuator '{identifier}' references unknown drive profile '{drive_profile}'"
                )
        if transmission is not None:
            used_transmissions.add(transmission)
            if identifier is not None:
                actuator_transmissions[identifier] = transmission
            if transmission not in transmission_ids:
                findings.append(
                    f"actuator '{identifier}' references unknown transmission '{transmission}'"
                )
        if (
            identifier is not None
            and drive_profile in profile_command_interfaces
            and drive_profile in profile_state_interfaces
            and transmission in transmission_interfaces
        ):
            required_interfaces = (
                profile_command_interfaces[drive_profile]
                | profile_state_interfaces[drive_profile]
            ) & CONVERSION_INTERFACES
            configured_interfaces = transmission_interfaces[transmission]
            for interface in sorted(required_interfaces - configured_interfaces):
                findings.append(
                    f"actuator '{identifier}' transmission '{transmission}' is missing "
                    f"conversion for '{interface}'"
                )
            for interface in sorted(configured_interfaces - required_interfaces):
                findings.append(
                    f"actuator '{identifier}' transmission '{transmission}' has unused "
                    f"conversion for '{interface}'"
                )
        if identifier is not None:
            addresses = actuator_addresses.get(identifier, [])
            if not addresses:
                findings.append(f"actuator '{identifier}' has no bus address")
            elif len(addresses) > 1:
                findings.append(
                    f"actuator '{identifier}' maps to multiple bus addresses: {addresses}"
                )
            elif drive_profile in profile_buses:
                address_bus = addresses[0].split(":", 1)[0]
                if profile_buses[drive_profile] != address_bus:
                    findings.append(
                        f"actuator '{identifier}' bus '{address_bus}' does not match "
                        f"drive profile '{drive_profile}' bus '{profile_buses[drive_profile]}'"
                    )

    for address_actuator in sorted(set(actuator_addresses) - identifiers):
        findings.append(f"bus address references unknown actuator '{address_actuator}'")
    for unused in sorted(profile_ids - used_profiles):
        findings.append(f"drive profile '{unused}' is not referenced by an actuator")
    for unused in sorted(transmission_ids - used_transmissions):
        findings.append(f"transmission '{unused}' is not referenced by an actuator")
    return identifiers, actuator_profiles, actuator_transmissions


def _validate_enable(
    document: Mapping[str, Any],
    findings: list[str],
    joints: Sequence[Mapping[str, Any]],
    joint_ids: set[str],
) -> None:
    enable = _mapping(document.get("enable"), "enable", findings)
    _check_keys(
        enable, "enable", {"batches", "lifecycle_policies"}, set(), findings
    )

    policy_entries = _sequence(
        enable.get("lifecycle_policies"), "enable.lifecycle_policies", findings
    )
    policies, policy_ids = _duplicate_ids(
        policy_entries,
        "enable.lifecycle_policies",
        "lifecycle policy",
        findings,
    )
    for index, policy in enumerate(policies):
        path = f"enable.lifecycle_policies[{index}]"
        _check_keys(
            policy,
            path,
            {"id", "disable_terminals", "fault_reset", "source"},
            set(),
            findings,
        )
        _nonempty_string(policy.get("id"), f"{path}.id", findings)
        terminals = _string_list(
            policy.get("disable_terminals"), f"{path}.disable_terminals", findings
        )
        for terminal in terminals:
            if terminal not in DISABLE_TERMINALS:
                findings.append(
                    f"{path}.disable_terminals: unexpected terminal '{terminal}'"
                )
        if "switch_on_disabled" not in terminals:
            findings.append(
                f"{path}.disable_terminals: switch_on_disabled is always required"
            )
        if policy.get("fault_reset") != "explicit_only":
            findings.append(f"{path}.fault_reset: must be 'explicit_only'")
        _nonempty_string(policy.get("source"), f"{path}.source", findings)

    batch_entries = _sequence(enable.get("batches"), "enable.batches", findings)
    if not batch_entries:
        findings.append("enable.batches: must contain at least one enable batch")
    if len(batch_entries) > MAX_ENABLE_BATCHES:
        findings.append(
            f"enable.batches: enable_manager supports at most {MAX_ENABLE_BATCHES} "
            "enable batches"
        )
    batches, _ = _duplicate_ids(
        batch_entries, "enable.batches", "enable batch", findings
    )
    batch_members: list[str] = []
    for index, batch in enumerate(batches):
        path = f"enable.batches[{index}]"
        _check_keys(batch, path, {"id", "joints"}, set(), findings)
        _nonempty_string(batch.get("id"), f"{path}.id", findings)
        members = _string_list(batch.get("joints"), f"{path}.joints", findings)
        if len(members) > MAX_ENABLE_BATCH_SIZE:
            findings.append(
                f"{path}.joints: enable_manager supports at most "
                f"{MAX_ENABLE_BATCH_SIZE} joints per batch"
            )
        batch_members.extend(members)
        for joint in members:
            if joint not in joint_ids:
                findings.append(f"enable batch references unknown joint '{joint}'")

    for joint, count in Counter(batch_members).items():
        if count > 1:
            findings.append(f"joint '{joint}' appears in multiple enable batches")

    used_policies: set[str] = set()
    managed_joints: set[str] = set()
    unmanaged_joints: set[str] = set()
    for joint in joints:
        name = joint.get("name")
        if not isinstance(name, str):
            continue
        policy = joint.get("lifecycle_policy")
        if isinstance(policy, str) and policy:
            managed_joints.add(name)
            used_policies.add(policy)
            if policy not in policy_ids:
                findings.append(
                    f"joint '{name}' references unknown lifecycle policy '{policy}'"
                )
        else:
            unmanaged_joints.add(name)

    batch_member_set = set(batch_members)
    for name in sorted(managed_joints - batch_member_set):
        findings.append(f"managed joint '{name}' is missing from an enable batch")
    for name in sorted(unmanaged_joints & batch_member_set):
        findings.append(
            f"joint '{name}' without lifecycle policy must not appear in an enable batch"
        )
    for unused in sorted(policy_ids - used_policies):
        findings.append(f"lifecycle policy '{unused}' is not referenced by a joint")


def _validate_joints(
    document: Mapping[str, Any],
    findings: list[str],
    actuator_ids: set[str],
    actuator_profiles: Mapping[str, str],
    actuator_transmissions: Mapping[str, str],
    profile_buses: Mapping[str, str],
    profile_command_interfaces: Mapping[str, set[str]],
    profile_state_interfaces: Mapping[str, set[str]],
    transmission_units: Mapping[str, str],
) -> tuple[list[Mapping[str, Any]], set[str]]:
    entries = _sequence(document.get("joints"), "joints", findings)
    joints: list[Mapping[str, Any]] = []
    names: list[str] = []
    actuator_joints: dict[str, list[str]] = {}
    role_counts: Counter[str] = Counter()
    for index, raw_joint in enumerate(entries):
        path = f"joints[{index}]"
        joint = _mapping(raw_joint, path, findings)
        joints.append(joint)
        _check_keys(
            joint,
            path,
            {"name", "actuator", "controller_role"},
            {"position_tolerance", "lifecycle_policy"},
            findings,
        )
        name = _nonempty_string(joint.get("name"), f"{path}.name", findings)
        actuator = _nonempty_string(joint.get("actuator"), f"{path}.actuator", findings)
        role = joint.get("controller_role")
        if role not in CONTROLLER_ROLES:
            findings.append(
                f"{path}.controller_role: must be one of {sorted(CONTROLLER_ROLES)}"
            )
        else:
            role_counts[str(role)] += 1
        if name is not None:
            names.append(name)
        if actuator is not None and name is not None:
            actuator_joints.setdefault(actuator, []).append(name)
            if actuator not in actuator_ids:
                findings.append(f"joint '{name}' references unknown actuator '{actuator}'")

        if role == "whole_body_trajectory":
            tolerance = joint.get("position_tolerance")
            if not _is_number(tolerance) or float(tolerance) <= 0.0:
                findings.append(f"{path}.position_tolerance: must be positive")
            if "lifecycle_policy" not in joint:
                findings.append(
                    f"whole_body_trajectory joint '{name}' requires a lifecycle policy"
                )
        else:
            if "position_tolerance" in joint:
                findings.append(f"{path}: track joints must not set position_tolerance")
            if role in {"left_track", "right_track"} and "lifecycle_policy" in joint:
                findings.append(f"{path}: track joints must not set lifecycle_policy")
        if "lifecycle_policy" in joint:
            _nonempty_string(
                joint.get("lifecycle_policy"), f"{path}.lifecycle_policy", findings
            )

        profile = actuator_profiles.get(actuator) if actuator is not None else None
        if role in ROLE_PROFILE_REQUIREMENTS and profile is not None:
            expected_bus, expected_commands, expected_states = ROLE_PROFILE_REQUIREMENTS[
                str(role)
            ]
            actual_bus = profile_buses.get(profile)
            if actual_bus is not None and actual_bus != expected_bus:
                findings.append(
                    f"{role} joint '{name}' requires drive profile bus "
                    f"'{expected_bus}', got '{actual_bus}'"
                )
            command_interfaces = profile_command_interfaces.get(profile)
            actual_commands = (
                command_interfaces & CONVERSION_INTERFACES
                if command_interfaces is not None
                else None
            )
            if actual_commands is not None and actual_commands != expected_commands:
                findings.append(
                    f"{role} joint '{name}' requires command motion interfaces "
                    f"{sorted(expected_commands)}, got {sorted(actual_commands)}"
                )
            state_interfaces = profile_state_interfaces.get(profile)
            actual_states = (
                state_interfaces & CONVERSION_INTERFACES
                if state_interfaces is not None
                else None
            )
            if actual_states is not None and actual_states != expected_states:
                findings.append(
                    f"{role} joint '{name}' requires state motion interfaces "
                    f"{sorted(expected_states)}, got {sorted(actual_states)}"
                )
            if role in {"left_track", "right_track"}:
                exact_commands = {"velocity"}
                exact_states = {"position", "velocity"}
                if (
                    command_interfaces is not None
                    and command_interfaces != exact_commands
                ):
                    findings.append(
                        f"{role} joint '{name}' command interfaces must be exactly "
                        f"{sorted(exact_commands)}, got {sorted(command_interfaces)}"
                    )
                if state_interfaces is not None and state_interfaces != exact_states:
                    findings.append(
                        f"{role} joint '{name}' state interfaces must be exactly "
                        f"{sorted(exact_states)}, got {sorted(state_interfaces)}"
                    )
            if role == "whole_body_trajectory":
                if command_interfaces is not None and "control_word" not in command_interfaces:
                    findings.append(
                        f"whole_body_trajectory joint '{name}' requires command "
                        "interface 'control_word'"
                    )
                if state_interfaces is not None and "status_word" not in state_interfaces:
                    findings.append(
                        f"whole_body_trajectory joint '{name}' requires state "
                        "interface 'status_word'"
                    )
            if role in {"left_track", "right_track"} and actuator is not None:
                transmission = actuator_transmissions.get(actuator)
                si_unit = transmission_units.get(transmission) if transmission else None
                if si_unit is not None and si_unit != "m":
                    findings.append(
                        f"{role} joint '{name}' transmission '{transmission}' requires "
                        f"linear SI unit 'm', got '{si_unit}'"
                    )

    for name, count in Counter(names).items():
        if count > 1:
            findings.append(f"joints: duplicate joint '{name}'")
    for actuator, mapped_joints in actuator_joints.items():
        if len(mapped_joints) > 1:
            findings.append(
                f"actuator '{actuator}' maps to multiple logical joints: {mapped_joints}"
            )
    for actuator in sorted(actuator_ids - set(actuator_joints)):
        findings.append(f"actuator '{actuator}' has no logical joint")
    whole_body_count = role_counts["whole_body_trajectory"]
    if whole_body_count == 0:
        findings.append("joints: must contain at least one whole_body_trajectory joint")
    if whole_body_count > MAX_ENABLE_AXES:
        findings.append(
            f"joints: enable_manager supports at most {MAX_ENABLE_AXES} "
            "whole_body_trajectory joints"
        )
    left_track_count = role_counts["left_track"]
    right_track_count = role_counts["right_track"]
    if (
        left_track_count == 0
        or right_track_count == 0
        or left_track_count != right_track_count
    ):
        findings.append(
            "joints: track roles left_track and right_track must have equal non-zero "
            f"counts (got {left_track_count} and {right_track_count})"
        )
    return joints, set(names)


def validate_manifest(
    document: object, *, repository_root: Path | None = None
) -> None:
    """Validate one parsed manifest and raise one aggregated fail-closed error."""
    findings: list[str] = []
    manifest = _mapping(document, "manifest", findings)
    _check_keys(manifest, "manifest", TOP_LEVEL_KEYS, set(), findings)
    if manifest.get("schema_version") != 1:
        findings.append("manifest.schema_version: only version 1 is supported")
    _nonempty_string(manifest.get("variant_id"), "manifest.variant_id", findings)

    actuator_addresses, _, _ = _validate_buses(manifest, findings)
    (
        profile_ids,
        profile_buses,
        profile_command_interfaces,
        profile_state_interfaces,
    ) = _validate_profiles(manifest, findings, repository_root)
    transmission_ids, transmission_interfaces, transmission_units = _validate_transmissions(
        manifest, findings
    )
    actuator_ids, actuator_profiles, actuator_transmissions = _validate_actuators(
        manifest,
        findings,
        profile_ids,
        transmission_ids,
        profile_buses,
        profile_command_interfaces,
        profile_state_interfaces,
        transmission_interfaces,
        actuator_addresses,
    )
    joints, joint_ids = _validate_joints(
        manifest,
        findings,
        actuator_ids,
        actuator_profiles,
        actuator_transmissions,
        profile_buses,
        profile_command_interfaces,
        profile_state_interfaces,
        transmission_units,
    )
    _validate_enable(manifest, findings, joints, joint_ids)

    if findings:
        raise DriverVariantError(findings)


def load_manifest(
    path: Path | str = DEFAULT_MANIFEST, *, repository_root: Path | None = None
) -> dict[str, Any]:
    """Load and validate a manifest without performing runtime or hardware I/O."""
    manifest_path = Path(path)
    try:
        text = manifest_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise DriverVariantError(f"{manifest_path}: cannot read manifest: {exc}") from exc
    document = parse_yaml_document(text, str(manifest_path))
    validate_manifest(document, repository_root=repository_root)
    return dict(document)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", nargs="?", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    try:
        document = load_manifest(args.manifest, repository_root=args.repository_root)
    except DriverVariantError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    print(f"PASS: driver variant {document['variant_id']} is valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
