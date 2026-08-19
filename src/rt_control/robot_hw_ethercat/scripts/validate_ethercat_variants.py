#!/usr/bin/env python3
"""Fail-closed validation for EtherCAT variant descriptors and slave profiles."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import re
import sys
from typing import Any, Iterable

import yaml


SUPPORTED_SCHEMA_VERSION = 1
SUPPORTED_SYSTEM = "ecat_arms"
VARIANT_KEYS = (
    "schema_version",
    "variant",
    "system",
    "extra_responders",
    "axes",
)
AXIS_KEYS = (
    "joint_name",
    "family",
    "ring_position",
    "profile",
    "mode_of_operation",
)
RESPONDER_KEYS = ("ring_position",)
FAMILY_REGISTRY_KEYS = ("schema_version", "interface_contracts", "families")
INTERFACE_CONTRACT_KEYS = (
    "required_command_interfaces",
    "required_state_interfaces",
)
COMMAND_INTERFACE_KEYS = ("name", "index", "sub_index", "type")
STATE_INTERFACE_KEYS = (
    "name",
    "index",
    "sub_index",
    "type",
    "mock_initial_value",
)
FAMILY_KEYS = ("identity_profile", "interface_contract", "certified_modes")
IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
VARIANT_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
PROFILE_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


class ValidationError(ValueError):
    """A stable, operator-readable descriptor validation error."""


class UniqueKeySafeLoader(yaml.SafeLoader):
    """SafeLoader variant that rejects duplicate mapping keys."""


@dataclass(frozen=True)
class RequiredInterface:
    """One public ros2_control interface bound to its PDO object index."""

    name: str
    index: int
    sub_index: int
    data_type: str
    mock_initial_value: float | None = None


@dataclass(frozen=True)
class InterfaceContract:
    """Required public command/state interfaces for one axis contract."""

    command: tuple[RequiredInterface, ...]
    state: tuple[RequiredInterface, ...]


@dataclass(frozen=True)
class FamilyContract:
    """Device identity, interface contract, and certified mode policy."""

    identity_profile: Path
    vendor_id: int
    product_id: int
    interface_contract: InterfaceContract
    certified_modes: frozenset[int]


def _construct_unique_mapping(
    loader: UniqueKeySafeLoader, node: yaml.MappingNode, deep: bool = False
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as exc:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                "found an unhashable mapping key",
                key_node.start_mark,
            ) from exc
        if duplicate:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"duplicate mapping key {key!r}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeySafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


def _load_yaml(path: Path) -> Any:
    try:
        return yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueKeySafeLoader)
    except OSError as exc:
        raise ValidationError(f"cannot read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ValidationError(f"invalid YAML in {path}: {exc}") from exc


def _mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValidationError(f"{context} must be a mapping")
    return value


def _sequence(value: Any, context: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValidationError(f"{context} must be a sequence")
    return value


def _exact_keys(mapping: dict[str, Any], expected: Iterable[str], context: str) -> None:
    expected_set = set(expected)
    actual_set = set(mapping)
    missing = sorted(expected_set - actual_set)
    unknown = sorted(actual_set - expected_set)
    if missing or unknown:
        raise ValidationError(
            f"{context} keys mismatch: missing={missing}, unknown={unknown}"
        )


def _integer(value: Any, context: str) -> int:
    if type(value) is not int:
        raise ValidationError(f"{context} must be an integer")
    return value


def _identifier(value: Any, context: str) -> str:
    if not isinstance(value, str) or IDENTIFIER_PATTERN.fullmatch(value) is None:
        raise ValidationError(f"{context} must be a ROS-style identifier")
    return value


def _ring_position(value: Any, context: str) -> int:
    position = _integer(value, context)
    if not 0 <= position <= 65535:
        raise ValidationError(f"{context} must be in [0, 65535]")
    return position


def _unsigned_integer(value: Any, context: str, maximum: int) -> int:
    number = _integer(value, context)
    if not 0 <= number <= maximum:
        raise ValidationError(f"{context} must be in [0, {maximum}]")
    return number


def _profile_identity(
    profile_path: Path, profile: dict[str, Any]
) -> tuple[int, int]:
    vendor_id = _unsigned_integer(
        profile.get("vendor_id"), f"{profile_path}: vendor_id", 0xFFFFFFFF
    )
    product_id = _unsigned_integer(
        profile.get("product_id"), f"{profile_path}: product_id", 0xFFFFFFFF
    )
    return vendor_id, product_id


def _required_interfaces(
    raw_interfaces: Any, context: str, *, state: bool
) -> tuple[RequiredInterface, ...]:
    entries = _sequence(raw_interfaces, context)
    if not entries:
        raise ValidationError(f"{context} must not be empty")

    required: list[RequiredInterface] = []
    seen_names: set[str] = set()
    for index, raw_entry in enumerate(entries):
        entry_context = f"{context}[{index}]"
        entry = _mapping(raw_entry, entry_context)
        _exact_keys(
            entry,
            STATE_INTERFACE_KEYS if state else COMMAND_INTERFACE_KEYS,
            entry_context,
        )
        name = _identifier(entry["name"], f"{entry_context}.name")
        if name in seen_names:
            raise ValidationError(f"{context}: duplicate interface name {name!r}")
        seen_names.add(name)
        object_index = _unsigned_integer(
            entry["index"], f"{entry_context}.index", 0xFFFF
        )
        sub_index = _unsigned_integer(
            entry["sub_index"], f"{entry_context}.sub_index", 0xFF
        )
        data_type = _identifier(entry["type"], f"{entry_context}.type")
        mock_initial_value: float | None = None
        if state:
            raw_initial_value = entry["mock_initial_value"]
            if isinstance(raw_initial_value, bool) or not isinstance(
                raw_initial_value, (int, float)
            ):
                raise ValidationError(
                    f"{entry_context}.mock_initial_value must be a finite number"
                )
            mock_initial_value = float(raw_initial_value)
            if not math.isfinite(mock_initial_value):
                raise ValidationError(
                    f"{entry_context}.mock_initial_value must be a finite number"
                )
        required.append(
            RequiredInterface(
                name=name,
                index=object_index,
                sub_index=sub_index,
                data_type=data_type,
                mock_initial_value=mock_initial_value,
            )
        )
    return tuple(required)


def _load_family_contracts(
    registry_path: Path, profile_dir: Path
) -> dict[str, FamilyContract]:
    registry = _mapping(_load_yaml(registry_path), str(registry_path))
    _exact_keys(registry, FAMILY_REGISTRY_KEYS, str(registry_path))
    schema_version = _integer(
        registry["schema_version"], f"{registry_path}: schema_version"
    )
    if schema_version != SUPPORTED_SCHEMA_VERSION:
        raise ValidationError(
            f"{registry_path}: unsupported schema_version {schema_version}"
        )

    raw_contracts = _mapping(
        registry["interface_contracts"], f"{registry_path}: interface_contracts"
    )
    if not raw_contracts:
        raise ValidationError(f"{registry_path}: interface_contracts must not be empty")
    interface_contracts: dict[str, InterfaceContract] = {}
    for raw_name, raw_contract in raw_contracts.items():
        name = _identifier(raw_name, f"{registry_path}: interface contract name")
        context = f"{registry_path}: interface_contracts.{name}"
        contract = _mapping(raw_contract, context)
        _exact_keys(contract, INTERFACE_CONTRACT_KEYS, context)
        interface_contracts[name] = InterfaceContract(
            command=_required_interfaces(
                contract["required_command_interfaces"],
                f"{context}.required_command_interfaces",
                state=False,
            ),
            state=_required_interfaces(
                contract["required_state_interfaces"],
                f"{context}.required_state_interfaces",
                state=True,
            ),
        )

    raw_families = _mapping(registry["families"], f"{registry_path}: families")
    if not raw_families:
        raise ValidationError(f"{registry_path}: families must not be empty")
    families: dict[str, FamilyContract] = {}
    seen_identities: dict[tuple[int, int], str] = {}
    for raw_name, raw_family in raw_families.items():
        name = _identifier(raw_name, f"{registry_path}: family name")
        context = f"{registry_path}: families.{name}"
        family = _mapping(raw_family, context)
        _exact_keys(family, FAMILY_KEYS, context)

        identity_profile_name = family["identity_profile"]
        if not isinstance(
            identity_profile_name, str
        ) or PROFILE_PATTERN.fullmatch(identity_profile_name) is None:
            raise ValidationError(
                f"{context}: invalid identity_profile {identity_profile_name!r}"
            )
        identity_profile = profile_dir / f"{identity_profile_name}.yaml"
        if not identity_profile.is_file():
            raise ValidationError(
                f"{context}: identity_profile does not exist: {identity_profile}"
            )
        identity_source = _mapping(
            _load_yaml(identity_profile), str(identity_profile)
        )
        vendor_id, product_id = _profile_identity(
            identity_profile, identity_source
        )
        identity = (vendor_id, product_id)
        if identity in seen_identities:
            raise ValidationError(
                f"{registry_path}: families {seen_identities[identity]!r} and "
                f"{name!r} share device identity "
                f"(vendor_id=0x{vendor_id:08X}, product_id=0x{product_id:08X})"
            )
        seen_identities[identity] = name

        interface_contract_name = _identifier(
            family["interface_contract"], f"{context}.interface_contract"
        )
        if interface_contract_name not in interface_contracts:
            raise ValidationError(
                f"{context}: unknown interface_contract "
                f"{interface_contract_name!r}"
            )
        raw_modes = _sequence(family["certified_modes"], f"{context}.certified_modes")
        if not raw_modes:
            raise ValidationError(f"{context}.certified_modes must not be empty")
        modes = tuple(
            _integer(raw_mode, f"{context}.certified_modes[{index}]")
            for index, raw_mode in enumerate(raw_modes)
        )
        if len(set(modes)) != len(modes):
            raise ValidationError(f"{context}.certified_modes contains duplicates")

        families[name] = FamilyContract(
            identity_profile=identity_profile,
            vendor_id=vendor_id,
            product_id=product_id,
            interface_contract=interface_contracts[interface_contract_name],
            certified_modes=frozenset(modes),
        )
    return families


def _pdo_channels(
    profile_path: Path, profile: dict[str, Any], pdo_name: str
) -> list[dict[str, Any]]:
    channels: list[dict[str, Any]] = []
    pdos = _sequence(profile.get(pdo_name), f"{profile_path}: {pdo_name}")
    for pdo_index, raw_pdo in enumerate(pdos):
        pdo = _mapping(raw_pdo, f"{profile_path}: {pdo_name}[{pdo_index}]")
        raw_channels = _sequence(
            pdo.get("channels"),
            f"{profile_path}: {pdo_name}[{pdo_index}].channels",
        )
        for channel_index, raw_channel in enumerate(raw_channels):
            channels.append(
                _mapping(
                    raw_channel,
                    f"{profile_path}: {pdo_name}[{pdo_index}].channels"
                    f"[{channel_index}]",
                )
            )
    return channels


def _validate_required_interface_bindings(
    profile_path: Path,
    profile: dict[str, Any],
    contract: InterfaceContract,
) -> None:
    for pdo_name, interface_key, requirements in (
        ("rpdo", "command_interface", contract.command),
        ("tpdo", "state_interface", contract.state),
    ):
        channels = _pdo_channels(profile_path, profile, pdo_name)
        for requirement in requirements:
            matches = [
                channel
                for channel in channels
                if channel.get(interface_key) == requirement.name
            ]
            if len(matches) != 1 or any(
                (
                    matches[0].get("index") != requirement.index,
                    matches[0].get("sub_index") != requirement.sub_index,
                    matches[0].get("type") != requirement.data_type,
                )
            ):
                raise ValidationError(
                    f"{profile_path}: required {interface_key} "
                    f"{requirement.name!r} at 0x{requirement.index:04X}:"
                    f"{requirement.sub_index} with type {requirement.data_type!r} "
                    f"must appear exactly once in {pdo_name}"
                )


def _validate_profile_mode(
    profile_path: Path, profile: dict[str, Any], mode: int
) -> None:
    sdo_entries = _sequence(profile.get("sdo"), f"{profile_path}: sdo")
    mode_sdos = 0
    for index, raw_entry in enumerate(sdo_entries):
        entry = _mapping(raw_entry, f"{profile_path}: sdo[{index}]")
        if entry.get("index") != 0x6060:
            continue
        mode_sdos += 1
        if entry.get("value") != mode:
            raise ValidationError(
                f"{profile_path}: SDO 0x6060 value {entry.get('value')!r} "
                f"does not match descriptor mode {mode}"
            )
    if mode_sdos == 0:
        raise ValidationError(f"{profile_path}: missing SDO 0x6060 mode assignment")

    rpdo_entries = _sequence(profile.get("rpdo"), f"{profile_path}: rpdo")
    for pdo_index, raw_pdo in enumerate(rpdo_entries):
        pdo = _mapping(raw_pdo, f"{profile_path}: rpdo[{pdo_index}]")
        channels = _sequence(
            pdo.get("channels"), f"{profile_path}: rpdo[{pdo_index}].channels"
        )
        for channel_index, raw_channel in enumerate(channels):
            channel = _mapping(
                raw_channel,
                f"{profile_path}: rpdo[{pdo_index}].channels[{channel_index}]",
            )
            if channel.get("index") != 0x6060:
                continue
            if channel.get("default") != mode:
                raise ValidationError(
                    f"{profile_path}: RPDO 0x6060 default "
                    f"{channel.get('default')!r} does not match descriptor mode {mode}"
                )


def validate_variant(variant_path: Path, profile_dir: Path) -> None:
    families = _load_family_contracts(profile_dir.parent / "families.yaml", profile_dir)
    descriptor = _mapping(_load_yaml(variant_path), str(variant_path))
    _exact_keys(descriptor, VARIANT_KEYS, str(variant_path))

    schema_version = _integer(
        descriptor["schema_version"], f"{variant_path}: schema_version"
    )
    if schema_version != SUPPORTED_SCHEMA_VERSION:
        raise ValidationError(
            f"{variant_path}: unsupported schema_version {schema_version}"
        )

    variant = descriptor["variant"]
    if not isinstance(variant, str) or VARIANT_PATTERN.fullmatch(variant) is None:
        raise ValidationError(f"{variant_path}: invalid EtherCAT variant identifier")
    if variant != variant_path.stem:
        raise ValidationError(
            f"{variant_path}: variant {variant!r} must match filename stem"
        )
    if descriptor["system"] != SUPPORTED_SYSTEM:
        raise ValidationError(
            f"{variant_path}: unsupported system {descriptor['system']!r}"
        )

    seen_positions: set[int] = set()
    responders = _sequence(
        descriptor["extra_responders"], f"{variant_path}: extra_responders"
    )
    for index, raw_responder in enumerate(responders):
        context = f"{variant_path}: extra_responders[{index}]"
        responder = _mapping(raw_responder, context)
        _exact_keys(responder, RESPONDER_KEYS, context)
        position = _ring_position(responder["ring_position"], f"{context}.ring_position")
        if position in seen_positions:
            raise ValidationError(f"{variant_path}: duplicate ring_position {position}")
        seen_positions.add(position)

    axes = _sequence(descriptor["axes"], f"{variant_path}: axes")
    if not axes:
        raise ValidationError(f"{variant_path}: axes must not be empty")
    seen_joints: set[str] = set()
    loaded_profiles: dict[Path, dict[str, Any]] = {}
    validated_profiles: dict[tuple[Path, str, int], None] = {}
    for index, raw_axis in enumerate(axes):
        context = f"{variant_path}: axes[{index}]"
        axis = _mapping(raw_axis, context)
        _exact_keys(axis, AXIS_KEYS, context)

        joint_name = _identifier(axis["joint_name"], f"{context}.joint_name")
        if joint_name in seen_joints:
            raise ValidationError(f"{variant_path}: duplicate joint_name {joint_name!r}")
        seen_joints.add(joint_name)

        position = _ring_position(axis["ring_position"], f"{context}.ring_position")
        if position in seen_positions:
            raise ValidationError(f"{variant_path}: duplicate ring_position {position}")
        seen_positions.add(position)

        family = _identifier(axis["family"], f"{context}.family")
        if family not in families:
            raise ValidationError(f"{context}: unsupported family {family!r}")
        family_contract = families[family]
        mode = _integer(axis["mode_of_operation"], f"{context}.mode_of_operation")
        if mode not in family_contract.certified_modes:
            raise ValidationError(
                f"{context}: uncertified mode_of_operation {mode} for family {family!r}"
            )

        profile_name = axis["profile"]
        if not isinstance(profile_name, str) or PROFILE_PATTERN.fullmatch(
            profile_name
        ) is None:
            raise ValidationError(f"{context}: invalid profile name {profile_name!r}")
        profile_path = profile_dir / f"{profile_name}.yaml"
        if not profile_path.is_file():
            raise ValidationError(f"{context}: profile does not exist: {profile_path}")
        resolved_profile_path = profile_path.resolve()
        if resolved_profile_path not in loaded_profiles:
            loaded_profiles[resolved_profile_path] = _mapping(
                _load_yaml(profile_path), str(profile_path)
            )
        profile = loaded_profiles[resolved_profile_path]
        vendor_id, product_id = _profile_identity(profile_path, profile)
        if (
            vendor_id != family_contract.vendor_id
            or product_id != family_contract.product_id
        ):
            raise ValidationError(
                f"{context}: profile device identity "
                f"(vendor_id=0x{vendor_id:08X}, product_id=0x{product_id:08X}) "
                f"does not match family {family!r} identity from "
                f"{family_contract.identity_profile}"
            )

        profile_key = (resolved_profile_path, family, mode)
        if profile_key not in validated_profiles:
            _validate_profile_mode(profile_path, profile, mode)
            _validate_required_interface_bindings(
                profile_path, profile, family_contract.interface_contract
            )
            validated_profiles[profile_key] = None

    expected_positions = list(range(len(seen_positions)))
    actual_positions = sorted(seen_positions)
    if actual_positions != expected_positions:
        raise ValidationError(
            f"{variant_path}: ring_positions must form contiguous range "
            f"0..{len(seen_positions) - 1}; got {actual_positions}"
        )


def _variant_paths(arguments: argparse.Namespace) -> list[Path]:
    if arguments.variant_file is not None:
        return [arguments.variant_file]
    paths = sorted(arguments.variants_dir.glob("*.yaml"))
    if not paths:
        raise ValidationError(
            f"no EtherCAT variant descriptors found in {arguments.variants_dir}"
        )
    return paths


def _parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--variant-file", type=Path)
    source.add_argument("--variants-dir", type=Path)
    parser.add_argument("--profile-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    arguments = _parse_arguments()
    try:
        for variant_path in _variant_paths(arguments):
            validate_variant(variant_path, arguments.profile_dir)
    except ValidationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
