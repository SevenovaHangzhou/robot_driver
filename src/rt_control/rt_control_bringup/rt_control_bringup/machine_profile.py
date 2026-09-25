"""Strict machine-profile and control-scope selection for RT-Control.

The machine manifest owns module composition and the allowed test surfaces. It
does not replace the hardware-package descriptors that own PDO/SDO and
mechanical values. Draft manifests can be inspected and counted, but they are
never considered runtime-ready.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
from typing import Any, Iterable, Mapping, Sequence

import yaml


_IDENTIFIER = re.compile(r"^[a-z][a-z0-9_]*$")
_STATUSES = frozenset({"absent", "draft", "ready"})
_TRANSPORTS = frozenset({"ethercat", "canopen", "damiao_can"})
_ROLES = frozenset({"actuator", "actuator_group", "state_sensor_group"})
_READINESS = frozenset({"full_scope", "partial_scope"})
_FAULT_REACTIONS = frozenset({"stop", "stop_and_inhibit"})
_TBD = "TBD"


class MachineProfileError(ValueError):
    """A machine manifest or selected scope cannot be used safely."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """SafeLoader variant that rejects duplicate mapping keys."""


def _construct_unique_mapping(
    loader: _UniqueKeyLoader,
    node: yaml.MappingNode,
    deep: bool = False,
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


def _path(parts: Sequence[object]) -> str:
    return ".".join(str(part) for part in parts)


def _mapping(value: Any, location: Sequence[object]) -> Mapping[str, Any]:
    if type(value) is not dict:
        raise MachineProfileError(f"{_path(location)} must be a mapping")
    return value


def _exact_keys(
    value: Mapping[str, Any], expected: Iterable[str], location: Sequence[object]
) -> None:
    expected_set = set(expected)
    actual_set = set(value)
    missing = sorted(expected_set - actual_set)
    unknown = sorted(actual_set - expected_set)
    if missing or unknown:
        raise MachineProfileError(
            f"{_path(location)} keys mismatch: missing={missing}, unknown={unknown}"
        )


def _list(value: Any, location: Sequence[object]) -> list[Any]:
    if type(value) is not list:
        raise MachineProfileError(f"{_path(location)} must be a list")
    return value


def _nonempty_list(value: Any, location: Sequence[object], label: str) -> list[Any]:
    items = _list(value, location)
    if not items:
        raise MachineProfileError(f"{_path(location)} must contain at least one {label}")
    return items


def _string(value: Any, location: Sequence[object], *, identifier: bool = False) -> str:
    if type(value) is not str or not value.strip() or value != value.strip():
        raise MachineProfileError(f"{_path(location)} must be a nonempty trimmed string")
    if identifier and _IDENTIFIER.fullmatch(value) is None:
        raise MachineProfileError(f"{_path(location)} must match ^[a-z][a-z0-9_]*$")
    return value


def _string_list(value: Any, location: Sequence[object]) -> tuple[str, ...]:
    values = _list(value, location)
    result: list[str] = []
    for index, item in enumerate(values):
        result.append(_string(item, (*location, index), identifier=True))
    return tuple(result)


def _integer(value: Any, location: Sequence[object], *, minimum: int | None = None) -> int:
    if type(value) is not int:
        raise MachineProfileError(f"{_path(location)} must be an integer")
    if minimum is not None and value < minimum:
        raise MachineProfileError(f"{_path(location)} must be >= {minimum}")
    return value


def _optional_integer(value: Any, location: Sequence[object]) -> int | None:
    if value is None:
        return None
    return _integer(value, location, minimum=0)


def _boolean(value: Any, location: Sequence[object]) -> bool:
    if type(value) is not bool:
        raise MachineProfileError(f"{_path(location)} must be a boolean")
    return value


def _placeholder_or_mapping(value: Any, location: Sequence[object]) -> Any:
    if value == _TBD:
        return value
    if type(value) is dict and value:
        return value
    raise MachineProfileError(
        f"{_path(location)} must be TBD or a structured mapping"
    )


def _contains_tbd(value: Any) -> bool:
    if value == _TBD:
        return True
    if isinstance(value, Mapping):
        return any(_contains_tbd(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return any(_contains_tbd(item) for item in value)
    return False


def _contains_unresolved_reference(value: Any) -> bool:
    return _contains_tbd(value) or (
        isinstance(value, Mapping) and value.get("verified") is False
    )


def _status(value: Any, location: Sequence[object]) -> str:
    status = _string(value, location)
    if status not in _STATUSES:
        raise MachineProfileError(
            f"{_path(location)} must be one of {sorted(_STATUSES)}"
        )
    return status


def _load_yaml(path: Path) -> Mapping[str, Any]:
    try:
        document = yaml.load(
            path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader
        )
    except OSError as error:
        raise MachineProfileError(
            f"could not read machine manifest {path}: {error}"
        ) from error
    except yaml.YAMLError as error:
        raise MachineProfileError(
            f"invalid machine manifest YAML {path}: {error}"
        ) from error
    return _mapping(document, (path.name,))


@dataclass(frozen=True)
class ModeGroup:
    name: str
    count: int
    mode_of_operation: int | None
    interface: str
    certified: bool


@dataclass(frozen=True)
class ModuleSpec:
    name: str
    role: str
    transport: str
    status: str
    master_id: int | None
    instance_count: int
    actuator_count: int
    sensor_count: int
    mode_groups: tuple[ModeGroup, ...]
    per_instance_mode_groups: Mapping[str, int]
    authority: str | None
    requires: tuple[str, ...]
    sensor_bindings: tuple[str, ...]
    hardware_identity: Any
    profile_ref: Any
    mechanical_parameters: Any
    pending_facts: tuple[str, ...]


@dataclass(frozen=True)
class BusLayout:
    status: str
    master_id: int | None
    positions: tuple[int, ...] | None
    node_ids: tuple[int, ...] | None


@dataclass(frozen=True)
class PhysicalLayout:
    ethercat: BusLayout
    canopen: BusLayout
    damiao_can: BusLayout


@dataclass(frozen=True)
class HardwareOptionSelection:
    name: str
    modules: tuple[str, ...]
    ethercat_layout: BusLayout | None
    pending_facts: tuple[str, ...]


@dataclass(frozen=True)
class HardwareOption:
    name: str
    default_selection: str
    selections: Mapping[str, HardwareOptionSelection]


@dataclass(frozen=True)
class PhysicalProfile:
    name: str
    status: str
    modules: tuple[str, ...]
    allowed_scopes: tuple[str, ...]
    hardware_options: Mapping[str, tuple[str, ...]]
    layout: PhysicalLayout
    pending_facts: tuple[str, ...]


@dataclass(frozen=True)
class GroupReference:
    module: str
    mode_group: str


@dataclass(frozen=True)
class ScopeSpec:
    name: str
    enabled_modules: tuple[str, ...]
    required_state_modules: tuple[str, ...]
    actuator_groups: tuple[GroupReference, ...]
    jtc_groups: tuple[GroupReference, ...]
    global_readiness: str


@dataclass(frozen=True)
class ControllerBinding:
    module: str
    name: str
    plugin: str
    package: str
    config_file: str
    actuator_groups: tuple[str, ...]
    required_state_modules: tuple[str, ...]


@dataclass(frozen=True)
class FaultDependency:
    source_module: str
    affected_modules: tuple[str, ...]
    reaction: str


@dataclass(frozen=True)
class RobotModelSpec:
    package: str
    xacro_file: str
    srdf_file: str
    joint_limits_file: str
    initial_positions_file: str
    end_effector: str
    source_revision: str


@dataclass(frozen=True)
class MachineManifest:
    path: Path
    variant: str
    status: str
    robot_model: RobotModelSpec
    functional_modules: tuple[str, ...]
    modules: Mapping[str, ModuleSpec]
    hardware_options: Mapping[str, HardwareOption]
    profiles: Mapping[str, PhysicalProfile]
    scopes: Mapping[str, ScopeSpec]
    controllers: Mapping[str, ControllerBinding]
    fault_dependencies: tuple[FaultDependency, ...]
    owner_refs: Mapping[str, str]


@dataclass(frozen=True)
class SelectedHardware:
    manifest_variant: str
    physical_profile: str
    control_scope: str
    hardware_options: Mapping[str, str]
    physical_modules: tuple[str, ...]
    active_modules: tuple[str, ...]
    inactive_modules: tuple[str, ...]
    transports: tuple[str, ...]
    bus_requirements: Mapping[str, str]
    ethercat_master_id: int | None
    ethercat_ring_positions: tuple[int, ...] | None
    canopen_node_ids: tuple[int, ...] | None
    damiao_can_node_ids: tuple[int, ...] | None
    actuator_count: int
    mode_counts: Mapping[int, int]
    group_counts: Mapping[str, int]
    actuator_groups: tuple[GroupReference, ...]
    jtc_groups: tuple[GroupReference, ...]
    jtc_mode_counts: Mapping[int, int]
    required_state_modules: tuple[str, ...]
    state_sensor_count: int
    controllers: tuple[ControllerBinding, ...]
    fault_dependencies: tuple[FaultDependency, ...]
    validation_status: str
    global_readiness: str
    runtime_blockers: tuple[str, ...]


def _parse_pending_facts(value: Any, location: Sequence[object]) -> tuple[str, ...]:
    values = _list(value, location)
    result: list[str] = []
    for index, item in enumerate(values):
        result.append(_string(item, (*location, index)))
    return tuple(result)


def _package_relative_file(
    value: Any, location: Sequence[object], suffix: str
) -> str:
    file_name = _string(value, location)
    path = Path(file_name)
    if (
        path.is_absolute()
        or len(path.parts) < 2
        or any(part == ".." for part in path.parts)
        or not file_name.endswith(suffix)
    ):
        raise MachineProfileError(
            f"{_path(location)} must be a package-relative {suffix} file"
        )
    return file_name


def _parse_robot_model(value: Any, location: Sequence[object]) -> RobotModelSpec:
    model = _mapping(value, location)
    _exact_keys(
        model,
        {
            "package",
            "xacro_file",
            "srdf_file",
            "joint_limits_file",
            "initial_positions_file",
            "end_effector",
            "source_revision",
        },
        location,
    )
    source_revision = _string(model["source_revision"], (*location, "source_revision"))
    if re.fullmatch(r"[0-9a-f]{40}", source_revision) is None:
        raise MachineProfileError(
            f"{_path((*location, 'source_revision'))} must be a full lowercase SHA"
        )
    return RobotModelSpec(
        package=_string(model["package"], (*location, "package"), identifier=True),
        xacro_file=_package_relative_file(
            model["xacro_file"], (*location, "xacro_file"), ".xacro"
        ),
        srdf_file=_package_relative_file(
            model["srdf_file"], (*location, "srdf_file"), ".srdf"
        ),
        joint_limits_file=_package_relative_file(
            model["joint_limits_file"], (*location, "joint_limits_file"), ".yaml"
        ),
        initial_positions_file=_package_relative_file(
            model["initial_positions_file"],
            (*location, "initial_positions_file"),
            ".yaml",
        ),
        end_effector=_string(
            model["end_effector"], (*location, "end_effector"), identifier=True
        ),
        source_revision=source_revision,
    )


def _parse_mode_groups(value: Any, location: Sequence[object]) -> tuple[ModeGroup, ...]:
    values = _list(value, location)
    groups: list[ModeGroup] = []
    seen: set[str] = set()
    expected = {"name", "count", "mode_of_operation", "interface", "certified"}
    for index, raw_group in enumerate(values):
        group_location = (*location, index)
        group = _mapping(raw_group, group_location)
        _exact_keys(group, expected, group_location)
        name = _string(group["name"], (*group_location, "name"), identifier=True)
        if name in seen:
            raise MachineProfileError(
                f"{_path(location)} contains duplicate mode group {name!r}"
            )
        seen.add(name)
        raw_count = group["count"]
        if type(raw_count) is not int or raw_count <= 0:
            raise MachineProfileError(
                f"{_path((*group_location, 'count'))} count must be a positive integer"
            )
        count = raw_count
        raw_mode = group["mode_of_operation"]
        mode = None if raw_mode is None else _integer(
            raw_mode, (*group_location, "mode_of_operation"), minimum=0
        )
        interface = _string(group["interface"], (*group_location, "interface"))
        certified = _boolean(group["certified"], (*group_location, "certified"))
        groups.append(ModeGroup(name, count, mode, interface, certified))
    return tuple(groups)


def _parse_modules(raw_modules: Any, location: Sequence[object]) -> dict[str, ModuleSpec]:
    modules_mapping = _mapping(raw_modules, location)
    if not modules_mapping:
        raise MachineProfileError(f"{_path(location)} must not be empty")
    expected = {
        "role",
        "transport",
        "status",
        "master_id",
        "instance_count",
        "actuator_count",
        "sensor_count",
        "mode_groups",
        "per_instance_mode_groups",
        "authority",
        "requires",
        "sensor_bindings",
        "hardware_identity",
        "profile_ref",
        "mechanical_parameters",
        "pending_facts",
    }
    modules: dict[str, ModuleSpec] = {}
    for raw_name, raw_module in modules_mapping.items():
        name = _string(raw_name, (*location, "<name>"), identifier=True)
        module_location = (*location, name)
        module = _mapping(raw_module, module_location)
        _exact_keys(module, expected, module_location)
        role = _string(module["role"], (*module_location, "role"))
        if role not in _ROLES:
            raise MachineProfileError(
                f"{_path(module_location)}.role is unsupported: {role!r}"
            )
        transport = _string(module["transport"], (*module_location, "transport"))
        if transport not in _TRANSPORTS:
            raise MachineProfileError(
                f"{_path(module_location)}.transport is unsupported: {transport!r}"
            )
        status = _status(module["status"], (*module_location, "status"))
        master_id = _optional_integer(
            module["master_id"], (*module_location, "master_id")
        )
        instance_count = _integer(
            module["instance_count"], (*module_location, "instance_count"), minimum=1
        )
        actuator_count = _integer(
            module["actuator_count"], (*module_location, "actuator_count"), minimum=0
        )
        sensor_count = _integer(
            module["sensor_count"], (*module_location, "sensor_count"), minimum=0
        )
        mode_groups = _parse_mode_groups(
            module["mode_groups"], (*module_location, "mode_groups")
        )
        raw_per_instance = _mapping(
            module["per_instance_mode_groups"],
            (*module_location, "per_instance_mode_groups"),
        )
        per_instance_mode_groups: dict[str, int] = {}
        for raw_name, raw_count in raw_per_instance.items():
            group_name = _string(
                raw_name,
                (*module_location, "per_instance_mode_groups", "<name>"),
                identifier=True,
            )
            per_instance_mode_groups[group_name] = _integer(
                raw_count,
                (*module_location, "per_instance_mode_groups", group_name),
                minimum=1,
            )
        authority = module["authority"]
        if authority is not None:
            authority = _string(authority, (*module_location, "authority"))
        requires = _string_list(module["requires"], (*module_location, "requires"))
        sensor_bindings = _string_list(
            module["sensor_bindings"], (*module_location, "sensor_bindings")
        )
        hardware_identity = _placeholder_or_mapping(
            module["hardware_identity"], (*module_location, "hardware_identity")
        )
        profile_ref = _placeholder_or_mapping(
            module["profile_ref"], (*module_location, "profile_ref")
        )
        mechanical_parameters = _placeholder_or_mapping(
            module["mechanical_parameters"],
            (*module_location, "mechanical_parameters"),
        )
        pending_facts = _parse_pending_facts(
            module["pending_facts"], (*module_location, "pending_facts")
        )

        if role == "state_sensor_group" and (actuator_count or mode_groups):
            raise MachineProfileError(
                f"{_path(module_location)} state sensor must not declare actuators or mode groups"
            )
        if (
            role == "state_sensor_group"
            and len(set(sensor_bindings)) != len(sensor_bindings)
        ):
            raise MachineProfileError(
                f"{_path(module_location)} sensor_bindings must be unique"
            )
        if role == "state_sensor_group" and authority is None:
            raise MachineProfileError(
                f"{_path(module_location)} state sensor requires an authority"
            )
        if role != "state_sensor_group" and sensor_bindings:
            raise MachineProfileError(
                f"{_path(module_location)} actuator module must not declare sensor_bindings"
            )
        if role != "state_sensor_group" and actuator_count <= 0:
            raise MachineProfileError(
                f"{_path(module_location)} actuator module must declare a positive actuator_count"
            )
        if (
            role != "state_sensor_group"
            and sum(group.count for group in mode_groups) != actuator_count
        ):
            raise MachineProfileError(
                f"{_path(module_location)} mode group counts must sum to actuator_count"
            )
        mode_group_names = {group.name for group in mode_groups}
        if set(per_instance_mode_groups) != mode_group_names:
            raise MachineProfileError(
                f"{_path(module_location)} per_instance_mode_groups must match mode_groups"
            )
        if role != "state_sensor_group" and any(
            per_instance_mode_groups[group.name] * instance_count != group.count
            for group in mode_groups
        ):
            raise MachineProfileError(
                f"{_path(module_location)} per-instance mode counts must expand to mode counts"
            )
        if role != "state_sensor_group" and transport == "ethercat" and any(
            group.mode_of_operation is None for group in mode_groups
        ):
            raise MachineProfileError(
                f"{_path(module_location)} EtherCAT mode groups require mode_of_operation"
            )
        if role != "state_sensor_group" and transport != "ethercat" and any(
            group.mode_of_operation is not None for group in mode_groups
        ):
            raise MachineProfileError(
                f"{_path(module_location)} non-EtherCAT mode groups must not "
                "declare mode_of_operation"
            )
        if role != "state_sensor_group" and sensor_count != 0:
            raise MachineProfileError(
                f"{_path(module_location)} actuator module must have sensor_count=0"
            )
        if role == "state_sensor_group" and sensor_count <= 0:
            raise MachineProfileError(
                f"{_path(module_location)} state sensor must declare a positive sensor_count"
            )
        if role == "state_sensor_group" and len(sensor_bindings) != sensor_count:
            raise MachineProfileError(
                f"{_path(module_location)} sensor_bindings must match sensor_count"
            )
        if role == "state_sensor_group" and instance_count != sensor_count:
            raise MachineProfileError(
                f"{_path(module_location)} instance_count must match sensor_count"
            )
        if transport == "ethercat" and master_id is None:
            raise MachineProfileError(
                f"{_path(module_location)} EtherCAT module requires master_id"
            )
        if transport != "ethercat" and master_id is not None:
            raise MachineProfileError(
                f"{_path(module_location)} non-EtherCAT module must have master_id=null"
            )
        modules[name] = ModuleSpec(
            name=name,
            role=role,
            transport=transport,
            status=status,
            master_id=master_id,
            instance_count=instance_count,
            actuator_count=actuator_count,
            sensor_count=sensor_count,
            mode_groups=mode_groups,
            per_instance_mode_groups=per_instance_mode_groups,
            authority=authority,
            requires=requires,
            sensor_bindings=sensor_bindings,
            hardware_identity=hardware_identity,
            profile_ref=profile_ref,
            mechanical_parameters=mechanical_parameters,
            pending_facts=pending_facts,
        )

    for module in modules.values():
        unknown = sorted(set(module.requires) - set(modules))
        if unknown:
            raise MachineProfileError(
                f"{module.name}.requires references unknown module(s): {unknown}"
            )
        if module.name in module.requires:
            raise MachineProfileError(f"{module.name}.requires must not contain itself")
    return modules


def _parse_bus_layout(
    raw_layout: Any,
    location: Sequence[object],
    *,
    transport: str,
    has_transport: bool,
) -> BusLayout:
    layout = _mapping(raw_layout, location)
    expected_keys = (
        {"status", "master_id", "ring_positions"}
        if transport == "ethercat"
        else {"status", "node_ids"}
    )
    _exact_keys(layout, expected_keys, location)
    status = _status(layout["status"], (*location, "status"))
    master_id = (
        _optional_integer(layout["master_id"], (*location, "master_id"))
        if transport == "ethercat"
        else None
    )
    raw_positions = layout.get("ring_positions", [])
    raw_nodes = layout.get("node_ids", [])

    positions: tuple[int, ...] | None
    if raw_positions == _TBD:
        positions = None
    else:
        position_values = _list(raw_positions, (*location, "ring_positions"))
        parsed_positions = tuple(
            _integer(item, (*location, "ring_positions", index), minimum=0)
            for index, item in enumerate(position_values)
        )
        if len(set(parsed_positions)) != len(parsed_positions):
            raise MachineProfileError(f"{_path(location)} ring_positions must be unique")
        if parsed_positions and set(parsed_positions) != set(range(len(parsed_positions))):
            raise MachineProfileError(
                f"{_path(location)} ring_positions must form a complete 0-based layout"
            )
        positions = parsed_positions

    node_ids: tuple[int, ...] | None
    if raw_nodes == _TBD:
        node_ids = None
    else:
        node_values = _list(raw_nodes, (*location, "node_ids"))
        minimum_node_id = 1 if transport == "canopen" else 0
        maximum_node_id = 127 if transport == "canopen" else 0x7FF
        parsed_nodes = tuple(
            _integer(
                item,
                (*location, "node_ids", index),
                minimum=minimum_node_id,
            )
            for index, item in enumerate(node_values)
        )
        if any(node > maximum_node_id for node in parsed_nodes):
            range_text = (
                "1..127" if transport == "canopen" else "0..0x7FF"
            )
            raise MachineProfileError(
                f"{_path(location)} node_ids must be in {range_text}"
            )
        if len(set(parsed_nodes)) != len(parsed_nodes):
            raise MachineProfileError(f"{_path(location)} node_ids must be unique")
        node_ids = parsed_nodes

    if not has_transport:
        if (
            status != "absent"
            or master_id is not None
            or positions not in {()}
            or node_ids not in {()}
        ):
            raise MachineProfileError(
                f"{_path(location)} must be absent with null/empty layout "
                "when transport is unused"
            )
    elif status == "absent":
        raise MachineProfileError(
            f"{_path(location)} cannot be absent for a selected transport"
        )
    elif transport == "ethercat" and positions == ():
        raise MachineProfileError(
            f"{_path(location)} EtherCAT layout must contain at least one ring position"
        )
    elif transport != "ethercat" and node_ids == ():
        raise MachineProfileError(
            f"{_path(location)} selected transport must contain at least one "
            "node ID"
        )
    return BusLayout(status, master_id, positions, node_ids)


def _parse_hardware_options(
    raw_options: Any,
    location: Sequence[object],
    modules: Mapping[str, ModuleSpec],
) -> dict[str, HardwareOption]:
    options_mapping = _mapping(raw_options, location)
    options: dict[str, HardwareOption] = {}
    for raw_name, raw_option in options_mapping.items():
        name = _string(raw_name, (*location, "<name>"), identifier=True)
        option_location = (*location, name)
        option = _mapping(raw_option, option_location)
        _exact_keys(option, {"default_selection", "selections"}, option_location)
        default_selection = _string(
            option["default_selection"],
            (*option_location, "default_selection"),
            identifier=True,
        )
        raw_selections = _mapping(
            option["selections"], (*option_location, "selections")
        )
        if not raw_selections:
            raise MachineProfileError(
                f"{_path((*option_location, 'selections'))} must not be empty"
            )
        selections: dict[str, HardwareOptionSelection] = {}
        for raw_selection_name, raw_selection in raw_selections.items():
            selection_name = _string(
                raw_selection_name,
                (*option_location, "selections", "<name>"),
                identifier=True,
            )
            selection_location = (*option_location, "selections", selection_name)
            selection = _mapping(raw_selection, selection_location)
            _exact_keys(
                selection,
                {"modules", "ethercat_layout", "pending_facts"},
                selection_location,
            )
            selected_modules = _string_list(
                selection["modules"], (*selection_location, "modules")
            )
            if len(set(selected_modules)) != len(selected_modules):
                raise MachineProfileError(
                    f"{_path(selection_location)} contains duplicate module"
                )
            unknown = sorted(set(selected_modules) - set(modules))
            if unknown:
                raise MachineProfileError(
                    f"{_path(selection_location)} references unknown module(s): {unknown}"
                )
            if any(
                modules[module_name].role != "state_sensor_group"
                for module_name in selected_modules
            ):
                raise MachineProfileError(
                    f"{_path(selection_location)} optional modules must be state sensors"
                )
            raw_ethercat_layout = selection["ethercat_layout"]
            ethercat_layout = None
            if raw_ethercat_layout is not None:
                if not selected_modules or any(
                    modules[module_name].transport != "ethercat"
                    for module_name in selected_modules
                ):
                    raise MachineProfileError(
                        f"{_path(selection_location)} EtherCAT override requires "
                        "EtherCAT sensor modules"
                    )
                ethercat_layout = _parse_bus_layout(
                    raw_ethercat_layout,
                    (*selection_location, "ethercat_layout"),
                    transport="ethercat",
                    has_transport=True,
                )
                master_ids = {
                    modules[module_name].master_id for module_name in selected_modules
                }
                if len(master_ids) != 1 or ethercat_layout.master_id not in master_ids:
                    raise MachineProfileError(
                        f"{_path(selection_location)} EtherCAT master mismatch"
                    )
            elif selected_modules:
                raise MachineProfileError(
                    f"{_path(selection_location)} sensor modules require a layout override"
                )
            pending_facts = _parse_pending_facts(
                selection["pending_facts"], (*selection_location, "pending_facts")
            )
            selections[selection_name] = HardwareOptionSelection(
                selection_name, selected_modules, ethercat_layout, pending_facts
            )
        if default_selection not in selections:
            raise MachineProfileError(
                f"{_path(option_location)} default_selection is not declared"
            )
        options[name] = HardwareOption(name, default_selection, selections)
    return options


def _parse_profiles(
    raw_profiles: Any,
    location: Sequence[object],
    modules: Mapping[str, ModuleSpec],
    hardware_options: Mapping[str, HardwareOption],
) -> dict[str, PhysicalProfile]:
    profiles_mapping = _mapping(raw_profiles, location)
    if not profiles_mapping:
        raise MachineProfileError(f"{_path(location)} must not be empty")
    expected = {
        "status", "modules", "allowed_scopes", "hardware_options", "layout",
        "pending_facts",
    }
    profiles: dict[str, PhysicalProfile] = {}
    for raw_name, raw_profile in profiles_mapping.items():
        name = _string(raw_name, (*location, "<name>"), identifier=True)
        profile_location = (*location, name)
        profile = _mapping(raw_profile, profile_location)
        _exact_keys(profile, expected, profile_location)
        status = _status(profile["status"], (*profile_location, "status"))
        selected_modules = _string_list(
            profile["modules"], (*profile_location, "modules")
        )
        if not selected_modules:
            raise MachineProfileError(f"{_path(profile_location)}.modules must not be empty")
        if len(set(selected_modules)) != len(selected_modules):
            raise MachineProfileError(f"{_path(profile_location)} contains duplicate module")
        unknown = sorted(set(selected_modules) - set(modules))
        if unknown:
            raise MachineProfileError(
                f"{_path(profile_location)} references unknown module(s): {unknown}"
            )
        for module_name in selected_modules:
            if modules[module_name].status == "absent":
                raise MachineProfileError(
                    f"{_path(profile_location)} cannot include absent module {module_name!r}"
                )
            missing = sorted(set(modules[module_name].requires) - set(selected_modules))
            if missing:
                raise MachineProfileError(
                    f"{_path(profile_location)} module {module_name!r} requires "
                    f"module(s): {missing}"
                )
        allowed_scopes = _string_list(
            profile["allowed_scopes"], (*profile_location, "allowed_scopes")
        )
        if not allowed_scopes:
            raise MachineProfileError(
                f"{_path(profile_location)}.allowed_scopes must not be empty"
            )
        if len(set(allowed_scopes)) != len(allowed_scopes):
            raise MachineProfileError(
                f"{_path(profile_location)} allowed_scopes must be unique"
            )
        raw_profile_options = _mapping(
            profile["hardware_options"], (*profile_location, "hardware_options")
        )
        if set(raw_profile_options) != set(hardware_options):
            raise MachineProfileError(
                f"{_path(profile_location)} hardware_options must list every option"
            )
        allowed_options: dict[str, tuple[str, ...]] = {}
        for option_name, raw_selections in raw_profile_options.items():
            option = hardware_options[option_name]
            selections = _string_list(
                raw_selections, (*profile_location, "hardware_options", option_name)
            )
            if not selections or len(set(selections)) != len(selections):
                raise MachineProfileError(
                    f"{_path((*profile_location, 'hardware_options', option_name))} "
                    "must contain unique selections"
                )
            unknown_selections = sorted(set(selections) - set(option.selections))
            if unknown_selections:
                raise MachineProfileError(
                    f"{_path(profile_location)} references unknown option selections: "
                    f"{unknown_selections}"
                )
            if option.default_selection not in selections:
                raise MachineProfileError(
                    f"{_path(profile_location)} must allow the default "
                    f"{option_name} selection"
                )
            for selection_name in selections:
                optional_modules = option.selections[selection_name].modules
                combined_modules = set(selected_modules) | set(optional_modules)
                for module_name in optional_modules:
                    missing = sorted(
                        set(modules[module_name].requires) - combined_modules
                    )
                    if missing:
                        raise MachineProfileError(
                            f"{_path(profile_location)} optional module {module_name!r} "
                            f"requires module(s): {missing}"
                        )
                optional_master_ids = {
                    modules[module_name].master_id
                    for module_name in optional_modules
                    if modules[module_name].transport == "ethercat"
                }
                base_master_ids = {
                    modules[module_name].master_id
                    for module_name in selected_modules
                    if modules[module_name].transport == "ethercat"
                }
                if optional_master_ids and base_master_ids != optional_master_ids:
                    raise MachineProfileError(
                        f"{_path(profile_location)} optional EtherCAT modules must "
                        "share one master with the profile"
                    )
            allowed_options[option_name] = selections
        layout = _mapping(profile["layout"], (*profile_location, "layout"))
        _exact_keys(
            layout,
            {"ethercat", "canopen", "damiao_can"},
            (*profile_location, "layout"),
        )
        transports = {modules[module_name].transport for module_name in selected_modules}
        ethercat = _parse_bus_layout(
            layout["ethercat"], (*profile_location, "layout", "ethercat"),
            transport="ethercat",
            has_transport="ethercat" in transports,
        )
        canopen = _parse_bus_layout(
            layout["canopen"], (*profile_location, "layout", "canopen"),
            transport="canopen",
            has_transport="canopen" in transports,
        )
        damiao = _parse_bus_layout(
            layout["damiao_can"], (*profile_location, "layout", "damiao_can"),
            transport="damiao_can",
            has_transport="damiao_can" in transports,
        )
        ethercat_master_ids = {
            modules[module_name].master_id
            for module_name in selected_modules
            if modules[module_name].transport == "ethercat"
        }
        if len(ethercat_master_ids) > 1:
            raise MachineProfileError(
                f"{_path(profile_location)} EtherCAT modules must share one master"
            )
        if ethercat_master_ids and ethercat.master_id not in ethercat_master_ids:
            raise MachineProfileError(
                f"{_path(profile_location)} EtherCAT layout master_id must match "
                "module master_id"
            )
        pending_facts = _parse_pending_facts(
            profile["pending_facts"], (*profile_location, "pending_facts")
        )
        profiles[name] = PhysicalProfile(
            name=name,
            status=status,
            modules=selected_modules,
            allowed_scopes=allowed_scopes,
            hardware_options=allowed_options,
            layout=PhysicalLayout(ethercat, canopen, damiao),
            pending_facts=pending_facts,
        )
    return profiles


def _parse_group_references(
    value: Any, location: Sequence[object]
) -> tuple[GroupReference, ...]:
    values = _list(value, location)
    result: list[GroupReference] = []
    seen: set[tuple[str, str]] = set()
    for index, raw_reference in enumerate(values):
        reference_location = (*location, index)
        reference = _mapping(raw_reference, reference_location)
        _exact_keys(reference, {"module", "mode_group"}, reference_location)
        module = _string(
            reference["module"], (*reference_location, "module"), identifier=True
        )
        mode_group = _string(
            reference["mode_group"], (*reference_location, "mode_group"), identifier=True
        )
        key = (module, mode_group)
        if key in seen:
            raise MachineProfileError(
                f"{_path(location)} contains duplicate actuator group {key!r}"
            )
        seen.add(key)
        result.append(GroupReference(module, mode_group))
    return tuple(result)


def _parse_scopes(
    raw_scopes: Any,
    location: Sequence[object],
    modules: Mapping[str, ModuleSpec],
) -> dict[str, ScopeSpec]:
    scopes_mapping = _mapping(raw_scopes, location)
    if not scopes_mapping:
        raise MachineProfileError(f"{_path(location)} must not be empty")
    expected = {
        "enabled_modules",
        "required_state_modules",
        "actuator_groups",
        "jtc_groups",
        "global_readiness",
    }
    scopes: dict[str, ScopeSpec] = {}
    for raw_name, raw_scope in scopes_mapping.items():
        name = _string(raw_name, (*location, "<name>"), identifier=True)
        scope_location = (*location, name)
        scope = _mapping(raw_scope, scope_location)
        _exact_keys(scope, expected, scope_location)
        enabled = _string_list(
            scope["enabled_modules"], (*scope_location, "enabled_modules")
        )
        if not enabled:
            raise MachineProfileError(
                f"{_path(scope_location)}.enabled_modules must not be empty"
            )
        if len(set(enabled)) != len(enabled):
            raise MachineProfileError(
                f"{_path(scope_location)} enabled_modules must be unique"
            )
        unknown = sorted(set(enabled) - set(modules))
        if unknown:
            raise MachineProfileError(
                f"{_path(scope_location)} references unknown module(s): {unknown}"
            )
        for module_name in enabled:
            missing = sorted(set(modules[module_name].requires) - set(enabled))
            if missing:
                raise MachineProfileError(
                    f"{_path(scope_location)} module {module_name!r} requires module(s): {missing}"
                )
        required_states = _string_list(
            scope["required_state_modules"], (*scope_location, "required_state_modules")
        )
        if len(set(required_states)) != len(required_states):
            raise MachineProfileError(
                f"{_path(scope_location)} required_state_modules must be unique"
            )
        if not set(required_states).issubset(enabled):
            raise MachineProfileError(
                f"{_path(scope_location)} required state modules must be enabled"
            )
        for module_name in required_states:
            if modules[module_name].role != "state_sensor_group":
                raise MachineProfileError(
                    f"{_path(scope_location)} required state module {module_name!r} "
                    "is not a state sensor"
                )
        actuator_groups = _parse_group_references(
            scope["actuator_groups"], (*scope_location, "actuator_groups")
        )
        jtc_groups = _parse_group_references(
            scope["jtc_groups"], (*scope_location, "jtc_groups")
        )
        actuator_group_keys = {(item.module, item.mode_group) for item in actuator_groups}
        jtc_group_keys = {(item.module, item.mode_group) for item in jtc_groups}
        if not jtc_group_keys.issubset(actuator_group_keys):
            raise MachineProfileError(
                f"{_path(scope_location)} jtc_groups must be actuator_groups"
            )
        for item in actuator_groups + jtc_groups:
            if item.module not in enabled:
                raise MachineProfileError(
                    f"{_path(scope_location)} group references module {item.module!r} "
                    "outside enabled_modules"
                )
            module = modules[item.module]
            group_names = {group.name for group in module.mode_groups}
            if item.mode_group not in group_names:
                raise MachineProfileError(
                    f"{_path(scope_location)} references unknown mode group "
                    f"{item.module}.{item.mode_group}"
                )
        for item in jtc_groups:
            group = next(
                group
                for group in modules[item.module].mode_groups
                if group.name == item.mode_group
            )
            if group.mode_of_operation != 8 or group.interface != "position":
                raise MachineProfileError(
                    f"{_path(scope_location)} jtc_groups may contain only CSP position groups"
                )
        expected_groups = {
            (module.name, group.name)
            for module in modules.values()
            if module.name in enabled and module.role != "state_sensor_group"
            for group in module.mode_groups
        }
        if actuator_group_keys != expected_groups:
            missing = sorted(expected_groups - actuator_group_keys)
            extra = sorted(actuator_group_keys - expected_groups)
            raise MachineProfileError(
                f"{_path(scope_location)} actuator_groups must cover enabled actuators "
                "exactly: "
                f"missing={missing}, extra={extra}"
            )
        readiness = _string(
            scope["global_readiness"], (*scope_location, "global_readiness")
        )
        if readiness not in _READINESS:
            raise MachineProfileError(
                f"{_path(scope_location)} global_readiness must be one of {sorted(_READINESS)}"
            )
        scopes[name] = ScopeSpec(
            name=name,
            enabled_modules=enabled,
            required_state_modules=required_states,
            actuator_groups=actuator_groups,
            jtc_groups=jtc_groups,
            global_readiness=readiness,
        )
    return scopes


def _validate_profile_scope_links(
    profiles: Mapping[str, PhysicalProfile], scopes: Mapping[str, ScopeSpec]
) -> None:
    for profile in profiles.values():
        for scope_name in profile.allowed_scopes:
            if scope_name not in scopes:
                raise MachineProfileError(
                    f"{profile.name}.allowed_scopes references unknown scope {scope_name!r}"
                )
            scope = scopes[scope_name]
            if not set(scope.enabled_modules).issubset(profile.modules):
                missing = sorted(set(scope.enabled_modules) - set(profile.modules))
                raise MachineProfileError(
                    f"{profile.name} cannot allow {scope_name}: missing module(s) {missing}"
                )


def _parse_controllers(
    raw_controllers: Any,
    location: Sequence[object],
    modules: Mapping[str, ModuleSpec],
    scopes: Mapping[str, ScopeSpec],
) -> dict[str, ControllerBinding]:
    controllers = _mapping(raw_controllers, location)
    bindings: dict[str, ControllerBinding] = {}
    used_names: set[str] = set()
    for raw_key, raw_binding in controllers.items():
        key = _string(raw_key, (*location, "<name>"), identifier=True)
        entry = (*location, key)
        binding = _mapping(raw_binding, entry)
        _exact_keys(
            binding,
            {"module", "name", "plugin", "package", "config_file",
             "actuator_groups", "required_state_modules"},
            entry,
        )
        module_name = _string(binding["module"], (*entry, "module"), identifier=True)
        if module_name not in modules:
            raise MachineProfileError(f"{_path(entry)} references unknown module {module_name!r}")
        if key != module_name or modules[module_name].role == "state_sensor_group":
            raise MachineProfileError(f"{_path(entry)} must bind its actuator module")
        name = _string(binding["name"], (*entry, "name"), identifier=True)
        if name in used_names:
            raise MachineProfileError(f"{_path(entry)} duplicates controller name {name!r}")
        used_names.add(name)
        package = _string(binding["package"], (*entry, "package"), identifier=True)
        plugin = _string(binding["plugin"], (*entry, "plugin"))
        if re.fullmatch(rf"{re.escape(package)}/[A-Za-z][A-Za-z0-9]*", plugin) is None:
            raise MachineProfileError(f"{_path(entry)} plugin must belong to {package}")
        config_file = _string(binding["config_file"], (*entry, "config_file"))
        path = Path(config_file)
        if (
            path.is_absolute()
            or len(path.parts) < 2
            or any(part == ".." for part in path.parts)
            or path.suffix != ".yaml"
        ):
            raise MachineProfileError(f"{_path(entry)} config_file must be package-relative")
        groups = _string_list(binding["actuator_groups"], (*entry, "actuator_groups"))
        expected = {group.name for group in modules[module_name].mode_groups}
        if not groups or len(set(groups)) != len(groups) or set(groups) != expected:
            raise MachineProfileError(f"{_path(entry)} actuator_groups must cover module exactly")
        states = _string_list(
            binding["required_state_modules"], (*entry, "required_state_modules")
        )
        expected_states = {
            required for required in modules[module_name].requires
            if modules[required].role == "state_sensor_group"
        }
        if len(set(states)) != len(states) or set(states) != expected_states:
            raise MachineProfileError(f"{_path(entry)} required state modules must match module dependencies")
        for scope in scopes.values():
            if module_name not in scope.enabled_modules:
                continue
            selected = {item.mode_group for item in scope.actuator_groups if item.module == module_name}
            if selected != expected or not set(states).issubset(scope.required_state_modules):
                raise MachineProfileError(
                    f"{_path(entry)} requires all actuator groups and required state modules "
                    f"in scope {scope.name}"
                )
            if any(item.module == module_name for item in scope.jtc_groups):
                raise MachineProfileError(
                    f"{_path(entry)} may not share its actuator groups with JTC"
                )
        bindings[key] = ControllerBinding(
            module_name, name, plugin, package, config_file, groups, states
        )
    return bindings


def _parse_fault_dependencies(
    raw_policy: Any,
    location: Sequence[object],
    modules: Mapping[str, ModuleSpec],
) -> tuple[FaultDependency, ...]:
    policy = _mapping(raw_policy, location)
    _exact_keys(policy, {"fault_dependencies"}, location)
    raw_dependencies = _list(
        policy["fault_dependencies"], (*location, "fault_dependencies")
    )
    dependencies: list[FaultDependency] = []
    seen: set[tuple[str, tuple[str, ...]]] = set()
    for index, raw_dependency in enumerate(raw_dependencies):
        dependency_location = (*location, "fault_dependencies", index)
        dependency = _mapping(raw_dependency, dependency_location)
        _exact_keys(
            dependency,
            {"source_module", "affected_modules", "reaction"},
            dependency_location,
        )
        source = _string(
            dependency["source_module"],
            (*dependency_location, "source_module"),
            identifier=True,
        )
        affected = _string_list(
            dependency["affected_modules"],
            (*dependency_location, "affected_modules"),
        )
        reaction = _string(
            dependency["reaction"], (*dependency_location, "reaction")
        )
        if source not in modules:
            raise MachineProfileError(
                f"{_path(dependency_location)} references unknown source module {source!r}"
            )
        unknown = sorted(set(affected) - set(modules))
        if unknown:
            raise MachineProfileError(
                f"{_path(dependency_location)} references unknown affected module(s): {unknown}"
            )
        if not affected or len(set(affected)) != len(affected) or source in affected:
            raise MachineProfileError(
                f"{_path(dependency_location)} affected_modules must be unique, nonempty, "
                "and exclude the source"
            )
        if reaction not in _FAULT_REACTIONS:
            raise MachineProfileError(
                f"{_path(dependency_location)} reaction must be one of "
                f"{sorted(_FAULT_REACTIONS)}"
            )
        for module_name in (source, *affected):
            if modules[module_name].role == "state_sensor_group":
                raise MachineProfileError(
                    f"{_path(dependency_location)} fault dependencies require actuator modules"
                )
        key = (source, affected)
        if key in seen:
            raise MachineProfileError(
                f"{_path(dependency_location)} duplicates a fault dependency"
            )
        seen.add(key)
        dependencies.append(FaultDependency(source, affected, reaction))
    return tuple(dependencies)


def _validate_known_v3_contract(manifest: MachineManifest) -> None:
    if manifest.variant != "alfa_v3":
        return
    if manifest.robot_model != RobotModelSpec(
        package="robot_description",
        xacro_file="urdf/robot_dual_gripper.urdf.xacro",
        srdf_file="srdf/robot.srdf",
        joint_limits_file="config/joint_limits_gripper.yaml",
        initial_positions_file="config/initial_positions_gripper.yaml",
        end_effector="gripper",
        source_revision="ec69ca04297896c1296720324d23cdb8f80d9e63",
    ):
        raise MachineProfileError("alfa_v3 robot model binding is inconsistent")
    expected_modules = {
        "arms",
        "wrist_force_sensors",
        "updown",
        "swerve_chassis",
        "swerve_encoders",
        "active_suspension",
        "head_gimbal",
    }
    if set(manifest.modules) != expected_modules:
        raise MachineProfileError(
            f"alfa_v3 modules must be exactly {sorted(expected_modules)}"
        )
    expected_profiles = {
        "arms_only",
        "arms_updown",
        "chassis_only",
        "full_robot",
        "head_only",
    }
    if set(manifest.profiles) != expected_profiles:
        raise MachineProfileError(
            f"alfa_v3 profiles must be exactly {sorted(expected_profiles)}"
        )
    arms = manifest.modules["arms"]
    if (
        arms.transport != "ethercat"
        or arms.master_id != 0
        or arms.instance_count != 2
        or arms.actuator_count != 16
        or {group.name: group.count for group in arms.mode_groups}
        != {"csp": 14, "gripper_pp": 2}
        or dict(arms.per_instance_mode_groups) != {"csp": 7, "gripper_pp": 1}
    ):
        raise MachineProfileError("alfa_v3 arms must contain 14 CSP and 2 PP actuators")
    if {group.mode_of_operation for group in arms.mode_groups} != {8, 1}:
        raise MachineProfileError("alfa_v3 arms mode assignments must be CSP=8 and PP=1")
    force_sensors = manifest.modules["wrist_force_sensors"]
    if (
        force_sensors.role != "state_sensor_group"
        or force_sensors.transport != "ethercat"
        or force_sensors.master_id != 0
        or force_sensors.instance_count != 2
        or force_sensors.sensor_count != 2
        or force_sensors.authority != "wrist_wrench"
        or force_sensors.sensor_bindings != ("left_wrist", "right_wrist")
    ):
        raise MachineProfileError(
            "alfa_v3 wrist force sensors must contain two EtherCAT state sensors"
        )
    updown = manifest.modules["updown"]
    if (
        updown.transport != "ethercat"
        or updown.master_id != 0
        or updown.instance_count != 1
        or updown.actuator_count != 1
        or len(updown.mode_groups) != 1
        or updown.mode_groups[0].mode_of_operation != 8
    ):
        raise MachineProfileError("alfa_v3 updown must contain one CSP actuator")
    chassis = manifest.modules["swerve_chassis"]
    if (
        chassis.transport != "ethercat"
        or chassis.master_id != 0
        or chassis.instance_count != 4
        or chassis.actuator_count != 8
        or {group.name: group.count for group in chassis.mode_groups}
        != {"steering_csp": 4, "drive_csv": 4}
        or dict(chassis.per_instance_mode_groups)
        != {"steering_csp": 1, "drive_csv": 1}
    ):
        raise MachineProfileError(
            "alfa_v3 chassis must contain four steering and four drive actuators"
        )
    encoders = manifest.modules["swerve_encoders"]
    if (
        encoders.transport != "canopen"
        or encoders.instance_count != 4
        or encoders.sensor_count != 4
        or encoders.authority != "steering_angle"
        or encoders.sensor_bindings
        != ("front_left", "front_right", "rear_left", "rear_right")
    ):
        raise MachineProfileError("alfa_v3 chassis must have four steering-angle encoder sensors")
    suspension = manifest.modules["active_suspension"]
    if (
        suspension.transport != "ethercat"
        or suspension.master_id != 0
        or suspension.instance_count != 1
        or suspension.actuator_count != 1
        or len(suspension.mode_groups) != 1
        or suspension.mode_groups[0].name != "csp"
        or suspension.mode_groups[0].count != 1
        or suspension.mode_groups[0].mode_of_operation != 8
        or dict(suspension.per_instance_mode_groups) != {"csp": 1}
    ):
        raise MachineProfileError(
            "alfa_v3 active_suspension must contain one CSP actuator"
        )
    head = manifest.modules["head_gimbal"]
    if (
        head.transport != "damiao_can"
        or head.actuator_count != 2
        or head.instance_count != 2
        or len(head.mode_groups) != 1
        or head.mode_groups[0].name != "vendor_can"
        or head.mode_groups[0].count != 2
        or head.mode_groups[0].mode_of_operation is not None
        or dict(head.per_instance_mode_groups) != {"vendor_can": 1}
    ):
        raise MachineProfileError("alfa_v3 head_gimbal must contain two actuators")
    expected_profile_modules = {
        "arms_only": ("arms",),
        "arms_updown": ("arms", "updown"),
        "chassis_only": (
            "swerve_chassis", "swerve_encoders", "active_suspension"
        ),
        "full_robot": (
            "arms",
            "updown",
            "swerve_chassis",
            "swerve_encoders",
            "active_suspension",
            "head_gimbal",
        ),
        "head_only": ("head_gimbal",),
    }
    for name, expected in expected_profile_modules.items():
        if manifest.profiles[name].modules != expected:
            raise MachineProfileError(f"alfa_v3 profile {name} has an unexpected module set")
    expected_scopes = {
        "arms_only": ("arms_only",),
        "arms_updown": ("arms_updown",),
        "chassis_only": ("chassis_only",),
        "full_robot": ("full",),
        "head_only": ("head_only",),
    }
    for name, expected in expected_scopes.items():
        if manifest.profiles[name].allowed_scopes != expected:
            raise MachineProfileError(
                f"alfa_v3 profile {name} must enable every physically present actuator"
            )
    if set(manifest.hardware_options) != {"force_sensors"}:
        raise MachineProfileError("alfa_v3 requires the force_sensors hardware option")
    force_option = manifest.hardware_options["force_sensors"]
    if (
        force_option.default_selection != "none"
        or set(force_option.selections) != {"none", "bluepoint_dual"}
        or force_option.selections["none"].modules
        or force_option.selections["none"].ethercat_layout is not None
        or force_option.selections["bluepoint_dual"].modules !=
        ("wrist_force_sensors",)
        or force_option.selections["bluepoint_dual"].ethercat_layout is None
    ):
        raise MachineProfileError("alfa_v3 force_sensors option is inconsistent")
    dual_profiles = {"arms_only", "arms_updown", "full_robot"}
    for profile_name, profile in manifest.profiles.items():
        expected = (
            ("none", "bluepoint_dual")
            if profile_name in dual_profiles else ("none",)
        )
        if profile.hardware_options["force_sensors"] != expected:
            raise MachineProfileError(
                f"alfa_v3 profile {profile_name} has invalid force_sensors choices"
            )
    if set(manifest.controllers) != {"swerve_chassis"}:
        raise MachineProfileError("alfa_v3 requires the swerve chassis controller binding")
    controller = manifest.controllers["swerve_chassis"]
    if (
        controller.plugin != "swerve_driver/SwerveController"
        or controller.package != "swerve_driver"
        or controller.name != "swerve_controller"
        or controller.actuator_groups != ("steering_csp", "drive_csv")
        or controller.required_state_modules != ("swerve_encoders",)
    ):
        raise MachineProfileError("alfa_v3 swerve controller binding is inconsistent")
    expected_dependencies = (
        FaultDependency(
            "active_suspension", ("swerve_chassis",), "stop_and_inhibit"
        ),
        FaultDependency("swerve_chassis", ("arms",), "stop"),
    )
    if manifest.fault_dependencies != expected_dependencies:
        raise MachineProfileError("alfa_v3 fault dependency policy is inconsistent")


def load_machine_manifest(path: str | Path) -> MachineManifest:
    """Load and validate one machine manifest without mutating its source."""
    manifest_path = Path(path)
    document = _load_yaml(manifest_path)
    root_location = (manifest_path.name,)
    required_keys = {
        "schema_version",
        "robot_variant",
        "status",
        "robot_model",
        "functional_modules",
        "modules",
        "hardware_options",
        "profiles",
        "scopes",
        "safety_policy",
        "owner_refs",
    }
    _exact_keys(
        document,
        required_keys | ({"controllers"} if "controllers" in document else set()),
        root_location,
    )
    schema_version = _integer(
        document["schema_version"], (*root_location, "schema_version"), minimum=1
    )
    if schema_version != 1:
        raise MachineProfileError(f"{_path(root_location)} schema_version must be 1")
    variant = _string(
        document["robot_variant"], (*root_location, "robot_variant"), identifier=True
    )
    status = _status(document["status"], (*root_location, "status"))
    robot_model = _parse_robot_model(
        document["robot_model"], (*root_location, "robot_model")
    )
    functional_modules = _string_list(
        document["functional_modules"], (*root_location, "functional_modules")
    )
    if not functional_modules:
        raise MachineProfileError(
            f"{_path(root_location)}.functional_modules must not be empty"
        )
    if len(set(functional_modules)) != len(functional_modules):
        raise MachineProfileError(
            f"{_path(root_location)}.functional_modules must be unique"
        )
    modules = _parse_modules(document["modules"], (*root_location, "modules"))
    unknown_functional = sorted(set(functional_modules) - set(modules))
    if unknown_functional:
        raise MachineProfileError(
            f"{_path(root_location)}.functional_modules references unknown module(s): "
            f"{unknown_functional}"
        )
    sensor_functional = [
        name for name in functional_modules if modules[name].role == "state_sensor_group"
    ]
    if sensor_functional:
        raise MachineProfileError(
            "functional_modules must contain actuator modules only: "
            f"{sensor_functional}"
        )
    hardware_options = _parse_hardware_options(
        document["hardware_options"], (*root_location, "hardware_options"), modules
    )
    profiles = _parse_profiles(
        document["profiles"], (*root_location, "profiles"), modules,
        hardware_options,
    )
    scopes = _parse_scopes(document["scopes"], (*root_location, "scopes"), modules)
    controllers = _parse_controllers(
        document.get("controllers", {}), (*root_location, "controllers"), modules, scopes
    )
    fault_dependencies = _parse_fault_dependencies(
        document["safety_policy"], (*root_location, "safety_policy"), modules
    )
    owner_refs_raw = _mapping(document["owner_refs"], (*root_location, "owner_refs"))
    owner_refs: dict[str, str] = {}
    for raw_name, raw_owner in owner_refs_raw.items():
        name = _string(
            raw_name, (*root_location, "owner_refs", "<name>"), identifier=True
        )
        owner_refs[name] = _string(raw_owner, (*root_location, "owner_refs", name))
    manifest = MachineManifest(
        path=manifest_path,
        variant=variant,
        status=status,
        robot_model=robot_model,
        functional_modules=functional_modules,
        modules=modules,
        hardware_options=hardware_options,
        profiles=profiles,
        scopes=scopes,
        controllers=controllers,
        fault_dependencies=fault_dependencies,
        owner_refs=owner_refs,
    )
    _validate_profile_scope_links(profiles, scopes)
    _validate_known_v3_contract(manifest)
    if variant == "alfa_v3" and set(owner_refs) != {
        "ethercat",
        "canopen",
        "damiao_can",
    }:
        raise MachineProfileError(
            "alfa_v3 owner_refs must declare ethercat, canopen, and damiao_can"
        )
    if variant == "alfa_v3" and manifest.functional_modules != (
        "arms",
        "updown",
        "swerve_chassis",
        "active_suspension",
        "head_gimbal",
    ):
        raise MachineProfileError(
            "alfa_v3 functional_modules must contain the five actuator modules"
        )
    return manifest


def _runtime_blockers(
    manifest: MachineManifest, profile: PhysicalProfile, scope: ScopeSpec,
    active_modules: tuple[str, ...], layout: PhysicalLayout,
    option_pending_facts: tuple[str, ...],
) -> tuple[str, ...]:
    blockers: list[str] = []
    if manifest.status != "ready":
        blockers.append(f"manifest status is {manifest.status}")
    if profile.status != "ready":
        blockers.append(f"profile {profile.name} status is {profile.status}")
    if profile.pending_facts:
        blockers.append(f"profile {profile.name} has pending facts")
    active_module_names = set(active_modules)
    selected_group_keys = {
        (item.module, item.mode_group) for item in scope.actuator_groups
    }
    for module_name in active_modules:
        module = manifest.modules[module_name]
        if module.status != "ready":
            blockers.append(f"module {module_name} status is {module.status}")
        if module.pending_facts:
            blockers.append(f"module {module_name} has pending facts")
        if any(
            _contains_unresolved_reference(value)
            for value in (
                module.hardware_identity,
                module.profile_ref,
                module.mechanical_parameters,
            )
        ):
            blockers.append(f"module {module_name} contains TBD or unverified values")
        for group in module.mode_groups:
            if (module_name, group.name) in selected_group_keys and not group.certified:
                blockers.append(f"mode group {module_name}.{group.name} is not certified")
        controller = manifest.controllers.get(module_name)
        if controller is not None and controller.config_file.endswith(".draft.yaml"):
            blockers.append(f"controller {controller.name} uses a draft config")

    transport_layouts = {
        "ethercat": layout.ethercat,
        "canopen": layout.canopen,
        "damiao_can": layout.damiao_can,
    }
    active_transports = {
        manifest.modules[name].transport for name in active_module_names
    }
    for transport in sorted(active_transports):
        layout = transport_layouts[transport]
        if layout.status != "ready":
            blockers.append(f"{transport} layout status is {layout.status}")
        if transport == "ethercat" and layout.positions is None:
            blockers.append("EtherCAT ring positions are TBD")
        if transport != "ethercat" and layout.node_ids is None:
            blockers.append(f"{transport} node IDs are TBD")
    if option_pending_facts:
        blockers.append("selected hardware option has pending facts")
    return tuple(dict.fromkeys(blockers))


def select_hardware(
    manifest_or_path: MachineManifest | str | Path,
    *,
    physical_profile: str,
    control_scope: str,
    hardware_options: Mapping[str, str] | None = None,
    require_runtime_ready: bool = False,
) -> SelectedHardware:
    """Select an immutable hardware surface from a physical profile and scope."""
    manifest = (
        manifest_or_path
        if isinstance(manifest_or_path, MachineManifest)
        else load_machine_manifest(manifest_or_path)
    )
    if physical_profile not in manifest.profiles:
        raise MachineProfileError(f"unknown physical profile {physical_profile!r}")
    profile = manifest.profiles[physical_profile]
    if control_scope not in manifest.scopes:
        raise MachineProfileError(f"unknown control scope {control_scope!r}")
    scope = manifest.scopes[control_scope]
    if control_scope not in profile.allowed_scopes:
        raise MachineProfileError(
            f"control scope {control_scope!r} is not allowed for physical profile "
            f"{physical_profile!r}"
        )

    requested_options = dict(hardware_options or {})
    unknown_options = sorted(set(requested_options) - set(manifest.hardware_options))
    if unknown_options:
        raise MachineProfileError(f"unknown hardware option(s): {unknown_options}")
    selected_options: dict[str, str] = {}
    optional_modules: list[str] = []
    option_pending_facts: list[str] = []
    selected_layout = profile.layout
    for option_name, option in manifest.hardware_options.items():
        selection_name = requested_options.get(option_name, option.default_selection)
        if selection_name not in profile.hardware_options[option_name]:
            raise MachineProfileError(
                f"hardware option {option_name}={selection_name} is not allowed for "
                f"physical profile {physical_profile}"
            )
        selection = option.selections[selection_name]
        selected_options[option_name] = selection_name
        optional_modules.extend(selection.modules)
        option_pending_facts.extend(selection.pending_facts)
        if selection.ethercat_layout is not None:
            selected_layout = PhysicalLayout(
                selection.ethercat_layout,
                selected_layout.canopen,
                selected_layout.damiao_can,
            )
    physical_modules = tuple((*profile.modules, *optional_modules))
    if len(set(physical_modules)) != len(physical_modules):
        raise MachineProfileError("selected hardware options duplicate a physical module")
    active_modules = tuple((*scope.enabled_modules, *optional_modules))
    inactive_modules = tuple(name for name in physical_modules if name not in active_modules)
    transports: list[str] = []
    for module_name in active_modules:
        transport = manifest.modules[module_name].transport
        if transport not in transports:
            transports.append(transport)
    bus_requirements = {
        "ethercat": (
            "not_required"
            if selected_layout.ethercat.status == "absent" else "required"
        ),
        "canopen": (
            "not_required"
            if selected_layout.canopen.status == "absent" else "required"
        ),
        "damiao_can": (
            "not_required"
            if selected_layout.damiao_can.status == "absent" else "required"
        ),
    }
    ethercat_modules = [
        manifest.modules[name]
        for name in active_modules
        if manifest.modules[name].transport == "ethercat"
    ]
    ethercat_master_id = ethercat_modules[0].master_id if ethercat_modules else None
    ethercat_ring_positions = selected_layout.ethercat.positions
    canopen_node_ids = selected_layout.canopen.node_ids
    damiao_can_node_ids = selected_layout.damiao_can.node_ids
    actuator_count = sum(manifest.modules[name].actuator_count for name in active_modules)
    mode_counts: dict[int, int] = {}
    group_counts: dict[str, int] = {}
    for item in scope.actuator_groups:
        group = next(
            group
            for group in manifest.modules[item.module].mode_groups
            if group.name == item.mode_group
        )
        group_counts[f"{item.module}.{item.mode_group}"] = group.count
        if group.mode_of_operation is not None:
            mode_counts[group.mode_of_operation] = (
                mode_counts.get(group.mode_of_operation, 0) + group.count
            )
    jtc_mode_counts: dict[int, int] = {}
    for item in scope.jtc_groups:
        group = next(
            group
            for group in manifest.modules[item.module].mode_groups
            if group.name == item.mode_group
        )
        if group.mode_of_operation is not None:
            jtc_mode_counts[group.mode_of_operation] = (
                jtc_mode_counts.get(group.mode_of_operation, 0) + group.count
            )
    required_state_modules = tuple((*scope.required_state_modules, *optional_modules))
    state_sensor_count = sum(
        manifest.modules[name].sensor_count for name in required_state_modules
    )
    controllers = tuple(
        manifest.controllers[name]
        for name in active_modules if name in manifest.controllers
    )
    active_module_names = set(active_modules)
    selected_fault_dependencies: list[FaultDependency] = []
    for dependency in manifest.fault_dependencies:
        affected = tuple(
            module_name
            for module_name in dependency.affected_modules
            if module_name in active_module_names
        )
        if dependency.source_module in active_module_names and affected:
            selected_fault_dependencies.append(
                FaultDependency(
                    dependency.source_module, affected, dependency.reaction
                )
            )
    blockers = _runtime_blockers(
        manifest, profile, scope, active_modules, selected_layout,
        tuple(option_pending_facts),
    )
    if require_runtime_ready and blockers:
        raise MachineProfileError(
            "selected hardware is not runtime-ready: " + "; ".join(blockers)
        )
    return SelectedHardware(
        manifest_variant=manifest.variant,
        physical_profile=physical_profile,
        control_scope=control_scope,
        hardware_options=selected_options,
        physical_modules=physical_modules,
        active_modules=active_modules,
        inactive_modules=inactive_modules,
        transports=tuple(transports),
        bus_requirements=bus_requirements,
        ethercat_master_id=ethercat_master_id,
        ethercat_ring_positions=ethercat_ring_positions,
        canopen_node_ids=canopen_node_ids,
        damiao_can_node_ids=damiao_can_node_ids,
        actuator_count=actuator_count,
        mode_counts=mode_counts,
        group_counts=group_counts,
        actuator_groups=scope.actuator_groups,
        jtc_groups=scope.jtc_groups,
        jtc_mode_counts=jtc_mode_counts,
        required_state_modules=required_state_modules,
        state_sensor_count=state_sensor_count,
        controllers=controllers,
        fault_dependencies=tuple(selected_fault_dependencies),
        validation_status="ready" if not blockers else "draft",
        global_readiness=scope.global_readiness,
        runtime_blockers=blockers,
    )


__all__ = [
    "BusLayout",
    "ControllerBinding",
    "FaultDependency",
    "GroupReference",
    "HardwareOption",
    "HardwareOptionSelection",
    "MachineManifest",
    "MachineProfileError",
    "ModeGroup",
    "ModuleSpec",
    "PhysicalLayout",
    "PhysicalProfile",
    "RobotModelSpec",
    "ScopeSpec",
    "SelectedHardware",
    "load_machine_manifest",
    "select_hardware",
]
