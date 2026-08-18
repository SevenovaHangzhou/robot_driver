#!/usr/bin/env python3
"""Validate structured runtime projections of the driver-variant manifest.

Phase 1 of ELECTRI-94 keeps the existing ROS/Xacro/YAML assets as runtime
inputs.  This module makes their duplication fail closed: it parses each
structured asset without importing ROS code or touching hardware and compares
the safety-relevant topology, ordering, mode and effective scale fields with
the versioned manifest.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

from tools import driver_variant


ECAT_XACRO = "src/rt_control/robot_hw_ethercat/urdf/ecat.ros2_control.xacro"
CONTROLLERS = "src/rt_control/rt_control_bringup/config/controllers.yaml"
CANOPEN_BUS = "src/rt_control/robot_hw_canopen/config/bus.yml"
CANOPEN_XACRO = (
    "src/rt_control/robot_hw_canopen/urdf/canopen.ros2_control.xacro"
)
MOCK_XACRO = "src/rt_control/rt_control_bringup/urdf/mock.ros2_control.xacro"
ROBOT_DESCRIPTION = "src/description/robot_description/urdf/robot.urdf.xacro"

_STATIC_SOURCES = (
    ECAT_XACRO,
    CONTROLLERS,
    CANOPEN_BUS,
    CANOPEN_XACRO,
    MOCK_XACRO,
    ROBOT_DESCRIPTION,
)
_SLAVE_SENSOR = re.compile(r"ethercat_slave_(\d+)\Z")
_XACRO_NS = "http://www.ros.org/wiki/xacro"


@dataclass(frozen=True)
class ConversionProjection:
    interface: str
    to_device: float
    from_device: float


@dataclass(frozen=True)
class ProfileProjection:
    identifier: str
    family: str
    bus: str
    config: str
    command_interfaces: tuple[str, ...]
    state_interfaces: tuple[str, ...]


@dataclass(frozen=True)
class EthercatAxisProjection:
    joint: str
    actuator: str
    ring_position: int
    profile: str
    transmission: str
    conversions: tuple[ConversionProjection, ...]


@dataclass(frozen=True)
class CanopenAxisProjection:
    joint: str
    actuator: str
    controller_role: str
    node_id: int
    operation_mode: int
    profile: str
    transmission: str
    conversions: tuple[ConversionProjection, ...]


@dataclass(frozen=True)
class ProjectionView:
    variant_id: str
    ethercat_master_id: int
    ring_positions: tuple[int, ...]
    ethercat_axes: tuple[EthercatAxisProjection, ...]
    canopen_interface: str
    canopen_master_node_id: int
    canopen_axes: tuple[CanopenAxisProjection, ...]
    whole_body_joints: tuple[str, ...]
    whole_body_si_units: tuple[str, ...]
    position_tolerances: tuple[float, ...]
    ready_to_switch_on_disable_terminal_joints: tuple[str, ...]
    profiles: tuple[ProfileProjection, ...]


def _conversion_tuple(transmission: Mapping[str, Any]) -> tuple[ConversionProjection, ...]:
    conversions = transmission["effective_conversion"]
    return tuple(
        ConversionProjection(
            interface=str(interface),
            to_device=float(values["to_device"]),
            from_device=float(values["from_device"]),
        )
        for interface, values in conversions.items()
    )


def derive_projection(document: Mapping[str, Any]) -> ProjectionView:
    """Normalize a validated manifest into one immutable comparison view."""
    profiles = {
        entry["id"]: ProfileProjection(
            identifier=entry["id"],
            family=entry["family"],
            bus=entry["bus"],
            config=entry["config"],
            command_interfaces=tuple(entry["command_interfaces"]),
            state_interfaces=tuple(entry["state_interfaces"]),
        )
        for entry in document["drive_profiles"]
    }
    transmissions = {
        entry["id"]: _conversion_tuple(entry)
        for entry in document["transmissions"]
    }
    transmission_units = {
        entry["id"]: str(entry["si_unit"])
        for entry in document["transmissions"]
    }
    actuators = {entry["id"]: entry for entry in document["actuators"]}
    joints_by_actuator = {entry["actuator"]: entry for entry in document["joints"]}

    ethercat_axes: list[EthercatAxisProjection] = []
    ring = document["buses"]["ethercat"]["ring"]
    for entry in ring:
        if entry["role"] != "drive":
            continue
        actuator_id = entry["actuator"]
        actuator = actuators[actuator_id]
        joint = joints_by_actuator[actuator_id]
        ethercat_axes.append(
            EthercatAxisProjection(
                joint=joint["name"],
                actuator=actuator_id,
                ring_position=int(entry["position"]),
                profile=actuator["drive_profile"],
                transmission=actuator["transmission"],
                conversions=transmissions[actuator["transmission"]],
            )
        )

    canopen_axes: list[CanopenAxisProjection] = []
    for entry in document["buses"]["canopen"]["nodes"]:
        actuator_id = entry["actuator"]
        actuator = actuators[actuator_id]
        joint = joints_by_actuator[actuator_id]
        canopen_axes.append(
            CanopenAxisProjection(
                joint=joint["name"],
                actuator=actuator_id,
                controller_role=joint["controller_role"],
                node_id=int(entry["node_id"]),
                operation_mode=int(entry["operation_mode"]),
                profile=actuator["drive_profile"],
                transmission=actuator["transmission"],
                conversions=transmissions[actuator["transmission"]],
            )
        )

    whole_body = [
        entry
        for entry in document["joints"]
        if entry["controller_role"] == "whole_body_trajectory"
    ]
    policies = {
        entry["id"]: set(entry["disable_terminals"])
        for entry in document["enable"]["lifecycle_policies"]
    }
    return ProjectionView(
        variant_id=document["variant_id"],
        ethercat_master_id=int(document["buses"]["ethercat"]["master_id"]),
        ring_positions=tuple(int(entry["position"]) for entry in ring),
        ethercat_axes=tuple(ethercat_axes),
        canopen_interface=document["buses"]["canopen"]["interface"],
        canopen_master_node_id=int(
            document["buses"]["canopen"]["master_node_id"]
        ),
        canopen_axes=tuple(canopen_axes),
        whole_body_joints=tuple(entry["name"] for entry in whole_body),
        whole_body_si_units=tuple(
            transmission_units[
                actuators[entry["actuator"]]["transmission"]
            ]
            for entry in whole_body
        ),
        position_tolerances=tuple(
            float(entry["position_tolerance"]) for entry in whole_body
        ),
        ready_to_switch_on_disable_terminal_joints=tuple(
            entry["name"]
            for entry in whole_body
            if "ready_to_switch_on"
            in policies[entry["lifecycle_policy"]]
        ),
        profiles=tuple(profiles.values()),
    )


def _required_sources(view: ProjectionView) -> tuple[str, ...]:
    profiles = tuple(
        profile.config for profile in view.profiles if profile.bus == "ethercat"
    )
    return _STATIC_SOURCES + profiles


def load_structured_sources(
    document: Mapping[str, Any], repository_root: Path
) -> dict[str, str]:
    """Read every phase-1 structured projection without executing it."""
    view = derive_projection(document)
    root = repository_root.resolve()
    findings: list[str] = []
    sources: dict[str, str] = {}
    for relative in _required_sources(view):
        path = (root / relative).resolve()
        try:
            path.relative_to(root)
        except ValueError:
            findings.append(f"{relative}: projection path escapes repository root")
            continue
        try:
            sources[relative] = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            findings.append(f"{relative}: cannot read projection: {exc}")
    if findings:
        raise driver_variant.DriverVariantError(findings)
    return sources


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _namespace(tag: str) -> str | None:
    if not tag.startswith("{") or "}" not in tag:
        return None
    return tag[1:].split("}", 1)[0]


def _xacro_tag(local_name: str) -> str:
    return f"{{{_XACRO_NS}}}{local_name}"


def _check_xacro_argument(
    root: ET.Element,
    name: str,
    expected_default: str,
    path: str,
    findings: list[str],
) -> None:
    matches = [
        child
        for child in root
        if child.tag == _xacro_tag("arg") and child.attrib.get("name") == name
    ]
    if len(matches) != 1:
        findings.append(
            f"{path}: xacro argument {name!r} must occur exactly once, "
            f"found {len(matches)}"
        )
        return
    actual_default = matches[0].attrib.get("default")
    if actual_default != expected_default:
        findings.append(
            f"{path}: xacro argument {name!r} default {actual_default!r} "
            f"does not match required {expected_default!r}"
        )


def _parse_xml(text: str, path: str, findings: list[str]) -> ET.Element | None:
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        findings.append(f"{path}: invalid XML: {exc}")
        return None
    if root.tag != "robot":
        findings.append(f"{path}: XML root must be an unnamespaced robot element")
    invalid_namespaces = sorted(
        {
            namespace
            for element in root.iter()
            if (namespace := _namespace(element.tag)) is not None
            and namespace != _XACRO_NS
        }
    )
    if invalid_namespaces:
        findings.append(
            f"{path}: xacro elements must use the canonical xacro namespace "
            f"{_XACRO_NS!r}, found {invalid_namespaces!r}"
        )
    return root


def _parse_yaml(
    text: str, path: str, findings: list[str]
) -> Mapping[str, Any] | None:
    try:
        document = driver_variant.parse_yaml_document(text, path)
    except driver_variant.DriverVariantError as exc:
        findings.extend(exc.findings)
        return None
    if not isinstance(document, Mapping):
        findings.append(f"{path}: YAML root must be a mapping")
        return None
    return document


def _ros2_control(
    root: ET.Element,
    name: str,
    path: str,
    findings: list[str],
) -> ET.Element | None:
    matches = [
        element
        for element in root
        if element.tag == "ros2_control" and element.attrib.get("name") == name
    ]
    if len(matches) != 1:
        findings.append(
            f"{path}: expected exactly one direct child ros2_control system "
            f"named '{name}', "
            f"found {len(matches)}"
        )
        return None
    return matches[0]


def _direct_children(parent: ET.Element, local_name: str) -> list[ET.Element]:
    return [child for child in parent if _local_name(child.tag) == local_name]


def _check_robot_description(
    text: str, view: ProjectionView, findings: list[str]
) -> None:
    root = _parse_xml(text, ROBOT_DESCRIPTION, findings)
    if root is None:
        return
    dynamic_xacro_elements = [
        _local_name(element.tag)
        for element in root.iter()
        if _namespace(element.tag) is not None
    ]
    if dynamic_xacro_elements:
        findings.append(
            f"{ROBOT_DESCRIPTION}: robot_description must remain statically "
            "enumerable; xacro elements are not supported by this phase-1 gate "
            f"({dynamic_xacro_elements!r})"
        )
    active_joint_elements = [
        element
        for element in root.iter()
        if element.tag == "joint"
        and element.attrib.get("type") not in {"fixed", "floating"}
    ]
    direct_active_joint_elements = [
        child
        for child in root
        if child.tag == "joint"
        and child.attrib.get("type") not in {"fixed", "floating"}
    ]
    direct_joint_ids = {id(element) for element in direct_active_joint_elements}
    nested_active_joints = [
        element.attrib.get("name", "")
        for element in active_joint_elements
        if id(element) not in direct_joint_ids
    ]
    if nested_active_joints:
        findings.append(
            f"{ROBOT_DESCRIPTION}: active joints must be direct robot children; "
            f"nested joints {nested_active_joints!r}"
        )
    dynamic_joint_attributes = [
        (element.attrib.get("name", ""), element.attrib.get("type", ""))
        for element in active_joint_elements
        if any(
            token in element.attrib.get(attribute, "")
            for attribute in ("name", "type")
            for token in ("${", "$(")
        )
    ]
    if dynamic_joint_attributes:
        findings.append(
            f"{ROBOT_DESCRIPTION}: active joint name/type must remain literal; "
            f"dynamic attributes are not supported {dynamic_joint_attributes!r}"
        )
    active_joint_names = [
        child.attrib.get("name", "") for child in direct_active_joint_elements
    ]
    duplicate_names = sorted(
        name
        for name, count in Counter(active_joint_names).items()
        if name and count > 1
    )
    if duplicate_names:
        findings.append(
            f"{ROBOT_DESCRIPTION}: duplicate active joints {duplicate_names!r}"
        )
    expected = set(view.whole_body_joints)
    actual = set(active_joint_names)
    if actual != expected:
        findings.append(
            f"{ROBOT_DESCRIPTION}: robot_description active joints "
            f"{sorted(actual)!r} do not match whole-body manifest "
            f"{sorted(expected)!r}"
        )
    active_joint_types = {
        child.attrib.get("name", ""): child.attrib.get("type", "")
        for child in direct_active_joint_elements
    }
    expected_units_by_type = {
        "revolute": "rad",
        "continuous": "rad",
        "prismatic": "m",
    }
    for joint, actual_unit in zip(
        view.whole_body_joints, view.whole_body_si_units, strict=True
    ):
        joint_type = active_joint_types.get(joint)
        expected_unit = expected_units_by_type.get(joint_type or "")
        if joint_type is not None and expected_unit is None:
            findings.append(
                f"{ROBOT_DESCRIPTION}: joint '{joint}' has unsupported active type "
                f"'{joint_type}'"
            )
        elif expected_unit is not None and actual_unit != expected_unit:
            findings.append(
                f"{ROBOT_DESCRIPTION}: joint '{joint}' type '{joint_type}' requires "
                f"SI unit '{expected_unit}', got '{actual_unit}'"
            )


def _parameter_text(
    parent: ET.Element,
    name: str,
    context: str,
    findings: list[str],
) -> str | None:
    matches = [
        child
        for child in parent
        if _local_name(child.tag) == "param" and child.attrib.get("name") == name
    ]
    if len(matches) != 1:
        findings.append(
            f"{context}: parameter '{name}' must occur exactly once, found {len(matches)}"
        )
        return None
    return (matches[0].text or "").strip()


def _as_int(value: str | None) -> int | None:
    if value is None:
        return None
    try:
        return int(value, 10)
    except ValueError:
        return None


def _is_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _number_matches(actual: object, expected: float) -> bool:
    return _is_number(actual) and float(actual) == float(expected)


def _sensor_positions(system: ET.Element) -> list[int | str]:
    positions: list[int | str] = []
    for sensor in _direct_children(system, "sensor"):
        name = sensor.attrib.get("name", "")
        match = _SLAVE_SENSOR.fullmatch(name)
        if match:
            positions.append(int(match.group(1)))
        elif name.startswith("ethercat_slave_"):
            positions.append(name)
    return positions


def _interface_names(parent: ET.Element, kind: str) -> tuple[str, ...]:
    return tuple(
        child.attrib.get("name", "")
        for child in parent
        if child.tag == kind
    )


def _check_sensor_interfaces(
    system: ET.Element,
    sensor_name: str,
    expected: tuple[str, ...],
    path: str,
    findings: list[str],
) -> None:
    sensors = [
        child
        for child in system
        if child.tag == "sensor" and child.attrib.get("name") == sensor_name
    ]
    if len(sensors) != 1:
        findings.append(
            f"{path}: {sensor_name} sensor must occur exactly once, found {len(sensors)}"
        )
        return
    actual = _interface_names(sensors[0], "state_interface")
    if actual != expected:
        findings.append(
            f"{path}: {sensor_name} interfaces {actual!r} do not match "
            f"required {expected!r}"
        )


def _check_ecat_xacro(
    text: str, view: ProjectionView, findings: list[str]
) -> None:
    root = _parse_xml(text, ECAT_XACRO, findings)
    if root is None:
        return
    _check_xacro_argument(
        root,
        "config_dir",
        "$(find robot_hw_ethercat)/config",
        ECAT_XACRO,
        findings,
    )
    system = _ros2_control(root, "ecat_arms", ECAT_XACRO, findings)
    if system is None:
        return

    hardware = _direct_children(system, "hardware")
    if len(hardware) != 1:
        findings.append(f"{ECAT_XACRO}: EtherCAT hardware must occur exactly once")
    else:
        actual_master = _as_int(
            _parameter_text(
                hardware[0], "master_id", "EtherCAT hardware", findings
            )
        )
        if actual_master != view.ethercat_master_id:
            findings.append(
                f"{ECAT_XACRO}: EtherCAT master_id {actual_master!r} does not match "
                f"manifest {view.ethercat_master_id}"
            )

    invocations = [
        child
        for child in system
        if _namespace(child.tag) == _XACRO_NS
        and _local_name(child.tag).endswith("_axis")
    ]
    for child in system:
        if child.tag in {"hardware", "sensor"} or child in invocations:
            continue
        if child.tag == "joint":
            findings.append(
                f"{ECAT_XACRO}: undeclared raw joint "
                f"{child.attrib.get('name')!r} in ecat_arms"
            )
        else:
            findings.append(
                f"{ECAT_XACRO}: undeclared ros2_control resource "
                f"{_local_name(child.tag)!r} in ecat_arms"
            )
    actual_joint_order = [item.attrib.get("joint_name") for item in invocations]
    expected_joint_order = [axis.joint for axis in view.ethercat_axes]
    if actual_joint_order != expected_joint_order:
        findings.append(
            f"{ECAT_XACRO}: EtherCAT joint order {actual_joint_order!r} does not "
            f"match manifest {expected_joint_order!r}"
        )
    profiles = _profile_map(view)
    for index, axis in enumerate(view.ethercat_axes):
        if index >= len(invocations):
            break
        invocation = invocations[index]
        if invocation.attrib.get("joint_name") != axis.joint:
            continue
        profile = profiles[axis.profile]
        expected_tag = _xacro_tag(f"{profile.family}_axis")
        if invocation.tag != expected_tag:
            findings.append(
                f"{ECAT_XACRO}: {axis.joint} macro family "
                f"{_local_name(invocation.tag)!r} does not match profile family "
                f"{profile.family!r}"
            )
        actual_position = _as_int(invocation.attrib.get("ring_position"))
        if actual_position != axis.ring_position:
            findings.append(
                f"{ECAT_XACRO}: {axis.joint} ring position {actual_position!r} "
                f"does not match manifest {axis.ring_position}"
            )
        actual_profile = invocation.attrib.get("profile")
        if actual_profile != axis.profile:
            findings.append(
                f"{ECAT_XACRO}: {axis.joint} profile {actual_profile!r} does not "
                f"match manifest {axis.profile!r}"
            )
        runtime_profile = f"{actual_profile}.yaml"
        configured_profile = PurePosixPath(profile.config).name
        if runtime_profile != configured_profile:
            findings.append(
                f"{ECAT_XACRO}: {profile.identifier} runtime profile path "
                f"{runtime_profile!r} does not match manifest config "
                f"{configured_profile!r}"
            )

    expected_sensor_positions = [axis.ring_position for axis in view.ethercat_axes]
    actual_sensor_positions = _sensor_positions(system)
    if actual_sensor_positions != expected_sensor_positions:
        findings.append(
            f"{ECAT_XACRO}: EtherCAT sensor positions {actual_sensor_positions!r} "
            f"do not match manifest {expected_sensor_positions!r}"
        )
    _check_sensor_interfaces(
        system,
        "ethercat_domain",
        ("process_data_age_ms",),
        ECAT_XACRO,
        findings,
    )
    _check_sensor_interfaces(
        system,
        "ethercat_master",
        ("link_up", "slaves_responding", "wc_error_count"),
        ECAT_XACRO,
        findings,
    )
    for position in expected_sensor_positions:
        _check_sensor_interfaces(
            system,
            f"ethercat_slave_{position}",
            ("al_state",),
            ECAT_XACRO,
            findings,
        )

    profiles_by_family: dict[str, list[ProfileProjection]] = {}
    for profile in view.profiles:
        if profile.bus == "ethercat":
            profiles_by_family.setdefault(profile.family, []).append(profile)
    for family, profiles in profiles_by_family.items():
        macro_name = f"{family}_axis"
        macro_matches = [
            child
            for child in root
            if child.tag == _xacro_tag("macro")
            and child.attrib.get("name") == macro_name
        ]
        if len(macro_matches) != 1:
            findings.append(
                f"{ECAT_XACRO}: {macro_name} macro must occur exactly once, "
                f"found {len(macro_matches)}"
            )
            continue
        macro = macro_matches[0]
        if macro.attrib.get("params", "").split() != [
            "joint_name",
            "ring_position",
            "profile",
        ]:
            findings.append(
                f"{ECAT_XACRO}: {macro_name} params must declare joint_name, "
                "ring_position and profile"
            )
        joints = _direct_children(macro, "joint")
        if len(joints) != 1:
            findings.append(
                f"{ECAT_XACRO}: macro '{macro_name}' must contain exactly one joint"
            )
            continue
        if joints[0].attrib.get("name") != "${joint_name}":
            findings.append(
                f"{ECAT_XACRO}: {macro_name} joint_name forwarding to joint name "
                "is required"
            )
        modules = _direct_children(joints[0], "ec_module")
        if len(modules) != 1:
            findings.append(
                f"{ECAT_XACRO}: {macro_name} must contain exactly one ec_module"
            )
        else:
            position = _parameter_text(
                modules[0], "position", macro_name, findings
            )
            if position != "${ring_position}":
                findings.append(
                    f"{ECAT_XACRO}: {macro_name} ring_position forwarding to "
                    "the ec_module position is required"
                )
            slave_config = _parameter_text(
                modules[0], "slave_config", macro_name, findings
            )
            if slave_config != "$(arg config_dir)/slaves/${profile}.yaml":
                findings.append(
                    f"{ECAT_XACRO}: {macro_name} profile forwarding to the "
                    "runtime slave config path is required"
                )
        command_interfaces = tuple(
            item.attrib.get("name", "")
            for item in _direct_children(joints[0], "command_interface")
        )
        state_interfaces = tuple(
            item.attrib.get("name", "")
            for item in _direct_children(joints[0], "state_interface")
        )
        for profile in profiles:
            if command_interfaces != profile.command_interfaces:
                findings.append(
                    f"{ECAT_XACRO}: {profile.identifier} command interfaces "
                    f"{command_interfaces!r} do not match manifest "
                    f"{profile.command_interfaces!r}"
                )
            if state_interfaces != profile.state_interfaces:
                findings.append(
                    f"{ECAT_XACRO}: {profile.identifier} state interfaces "
                    f"{state_interfaces!r} do not match manifest "
                    f"{profile.state_interfaces!r}"
                )


def _mapping_child(
    parent: Mapping[str, Any] | None,
    key: str,
    context: str,
    findings: list[str],
) -> Mapping[str, Any] | None:
    if parent is None:
        return None
    value = parent.get(key)
    if not isinstance(value, Mapping):
        findings.append(f"{context}.{key}: must be a mapping")
        return None
    return value


def _check_controllers(
    text: str, view: ProjectionView, findings: list[str]
) -> None:
    root = _parse_yaml(text, CONTROLLERS, findings)
    if root is None:
        return
    jsb = _mapping_child(root, "joint_state_broadcaster", CONTROLLERS, findings)
    jsb_params = _mapping_child(jsb, "ros__parameters", "joint_state_broadcaster", findings)
    whole_body_interfaces = sorted(
        {
            conversion.interface
            for axis in view.ethercat_axes
            for conversion in axis.conversions
        }
    )
    if jsb_params is not None:
        actual = jsb_params.get("joints")
        if actual != list(view.whole_body_joints):
            findings.append(
                f"{CONTROLLERS}: joint_state_broadcaster joint order {actual!r} "
                f"does not match manifest {list(view.whole_body_joints)!r}"
            )
        if jsb_params.get("interfaces") != whole_body_interfaces:
            findings.append(
                f"{CONTROLLERS}: joint_state_broadcaster interfaces "
                f"{jsb_params.get('interfaces')!r} do not match manifest-owned "
                f"motion interfaces {whole_body_interfaces!r}"
            )

    jtc = _mapping_child(root, "whole_body_jtc", CONTROLLERS, findings)
    jtc_params = _mapping_child(jtc, "ros__parameters", "whole_body_jtc", findings)
    if jtc_params is not None:
        actual_joints = jtc_params.get("joints")
        if actual_joints != list(view.whole_body_joints):
            findings.append(
                f"{CONTROLLERS}: whole_body_jtc joint order {actual_joints!r} "
                f"does not match manifest {list(view.whole_body_joints)!r}"
            )
        for key in ("command_interfaces", "state_interfaces"):
            if jtc_params.get(key) != whole_body_interfaces:
                findings.append(
                    f"{CONTROLLERS}: whole_body_jtc {key} "
                    f"{jtc_params.get(key)!r} do not match manifest-owned motion "
                    f"interfaces {whole_body_interfaces!r}"
                )
        for key in (
            "allow_partial_joints_goal",
            "open_loop_control",
            "set_last_command_interface_value_as_state_on_activation",
        ):
            if jtc_params.get(key) is not False:
                findings.append(
                    f"{CONTROLLERS}: whole_body_jtc {key} must remain false"
                )
        consistency = _mapping_child(
            jtc_params,
            "trajectory_start_consistency_check",
            "whole_body_jtc.ros__parameters",
            findings,
        )
        if consistency is not None:
            if consistency.get("enabled") is not True:
                findings.append(
                    f"{CONTROLLERS}: trajectory_start_consistency_check.enabled "
                    "must remain true"
                )
            actual_tolerances = consistency.get("position_tolerances")
            expected_tolerances = list(view.position_tolerances)
            if (
                not isinstance(actual_tolerances, list)
                or len(actual_tolerances) != len(expected_tolerances)
                or any(
                    not _number_matches(actual, expected)
                    for actual, expected in zip(
                        actual_tolerances, expected_tolerances
                    )
                )
            ):
                findings.append(
                    f"{CONTROLLERS}: position_tolerances {actual_tolerances!r} "
                    f"do not match manifest {expected_tolerances!r}"
                )
            if consistency.get("feedback_age_state_interface") != (
                "ethercat_domain/process_data_age_ms"
            ):
                findings.append(
                    f"{CONTROLLERS}: trajectory_start_consistency_check "
                    "feedback_age_state_interface must remain "
                    "ethercat_domain/process_data_age_ms"
                )
            if not _number_matches(consistency.get("max_feedback_age_ms"), 500.0):
                findings.append(
                    f"{CONTROLLERS}: trajectory_start_consistency_check "
                    "max_feedback_age_ms must remain 500.0"
                )
        constraints = _mapping_child(
            jtc_params, "constraints", "whole_body_jtc.ros__parameters", findings
        )
        if constraints is not None:
            if not _number_matches(constraints.get("goal_time"), 0.5):
                findings.append(
                    f"{CONTROLLERS}: whole_body_jtc constraints.goal_time must "
                    "remain 0.5"
                )
            if not _number_matches(
                constraints.get("stopped_velocity_tolerance"), 0.01
            ):
                findings.append(
                    f"{CONTROLLERS}: whole_body_jtc "
                    "constraints.stopped_velocity_tolerance must remain 0.01"
                )

    diff = _mapping_child(root, "diff_drive_controller", CONTROLLERS, findings)
    diff_params = _mapping_child(
        diff, "ros__parameters", "diff_drive_controller", findings
    )
    if diff_params is None:
        return
    left = [
        axis.joint
        for axis in view.canopen_axes
        if axis.controller_role == "left_track"
    ]
    right = [
        axis.joint
        for axis in view.canopen_axes
        if axis.controller_role == "right_track"
    ]
    for key, expected in (("left_wheel_names", left), ("right_wheel_names", right)):
        actual = diff_params.get(key)
        if actual != expected:
            findings.append(
                f"{CONTROLLERS}: {key} {actual!r} does not match manifest {expected!r}"
            )
    wheels_per_side = diff_params.get("wheels_per_side")
    if len(left) != len(right) or wheels_per_side != len(left):
        findings.append(
            f"{CONTROLLERS}: wheels_per_side {wheels_per_side!r} does not match "
            f"manifest left/right counts {len(left)}/{len(right)}"
        )
    if not _number_matches(diff_params.get("wheel_radius"), 1.0):
        findings.append(
            f"{CONTROLLERS}: wheel_radius must remain 1.0 for linear-SI track joints"
        )

    enable = _mapping_child(root, "enable_manager", CONTROLLERS, findings)
    enable_params = _mapping_child(
        enable, "ros__parameters", "enable_manager", findings
    )
    if enable_params is not None:
        key = "ready_to_switch_on_disable_terminal_joints"
        expected = list(view.ready_to_switch_on_disable_terminal_joints)
        if key not in enable_params:
            findings.append(f"{CONTROLLERS}: {key} is missing")
        elif enable_params.get(key) != expected:
            findings.append(
                f"{CONTROLLERS}: {key} {enable_params.get(key)!r} does not "
                f"match manifest {expected!r}"
            )


def _profile_map(view: ProjectionView) -> dict[str, ProfileProjection]:
    return {profile.identifier: profile for profile in view.profiles}


def _conversion_map(
    conversions: Sequence[ConversionProjection],
) -> dict[str, ConversionProjection]:
    return {conversion.interface: conversion for conversion in conversions}


def _normalize_relative_path(
    value: object, context: str, findings: list[str]
) -> str | None:
    if not isinstance(value, str) or not value:
        findings.append(f"{context}: must be a non-empty relative path")
        return None
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts:
        findings.append(f"{context}: must not be absolute or escape with '..'")
        return None
    return str(path)


def _check_canopen_bus(
    text: str, view: ProjectionView, findings: list[str]
) -> None:
    root = _parse_yaml(text, CANOPEN_BUS, findings)
    if root is None:
        return
    master = _mapping_child(root, "master", CANOPEN_BUS, findings)
    if master is not None and master.get("node_id") != view.canopen_master_node_id:
        findings.append(
            f"{CANOPEN_BUS}: CANopen master node {master.get('node_id')!r} does "
            f"not match manifest {view.canopen_master_node_id}"
        )

    profiles = _profile_map(view)
    bus_directory = PurePosixPath(CANOPEN_BUS).parent
    defaults = _mapping_child(root, "defaults", CANOPEN_BUS, findings)
    default_dcf = defaults.get("dcf") if defaults is not None else None

    nodes = _mapping_child(root, "nodes", CANOPEN_BUS, findings)
    if nodes is not None:
        actual_names = set(nodes)
        expected_names = {axis.joint for axis in view.canopen_axes}
        if actual_names != expected_names:
            findings.append(
                f"{CANOPEN_BUS}: CANopen joint mapping {sorted(actual_names)!r} "
                f"does not match manifest {sorted(expected_names)!r}"
            )
        for axis in view.canopen_axes:
            node = nodes.get(axis.joint)
            if not isinstance(node, Mapping):
                findings.append(f"{CANOPEN_BUS}: {axis.joint} node mapping is missing")
                continue
            if node.get("node_id") != axis.node_id:
                findings.append(
                    f"{CANOPEN_BUS}: {axis.joint} node_id {node.get('node_id')!r} "
                    f"does not match manifest {axis.node_id}"
                )
            if node.get("operation_mode") != axis.operation_mode:
                findings.append(
                    f"{CANOPEN_BUS}: {axis.joint} operation_mode "
                    f"{node.get('operation_mode')!r} does not match manifest "
                    f"{axis.operation_mode}"
                )
            if node.get("mandatory") is not True:
                findings.append(
                    f"{CANOPEN_BUS}: {axis.joint} mandatory must be true"
                )
            for interface, conversion in _conversion_map(axis.conversions).items():
                prefix = "pos" if interface == "position" else interface[:3]
                for direction, expected in (
                    ("to_dev", conversion.to_device),
                    ("from_dev", conversion.from_device),
                ):
                    key = f"scale_{prefix}_{direction}"
                    if not _number_matches(node.get(key), expected):
                        direction_name = (
                            "to_device" if direction == "to_dev" else "from_device"
                        )
                        findings.append(
                            f"{CANOPEN_BUS}: {axis.joint} {interface} "
                            f"{direction_name} {node.get(key)!r} does not match "
                            f"manifest {expected!r}"
                        )
            profile = profiles[axis.profile]
            try:
                expected_dcf = str(
                    PurePosixPath(profile.config).relative_to(bus_directory)
                )
            except ValueError:
                findings.append(
                    f"{CANOPEN_BUS}: profile {profile.identifier} config is outside "
                    "the CANopen config directory"
                )
                continue
            effective_dcf = node.get("dcf", default_dcf)
            actual_dcf = _normalize_relative_path(
                effective_dcf,
                f"{CANOPEN_BUS}: {axis.joint} effective dcf",
                findings,
            )
            if actual_dcf is not None and actual_dcf != expected_dcf:
                findings.append(
                    f"{CANOPEN_BUS}: {axis.joint} effective dcf {actual_dcf!r} "
                    f"does not match manifest {expected_dcf!r}"
                )


def _check_canopen_xacro(
    text: str, view: ProjectionView, findings: list[str]
) -> None:
    root = _parse_xml(text, CANOPEN_XACRO, findings)
    if root is None:
        return
    _check_xacro_argument(
        root,
        "canopen_config_dir",
        "$(find robot_hw_canopen)/config",
        CANOPEN_XACRO,
        findings,
    )
    interface_args = [
        child
        for child in root
        if child.tag == _xacro_tag("arg")
        and child.attrib.get("name") == "can_interface_name"
    ]
    if len(interface_args) != 1 or interface_args[0].attrib.get("default") != view.canopen_interface:
        actual = interface_args[0].attrib.get("default") if len(interface_args) == 1 else None
        findings.append(
            f"{CANOPEN_XACRO}: CANopen interface {actual!r} does not match "
            f"manifest {view.canopen_interface!r}"
        )
    system = _ros2_control(
        root, "canopen_mobile_axes", CANOPEN_XACRO, findings
    )
    if system is None:
        return
    hardware = _direct_children(system, "hardware")
    if len(hardware) != 1:
        findings.append(
            f"{CANOPEN_XACRO}: CANopen hardware block must occur exactly once"
        )
    else:
        plugins = _direct_children(hardware[0], "plugin")
        plugin = (plugins[0].text or "").strip() if len(plugins) == 1 else None
        if plugin != "canopen_ros2_control/Cia402System":
            findings.append(
                f"{CANOPEN_XACRO}: hardware plugin must be "
                "canopen_ros2_control/Cia402System"
            )
        expected_parameters = {
            "bus_config": "$(arg canopen_config_dir)/bus.yml",
            "master_config": "$(arg canopen_config_dir)/master.dcf",
            "master_bin": "$(arg canopen_config_dir)/master.bin",
            "can_interface_name": "$(arg can_interface_name)",
        }
        for name, expected in expected_parameters.items():
            actual = _parameter_text(
                hardware[0], name, "CANopen hardware", findings
            )
            if actual != expected:
                suffix = (
                    "must reference the can_interface_name argument"
                    if name == "can_interface_name"
                    else f"must reference {expected.rsplit('/', 1)[-1]}"
                )
                findings.append(
                    f"{CANOPEN_XACRO}: {name} {actual!r} {suffix}"
                )
    for child in system:
        if child.tag not in {"hardware", "joint"}:
            findings.append(
                f"{CANOPEN_XACRO}: undeclared ros2_control resource "
                f"{_local_name(child.tag)!r} in canopen_mobile_axes"
            )
    joints = _direct_children(system, "joint")
    actual_order = [joint.attrib.get("name") for joint in joints]
    expected_order = [axis.joint for axis in view.canopen_axes]
    if actual_order != expected_order:
        findings.append(
            f"{CANOPEN_XACRO}: CANopen joint order {actual_order!r} does not "
            f"match manifest {expected_order!r}"
        )
    for index, axis in enumerate(view.canopen_axes):
        if index >= len(joints) or joints[index].attrib.get("name") != axis.joint:
            continue
        node_id = _as_int(
            _parameter_text(joints[index], "node_id", axis.joint, findings)
        )
        mode = _as_int(
            _parameter_text(
                joints[index], "operation_mode", axis.joint, findings
            )
        )
        if node_id != axis.node_id:
            findings.append(
                f"{CANOPEN_XACRO}: {axis.joint} node_id {node_id!r} does not "
                f"match manifest {axis.node_id}"
            )
        if mode != axis.operation_mode:
            findings.append(
                f"{CANOPEN_XACRO}: {axis.joint} operation_mode {mode!r} does not "
                f"match manifest {axis.operation_mode}"
            )


def _mock_slave_count(
    system: ET.Element, findings: list[str]
) -> float | None:
    masters = [
        sensor
        for sensor in _direct_children(system, "sensor")
        if sensor.attrib.get("name") == "ethercat_master"
    ]
    if len(masters) != 1:
        findings.append(f"{MOCK_XACRO}: mock ethercat_master sensor must occur once")
        return None
    interfaces = [
        item
        for item in _direct_children(masters[0], "state_interface")
        if item.attrib.get("name") == "slaves_responding"
    ]
    if len(interfaces) != 1:
        findings.append(f"{MOCK_XACRO}: slaves_responding interface must occur once")
        return None
    text = _parameter_text(
        interfaces[0], "initial_value", "mock slaves_responding", findings
    )
    try:
        return float(text) if text is not None else None
    except ValueError:
        return None


def _check_mock_xacro(
    text: str, view: ProjectionView, findings: list[str]
) -> None:
    root = _parse_xml(text, MOCK_XACRO, findings)
    if root is None:
        return
    macro_matches = [
        child
        for child in root
        if child.tag == _xacro_tag("macro")
        and child.attrib.get("name") == "mock_ecat_axis"
    ]
    if len(macro_matches) != 1:
        findings.append(
            f"{MOCK_XACRO}: mock_ecat_axis macro must occur exactly once, "
            f"found {len(macro_matches)}"
        )
    else:
        macro = macro_matches[0]
        if macro.attrib.get("params", "").split() != ["joint_name"]:
            findings.append(
                f"{MOCK_XACRO}: mock_ecat_axis params must declare joint_name"
            )
        macro_joints = [child for child in macro if child.tag == "joint"]
        if len(macro_joints) != 1:
            findings.append(
                f"{MOCK_XACRO}: mock_ecat_axis must contain exactly one direct joint"
            )
        elif macro_joints[0].attrib.get("name") != "${joint_name}":
            findings.append(
                f"{MOCK_XACRO}: mock_ecat_axis joint_name forwarding to joint name "
                "is required"
            )
    ecat = _ros2_control(root, "ecat_arms", MOCK_XACRO, findings)
    if ecat is not None:
        invocations = [
            child for child in ecat if child.tag == _xacro_tag("mock_ecat_axis")
        ]
        for child in ecat:
            if child.tag in {"hardware", "sensor"} or child in invocations:
                continue
            if child.tag == "joint":
                findings.append(
                    f"{MOCK_XACRO}: mock undeclared raw joint "
                    f"{child.attrib.get('name')!r} in ecat_arms"
                )
            else:
                findings.append(
                    f"{MOCK_XACRO}: mock undeclared ros2_control resource "
                    f"{_local_name(child.tag)!r} in ecat_arms"
                )
        actual_joints = [child.attrib.get("joint_name") for child in invocations]
        expected_joints = list(view.whole_body_joints)
        if actual_joints != expected_joints:
            findings.append(
                f"{MOCK_XACRO}: mock EtherCAT joint order {actual_joints!r} does "
                f"not match manifest {expected_joints!r}"
            )
        actual_sensors = _sensor_positions(ecat)
        expected_sensors = [axis.ring_position for axis in view.ethercat_axes]
        if actual_sensors != expected_sensors:
            findings.append(
                f"{MOCK_XACRO}: mock EtherCAT sensor positions "
                f"{actual_sensors!r} do not match manifest {expected_sensors!r}"
            )
        _check_sensor_interfaces(
            ecat,
            "ethercat_domain",
            ("process_data_age_ms",),
            MOCK_XACRO,
            findings,
        )
        _check_sensor_interfaces(
            ecat,
            "ethercat_master",
            ("link_up", "slaves_responding", "wc_error_count"),
            MOCK_XACRO,
            findings,
        )
        for position in expected_sensors:
            _check_sensor_interfaces(
                ecat,
                f"ethercat_slave_{position}",
                ("al_state",),
                MOCK_XACRO,
                findings,
            )
        slave_count = _mock_slave_count(ecat, findings)
        if slave_count != float(len(view.ring_positions)):
            findings.append(
                f"{MOCK_XACRO}: slaves_responding {slave_count!r} does not match "
                f"full ring size {len(view.ring_positions)}"
            )

    canopen = _ros2_control(root, "canopen_mobile_axes", MOCK_XACRO, findings)
    if canopen is None:
        return
    for child in canopen:
        if child.tag not in {"hardware", "joint"}:
            findings.append(
                f"{MOCK_XACRO}: mock undeclared ros2_control resource "
                f"{_local_name(child.tag)!r} in canopen_mobile_axes"
            )
    joints = _direct_children(canopen, "joint")
    actual_joint_names = [joint.attrib.get("name") for joint in joints]
    expected_joint_names = [axis.joint for axis in view.canopen_axes]
    if actual_joint_names != expected_joint_names:
        findings.append(
            f"{MOCK_XACRO}: mock CANopen joints {actual_joint_names!r} do not "
            f"match manifest {expected_joint_names!r}"
        )
    profiles = _profile_map(view)
    for index, axis in enumerate(view.canopen_axes):
        if index >= len(joints) or joints[index].attrib.get("name") != axis.joint:
            continue
        profile = profiles[axis.profile]
        command_interfaces = tuple(
            item.attrib.get("name", "")
            for item in _direct_children(joints[index], "command_interface")
        )
        state_interfaces = tuple(
            item.attrib.get("name", "")
            for item in _direct_children(joints[index], "state_interface")
        )
        if command_interfaces != profile.command_interfaces:
            findings.append(
                f"{MOCK_XACRO}: {axis.joint} command interfaces "
                f"{command_interfaces!r} do not match manifest "
                f"{profile.command_interfaces!r}"
            )
        if state_interfaces != profile.state_interfaces:
            findings.append(
                f"{MOCK_XACRO}: {axis.joint} state interfaces "
                f"{state_interfaces!r} do not match manifest "
                f"{profile.state_interfaces!r}"
            )


def _pdo_channels(
    root: Mapping[str, Any], pdo_name: str
) -> list[Mapping[str, Any]]:
    result: list[Mapping[str, Any]] = []
    pdos = root.get(pdo_name)
    if not isinstance(pdos, list):
        return result
    for pdo in pdos:
        if not isinstance(pdo, Mapping):
            continue
        channels = pdo.get("channels")
        if not isinstance(channels, list):
            continue
        result.extend(channel for channel in channels if isinstance(channel, Mapping))
    return result


def _profile_conversions(
    profile_id: str, view: ProjectionView, findings: list[str]
) -> dict[str, ConversionProjection]:
    used = [
        axis.conversions
        for axis in (*view.ethercat_axes, *view.canopen_axes)
        if axis.profile == profile_id
    ]
    expected: dict[str, ConversionProjection] = {}
    for conversions in used:
        for conversion in conversions:
            existing = expected.get(conversion.interface)
            if existing is not None and existing != conversion:
                findings.append(
                    f"drive profile {profile_id} is assigned conflicting "
                    f"{conversion.interface} conversions"
                )
            expected[conversion.interface] = conversion
    return expected


def _check_profile(
    profile: ProfileProjection,
    text: str,
    view: ProjectionView,
    findings: list[str],
) -> None:
    root = _parse_yaml(text, profile.config, findings)
    if root is None:
        return
    if root.get("auto_fault_reset") is not False:
        findings.append(
            f"{profile.config}: {profile.identifier} auto_fault_reset must be false"
        )
    if root.get("auto_state_transitions") is not False:
        findings.append(
            f"{profile.config}: {profile.identifier} auto_state_transitions must be false"
        )
    rpdo = _pdo_channels(root, "rpdo")
    tpdo = _pdo_channels(root, "tpdo")
    for direction, channels, key, expected_interfaces in (
        ("command", rpdo, "command_interface", profile.command_interfaces),
        ("state", tpdo, "state_interface", profile.state_interfaces),
    ):
        actual_interfaces: list[str] = []
        for channel in channels:
            if key not in channel:
                continue
            interface = channel.get(key)
            if not isinstance(interface, str) or not interface:
                findings.append(
                    f"{profile.config}: {profile.identifier} {direction} interface "
                    "names must be non-empty strings"
                )
                continue
            if interface in driver_variant.PROFILE_INTERFACES:
                actual_interfaces.append(interface)
        actual_multiset = Counter(actual_interfaces)
        expected_multiset = Counter(expected_interfaces)
        if actual_multiset != expected_multiset:
            findings.append(
                f"{profile.config}: {profile.identifier} {direction} interfaces "
                f"{dict(actual_multiset)!r} do not match manifest "
                f"{dict(expected_multiset)!r}"
            )
    for interface, conversion in _profile_conversions(
        profile.identifier, view, findings
    ).items():
        commands = [
            channel
            for channel in rpdo
            if channel.get("command_interface") == interface
        ]
        states = [
            channel
            for channel in tpdo
            if channel.get("state_interface") == interface
        ]
        if len(commands) != 1:
            findings.append(
                f"{profile.config}: {profile.identifier} {interface} must have "
                f"exactly one command channel, found {len(commands)}"
            )
        elif not _number_matches(commands[0].get("factor"), conversion.to_device):
            findings.append(
                f"{profile.config}: {profile.identifier} {interface} to_device "
                f"{commands[0].get('factor')!r} does not match manifest "
                f"{conversion.to_device!r}"
            )
        if len(states) != 1:
            findings.append(
                f"{profile.config}: {profile.identifier} {interface} must have "
                f"exactly one state channel, found {len(states)}"
            )
        elif not _number_matches(states[0].get("factor"), conversion.from_device):
            findings.append(
                f"{profile.config}: {profile.identifier} {interface} from_device "
                f"{states[0].get('factor')!r} does not match manifest "
                f"{conversion.from_device!r}"
            )


def validate_structured_projections(
    document: Mapping[str, Any], sources: Mapping[str, str]
) -> None:
    """Aggregate every structured projection drift into one fail-closed error."""
    view = derive_projection(document)
    findings: list[str] = []
    required = _required_sources(view)
    for path in required:
        if path not in sources:
            findings.append(f"{path}: missing projection source")

    if ECAT_XACRO in sources:
        _check_ecat_xacro(sources[ECAT_XACRO], view, findings)
    if CONTROLLERS in sources:
        _check_controllers(sources[CONTROLLERS], view, findings)
    if CANOPEN_BUS in sources:
        _check_canopen_bus(sources[CANOPEN_BUS], view, findings)
    if CANOPEN_XACRO in sources:
        _check_canopen_xacro(sources[CANOPEN_XACRO], view, findings)
    if MOCK_XACRO in sources:
        _check_mock_xacro(sources[MOCK_XACRO], view, findings)
    if ROBOT_DESCRIPTION in sources:
        _check_robot_description(sources[ROBOT_DESCRIPTION], view, findings)
    for profile in view.profiles:
        if profile.bus == "ethercat" and profile.config in sources:
            _check_profile(
                profile, sources[profile.config], view, findings
            )

    if findings:
        raise driver_variant.DriverVariantError(findings)
