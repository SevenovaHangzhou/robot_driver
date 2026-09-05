"""Strict loader for the versioned RT-Control hardware composition slice."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence

import yaml


class HardwareCompositionError(ValueError):
    """The hardware-composition file cannot safely describe this robot."""


_VARIANT_NAME = re.compile(r"^[a-z][a-z0-9_]*$")


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


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


@dataclass(frozen=True)
class EthercatAxis:
    joint_name: str
    ring_position: int


@dataclass(frozen=True)
class EthercatSensor:
    sensor_name: str
    ring_position: int
    wrench_topic: str = ""
    raw_topic: str = ""
    frame_id: str = ""


@dataclass(frozen=True)
class EthercatComposition:
    expected_responders: int
    axes: tuple[EthercatAxis, ...]
    sensors: tuple[EthercatSensor, ...]


@dataclass(frozen=True)
class CanopenNode:
    joint_name: str
    node_id: int
    side: str


@dataclass(frozen=True)
class CanopenComposition:
    nodes: tuple[CanopenNode, ...]


@dataclass(frozen=True)
class HardwareComposition:
    schema_version: int
    ethercat: EthercatComposition
    canopen: CanopenComposition

    def diagnostics_parameters(self) -> dict[str, Any]:
        """Return the only flattened view consumed by ``rt_diagnostics``."""
        return {
            "ethercat_joint_names": [
                axis.joint_name for axis in self.ethercat.axes
            ],
            "ethercat_ring_positions": [
                axis.ring_position for axis in self.ethercat.axes
            ],
            "ethercat_sensor_names": [
                sensor.sensor_name for sensor in self.ethercat.sensors
            ],
            "ethercat_sensor_ring_positions": [
                sensor.ring_position for sensor in self.ethercat.sensors
            ],
            "ethercat_expected_responders": self.ethercat.expected_responders,
            "canopen_node_ids": [node.node_id for node in self.canopen.nodes],
        }

    def x503_parameters(self) -> dict[str, Any]:
        """Return descriptor-owned X503 ROS bridge metadata."""
        sensors = [sensor for sensor in self.ethercat.sensors if sensor.wrench_topic]
        return {
            "sensor_names": [sensor.sensor_name for sensor in sensors],
            "wrench_topics": [sensor.wrench_topic for sensor in sensors],
            "raw_topics": [sensor.raw_topic for sensor in sensors],
            "frame_ids": [sensor.frame_id for sensor in sensors],
            "slave_positions": [sensor.ring_position for sensor in sensors],
        }


def _path(path: Sequence[object]) -> str:
    return ".".join(str(component) for component in path)


def _mapping(
    value: Any,
    path: tuple[object, ...],
    required_fields: frozenset[str],
) -> Mapping[str, Any]:
    if type(value) is not dict:
        raise HardwareCompositionError(f"{_path(path)} must be a mapping")

    keys = set(value)
    missing = required_fields - keys
    if missing:
        names = ", ".join(sorted(missing))
        raise HardwareCompositionError(f"{_path(path)} is missing fields: {names}")
    unknown = keys - required_fields
    if unknown:
        names = ", ".join(sorted(repr(name) for name in unknown))
        raise HardwareCompositionError(f"{_path(path)} has unknown fields: {names}")
    return value


def _mapping_with_required(
    value: Any,
    path: tuple[object, ...],
    required_fields: frozenset[str],
) -> Mapping[str, Any]:
    if type(value) is not dict:
        raise HardwareCompositionError(f"{_path(path)} must be a mapping")
    missing = required_fields - set(value)
    if missing:
        names = ", ".join(sorted(missing))
        raise HardwareCompositionError(f"{_path(path)} is missing fields: {names}")
    return value


def _nonempty_list(value: Any, path: tuple[object, ...], label: str) -> list[Any]:
    if type(value) is not list:
        raise HardwareCompositionError(f"{_path(path)} must be a list")
    if not value:
        raise HardwareCompositionError(f"{_path(path)} must contain at least one {label}")
    return value


def _list(value: Any, path: tuple[object, ...]) -> list[Any]:
    if type(value) is not list:
        raise HardwareCompositionError(f"{_path(path)} must be a list")
    return value


def _integer(value: Any, path: tuple[object, ...]) -> int:
    if type(value) is not int:
        raise HardwareCompositionError(f"{_path(path)} must be an integer")
    return value


def _joint_name(value: Any, path: tuple[object, ...]) -> str:
    if type(value) is not str or not value.strip() or value != value.strip():
        raise HardwareCompositionError(
            f"{_path(path)} joint_name must be a nonempty, trimmed string"
        )
    return value


def _trimmed_string(value: Any, path: tuple[object, ...]) -> str:
    if type(value) is not str or not value.strip() or value != value.strip():
        raise HardwareCompositionError(
            f"{_path(path)} must be a nonempty, trimmed string"
        )
    return value


def _load_unique_yaml(path: str | Path, label: str) -> Mapping[str, Any]:
    config_path = Path(path)
    try:
        contents = config_path.read_text(encoding="utf-8")
    except OSError as error:
        raise HardwareCompositionError(
            f"could not read {label} {config_path}: {error}"
        ) from error

    try:
        document = yaml.load(contents, Loader=_UniqueKeyLoader)
    except yaml.YAMLError as error:
        raise HardwareCompositionError(f"invalid {label} YAML: {error}") from error

    if type(document) is not dict:
        raise HardwareCompositionError(f"{label} must be a mapping")
    return document


def variant_descriptor_path(package_share: str | Path, variant: str) -> Path:
    """Resolve a package-owned descriptor without accepting path syntax."""
    if type(variant) is not str or _VARIANT_NAME.fullmatch(variant) is None:
        raise HardwareCompositionError(
            "variant must match ^[a-z][a-z0-9_]*$"
        )
    return Path(package_share) / "variants" / f"{variant}.yaml"


def _descriptor_header(
    document: Mapping[str, Any],
    path: Path,
    expected_system: str,
    fields: frozenset[str],
) -> Mapping[str, Any]:
    root = _mapping_with_required(document, (path.name,), fields)
    schema_version = _integer(root["schema_version"], (path.name, "schema_version"))
    if schema_version != 1:
        raise HardwareCompositionError(
            f"{path.name}.schema_version must be 1"
        )
    variant = _trimmed_string(root["variant"], (path.name, "variant"))
    if _VARIANT_NAME.fullmatch(variant) is None:
        raise HardwareCompositionError(f"{path.name}.variant has an invalid name")
    if variant != path.stem:
        raise HardwareCompositionError(
            f"{path.name}.variant must match descriptor filename {path.stem!r}"
        )
    system = _trimmed_string(root["system"], (path.name, "system"))
    if system != expected_system:
        raise HardwareCompositionError(
            f"{path.name}.system must be {expected_system!r}"
        )
    return root


def _parse_ethercat_variant(path: str | Path) -> EthercatComposition:
    descriptor_path = Path(path)
    root = _descriptor_header(
        _load_unique_yaml(descriptor_path, "EtherCAT variant descriptor"),
        descriptor_path,
        "ecat_arms",
        frozenset(
            {
                "schema_version",
                "variant",
                "system",
                "extra_responders",
                "sensors",
                "axes",
            }
        ),
    )
    raw_axes = _nonempty_list(
        root["axes"], (descriptor_path.name, "axes"), "axis"
    )
    raw_extra = _list(
        root["extra_responders"],
        (descriptor_path.name, "extra_responders"),
    )
    raw_sensors = _list(
        root["sensors"],
        (descriptor_path.name, "sensors"),
    )

    resource_names: set[str] = set()
    ring_positions: set[int] = set()
    axes: list[EthercatAxis] = []
    for index, raw_axis in enumerate(raw_axes):
        axis_path = (descriptor_path.name, "axes", index)
        axis = _mapping_with_required(
            raw_axis,
            axis_path,
            frozenset({"joint_name", "ring_position"}),
        )
        joint_name = _joint_name(axis["joint_name"], axis_path + ("joint_name",))
        ring_position = _integer(
            axis["ring_position"], axis_path + ("ring_position",)
        )
        if not 0 <= ring_position <= 65534:
            raise HardwareCompositionError(
                f"{_path(axis_path + ('ring_position',))} must be in 0..65534"
            )
        if joint_name in resource_names:
            raise HardwareCompositionError("EtherCAT joint_name values must be unique")
        if ring_position in ring_positions:
            raise HardwareCompositionError(
                "EtherCAT ring_position values must be unique"
            )
        resource_names.add(joint_name)
        ring_positions.add(ring_position)
        axes.append(EthercatAxis(joint_name, ring_position))

    sensors: list[EthercatSensor] = []
    for index, raw_sensor in enumerate(raw_sensors):
        sensor_path = (descriptor_path.name, "sensors", index)
        sensor = _mapping_with_required(
            raw_sensor,
            sensor_path,
            frozenset({"sensor_name", "ring_position"}),
        )
        sensor_name = _trimmed_string(
            sensor["sensor_name"], sensor_path + ("sensor_name",)
        )
        ring_position = _integer(
            sensor["ring_position"], sensor_path + ("ring_position",)
        )
        if not 0 <= ring_position <= 65534:
            raise HardwareCompositionError(
                f"{_path(sensor_path + ('ring_position',))} must be in 0..65534"
            )
        if sensor_name in resource_names:
            if any(existing.sensor_name == sensor_name for existing in sensors):
                raise HardwareCompositionError(
                    "EtherCAT sensor_name values must be unique"
                )
            raise HardwareCompositionError(
                "EtherCAT resource names must be unique across axes and sensors"
            )
        if ring_position in ring_positions:
            raise HardwareCompositionError(
                "EtherCAT ring_position values must be unique"
            )
        resource_names.add(sensor_name)
        ring_positions.add(ring_position)
        wrench_topic = ""
        raw_topic = ""
        frame_id = ""
        if "wrench_topic" in sensor:
            wrench_topic = _trimmed_string(
                sensor["wrench_topic"], sensor_path + ("wrench_topic",)
            )
        if "raw_topic" in sensor:
            raw_topic = _trimmed_string(
                sensor["raw_topic"], sensor_path + ("raw_topic",)
            )
        if "frame_id" in sensor:
            frame_id = _trimmed_string(
                sensor["frame_id"], sensor_path + ("frame_id",)
            )
        if bool(wrench_topic) != bool(raw_topic) or bool(wrench_topic) != bool(frame_id):
            raise HardwareCompositionError(
                f"{_path(sensor_path)} X503 ROS metadata must provide wrench_topic, raw_topic and frame_id together"
            )
        sensors.append(EthercatSensor(sensor_name, ring_position, wrench_topic, raw_topic, frame_id))

    for index, raw_responder in enumerate(raw_extra):
        responder_path = (descriptor_path.name, "extra_responders", index)
        responder = _mapping_with_required(
            raw_responder, responder_path, frozenset({"ring_position"})
        )
        ring_position = _integer(
            responder["ring_position"], responder_path + ("ring_position",)
        )
        if not 0 <= ring_position <= 65534:
            raise HardwareCompositionError(
                f"{_path(responder_path + ('ring_position',))} must be in 0..65534"
            )
        if ring_position in ring_positions:
            raise HardwareCompositionError(
                "EtherCAT ring_position values must be unique"
            )
        ring_positions.add(ring_position)

    expected_responders = len(ring_positions)
    if ring_positions != set(range(expected_responders)):
        raise HardwareCompositionError(
            "EtherCAT ring positions must form one complete 0-based ring"
        )
    return EthercatComposition(expected_responders, tuple(axes), tuple(sensors))


def _parse_canopen_variant(path: str | Path) -> CanopenComposition:
    descriptor_path = Path(path)
    root = _descriptor_header(
        _load_unique_yaml(descriptor_path, "CANopen variant descriptor"),
        descriptor_path,
        "canopen_mobile_axes",
        frozenset({"schema_version", "variant", "system", "nodes"}),
    )
    raw_nodes = _nonempty_list(
        root["nodes"], (descriptor_path.name, "nodes"), "node"
    )
    joint_names: set[str] = set()
    node_ids: set[int] = set()
    nodes: list[CanopenNode] = []
    for index, raw_node in enumerate(raw_nodes):
        node_path = (descriptor_path.name, "nodes", index)
        node = _mapping_with_required(
            raw_node,
            node_path,
            frozenset({"joint_name", "node_id", "side"}),
        )
        joint_name = _joint_name(node["joint_name"], node_path + ("joint_name",))
        node_id = _integer(node["node_id"], node_path + ("node_id",))
        side = _trimmed_string(node["side"], node_path + ("side",))
        if not 1 <= node_id <= 127:
            raise HardwareCompositionError(
                f"{_path(node_path + ('node_id',))} must be in 1..127"
            )
        if side not in {"left", "right"}:
            raise HardwareCompositionError(
                f"{_path(node_path + ('side',))} must be 'left' or 'right'"
            )
        if joint_name in joint_names:
            raise HardwareCompositionError("CANopen joint_name values must be unique")
        if node_id in node_ids:
            raise HardwareCompositionError("CANopen node_id values must be unique")
        joint_names.add(joint_name)
        node_ids.add(node_id)
        nodes.append(CanopenNode(joint_name, node_id, side))
    return CanopenComposition(tuple(nodes))


def load_hardware_variants(
    ethercat_descriptor: str | Path,
    canopen_descriptor: str | Path,
) -> HardwareComposition:
    """Compose the diagnostic view from two hardware-package descriptors."""
    ethercat = _parse_ethercat_variant(ethercat_descriptor)
    canopen = _parse_canopen_variant(canopen_descriptor)
    ethercat_resources = {axis.joint_name for axis in ethercat.axes} | {
        sensor.sensor_name for sensor in ethercat.sensors
    }
    overlap = ethercat_resources & {node.joint_name for node in canopen.nodes}
    if overlap:
        names = ", ".join(sorted(overlap))
        raise HardwareCompositionError(
            f"resource names must be unique across buses: {names}"
        )
    return HardwareComposition(1, ethercat, canopen)


def _controller_parameters(
    document: Mapping[str, Any], controller_name: str
) -> Mapping[str, Any]:
    controller = _mapping_with_required(
        document.get(controller_name),
        (controller_name,),
        frozenset({"ros__parameters"}),
    )
    parameters = controller["ros__parameters"]
    if type(parameters) is not dict:
        raise HardwareCompositionError(
            f"{controller_name}.ros__parameters must be a mapping"
        )
    return parameters


def _string_list(value: Any, path: tuple[object, ...]) -> list[str]:
    if type(value) is not list:
        raise HardwareCompositionError(f"{_path(path)} must be a list")
    result: list[str] = []
    for index, item in enumerate(value):
        result.append(_joint_name(item, path + (index,)))
    return result


def _require_joint_order(
    parameters: Mapping[str, Any],
    field: str,
    expected: list[str],
    controller_name: str,
) -> None:
    actual = _string_list(
        parameters.get(field), (controller_name, "ros__parameters", field)
    )
    if actual != expected:
        raise HardwareCompositionError(
            f"{controller_name}.{field} must exactly match selected variant order"
        )


def validate_controller_compatibility(
    composition: HardwareComposition,
    controllers_path: str | Path,
) -> None:
    """Reject a selected hardware variant before any ROS node can start."""
    document = _load_unique_yaml(controllers_path, "controller configuration")
    ethercat_joints = [axis.joint_name for axis in composition.ethercat.axes]

    jsb = _controller_parameters(document, "joint_state_broadcaster")
    _require_joint_order(jsb, "joints", ethercat_joints, "joint_state_broadcaster")
    if jsb.get("interfaces") != ["position"]:
        raise HardwareCompositionError(
            "joint_state_broadcaster.interfaces must be exactly ['position']"
        )

    jtc = _controller_parameters(document, "whole_body_jtc")
    _require_joint_order(jtc, "joints", ethercat_joints, "whole_body_jtc")
    if jtc.get("command_interfaces") != ["position"]:
        raise HardwareCompositionError(
            "whole_body_jtc.command_interfaces must be exactly ['position']"
        )
    if jtc.get("state_interfaces") != ["position"]:
        raise HardwareCompositionError(
            "whole_body_jtc.state_interfaces must be exactly ['position']"
        )
    consistency = _mapping_with_required(
        jtc.get("trajectory_start_consistency_check"),
        ("whole_body_jtc", "trajectory_start_consistency_check"),
        frozenset({"enabled", "position_tolerances"}),
    )
    if consistency["enabled"] is not True:
        raise HardwareCompositionError(
            "whole_body_jtc trajectory consistency check must be enabled"
        )
    tolerances = consistency["position_tolerances"]
    if type(tolerances) is not list or len(tolerances) != len(ethercat_joints):
        raise HardwareCompositionError(
            "whole_body_jtc.position_tolerances must have one value per selected joint"
        )
    if any(
        type(value) not in {int, float}
        or not math.isfinite(float(value))
        or value <= 0.0
        for value in tolerances
    ):
        raise HardwareCompositionError(
            "whole_body_jtc.position_tolerances must be finite positive numbers"
        )

    enable = _controller_parameters(document, "enable_manager")
    _require_joint_order(enable, "managed_joints", ethercat_joints, "enable_manager")
    flat_batches = _string_list(
        enable.get("enable_batch_joint_names"),
        ("enable_manager", "enable_batch_joint_names"),
    )
    batch_sizes = enable.get("enable_batch_sizes")
    if (
        type(batch_sizes) is not list
        or not batch_sizes
        or any(type(size) is not int or not 1 <= size <= 3 for size in batch_sizes)
        or sum(batch_sizes) != len(flat_batches)
        or len(flat_batches) != len(set(flat_batches))
        or set(flat_batches) != set(ethercat_joints)
    ):
        raise HardwareCompositionError(
            "enable batches must cover every selected EtherCAT joint exactly once"
        )
    ready_terminal = _string_list(
        enable.get("ready_to_switch_on_disable_terminal_joints"),
        ("enable_manager", "ready_to_switch_on_disable_terminal_joints"),
    )
    if len(ready_terminal) != len(set(ready_terminal)) or not set(
        ready_terminal
    ).issubset(ethercat_joints):
        raise HardwareCompositionError(
            "enable-manager disable terminal policy must be a unique joint subset"
        )

    diff_drive = _controller_parameters(document, "diff_drive_controller")
    expected_left = [
        node.joint_name for node in composition.canopen.nodes if node.side == "left"
    ]
    expected_right = [
        node.joint_name for node in composition.canopen.nodes if node.side == "right"
    ]
    if not expected_left or len(expected_left) != len(expected_right):
        raise HardwareCompositionError(
            "selected CANopen variant must provide equal nonempty left/right wheels"
        )
    _require_joint_order(
        diff_drive,
        "left_wheel_names",
        expected_left,
        "diff_drive_controller",
    )
    _require_joint_order(
        diff_drive,
        "right_wheel_names",
        expected_right,
        "diff_drive_controller",
    )
    wheels_per_side = diff_drive.get("wheels_per_side")
    if type(wheels_per_side) is not int or wheels_per_side != len(expected_left):
        raise HardwareCompositionError(
            "diff_drive_controller.wheels_per_side must match selected variant"
        )
