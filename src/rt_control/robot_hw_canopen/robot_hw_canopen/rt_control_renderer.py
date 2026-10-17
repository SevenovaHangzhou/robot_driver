"""CiA406 read-only sensor renderer; no bus probes and no composer dependency."""

from collections.abc import Mapping
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from jsonschema import Draft7Validator
import yaml

RENDERER_API_VERSION = 1


def _share():
    return Path(get_package_share_directory("robot_hw_canopen"))


def _validate(data, name):
    schema = json.loads((_share() / "schema" / name).read_text())
    Draft7Validator(schema).validate(data)
    return deepcopy(dict(data))


def validate_module(module: Mapping, data: Mapping) -> Mapping:
    return {**deepcopy(dict(data)), "spec": _validate(data["spec"], "encoder_module.schema.json")}


def validate_unit(module: Mapping, data: Mapping | None) -> Mapping:
    if data is None:
        raise ValueError("External encoder calibration is required")
    return {**deepcopy(dict(data)), "spec": _validate(data["spec"], "encoder_unit.schema.json")}


def validate_device(data: Mapping, *, render_mode: str) -> Mapping:
    missing = [
        key
        for key in ("counts_per_revolution", "distinguishable_revolutions")
        if key not in data or data[key] == "TBD"
    ]
    if missing:
        raise ValueError("Required encoder facts: " + ", ".join(missing))
    return deepcopy(dict(data))


def _param(parent, name, value):
    ET.SubElement(parent, "param", name=name).text = str(value)


def render(request: Mapping) -> Mapping:
    machine = validate_module(request["module"], request["machine"])
    unit = validate_unit(request["module"], request["unit"])
    spec = machine["spec"]
    families = yaml.safe_load((_share() / "config/profiles/families.yaml").read_text())["families"]
    for reference in request["module"]["device_refs"]:
        if request["devices"][reference]["family"] not in families:
            raise ValueError(f"Unsupported device family: {reference}")
    profile = yaml.safe_load(
        (_share() / "config/profiles" / (spec["profile"] + ".yaml")).read_text()
    )
    deployment = request["deployment"]
    mock = request["use_mock_hardware"]
    calibration = {item["module_position"]: item for item in unit["spec"]["modules"]}
    if len(calibration) != len(unit["spec"]["modules"]):
        raise ValueError("Duplicate encoder calibration module_position")
    encoder_names = {}
    assembly = request["devices"][spec["assembly"]]
    bus = {
        "options": {"dcf_path": request["runtime_dir"]},
        "master": {
            "node_id": deployment["canopen"]["master_node_id"],
            "driver": profile["master_driver"]["class"],
            "package": profile["master_driver"]["package"],
            "sync_period": request["transport_layout"]["sync_period_us"],
        },
    }
    sensors, claims, missing = [], [], []
    for item in spec["nodes"]:
        device = request["devices"][item["device"]]
        name = item["resource_name"]
        position = item["module_position"]
        if position in encoder_names or name in encoder_names.values():
            raise ValueError("Duplicate encoder resource or module_position")
        if position not in calibration:
            raise ValueError("Missing encoder module calibration")
        encoder_names[position] = name
        values = {
            "node_id": item["node_id"],
            "counts_per_revolution": device["counts_per_revolution"],
            "ring_gear_teeth": assembly["external_encoder_ring_teeth"],
            "pinion_gear_teeth": assembly["external_encoder_pinion_teeth"],
            "direction": calibration[position]["external_encoder_direction"],
            "installation_offset_rad": calibration[position][
                "external_encoder_installation_offset_rad"
            ],
            # Multi-turn counter range; the decoder unwraps the counter rollover.
            "encoder_revolutions": device["distinguishable_revolutions"],
        }
        missing += [name + "." + key for key, value in values.items() if value == "TBD"]
        sensor = ET.Element("sensor", name=name)
        for interface in profile["state_interfaces"]:
            element = ET.SubElement(sensor, "state_interface", name=interface)
            if mock:
                _param(element, "initial_value", 0)
        if not mock:
            for key, value in values.items():
                _param(sensor, key, value)
        sensors.append(sensor)
        if type(item["node_id"]) is int:
            claims.append(f"node:{item['node_id']}")
        bus[name] = {
            "node_id": item["node_id"],
            "dcf": deployment["canopen"]["eds_files"][name],
            "driver": profile["node_driver"]["class"],
            "package": profile["node_driver"]["package"],
            "tpdo": {
                1: {
                    "enabled": True,
                    "transmission": 1,
                    "mapping": [
                        {
                            "index": profile["position_pdo"]["index"],
                            "sub_index": profile["position_pdo"]["subindex"],
                        }
                    ],
                }
            },
        }
    if not mock:
        missing += ["machine.verified"] if machine["verified"] is not True else []
        missing += ["unit.verified"] if unit["verified"] is not True else []
        missing += (
            ["deployment.can_interface"]
            if deployment["can_interfaces"][machine["module"]] == "TBD"
            else []
        )
        missing += [
            "deployment.canopen." + key
            for key in ("master_node_id", "master_config", "master_bin")
            if deployment["canopen"].get(key, "TBD") == "TBD"
        ]
        missing += [
            "EDS." + name
            for name, value in deployment["canopen"]["eds_files"].items()
            if value == "TBD"
        ]
        if missing:
            raise ValueError("Missing real encoder facts: " + ", ".join(missing))
        for item in spec["nodes"]:
            validate_device(request["devices"][item["device"]], render_mode=request["render_mode"])
    params = {"control_frequency": request["bus_cycle_hz"]}
    if not mock:
        params.update(
            bus_config=str(Path(request["runtime_dir"]) / "canopen_bus.yaml"),
            master_config=deployment["canopen"]["master_config"],
            master_bin=deployment["canopen"]["master_bin"],
            can_interface_name=deployment["can_interfaces"][machine["module"]],
            ready_feedback_age_ms=profile["ready_feedback_age_ms"],
        )
    return {
        "hardware_system_id": machine["master_system"],
        "hardware_plugin": "mock_components/GenericSystem" if mock else profile["plugin"],
        "hardware_params": params,
        "joints": [],
        "sensors": sensors,
        "managed_joints": [],
        "actuator_groups": {},
        "parameter_exports": {"steering_encoders": encoder_names},
        "resource_claims": claims,
        "runtime_files": {"canopen_bus.yaml": yaml.safe_dump(bus, sort_keys=False)},
        "expected_topology": {
            "node_ids": [item["node_id"] for item in spec["nodes"]],
            "preflight_required": False,
        },
    }


def preflight(expected_topology: Mapping, *, run=None) -> None:
    """No new CAN device check is introduced by this draft migration."""
