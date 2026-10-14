"""Kinco cyclic fragments; real facts and explicit fixture values remain separate."""

from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

COMMANDS = (
    "position",
    "velocity",
    "mode_of_operation",
    "control_word",
    "write_sequence",
    "write_mask",
)
STATES = (
    "position",
    "velocity",
    "mode_of_operation_display",
    "status_word",
    "feedback_age_ms",
    "mode_switch_ready",
    "mode_request_ack",
    "command_fresh",
    "mode_request_error",
    "feedback_sequence",
    "sent_sequence",
    "feedback_sequence_at_send",
    "sent_velocity",
)


def conversion_factors(counts, ratio, sign, velocity_dec):
    position = sign * counts * ratio / (2.0 * math.pi)
    velocity = (
        sign
        * counts
        * ratio
        * 60
        * velocity_dec["numerator"]
        / (2.0 * math.pi * velocity_dec["denominator"])
    )
    return position, velocity


def _param(parent, key, value):
    ET.SubElement(parent, "param", name=key).text = str(value)


def missing_facts(request, profile):
    machine, unit = request["machine"], request["unit"]
    spec = machine["spec"]
    fields = {
        "machine.verified": machine["verified"],
        "unit.verified": None if unit is None else unit["verified"],
        "deployment.ethercat.master_id": request["deployment"]["ethercat"]["master_id"],
    }
    fields.update({"profile.identity." + key: value for key, value in profile["identity"].items()})
    fields.update({"profile.adapter." + key: value for key, value in profile["adapter"].items()})
    fields.update(
        {
            "profile." + key: profile[key]
            for key in ("dc_cycle_ns", "assign_activate", "pdo_watchdog_ms")
        }
    )
    assembly = request["devices"][spec["assembly"]]
    fields["assembly.max_abs_velocity_rad_s"] = assembly["spec"]["max_abs_velocity_rad_s"]
    fields.update(
        {"assembly.drive." + key: value for key, value in assembly["spec"]["drive"].items()}
    )
    for axis in spec["axes"]:
        label = f"{axis['module_position']}.{axis['role']}"
        for key in ("ring_position", "robot_model_joint", "direction"):
            fields[label + "." + key] = axis[key]
        fields[label + ".motor_encoder_counts_per_revolution"] = request["devices"][axis["device"]][
            "motor_encoder_counts_per_revolution"
        ]
    if unit is not None:
        for module in unit["spec"]["modules"]:
            for key in (
                "effective_wheel_radius_m",
                "steering_zero_counts",
                "drive_zero_counts",
                "backlash_rad",
            ):
                fields[module["module_position"] + "." + key] = module[key]
    return [
        path for path, value in fields.items() if value is None or value is False or value == "TBD"
    ]


def render(request, share, validate_module, validate_unit):
    machine = validate_module(request["module"], request["machine"])
    unit = validate_unit(request["module"], request["unit"])
    spec = machine["spec"]
    profile = yaml.safe_load((share / "config/profiles" / (spec["profile"] + ".yaml")).read_text())
    mock = request["use_mock_hardware"]
    missing = missing_facts(request, profile)
    if not mock and missing:
        raise ValueError("Missing real chassis facts: " + ", ".join(missing))
    calibrations = {module["module_position"]: module for module in unit["spec"]["modules"]}
    assembly = request["devices"][spec["assembly"]]
    families = yaml.safe_load((share / "config/profiles/families.yaml").read_text())["families"]
    for reference in request["module"]["device_refs"]:
        if request["devices"][reference]["family"] not in families:
            raise ValueError(f"Unsupported device family: {reference}")
    known_positions = all(type(axis["ring_position"]) is int for axis in spec["axes"])
    axes = (
        sorted(spec["axes"], key=lambda axis: axis["ring_position"])
        if known_positions
        else spec["axes"]
    )
    if len(axes) != request["module"]["actuator_count"]:
        raise ValueError("Axis count differs from declared module")
    joints, managed, runtime_files, responders = [], [], {}, []
    groups = {group["name"]: [] for group in request["module"]["mode_groups"]}
    for axis in axes:
        device = request["devices"][axis["device"]]
        family = families[device["family"]]
        if (
            axis["mode_of_operation"] not in family["certified_modes"]
            or axis["interface_contract"] not in family["interface_contracts"]
        ):
            raise ValueError("Unsupported Kinco mode/interface contract")
        name = (
            axis["mock_joint_name"]
            if mock and axis["robot_model_joint"] == "TBD"
            else axis["robot_model_joint"]
        )
        joint = ET.Element("joint", name=name)
        for interface in COMMANDS:
            element = ET.SubElement(joint, "command_interface", name=interface)
            _param(
                element,
                "initial_value",
                axis["mode_of_operation"] if interface == "mode_of_operation" else 0,
            )
        for interface in STATES:
            element = ET.SubElement(joint, "state_interface", name=interface)
            if mock:
                _param(
                    element,
                    "initial_value",
                    64
                    if interface == "status_word"
                    else axis["mode_of_operation"]
                    if interface == "mode_of_operation_display"
                    else 0,
                )
        config = deepcopy(profile["driver_config"])
        if mock:
            config.update(
                vendor_id=profile["mock_identity"]["vendor_id"],
                product_id=profile["mock_identity"]["product_code"],
                assign_activate=profile["mock_identity"]["assign_activate"],
            )
            adapter = deepcopy(profile["mock_adapter"])
        else:
            role = axis["role"]
            ratio = assembly[
                "steering_motor_turns_per_axis_turn"
                if role == "steering"
                else "drive_motor_turns_per_wheel_turn"
            ]
            factors = conversion_factors(
                device["motor_encoder_counts_per_revolution"],
                ratio,
                axis["direction"],
                profile["velocity_dec"],
            )
            calibration = calibrations[axis["module_position"]]
            bounds = assembly["spec"][role]
            minimum = (
                bounds["software_min_angle_rad"]
                if role == "steering"
                else bounds["min_position_rad"]
            )
            maximum = (
                bounds["software_max_angle_rad"]
                if role == "steering"
                else bounds["max_position_rad"]
            )
            adapter = {
                "verified": True,
                "mock_only": False,
                "position_counts_per_unit": factors[0],
                "velocity_counts_per_unit": factors[1],
                "position_offset_counts": calibration[
                    "steering_zero_counts" if role == "steering" else "drive_zero_counts"
                ],
                "min_position": minimum,
                "max_position": maximum,
                "max_abs_velocity": assembly["spec"]["max_abs_velocity_rad_s"],
                **profile["adapter"],
            }
            config.update(
                vendor_id=profile["identity"]["vendor_id"],
                product_id=profile["identity"]["product_code"],
                assign_activate=profile["assign_activate"],
            )
        config["kinco_cyclic_mode"] = adapter
        relative = name + ".yaml"
        runtime_files[relative] = yaml.safe_dump(config, sort_keys=False)
        if not mock:
            for key, value in {
                "name": name + "_drive",
                "plugin": profile["driver_plugin"],
                "alias": 0,
                "position": axis["ring_position"],
                "mode_of_operation": axis["mode_of_operation"],
                "slave_config": str(Path(request["runtime_dir"]) / relative),
            }.items():
                _param(joint, "ec_module." + key, value)
        joints.append(joint)
        managed.append(name)
        for group in request["module"]["mode_groups"]:
            if group["mode_of_operation"] == axis["mode_of_operation"]:
                groups[group["name"]].append(name)
        if type(axis["ring_position"]) is int:
            responders.append({"ring_position": axis["ring_position"], **profile["identity"]})
    topology = {
        "master_id": request["deployment"]["ethercat"]["master_id"],
        "responders": responders,
        "axes": [],
        "branches": [],
        "internal_ports": [],
        "preflight_required": False,
    }
    from .master_health import sensors as health_sensors

    params = {
        "master_id": topology["master_id"],
        "control_frequency": request["bus_cycle_hz"],
        "startup_bus_timeout_ms": 70000,
        "preload_timeout_ms": 5000,
    }
    if mock:
        params = {"calculate_dynamics": "false", **params}
    return {
        "hardware_system_id": machine["master_system"],
        "hardware_plugin": "mock_components/GenericSystem"
        if mock
        else "ethercat_driver/EthercatDriver",
        "hardware_params": params,
        "joints": joints,
        "sensors": health_sensors(request, axes),
        "managed_joints": managed,
        "actuator_groups": groups,
        "resource_claims": [f"ring:{item['ring_position']}" for item in responders],
        "runtime_files": runtime_files,
        "expected_topology": topology,
    }
