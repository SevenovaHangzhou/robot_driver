"""EtherCAT owner's offline fragments and explicit read-only preflight (API 1)."""

from collections.abc import Mapping
from copy import deepcopy
from pathlib import Path
import math
import re
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from jsonschema import Draft7Validator
import json
import yaml

RENDERER_API_VERSION = 1


def _share():
    return Path(get_package_share_directory("robot_hw_ethercat"))


def _private_validate(data, filename):
    schema = json.loads((_share() / "schema" / filename).read_text())
    Draft7Validator(schema).validate(data)
    return deepcopy(dict(data))


def validate_module(module: Mapping, data: Mapping) -> Mapping:
    schema = (
        "kinco_module.schema.json" if data["spec"].get("family") == "kinco" else "arms.schema.json"
    )
    return {**deepcopy(dict(data)), "spec": _private_validate(data["spec"], schema)}


def validate_unit(module: Mapping, data: Mapping | None) -> Mapping:
    if data is None:
        raise ValueError("An output-encoder zero capture is required")
    schema = "kinco_unit.schema.json" if "modules" in data["spec"] else "arms_zero.schema.json"
    return {**deepcopy(dict(data)), "spec": _private_validate(data["spec"], schema)}


def validate_device(data: Mapping, *, render_mode: str) -> Mapping:
    """Only output resolution is required for CSP and hold-only PP position scaling."""
    field = (
        "motor_encoder_counts_per_revolution"
        if data["family"] == "kinco"
        else "output_encoder_counts_per_revolution"
    )
    value = data.get(field)
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"Required {field} must be positive and finite")
    return deepcopy(dict(data))


def _profile(name):
    return yaml.safe_load((_share() / "config/profiles" / (name + ".yaml")).read_text())


def _param(parent, name, value):
    ET.SubElement(parent, "param", name=name).text = str(value)


def _interface(parent, kind, name, initial=None):
    element = ET.SubElement(parent, kind, name=name)
    if initial is not None:
        _param(element, "initial_value", initial)


def position_factors(counts, direction, zero_counts):
    """Output-side encoder conversion; reduction ratio deliberately has no role."""
    radians_per_count = (2.0 * math.pi) / counts
    return {
        "command_factor": direction * (counts / (2.0 * math.pi)),
        "command_offset": zero_counts,
        "feedback_factor": direction * radians_per_count,
        "feedback_offset": -direction * zero_counts * radians_per_count,
    }


def render(request: Mapping) -> Mapping:
    """Render content only. The registry owns the shared request/result envelope."""
    machine = request["machine"]
    if machine["spec"].get("family") == "kinco":
        from .kinco_renderer import render as render_kinco

        return render_kinco(request, _share(), validate_module, validate_unit)
    unit = request["unit"]
    module = request["module"]
    spec = validate_module(module, machine)["spec"]
    zero = validate_unit(module, unit)["spec"]
    mock = request["use_mock_hardware"]
    stationary = request["render_mode"] == "stationary_enable"
    axes = sorted(
        ((side, axis) for side, branch in spec["branches"].items() for axis in branch["axes"]),
        key=lambda entry: entry[1]["ring_position"],
    )
    zeros = {entry["ring_position"]: entry for entry in zero["axes"]}
    if len(axes) != module["actuator_count"] or any(
        sum(axis["mode_of_operation"] == group["mode_of_operation"] for _, axis in axes)
        != group["count"]
        for group in module["mode_groups"]
    ):
        raise ValueError("Axis count or mode groups differ from the declared module")
    if len(zeros) != len(zero["axes"]) or set(zeros) != {axis["ring_position"] for _, axis in axes}:
        raise ValueError("The zero capture must cover unique physical axis positions")
    if stationary:
        if (
            zero["confirmed_by"] != "user"
            or zero["external_transmission_ratio"] != 1.0
            or zero["position_object"] != 0x6064
        ):
            raise ValueError("Stationary enable requires the confirmed output-encoder zero capture")
    elif not mock:
        if machine["verified"] is not True:
            raise ValueError("Arm motion calibration is not verified")
        if spec["branch_mapping_verified"] is not True:
            raise ValueError("Arm branch mapping is not verified")
        if any(
            type(axis["direction"]) is not int or axis["direction"] not in (-1, 1)
            for _, axis in axes
            if axis["mode_of_operation"] == 8
        ):
            raise ValueError("Every CSP direction must be verified as +1 or -1")
    families = _profile("families")["families"]
    runtime_files, joints, managed, expected_axes = {}, [], [], []
    for side, axis in axes:
        device = request["devices"][axis["device"]]
        if not mock:
            device = validate_device(device, render_mode=request["render_mode"])
        family = families[device["family"]]
        mode = axis["mode_of_operation"]
        if mode not in family["certified_modes"]:
            raise ValueError(f"Uncertified mode {mode} for {device['family']}")
        name = (
            side + "_" + axis["physical_joint"].lower() if stationary else axis["robot_model_joint"]
        )
        managed.append(name)
        joint = ET.Element("joint", name=name)
        record = _profile(
            spec["drive_profiles"]["gripper_pp" if stationary or mode == 1 else "csp"]
        )
        profile = deepcopy(record["driver_config"])
        profile["sdo"][0]["value"] = mode
        raw_zero = zeros[axis["ring_position"]]["zero_counts"]
        if not stationary and mode == 8:
            _interface(joint, "command_interface", "position")
        _interface(joint, "command_interface", "control_word", 0)
        state = "position_raw" if stationary or mode == 1 else "position"
        _interface(
            joint,
            "state_interface",
            state,
            (raw_zero if state == "position_raw" else 0.0) if mock else None,
        )
        _interface(joint, "state_interface", "digital_inputs", 0 if mock else None)
        _interface(joint, "state_interface", "status_word", 64 if mock else None)
        if (
            not stationary
            and mode == 8
            and "output_encoder_counts_per_revolution" in device
            and device["output_encoder_counts_per_revolution"] != "TBD"
        ):
            direction = 1 if mock else axis["direction"]
            factors = position_factors(
                device["output_encoder_counts_per_revolution"], direction, raw_zero
            )
            target, actual = profile["rpdo"][0]["channels"][0], profile["tpdo"][0]["channels"][0]
            target.update(
                command_interface="position",
                factor=factors["command_factor"],
                offset=factors["command_offset"],
            )
            actual.update(
                state_interface="position",
                factor=factors["feedback_factor"],
                offset=factors["feedback_offset"],
            )
        relative = name + ".yaml"
        runtime_files[relative] = yaml.safe_dump(profile, sort_keys=False)
        if not mock:
            for key, value in {
                "name": name + "_drive",
                "plugin": "ethercat_generic_plugins/EcCiA402Drive",
                "alias": 0,
                "position": axis["ring_position"],
                "mode_of_operation": mode,
                "slave_config": str(Path(request["runtime_dir"]) / relative),
            }.items():
                _param(joint, "ec_module." + key, value)
        joints.append(joint)
        policy = (
            record["encoder_policies"][axis["encoder_policy"]] if "encoder_policy" in axis else {}
        )
        expected_axes.append(
            {
                "ring_position": axis["ring_position"],
                **record["identity"],
                "allowed_errors": policy.get("startup_battery_reset_codes", []),
            }
        )
    responders = []
    for responder in spec["extra_responders"]:
        record = _profile(responder["profile"])
        responders.append(
            {
                "ring_position": responder["ring_position"],
                **record["identities"][responder["identity_key"]],
            }
        )
    master_id = request["deployment"]["ethercat"]["master_id"]
    if not mock and master_id == "TBD":
        raise ValueError("Required deployment.ethercat.master_id is TBD")
    expected = {
        "master_id": master_id,
        "responders": sorted(responders + expected_axes, key=lambda item: item["ring_position"]),
        "axes": expected_axes,
        "branches": [
            {
                "main_esc_port": branch["main_esc_port"],
                "downstream": min(axis["ring_position"] for axis in branch["axes"]),
            }
            for branch in spec["branches"].values()
        ],
        "internal_ports": [
            {"main_esc_port": 2, "downstream": item["ring_position"]}
            for item in spec["extra_responders"]
            if item["identity_key"] == "coupler_subdevice"
        ],
        "preflight_required": stationary,
    }
    from .master_health import sensors as health_sensors

    sensors = health_sensors(request, [axis for _, axis in axes])
    hardware_params = {}
    if mock and not stationary:
        hardware_params["calculate_dynamics"] = "false"
    hardware_params.update(
        master_id=master_id,
        control_frequency=request["bus_cycle_hz"],
        startup_bus_timeout_ms=70000,
        preload_timeout_ms=5000,
    )
    groups = {
        group["name"]: [
            managed[index]
            for index, (_, axis) in enumerate(axes)
            if axis["mode_of_operation"] == group["mode_of_operation"]
        ]
        for group in module["mode_groups"]
    }
    return {
        "hardware_system_id": machine["master_system"],
        "hardware_plugin": "mock_components/GenericSystem"
        if mock
        else "ethercat_driver/EthercatDriver",
        "hardware_params": hardware_params,
        "joints": joints,
        "sensors": sensors,
        "runtime_files": runtime_files,
        "managed_joints": managed,
        "actuator_groups": groups,
        "resource_claims": [f"ring:{item['ring_position']}" for item in expected["responders"]],
        "expected_topology": expected,
    }


def preflight(expected_topology: Mapping, *, run=None) -> None:
    """Reproduce the old stationary checks. Motion has no new admission preflight."""
    topology = expected_topology
    topology = {
        **topology,
        "responders": sorted(topology["responders"], key=lambda item: item["ring_position"]),
    }
    if not topology["preflight_required"]:
        return
    execute = subprocess.check_output if run is None else run
    master_id = str(topology["master_id"])

    def output(*arguments):
        return execute(["ethercat", *arguments], text=True, timeout=10)

    master = output("master", "-m", master_id)
    if not re.search(r"Active:\s+no", master) or not re.search(
        rf"Slaves:\s+{len(topology['responders'])}\b", master
    ):
        raise RuntimeError("Expected inactive master with the declared responder count")
    slaves = output("slaves", "-m", master_id, "-v")
    blocks = re.findall(
        rf"=== Master {master_id}, Slave (\d+) ===(.*?)(?==== Master|\Z)", slaves, re.S
    )
    if [int(position) for position, _ in blocks] != [
        item["ring_position"] for item in topology["responders"]
    ]:
        raise RuntimeError("Unexpected EtherCAT enumeration")
    for (_, block), identity in zip(blocks, topology["responders"]):
        for label, key in (
            ("Vendor Id", "vendor_id"),
            ("Product code", "product_code"),
            ("Revision number", "revision_number"),
        ):
            match = re.search(label + r":\s+(0x[0-9a-fA-F]+)", block)
            if not match or int(match.group(1), 16) != identity[key]:
                raise RuntimeError("Unexpected slave identity")
        if "State: PREOP" not in block:
            raise RuntimeError("Unexpected slave AL state")
    main = blocks[0][1]
    for branch in topology["branches"] + topology["internal_ports"]:
        match = re.search(
            rf"^\s+{branch['main_esc_port']}\s+(?:MII|EBUS)\s+up\s+open\s+yes\s+(\d+)\b", main, re.M
        )
        if not match or int(match.group(1)) != branch["downstream"]:
            raise RuntimeError("Branch wiring differs from the declared layout")
    for axis in topology["axes"]:

        def value(index):
            return int(
                output(
                    "upload",
                    "-m",
                    master_id,
                    "-p",
                    str(axis["ring_position"]),
                    "-t",
                    "uint16",
                    hex(index),
                    "0",
                ).split()[-1]
            )

        status, error = value(0x6041), value(0x603F)
        if status & 8 or status & 0x6F == 0x27 or value(0x6040) != 0:
            raise RuntimeError("Axis is faulted or already controlled")
        if error != 0 and error not in axis["allowed_errors"]:
            raise RuntimeError(f"Unqualified error 0x{error:04x}")
