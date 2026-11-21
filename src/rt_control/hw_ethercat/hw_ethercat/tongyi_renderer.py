"""Tongyi XMC_ESC linear axes in Profile Position mode (family `tongyi`).

Facts and their owners: the fixed PDO layout, velocity unit and adapter timings live in the
protocol profile; counts per motor turn in the device file; lead, stroke and the speed limit in
the machine module; the encoder zero in the unit calibration. A real start needs all of them.
"""

from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from infra_renderer import xml_param
import yaml

from .master_health import master_diagnostic, offline_exports, sensors as health_sensors
from .profiles import load_profile, master_parameters, pdo_objects

# Interface names and order of hw_ethercat/TongyiPpSlave (tongyi_pp_slave.hpp).
COMMANDS = ("position", "profile_velocity", "pp_sequence", "pp_halt", "control_word")
STATES = (
    "position",
    "velocity",
    "status_word",
    "error_code",
    "mode_of_operation_display",
    "pp_sequence",
    "pp_state",
    "velocity_raw",
    "current_raw",
    "torque_raw",
    "bridge_temperature_raw",
    "motor_temperature_raw",
    "digital_inputs",
)
PROFILE_POSITION = 1


def _tbd(value):
    return value is None or value is False or value == "TBD"


def missing_facts(request, profile, calibrations):
    machine, spec = request["machine"], request["machine"]["spec"]
    fields = {
        "machine.verified": machine["verified"],
        "unit.verified": None if request["unit"] is None else request["unit"]["verified"],
        "deployment.ethercat.master_id": request["deployment"]["ethercat"]["master_id"],
    }
    for key in (
        "metres_per_motor_turn",
        "stroke_min_m",
        "stroke_max_m",
        "max_velocity_m_s",
        "profile_acceleration_ms_per_krpm",
        "profile_deceleration_ms_per_krpm",
    ):
        fields["spec." + key] = spec[key]
    for axis in spec["axes"]:
        label = axis["robot_model_joint"]
        for key in ("ring_position", "direction"):
            fields[f"{label}.{key}"] = axis[key]
        fields[f"{label}.motor_encoder_counts_per_revolution"] = request["devices"][axis["device"]][
            "motor_encoder_counts_per_revolution"
        ]
        fields[f"{label}.zero_counts"] = calibrations.get(axis["ring_position"], {}).get(
            "zero_counts"
        )
    return [path for path, value in fields.items() if _tbd(value)]


def _joint(name, mock):
    joint = ET.Element("joint", name=name)
    for interface in COMMANDS:
        element = ET.SubElement(joint, "command_interface", name=interface)
        # A position target starts as "no producer"; 0 would be a real target.
        xml_param(element, "initial_value", "nan" if interface == "position" else 0)
    for interface in STATES:
        element = ET.SubElement(joint, "state_interface", name=interface)
        if mock:
            initial = {"status_word": 64, "mode_of_operation_display": PROFILE_POSITION}
            xml_param(element, "initial_value", initial.get(interface, 0))
    return joint


def _pp(spec, profile, device, axis, zero):
    """The plugin's `pp` block: signed counts per metre and the drive's velocity scale."""
    counts = device["motor_encoder_counts_per_revolution"] / spec["metres_per_motor_turn"]
    # 0x6081/0x607F are in `profile_velocity_unit_rpm` of motor speed.
    per_mps = 60.0 / spec["metres_per_motor_turn"] / profile["profile_velocity_unit_rpm"]
    maximum = spec["max_velocity_m_s"] * per_mps
    if not math.isclose(maximum, round(maximum), abs_tol=1e-9):
        raise ValueError("Maximum velocity is not a whole drive velocity unit")
    return {
        "verified": True,
        "counts_per_metre": axis["direction"] * counts,
        "zero_counts": zero,
        "min_position": spec["stroke_min_m"],
        "max_position": spec["stroke_max_m"],
        "max_velocity": spec["max_velocity_m_s"],
        "profile_velocity_per_metre_per_second": per_mps,
        "max_profile_velocity": int(round(maximum)),
        **profile["adapter"],
    }


def render(request, share, validate_module, validate_unit, validate_device):
    machine = validate_module(request["module"], request["machine"])
    unit = validate_unit(request["module"], request["unit"])
    spec = machine["spec"]
    profile = load_profile(share, spec["profile"])
    identity = profile["identities"][spec["identity_key"]]
    mock = request["use_mock_hardware"]
    calibrations = {item["ring_position"]: item for item in unit.get("spec", {}).get("axes", [])}
    if len(calibrations) != len(unit.get("spec", {}).get("axes", [])):
        raise ValueError("Duplicate Tongyi calibration ring_position")
    missing = missing_facts(request, profile, calibrations)
    if not mock and missing:
        raise ValueError("Missing real Tongyi PP facts: " + ", ".join(missing))
    axes = spec["axes"]
    if len(axes) != request["module"]["actuator_count"]:
        raise ValueError("Axis count differs from declared module")
    master_id = request["deployment"]["ethercat"]["master_id"]
    joints, managed, runtime_files, responders = [], [], {}, []
    preflight_axes, pdo_checks, position_windows, diagnostic_devices = [], [], [], []
    groups = {group["name"]: [] for group in request["module"]["mode_groups"]}
    for axis in axes:
        name = axis["robot_model_joint"]
        config = deepcopy(profile["driver_config"])
        if mock:
            config.update(
                vendor_id=profile["mock_identity"]["vendor_id"],
                product_id=profile["mock_identity"]["product_code"],
                assign_activate=profile["mock_identity"]["assign_activate"],
            )
        else:
            device = validate_device(request["devices"][axis["device"]], render_mode="")
            zero = calibrations[axis["ring_position"]]["zero_counts"]
            config.update(
                vendor_id=identity["vendor_id"],
                product_id=identity["product_code"],
                assign_activate=profile["assign_activate"],
                pp=_pp(spec, profile, device, axis, zero),
                sdo=[
                    {"index": index, "sub_index": 0, "type": "uint32", "value": spec[key]}
                    for index, key in (
                        (0x6083, "profile_acceleration_ms_per_krpm"),
                        (0x6084, "profile_deceleration_ms_per_krpm"),
                    )
                ],
            )
        joint = _joint(name, mock)
        relative = name + ".yaml"
        runtime_files[relative] = yaml.safe_dump(config, sort_keys=False)
        if not mock:
            for key, value in {
                "name": name + "_drive",
                "plugin": profile["driver_plugin"],
                "alias": 0,
                "position": axis["ring_position"],
                "mode_of_operation": PROFILE_POSITION,
                "slave_config": str(Path(request["runtime_dir"]) / relative),
            }.items():
                xml_param(joint, "ec_module." + key, value)
        joints.append(joint)
        managed.append(name)
        for group in request["module"]["mode_groups"]:
            if group["mode_of_operation"] == PROFILE_POSITION:
                groups[group["name"]].append(name)
        diagnostic_devices.append(
            {
                "name": name,
                "role": "drive",
                "device": axis["device"],
                "address": {"master_id": master_id, "ring_position": axis["ring_position"]},
                "state": {
                    "al_state": f"ethercat_slave_{axis['ring_position']}/al_state",
                    "status_word": f"{name}/status_word",
                    "error_code": f"{name}/error_code",
                },
            }
        )
        if type(axis["ring_position"]) is int:
            responders.append({"ring_position": axis["ring_position"], **identity})
        if not mock:
            ring = axis["ring_position"]
            preflight_axes.append({"ring_position": ring, "allowed_errors": []})
            pdo_checks.append({"ring_position": ring, "objects": pdo_objects(config)})
            # The encoder is absolute: a start outside the hard stops means a wrong zero.
            pp = config["pp"]
            ends = sorted(
                pp["zero_counts"] + pp["counts_per_metre"] * bound
                for bound in (
                    spec["stroke_min_m"] - spec["hard_stop_margin_m"],
                    spec["stroke_max_m"] + spec["hard_stop_margin_m"],
                )
            )
            position_windows.append({"ring_position": ring, "raw_min": ends[0], "raw_max": ends[1]})
    for extra in spec.get("extra_responders", []):
        record = load_profile(share, extra["profile"])
        responders.append(
            {"ring_position": extra["ring_position"], **record["identities"][extra["identity_key"]]}
        )
    responders.sort(key=lambda item: item["ring_position"])
    topology = {
        "master_id": master_id,
        "responders": responders,
        "axes": preflight_axes,
        "branches": [],
        "internal_ports": [],
        "kind": "cia402_topology",
        "pdo_checks": pdo_checks,
        "position_windows": position_windows,
        "preflight_required": not mock,
    }
    exports = {"joint_names": managed, **offline_exports(request, axes)}
    if len(managed) == 1:
        exports["joint_name"] = managed[0]
    # Commands of a motion producer; control_word stays with enable_manager.
    exports["motion_command_interfaces"] = [name for name in COMMANDS if name != "control_word"]
    params = {
        "master_id": master_id,
        "control_frequency": request["bus_cycle_hz"],
        **master_parameters(share),
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
        "parameter_exports": exports,
        "actuator_groups": groups,
        "resource_claims": [f"ring:{item['ring_position']}" for item in responders],
        "runtime_files": runtime_files,
        "expected_topology": topology,
        "diagnostic_devices": master_diagnostic(request, master_id) + diagnostic_devices,
    }
