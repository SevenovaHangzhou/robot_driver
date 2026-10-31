"""Output-encoder CiA402 arm fragments; selected through the family registry."""

from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from infra_renderer import xml_param
import yaml

from .profiles import master_parameters


def _profile(share, name):
    return yaml.safe_load((share / "config/profiles" / (name + ".yaml")).read_text())


def _interface(parent, kind, name, initial=None):
    element = ET.SubElement(parent, kind, name=name)
    if initial is not None:
        xml_param(element, "initial_value", initial)


def position_factors(counts, direction, zero_counts):
    """Output-side encoder conversion; reduction ratio deliberately has no role."""
    radians_per_count = (2.0 * math.pi) / counts
    return {
        "command_factor": direction * (counts / (2.0 * math.pi)),
        "command_offset": zero_counts,
        "feedback_factor": direction * radians_per_count,
        "feedback_offset": -direction * zero_counts * radians_per_count,
    }


BASE_STATES = {"position", "position_raw", "digital_inputs", "status_word"}


def _render_diagnostic_channels(joint, profile, factors, direction, mock):
    """Extra TxPDO feedback and RxPDO mirrors declared by the profile become joint state interfaces.

    `scale` (renderer-only): velocity = output-side rad/s with the axis direction, direction = raw
    value times the axis direction. Without verified position factors a velocity channel is kept
    raw (counts/s); the scale key never reaches the driver configuration.
    """
    for pdo in profile.get("rpdo", []):
        for channel in pdo["channels"]:
            mirror = channel.get("mirror_state_interface")
            if mirror:
                _interface(joint, "state_interface", mirror, 0.0 if mock else None)
    for pdo in profile.get("tpdo", []):
        for channel in pdo["channels"]:
            scale = channel.pop("scale", None)
            name = channel.get("state_interface")
            if not name or name in BASE_STATES:
                continue
            _interface(joint, "state_interface", name, 0.0 if mock else None)
            if scale == "velocity" and factors is not None:
                channel["factor"] = factors["feedback_factor"]
            elif scale == "direction":
                channel["factor"] = float(direction)
            elif scale not in (None, "velocity"):
                raise ValueError(f"Unknown channel scale {scale!r} for {name}")


def render(request, share, validate_module, validate_unit, validate_device):
    """Render content only. The registry owns the shared request/result envelope."""
    machine = request["machine"]
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
    master_id = request["deployment"]["ethercat"]["master_id"]
    if not mock and master_id == "TBD":
        raise ValueError("Required deployment.ethercat.master_id is TBD")
    families = _profile(share, "families")["families"]
    runtime_files, joints, managed, expected_axes, diagnostic_devices = {}, [], [], [], []
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
        profile_name = spec["drive_profiles"]["gripper_pp" if stationary or mode == 1 else "csp"]
        # Only the motion CSP path accepts an explicit per-axis feedback profile.
        # Stationary-enable and PP retain their existing profiles and safety behavior.
        if not stationary and mode == 8:
            profile_name = axis.get("csp_profile", profile_name)
        record = _profile(share, profile_name)
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
        factors = None
        direction = 1 if mock or stationary else axis.get("direction", 1)
        if (
            not stationary
            and mode == 8
            and "output_encoder_counts_per_revolution" in device
            and device["output_encoder_counts_per_revolution"] != "TBD"
        ):
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
        _render_diagnostic_channels(joint, profile, factors, direction, mock)
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
                xml_param(joint, "ec_module." + key, value)
        joints.append(joint)
        policy = (
            record["encoder_policies"][axis["encoder_policy"]] if "encoder_policy" in axis else {}
        )
        expected_axes.append(
            {
                "ring_position": axis["ring_position"],
                **record["identity"],
                # Profile-wide startup codes (e.g. 0xA000 after the master left OP) plus the
                # battery codes of the axis encoder policy; enable_manager owns the CiA402 reset.
                "allowed_errors": sorted(
                    {
                        *record.get("startup_reset", {}).get("codes", []),
                        *policy.get("startup_battery_reset_codes", []),
                    }
                ),
            }
        )
        rendered_states = {
            item.attrib["name"] for item in joint.findall("state_interface")
        }
        diagnostic_devices.append(
            {
                "name": name,
                "role": "drive",
                "device": axis["device"],
                "address": {"master_id": master_id, "ring_position": axis["ring_position"]},
                "identity": dict(record["identity"]),
                "state": {
                    "al_state": f"ethercat_slave_{axis['ring_position']}/al_state",
                    **{
                        state_name: f"{name}/{state_name}"
                        for state_name in ("status_word", "error_code")
                        if state_name in rendered_states
                    },
                },
                "allowed_startup_codes": expected_axes[-1]["allowed_errors"],
            }
        )
    responders = []
    for responder in spec["extra_responders"]:
        record = _profile(share, responder["profile"])
        responders.append(
            {
                "ring_position": responder["ring_position"],
                **record["identities"][responder["identity_key"]],
            }
        )
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
        "kind": "cia402_topology",
        "preflight_required": stationary or spec.get("topology_preflight", False),
    }
    if spec.get("topology_preflight", False) and not stationary:
        expected["topology_only"] = True
    from .master_health import responder_count, sensors as health_sensors

    sensors = health_sensors(request, [axis for _, axis in axes])
    slaves = responder_count(request)
    if slaves is not None:
        diagnostic_devices.append(
            {
                "name": f"master{master_id}",
                "role": "ethercat_master",
                "bus": True,
                "address": {"master_id": master_id},
                "state": {
                    "link_up": "ethercat_master/link_up",
                    "slaves_responding": "ethercat_master/slaves_responding",
                    "wc_error_count": "ethercat_master/wc_error_count",
                    "process_data_age_ms": "ethercat_domain/process_data_age_ms",
                },
                "expected": {"slaves_responding": slaves},
            }
        )
    hardware_params = {}
    if mock and not stationary:
        hardware_params["calculate_dynamics"] = "false"
    hardware_params.update(
        master_id=master_id,
        control_frequency=request["bus_cycle_hz"],
        **master_parameters(share),
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
        "diagnostic_devices": diagnostic_devices,
    }
