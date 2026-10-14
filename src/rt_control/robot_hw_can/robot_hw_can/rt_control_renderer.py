"""DaMiao Mapping renderer: owner shape checks and offline XML only."""

from collections.abc import Mapping
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from jsonschema import Draft7Validator

RENDERER_API_VERSION = 1


def _share():
    return Path(get_package_share_directory("robot_hw_can"))


def _validate(data, file):
    Draft7Validator(json.loads((_share() / "schema" / file).read_text())).validate(data)
    return deepcopy(dict(data))


def validate_module(module: Mapping, data: Mapping) -> Mapping:
    spec = _validate(data["spec"], "head.schema.json")
    if [joint["robot_model_joint"] for joint in spec["joints"]] != [
        "head_joint",
        "head_pitch_joint",
    ]:
        raise ValueError("Head joints must retain the declared model order")
    for key in ("can_id", "master_id", "robot_model_joint"):
        values = [joint[key] for joint in spec["joints"]]
        if len(set(values)) != len(values):
            raise ValueError(f"Head {key} must be unique")
    return {**deepcopy(dict(data)), "spec": spec}


def validate_unit(module: Mapping, data: Mapping | None) -> Mapping:
    # The current head protocol reads the device's established zero; no new offset is applied.
    if data is None:
        return {}
    return {**deepcopy(dict(data)), "spec": _validate(data["spec"], "head_zero.schema.json")}


def validate_device(data: Mapping, *, render_mode: str) -> Mapping:
    if data["family"] != "damiao":
        raise ValueError("Head device must use the declared protocol family")
    return deepcopy(dict(data))


def _param(parent, name, value):
    ET.SubElement(parent, "param", name=name).text = str(value)


def render(request: Mapping) -> Mapping:
    machine = validate_module(request["module"], request["machine"])
    validate_unit(request["module"], request["unit"])
    mock = request["use_mock_hardware"]
    if not mock and machine["verified"] is not True:
        raise ValueError("D-b: head_gimbal configuration is not verified")
    spec = machine["spec"]
    params = (
        {"calculate_dynamics": "true"}
        if mock
        else {
            "can_interface": request["deployment"]["can_interfaces"][
                request["module"]["can_interface_key"]
            ],
            **{
                key: spec[key]
                for key in (
                    "configure_timeout_ms",
                    "feedback_timeout_ms",
                    "transition_timeout_ms",
                    "command_rate_hz",
                    "disabled_poll_interval_ms",
                    "max_rx_frames_per_cycle",
                )
            },
        }
    )
    if not mock and params["can_interface"] == "TBD":
        raise ValueError("Required head CAN interface is TBD")
    joints = []
    limits = request["robot_model"]["joint_limits"]
    limits = limits["joints"]
    for entry in spec["joints"]:
        validate_device(request["devices"][entry["device"]], render_mode=request["render_mode"])
        joint = ET.Element("joint", name=entry["robot_model_joint"])
        for key in (
            "can_id",
            "master_id",
            "velocity_limit",
            "acceleration_krad_s2",
            "deceleration_krad_s2",
            "maximum_speed_rad_s",
        ):
            _param(joint, key, entry[key])
        bound = limits[entry["robot_model_joint"]]
        lower, upper = bound["lower_position_rad"], bound["upper_position_rad"]
        if lower >= upper:
            raise ValueError("Head model limits require min < max")
        command = ET.SubElement(joint, "command_interface", name="position")
        _param(command, "min", lower)
        _param(command, "max", upper)
        for name in (
            "position",
            "velocity",
            "effort",
            "fault_code",
            "mos_temperature",
            "motor_temperature",
            "feedback_age_ms",
            "enabled",
        ):
            _param(ET.SubElement(joint, "state_interface", name=name), "initial_value", 0.0)
        joints.append(joint)
    control = ET.Element("gpio", name="damiao_head_control")
    for name in ("enable_request", "reset_generation"):
        _param(ET.SubElement(control, "command_interface", name=name), "initial_value", 0.0)
    for name in ("phase", "fault_latched", "handled_reset_generation", "motion_allowed"):
        _param(ET.SubElement(control, "state_interface", name=name), "initial_value", 0.0)
    # This protocol does not consume an EtherCAT-style control_frequency parameter.
    return {
        "hardware_system_id": machine["master_system"],
        "hardware_plugin": "mock_components/GenericSystem" if mock else "robot_hw_can/DamiaoSystem",
        "hardware_params": params,
        "joints": joints,
        "sensors": [],
        "gpios": [control],
        "managed_joints": [],
        "actuator_groups": {
            group["name"]: [entry["robot_model_joint"] for entry in spec["joints"]]
            for group in request["module"]["mode_groups"]
        },
        "resource_claims": [f"can:{entry['can_id']}" for entry in spec["joints"]],
        "runtime_files": {},
        "expected_topology": {"preflight_required": False},
    }


def preflight(expected_topology: Mapping, *, run=None) -> None:
    """No additional head CAN preflight is introduced by the migration."""
