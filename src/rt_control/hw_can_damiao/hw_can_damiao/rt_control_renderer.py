"""DaMiao Mapping renderer: owner shape checks, offline XML and the startup CAN preflight."""

from collections.abc import Mapping
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from infra_renderer import validated_spec, xml_param
from infra_socketcan.link import link_problem

RENDERER_API_VERSION = 1


def _share():
    return Path(get_package_share_directory("hw_can_damiao"))


def validate_module(module: Mapping, data: Mapping) -> Mapping:
    machine = validated_spec(data, _share() / "schema/head.schema.json")
    spec = machine["spec"]
    if spec["control_name"] in {joint["robot_model_joint"] for joint in spec["joints"]}:
        raise ValueError("Control GPIO and joint names must be distinct")
    for key in ("can_id", "master_id", "robot_model_joint"):
        values = [joint[key] for joint in spec["joints"]]
        if len(set(values)) != len(values):
            raise ValueError(f"Head {key} must be unique")
    return machine


def validate_unit(module: Mapping, data: Mapping | None) -> Mapping:
    # The current head protocol reads the device's established zero; no new offset is applied.
    if data is None:
        return {}
    return validated_spec(data, _share() / "schema/head_zero.schema.json")


def validate_device(data: Mapping, *, render_mode: str) -> Mapping:
    if data["family"] != "damiao":
        raise ValueError("Head device must use the declared protocol family")
    return deepcopy(dict(data))


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
            xml_param(joint, key, entry[key])
        bound = limits[entry["robot_model_joint"]]
        lower, upper = bound["lower_position_rad"], bound["upper_position_rad"]
        if lower >= upper:
            raise ValueError("Head model limits require min < max")
        command = ET.SubElement(joint, "command_interface", name="position")
        xml_param(command, "min", lower)
        xml_param(command, "max", upper)
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
            xml_param(ET.SubElement(joint, "state_interface", name=name), "initial_value", 0.0)
        joints.append(joint)
    control = ET.Element("gpio", name=spec["control_name"])
    for name in ("enable_request", "reset_generation"):
        xml_param(ET.SubElement(control, "command_interface", name=name), "initial_value", 0.0)
    for name in ("phase", "fault_latched", "handled_reset_generation", "motion_allowed"):
        xml_param(ET.SubElement(control, "state_interface", name=name), "initial_value", 0.0)
    # This protocol does not consume an EtherCAT-style control_frequency parameter.
    return {
        "hardware_system_id": machine["master_system"],
        "hardware_plugin": "mock_components/GenericSystem" if mock else "hw_can_damiao/DamiaoSystem",
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
        "diagnostic_devices": [
            {
                "name": entry["robot_model_joint"],
                "role": "head_motor",
                "device": entry["device"],
                "address": {"can_id": entry["can_id"]},
                "state": {
                    "fault_code": f"{entry['robot_model_joint']}/fault_code",
                    "feedback_age_ms": f"{entry['robot_model_joint']}/feedback_age_ms",
                },
                "expected": {"max_feedback_age_ms": spec["feedback_timeout_ms"]},
            }
            for entry in spec["joints"]
        ],
        "parameter_exports": {
            "joint_names": [entry["robot_model_joint"] for entry in spec["joints"]],
            "control_name": spec["control_name"],
            # The DaMiao plugin sequences enable itself; enable_manager drives it by handshake.
            "handshake_devices": [spec["control_name"]],
            "feedback_age_interfaces": [
                f"{entry['robot_model_joint']}/feedback_age_ms" for entry in spec["joints"]
            ],
        },
        "runtime_files": {},
        "expected_topology": {
            "preflight_required": not mock,
            "can_interface": None if mock else params["can_interface"],
            "motors": [
                {"can_id": entry["can_id"], "master_id": entry["master_id"]}
                for entry in spec["joints"]
            ],
            "timeout_ms": spec["configure_timeout_ms"],
        },
    }


def preflight(expected_topology: Mapping, *, run=None) -> list:
    """Interface up and error-active, then one disable command per motor (user 2026-11-04 allows
    these CAN frames): a motor that does not answer is offline, an ERR nibble above 1 a fault."""
    if not expected_topology.get("preflight_required"):
        return []
    interface = expected_topology["can_interface"]
    problem = link_problem(interface, run)
    if problem is not None:
        return [
            {"address": {"can_interface": interface}, "problem": problem[0],
             "detail": problem[1], "blocking": True}
        ]
    execute = subprocess.check_output if run is None else run
    tool = Path(get_package_prefix("hw_can_damiao")) / "lib/hw_can_damiao/damiao_preflight"
    motors = expected_topology["motors"]
    output = execute(
        [str(tool), interface, str(expected_topology["timeout_ms"]),
         *(f"{m['can_id']}:{m['master_id']}" for m in motors)],
        text=True,
        timeout=5 + len(motors) * expected_topology["timeout_ms"] / 1000,
    )
    findings = []
    for line in output.splitlines():
        reply = json.loads(line)
        address = {"can_id": reply["can_id"]}
        if not reply["answered"]:
            findings.append({"address": address, "problem": "offline", "blocking": True,
                             "detail": f"DaMiao motor {reply['can_id']} did not answer on {interface}"})
        elif reply["status"] > 1:
            findings.append({"address": address, "problem": "fault_code", "blocking": True,
                             "fault_code": reply["status"],
                             "detail": f"DaMiao motor {reply['can_id']} reports ERR {reply['status']}"})
    return findings


def select_startup(request):
    from .presence import probe_module
    rows = []
    for name, machine in request['modules'].items():
        declaration = request['declarations'][name]
        mode = request['choices'].get(name, 'auto')
        if mode not in ('auto', 'none', 'required'):
            raise ValueError(f'Unknown startup selection for {name}: {mode}')
        if mode == 'auto' and request['mock'] and request.get('probe') is None:
            raise ValueError('Mock head discovery requires explicit probe evidence')
        evidence = ((request.get('probe') or probe_module)(machine, declaration, request['deployment'])
                    if mode == 'auto' else {'present': None if mode == 'none' else True,
                                           'reason': mode})
        selected = bool(evidence['present'])
        if not selected and declaration['startup']['policy'] == 'required':
            raise ValueError(f'Required module unavailable: {name}')
        spec = machine['spec']
        joints = [joint['robot_model_joint'] for joint in spec['joints']]
        rows.append(dict(module=name, component='', option=declaration['startup'].get('option', ''), label=declaration['startup']['label'],
            policy=declaration['startup']['policy'], present=evidence['present'], selected=selected,
            reason='excluded' if mode == 'none' else 'present' if selected else 'absent',
            detail=evidence, domain=None, joints=joints,
            health_interfaces=[f'{joint}/feedback_age_ms' for joint in joints] +
                [f"{spec['control_name']}/fault_latched"]))
    return dict(modules=request['modules'], rows=rows,
                removed=[row['module'] for row in rows if not row['selected']])
