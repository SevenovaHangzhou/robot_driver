"""Compose a stationary 16-axis enable test without motion command interfaces.

The pinned CiA402 plugin preloads actual raw position before enable and holds
that target when no position command is mapped. PP receives no new-setpoint bit.
This is independent of the unqualified PP motion/force Action configuration.
"""

from dataclasses import dataclass
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import re
import subprocess

import yaml


@dataclass(frozen=True)
class StationaryEnable:
    robot_description: str
    controllers: dict


def _param(parent, name, value):
    ET.SubElement(parent, "param", name=name).text = str(value)


def verify_stationary_bus():
    """Read-only preflight before requesting the master; never resets or enables."""
    def output(*arguments):
        return subprocess.check_output(["ethercat", *arguments], text=True, timeout=10)

    master = output("master", "-m", "0")
    if not re.search(r"Active:\s+no", master) or not re.search(r"Slaves:\s+18\b", master):
        raise RuntimeError("Expected inactive master 0 with exactly 18 responders")
    slaves = output("slaves", "-m", "0", "-v")
    blocks = re.findall(r"=== Master 0, Slave (\d+) ===(.*?)(?==== Master|\Z)", slaves, re.S)
    if [int(position) for position, _ in blocks] != list(range(18)):
        raise RuntimeError("Unexpected EtherCAT enumeration")
    for position, block in blocks:
        p = int(position)
        vendor, product, revision = (0x00100000, 0x10F40931 if p == 0 else 0x10F40932, 0x10000) if p in (0, 17) else (0x5A65726F, 0x29252, 1)
        values = []
        for label in ("Vendor Id", "Product code", "Revision number"):
            match = re.search(label + r":\s+(0x[0-9a-fA-F]+)", block)
            if not match:
                raise RuntimeError("Missing slave identity")
            values.append(int(match.group(1), 16))
        if tuple(values) != (vendor, product, revision) or "State: PREOP" not in block:
            raise RuntimeError(f"Unexpected identity or AL state at position {p}")
    # Main ESC port 3 is X2/left; port 1 is X3/right; port 2 is the internal child.
    main = blocks[0][1]
    for port, downstream in ((3, 1), (1, 9), (2, 17)):
        match = re.search(rf"^\s+{port}\s+(?:MII|EBUS)\s+up\s+open\s+yes\s+(\d+)\b", main, re.M)
        if not match or int(match.group(1)) != downstream:
            raise RuntimeError("Arm branch wiring differs from the confirmed X2/X3 layout")
    for p in range(1, 17):
        def value(index, kind):
            return int(output("upload", "-m", "0", "-p", str(p), "-t", kind, hex(index), "0").split()[-1])
        status, error = value(0x6041, "uint16"), value(0x603F, "uint16")
        if status & 8 or status & 0x6F == 0x27 or value(0x6040, "uint16") != 0:
            raise RuntimeError(f"Axis {p} is faulted or already controlled")
        if error != 0 and not (p in (3, 12) and error == 0x730F):
            raise RuntimeError(f"Unqualified error 0x{error:04x} at axis {p}")


def build_stationary_enable(hardware_share, runtime_dir, *, use_mock_hardware=False):
    hardware_share, runtime_dir = Path(hardware_share), Path(runtime_dir)
    machines = hardware_share / "config/machines"
    inventory = yaml.safe_load((machines / "alfa_v3_arms_only.draft.yaml").read_text())
    zero = yaml.safe_load((machines / "alfa_v3_mechanical_zero.yaml").read_text())
    template = yaml.safe_load((machines / "zeroerr_stationary.yaml").read_text())
    if (zero.get("confirmed_by") != "user" or zero.get("external_transmission_ratio") != 1.0
            or zero.get("position_object") != 0x6064):
        raise ValueError("Stationary enable requires the confirmed output-encoder zero capture")
    if len(zero["axes"]) != 16:
        raise ValueError("A zero capture for all sixteen axes is required")
    zeros = {axis["ring_position"]: axis for axis in zero["axes"]}
    if set(zeros) != set(range(1, 17)):
        raise ValueError("The zero capture must cover sixteen unique physical positions")
    axes = [(side, axis) for side in ("left", "right") for axis in inventory["branches"][side]["axes"]]
    if [axis["ring_position"] for _, axis in axes] != list(range(1, 17)):
        raise ValueError("Unexpected physical axis order")
    if [axis["mode_of_operation"] for _, axis in axes] != [8] * 7 + [1] + [8] * 7 + [1]:
        raise ValueError("Expected fourteen CSP axes and two PP grippers")
    if inventory["master_id"] != 0 or [x["ring_position"] for x in inventory["extra_responders"]] != [0, 17]:
        raise ValueError("Unexpected master or branch coupler positions")
    runtime_dir.mkdir(parents=True, exist_ok=True)
    robot = ET.Element("robot", name="alfa_v3_stationary_enable")
    ET.SubElement(robot, "link", name="enable_test_base")
    system = ET.SubElement(robot, "ros2_control", name="ecat_arms", type="system")
    hardware = ET.SubElement(system, "hardware")
    ET.SubElement(hardware, "plugin").text = (
        "mock_components/GenericSystem" if use_mock_hardware else "ethercat_driver/EthercatDriver")
    for name, value in {"master_id": 0, "control_frequency": 250,
                        "startup_bus_timeout_ms": 70000, "preload_timeout_ms": 5000}.items():
        _param(hardware, name, value)
    names = []
    for side, axis in axes:
        position = axis["ring_position"]
        mode = axis["mode_of_operation"]
        raw_zero = zeros[position]["zero_counts"]
        if type(raw_zero) is not int or not -(2**31) <= raw_zero < 2**31:
            raise ValueError("Invalid raw zero count")
        # These are commissioning resource identifiers; no TF/Robot Model is published.
        name = side + "_" + axis["physical_joint"].lower()
        names.append(name)
        joint = ET.SubElement(system, "joint", name=name)
        command = ET.SubElement(joint, "command_interface", name="control_word")
        _param(command, "initial_value", 0)
        for state_name, initial in [("position_raw", raw_zero), ("digital_inputs", 0), ("status_word", 64)]:
            state = ET.SubElement(joint, "state_interface", name=state_name)
            if use_mock_hardware:
                _param(state, "initial_value", initial)
        config = deepcopy(template)
        config["sdo"][0]["value"] = mode
        path = runtime_dir / f"{name}.yaml"
        path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
        if not use_mock_hardware:
            for key, value in {"name": name + "_drive", "plugin": "ethercat_generic_plugins/EcCiA402Drive",
                               "alias": 0, "position": position, "mode_of_operation": mode,
                               "slave_config": path}.items():
                _param(joint, "ec_module." + key, value)
    sensors = {"ethercat_domain": {"process_data_age_ms": 0},
               "ethercat_master": {"link_up": 1, "slaves_responding": 18, "wc_error_count": 0}}
    sensors.update({f"ethercat_slave_{p}": {"al_state": 8} for p in range(1, 17)})
    for name, interfaces in sensors.items():
        sensor = ET.SubElement(system, "sensor", name=name)
        for key, value in interfaces.items():
            state = ET.SubElement(sensor, "state_interface", name=key)
            if use_mock_hardware:
                _param(state, "initial_value", value)
    controllers = {
        "controller_manager": {"ros__parameters": {
            "update_rate": 250, "thread_priority": 80,
            "enable_manager": {"type": "enable_manager/EnableManagerController"},
            "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            "rt_internal_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
        }},
        "enable_manager": {"ros__parameters": {
            "managed_joints": names, "enable_batch_joint_names": list(names),
            "enable_batch_sizes": [1] * 16, "disable_terminal_policy": "switch_on_disabled",
            "enable_only": True, "jtc_name": "",
            "batch_timeout": 4.0, "disable_stage_timeout": 4.0,
            "fault_reset_timeout": 4.0, "inter_batch_delay": 0.2,
            "service_result_timeout_ms": 90000,
        }},
        "joint_state_broadcaster": {"ros__parameters": {"use_local_topics": True}},
        "rt_internal_state_broadcaster": {"ros__parameters": {"use_local_topics": True}},
    }
    return StationaryEnable(ET.tostring(robot, encoding="unicode"), controllers)
