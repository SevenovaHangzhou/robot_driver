"""Compose a sensor-only Blue Point raw-data bench, without actuator modules."""

from dataclasses import dataclass
from pathlib import Path
import re
import subprocess
import uuid
import xml.etree.ElementTree as ET

import yaml


@dataclass(frozen=True)
class ForceSensorRuntime:
    robot_description: str
    controllers: dict
    controller_names: tuple[str, ...]


def _param(parent, name, value):
    ET.SubElement(parent, "param", name=name).text = str(value)


def _read(argv):
    result = subprocess.run(argv, capture_output=True, text=True, timeout=5, check=True)
    return result.stdout


def check_bench(sensors, profiles, run=_read):
    """Only mailbox/discovery reads; never start a master or change slave state."""
    master = run(["ethercat", "master", "-m", "0"])
    for pattern in (r"Phase:\s*Idle\b", r"Active:\s*no\b", r"Link:\s*UP\b", r"Slaves:\s*6\b"):
        if not re.search(pattern, master):
            raise ValueError("Force bench requires an idle master, link UP and exactly six responders")
    for sensor, profile in zip(sensors, profiles):
        position = sensor["ring_position"]
        details = run(["ethercat", "slaves", "-m", "0", "-p", str(position), "-v"])
        expected = {
            "Vendor Id": profile["vendor_id"],
            "Product code": profile["product_id"],
            "Revision number": profile["metadata"]["revision_id"],
        }
        for label, value in expected.items():
            match = re.search(re.escape(label) + r":\s*(0x[0-9a-fA-F]+)", details)
            if match is None or int(match[1], 16) != value:
                raise ValueError(f"Force sensor {position} identity mismatch: {label}")
        if not re.search(r"State:\s*PREOP\b", details) or not re.search(r"Flag:\s*\+", details):
            raise ValueError(f"Force sensor {position} must be error-free PREOP")
        pdos = run(["ethercat", "pdos", "-m", "0", "-p", str(position)])
        entries = [(int(i, 16), int(s, 16), int(bits)) for i, s, bits in re.findall(
            r"PDO entry (0x[0-9a-fA-F]+):([0-9a-fA-F]+),\s*(\d+) bit", pdos)]
        expected_entries = [
            (channel["index"], channel["sub_index"], 32)
            for direction in ("rpdo", "tpdo")
            for pdo in profile[direction]
            for channel in pdo["channels"]
        ]
        if entries != expected_entries:
            raise ValueError(f"Force sensor {position} PDO layout mismatch")
        for label, index in (("RxPDO", 0x1600), ("TxPDO", 0x1A00)):
            if not re.search(label + rf"\s+0x{index:04x}\b", pdos, re.IGNORECASE):
                raise ValueError(f"Force sensor {position} missing {label}")


def build_force_sensor_runtime(*, hardware_share, bringup_share, runtime_dir,
                               use_mock_hardware=True, sync_mode="", run=_read):
    hardware_share, bringup_share, runtime_dir = map(Path, (hardware_share, bringup_share, runtime_dir))
    document = yaml.safe_load((bringup_share / "config/bluepoint_force_sensors.draft.yaml").read_text())
    sensors = document["sensors"]
    if [(s["side"], s["ring_position"]) for s in sensors] != [("left", 2), ("right", 4)]:
        raise ValueError("This entry point supports only the confirmed left=2/right=4 bench")
    profiles = []
    for sensor in sensors:
        name = sensor["profile"]
        if not re.fullmatch(r"[a-zA-Z0-9_]+", name):
            raise ValueError("Invalid force sensor profile name")
        profile = yaml.safe_load((hardware_share / f"config/slaves/{name}.yaml").read_text())
        if not profile.get("use_slave_pdo_defaults") or profile.get("sdo"):
            raise ValueError("Force bench requires preserved PDOs and no startup SDO writes")
        for pdo in profile["rpdo"]:
            for channel in pdo["channels"]:
                if channel.get("default") != 0 or "command_interface" in channel:
                    raise ValueError("Force bench outputs must remain fixed zero")
        profiles.append(profile)
    if not use_mock_hardware:
        if sync_mode not in ("sm", "dc"):
            raise ValueError("Real force bench requires explicit sync_mode:=sm or dc")
        check_bench(sensors, profiles, run)

    runtime_dir.mkdir(parents=True, exist_ok=True)
    robot = ET.Element("robot", name="bluepoint_raw_bench")
    ET.SubElement(robot, "link", name="force_bench_base")
    system = ET.SubElement(robot, "ros2_control", name="bluepoint_raw_bench", type="system")
    hardware = ET.SubElement(system, "hardware")
    ET.SubElement(hardware, "plugin").text = (
        "mock_components/GenericSystem" if use_mock_hardware else "ethercat_driver/EthercatDriver")
    for name, value in {"master_id": 0, "control_frequency": 1000,
                        "startup_bus_timeout_ms": 70000, "preload_timeout_ms": 5000}.items():
        _param(hardware, name, value)
    if use_mock_hardware:
        _param(hardware, "calculate_dynamics", "false")
        _param(hardware, "mock_sensor_commands", "false")

    def state_sensor(name, values):
        element = ET.SubElement(system, "sensor", name=name)
        for interface, initial in values.items():
            field = ET.SubElement(element, "state_interface", name=interface)
            if use_mock_hardware:
                _param(field, "initial_value", initial)
        return element

    state_sensor("ethercat_master", {"link_up": 1, "slaves_responding": 6, "wc_error_count": 0})
    state_sensor("ethercat_domain", {"process_data_age_ms": 0})
    controllers = {"controller_manager": {"ros__parameters": {"update_rate": 1000}}}
    startup_id = "force-bench-" + str(uuid.uuid4())
    names = []
    for sensor, profile in zip(sensors, profiles):
        position = sensor["ring_position"]
        state_sensor(f"ethercat_slave_{position}", {"al_state": 8})
        values = {c["state_interface"]: 0 for p in profile["tpdo"] for c in p["channels"]}
        component = state_sensor(sensor["sensor_name"], values)
        if not use_mock_hardware:
            profile["assign_activate"] = 0x0300 if sync_mode == "dc" else 0
            profile_path = runtime_dir / f"{sensor['profile']}.yaml"
            profile_path.write_text(yaml.safe_dump(profile, sort_keys=False))
            for key, value in {"name": sensor["sensor_name"] + "_device",
                               "plugin": "ethercat_generic_plugins/GenericEcSlave", "alias": 0,
                               "position": position, "slave_config": profile_path}.items():
                _param(component, "ec_module." + key, value)
        name = sensor["controller_name"]
        names.append(name)
        controllers["controller_manager"]["ros__parameters"][name] = {"type": document["plugin"]}
        params = {key: sensor[key] for key in (
            "sensor_name", "value_interfaces", "auxiliary_interfaces", "scale_factors",
            "decimals", "unit_codes", "validity_policy", "snapshot_source", "wrench_topic",
            "raw_topic", "sensor_state_topic", "diagnostic_name")}
        params.update(calibration_valid=False, startup_id=startup_id,
                      calibration_topic=document["calibration_topic"],
                      # Local label for raw-only operation; no physical TF is asserted.
                      frame_id=sensor["sensor_name"] + "_raw",
                      link_interface="ethercat_master/link_up",
                      al_state_interface=f"ethercat_slave_{position}/al_state")
        controllers[name] = {"ros__parameters": params}
    return ForceSensorRuntime(ET.tostring(robot, encoding="unicode"), controllers, tuple(names))
