"""Passive force fragments and the old read-only bench checks."""

from copy import deepcopy
import math
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

from infra_renderer import xml_param
import yaml

from .profiles import master_parameters


def render(request, share, validate_module, validate_unit, validate_device):
    machine = validate_module(request["module"], request["machine"])
    spec = machine["spec"]
    mock = request["use_mock_hardware"]
    bench = request["hardware_system_id"] == "bluepoint_raw_bench"
    if not bench:
        if spec["full_ring_positions"] == "TBD":
            raise ValueError("Full-machine force sensor ring positions are TBD")
        if len(spec["full_ring_positions"]) != len(spec["sensors"]):
            raise ValueError("Full-machine force topology must cover every sensor")
        for item, position in zip(spec["sensors"], spec["full_ring_positions"]):
            item["ring_position"] = position
    if not mock and request.get("sync_mode", "") not in ("sm", "dc"):
        raise ValueError("Real force bench requires explicit sync_mode sm or dc")
    sensors, files, parameters, profiles = [], {}, {}, []
    contexts = {}
    count = len(spec["sensors"]) + len(spec["bench_extra_responders"])

    def state_sensor(name, values):
        sensor = ET.Element("sensor", name=name)
        for name, value in values.items():
            interface = ET.SubElement(sensor, "state_interface", name=name)
            if mock:
                xml_param(interface, "initial_value", value)
        sensors.append(sensor)
        return sensor

    state_sensor("ethercat_master", {"link_up": 1, "slaves_responding": count, "wc_error_count": 0})
    state_sensor("ethercat_domain", {"process_data_age_ms": 0})
    for item in spec["sensors"]:
        record = yaml.safe_load(
            (share / "config/profiles" / (item["profile"] + ".yaml")).read_text()
        )
        profile = deepcopy(record["driver_config"])
        profile["metadata"].update(
            side=item["side"],
            observed_ring_position=item["ring_position"],
            ring_positions=[item["ring_position"]],
        )
        if not profile.get("use_slave_pdo_defaults") or profile.get("sdo"):
            raise ValueError("Force bench requires preserved PDOs and no startup SDO writes")
        if any(
            channel.get("default") != 0 or "command_interface" in channel
            for pdo in profile["rpdo"]
            for channel in pdo["channels"]
        ):
            raise ValueError("Force bench outputs must remain fixed zero")
        scale = []
        for name, size in (("force_raw_per_newton", 3), ("torque_raw_per_newton_metre", 3)):
            raw = record[name]
            if type(raw) not in (int, float) or not math.isfinite(raw) or raw <= 0:
                raise ValueError("Force scale must be positive")
            scale += [1 / raw] * size
        position = item["ring_position"]
        if not mock and position == "TBD":
            raise ValueError("Required force sensor ring_position is TBD")
        state_sensor(f"ethercat_slave_{position}", {"al_state": 8})
        values = {
            channel["state_interface"]: 0 for pdo in profile["tpdo"] for channel in pdo["channels"]
        }
        component = state_sensor(item["sensor_name"], values)
        if not mock:
            profile["assign_activate"] = 0x0300 if request["sync_mode"] == "dc" else 0
            # Exact legacy filenames are declaration data, not composer special cases.
            relative = item["profile"] + "_" + item["side"] + ".yaml"
            files[relative] = yaml.safe_dump(profile, sort_keys=False)
            for key, value in {
                "name": item["sensor_name"] + "_device",
                "plugin": "ethercat_generic_plugins/GenericEcSlave",
                "alias": 0,
                "position": position,
                "slave_config": str(Path(request["runtime_dir"]) / relative),
            }.items():
                xml_param(component, "ec_module." + key, value)
        controller_name = item["controller_name"]
        if controller_name in parameters:
            raise ValueError("Duplicate force sensor controller binding")
        contexts[controller_name] = {
            "binding": item["binding"], "sensor_name": item["sensor_name"],
            "family": spec["family"],
        }
        parameters[controller_name] = {
            "sensor_name": item["sensor_name"],
            "value_interfaces": [
                item["sensor_name"] + "/" + name
                for name in record["broadcaster"]["value_interface_names"]
            ],
            "auxiliary_interfaces": [
                item["sensor_name"] + "/" + name
                for name in record["broadcaster"]["auxiliary_interface_names"]
            ],
            "scale_factors": scale,
            "decimals": record["decimals"],
            "unit_codes": record["unit_codes"],
            "validity_policy": record["broadcaster"]["validity_policy"],
            "snapshot_source": record["broadcaster"]["snapshot_source"],
            "calibration_valid": False,
            "frame_id": item["sensor_name"] + "_raw",
            "link_interface": "ethercat_master/link_up",
            "al_state_interface": f"ethercat_slave_{position}/al_state",
        }
        profiles.append(
            {
                "ring_position": position,
                "vendor_id": profile["vendor_id"],
                "product_id": profile["product_id"],
                "revision": profile["metadata"]["revision_id"],
                "pdo_entries": [
                    [channel["index"], channel["sub_index"], 32]
                    for direction in ("rpdo", "tpdo")
                    for pdo in profile[direction]
                    for channel in pdo["channels"]
                ],
            }
        )
    params = {
        "master_id": request["deployment"]["ethercat"]["master_id"],
        "control_frequency": request["bus_cycle_hz"],
        **master_parameters(share),
    }
    if mock:
        params.update(calculate_dynamics="false", mock_sensor_commands="false")
    return {
        "hardware_system_id": machine["master_system"],
        "hardware_plugin": "mock_components/GenericSystem"
        if mock
        else "ethercat_driver/EthercatDriver",
        "hardware_params": params,
        "joints": [],
        "sensors": sensors,
        "managed_joints": [],
        "actuator_groups": {},
        "runtime_files": files,
        "controller_parameters": parameters,
        "controller_context": contexts,
        "expected_topology": {
            "kind": "passive_force_bench",
            "master_id": params["master_id"],
            "responder_count": count,
            "sensors": profiles,
            "preflight_required": True,
        },
    }


def preflight(topology, *, run=None):
    """Read-only bench checks; one finding per problem, a master problem ends the scan."""

    def read(argv):
        return subprocess.run(argv, capture_output=True, text=True, timeout=5, check=True).stdout

    execute = read if run is None else run
    master_id = str(topology["master_id"])
    bus = {"master_id": topology["master_id"]}
    master = execute(["ethercat", "master", "-m", master_id])
    for pattern in (
        r"Phase:\s*Idle\b",
        r"Active:\s*no\b",
        r"Link:\s*UP\b",
        rf"Slaves:\s*{topology['responder_count']}\b",
    ):
        if not re.search(pattern, master):
            return [
                {
                    "address": bus,
                    "problem": "offline",
                    "detail": "Force bench requires idle master, link UP and declared responders",
                    "blocking": True,
                }
            ]
    findings = []
    for sensor in topology["sensors"]:
        position = str(sensor["ring_position"])
        address = {**bus, "ring_position": sensor["ring_position"]}

        def problem(kind, detail):
            findings.append(
                {"address": address, "problem": kind, "detail": detail, "blocking": True}
            )

        details = execute(["ethercat", "slaves", "-m", master_id, "-p", position, "-v"])
        identity = True
        for label, key in (
            ("Vendor Id", "vendor_id"),
            ("Product code", "product_id"),
            ("Revision number", "revision"),
        ):
            match = re.search(re.escape(label) + r":\s*(0x[0-9a-fA-F]+)", details)
            if match is None or int(match[1], 16) != sensor[key]:
                identity = False
        if not identity:
            problem("identity_mismatch", "Force sensor identity mismatch")
            continue
        if not re.search(r"State:\s*PREOP\b", details) or not re.search(r"Flag:\s*\+", details):
            problem("offline", "Force sensor must be error-free PREOP")
            continue
        pdos = execute(["ethercat", "pdos", "-m", master_id, "-p", position])
        entries = [
            [int(i, 16), int(s, 16), int(bits)]
            for i, s, bits in re.findall(
                r"PDO entry (0x[0-9a-fA-F]+):([0-9a-fA-F]+),\s*(\d+) bit", pdos
            )
        ]
        if entries != sensor["pdo_entries"]:
            problem("configuration_mismatch", "Force sensor PDO layout mismatch")
        for label, index in (("RxPDO", 0x1600), ("TxPDO", 0x1A00)):
            if not re.search(label + rf"\s+0x{index:04x}\b", pdos, re.IGNORECASE):
                problem("configuration_mismatch", "Force sensor missing PDO")
    return findings
