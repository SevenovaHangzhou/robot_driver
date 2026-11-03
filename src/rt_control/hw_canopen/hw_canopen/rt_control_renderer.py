"""CiA406 read-only sensor renderer; no bus probes and no composer dependency."""

from collections.abc import Mapping
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from infra_renderer import validated_spec, xml_param
import yaml

RENDERER_API_VERSION = 1


def _share():
    return Path(get_package_share_directory("hw_canopen"))


def validate_module(module: Mapping, data: Mapping) -> Mapping:
    return validated_spec(data, _share() / "schema/encoder_module.schema.json")


def validate_unit(module: Mapping, data: Mapping | None) -> Mapping:
    if data is None:
        raise ValueError("External encoder calibration is required")
    return validated_spec(data, _share() / "schema/encoder_unit.schema.json")


def validate_device(data: Mapping, *, render_mode: str) -> Mapping:
    missing = [
        key
        for key in ("counts_per_revolution", "distinguishable_revolutions")
        if key not in data or data[key] == "TBD"
    ]
    if missing:
        raise ValueError("Required encoder facts: " + ", ".join(missing))
    return deepcopy(dict(data))


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
    canopen = deployment["canopen"]
    # The generated bundle (EDS copies, master DCF/bin) lives in a deployment directory.
    bundle_dir = request["runtime_dir"] if mock else canopen.get("bundle_dir", "TBD")
    bus = {
        "options": {"dcf_path": bundle_dir},
        "master": {
            "node_id": canopen["master_node_id"],
            "baudrate": canopen.get("bitrate_kbps", "TBD"),
            "sync_period": request["transport_layout"]["sync_period_us"],
            **deepcopy(profile["master_settings"]),
            "driver": profile["master_driver"]["class"],
            "package": profile["master_driver"]["package"],
        },
    }
    sources = {}
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
                xml_param(element, "initial_value", 0)
        if not mock:
            for key, value in values.items():
                xml_param(sensor, key, value)
        sensors.append(sensor)
        if type(item["node_id"]) is int:
            claims.append(f"node:{item['node_id']}")
        node = deepcopy(profile["node_settings"])
        if type(item["node_id"]) is int:
            node["tpdo"][2]["cob_id"] = profile["tpdo2_disabled_cob_id_base"] + item["node_id"]
        # ros2_canopen joins dcf_path and dcf: the runtime name must be relative.
        bus[name] = {
            **node,
            "node_id": item["node_id"],
            "driver": profile["node_driver"]["class"],
            "package": profile["node_driver"]["package"],
            "dcf": f"{name}.eds",
        }
        sources[name] = canopen["eds_files"][name]
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
            for key in ("master_node_id", "bitrate_kbps", "bundle_dir")
            if canopen.get(key, "TBD") == "TBD"
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
    bus_text = yaml.safe_dump(bus, sort_keys=False)
    if not mock:
        bundle = Path(bundle_dir)
        params.update(
            bus_config=str(bundle / "bus.yml"),
            master_config=str(bundle / "master.dcf"),
            master_bin=str(bundle / "master.bin"),
            can_interface_name=deployment["can_interfaces"][machine["module"]],
            ready_feedback_age_ms=profile["ready_feedback_age_ms"],
        )
        # EDS sources are resolved at composition time: "<package>:<share-relative path>".
        sources = {name: _resolve_eds(value) for name, value in sources.items()}
    interface = None if mock else deployment["can_interfaces"][machine["module"]]
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
        "runtime_files": {
            "canopen_bus.yaml": bus_text,
            "canopen_sources.yaml": yaml.safe_dump(sources, sort_keys=True),
        },
        "expected_topology": {
            "node_ids": [item["node_id"] for item in spec["nodes"]],
            "can_interface": interface,
            "can_usb_serial": None
            if mock
            else deployment.get("host", {}).get("can_usb_serials", {}).get(interface),
            "bundle_dir": None if mock else bundle_dir,
            "bus_sha256": hashlib.sha256(bus_text.encode("utf-8")).hexdigest(),
            "preflight_required": not mock,
        },
    }


def _resolve_eds(reference):
    package, _, relative = reference.partition(":")
    if not package or not relative or relative.startswith("/") or ".." in relative.split("/"):
        raise ValueError(f"EDS reference must be <package>:<share-relative path>: {reference}")
    share = _share() if package == "hw_canopen" else Path(get_package_share_directory(package))
    path = share / relative
    if not path.is_file():
        raise ValueError(f"EDS file not installed: {reference}")
    return str(path)


def preflight(expected_topology: Mapping, *, run=None) -> list:
    """Read-only: interface up/error-active and identity, bundle generated from this source."""
    if not expected_topology.get("preflight_required"):
        return []
    interface = expected_topology["can_interface"]
    address = {"can_interface": interface}

    def finding(problem, detail):
        return [{"address": address, "problem": problem, "detail": detail, "blocking": True}]

    bundle = Path(expected_topology["bundle_dir"])
    for name in ("bus.yml", "master.dcf", "master.bin", "source.sha256"):
        if not (bundle / name).is_file():
            return finding(
                "configuration_mismatch",
                f"CANopen bundle incomplete: {bundle / name}; run generate_canopen_bundle",
            )
    if (bundle / "source.sha256").read_text().strip() != expected_topology["bus_sha256"]:
        return finding(
            "configuration_mismatch", f"CANopen bundle {bundle} is stale; run generate_canopen_bundle"
        )
    execute = subprocess.check_output if run is None else run
    link = json.loads(
        execute(["ip", "-details", "-json", "link", "show", interface], text=True, timeout=5)
    )[0]
    if "UP" not in link.get("flags", []):
        return finding("offline", f"CAN interface {interface} is not up")
    state = link.get("linkinfo", {}).get("info_data", {}).get("state")
    if state != "ERROR-ACTIVE":
        return finding("offline", f"CAN interface {interface} is not error-active: {state}")
    serial = expected_topology.get("can_usb_serial")
    if serial:
        device = Path("/sys/class/net", interface, "device").resolve()
        found = [
            (path / "serial").read_text().strip()
            for path in [device, *device.parents]
            if (path / "serial").is_file()
        ]
        if serial not in found:
            return finding(
                "identity_mismatch", f"CAN interface {interface} is not the declared USB adapter"
            )
    return []
