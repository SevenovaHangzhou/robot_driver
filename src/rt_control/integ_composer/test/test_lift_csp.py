"""Advantech lift in CSP (ELECTRI-242): the Tongyi drive on the upstream CiA402 plugin over its fixed
PDO, rendered alone through the protocol owner; the full-machine scope uses the same fragment."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

from integ_composer.renderer_registry import render_checked
from integ_composer.schema.manifest_load import read_with_parts
from integ_composer.schema.primitives import load_yaml

PACKAGE = Path(__file__).resolve().parents[1]
RT = PACKAGE.parent
DATA = RT / "hwcfg_machines"
sys.path.insert(0, str(RT / "hw_ethercat"))
from hw_ethercat import rt_control_renderer as OWNER

COUNTS_PER_METRE = 65535 / 0.005
ZERO = 8887150


def request(tmp_path, mock):
    manifest = read_with_parts(DATA / "machines/Alfa_V3/manifest.yaml")
    descriptor = manifest["modules"]["updown"]
    machine = load_yaml(DATA / descriptor["config_file"])
    return {
        "module": descriptor,
        "machine": machine,
        "devices": {path: load_yaml(DATA / path) for path in descriptor["device_refs"]},
        "unit": load_yaml(DATA / "units/Alfa_V3-Advantech-01/updown_zero.yaml"),
        "deployment": load_yaml(DATA / "units/Alfa_V3-Advantech-01/deployment.yaml"),
        "robot_model": {"joint_limits": {"joints": {}}, "joints": ["updown"]},
        "use_mock_hardware": mock,
        "bus_cycle_hz": 1000,
        "runtime_dir": str(tmp_path),
        "render_mode": "normal",
        "hardware_system_id": "ecat_arms",
        "transport_layout": manifest["layout"]["ethercat"],
        "peer_machines": [machine],
    }


@pytest.fixture
def render(monkeypatch, tmp_path):
    monkeypatch.setattr(OWNER, "_share", lambda: RT / "hw_ethercat")

    def run(mock):
        schemas = PACKAGE / "schema"
        return render_checked(OWNER, request(tmp_path, mock), schemas, DATA)

    return run


def interfaces(joint, kind):
    return [element.attrib["name"] for element in joint.findall(kind)]


def test_real_lift_runs_the_cia402_drive_in_csp(render):
    result = render(mock=False)
    (joint,) = result["joints"]
    assert joint.attrib["name"] == "updown"
    assert interfaces(joint, "command_interface") == ["position", "control_word"]
    assert interfaces(joint, "state_interface") == [
        "position",
        "status_word",
        "error_code",
        "mode_of_operation_display",
    ]
    params = {element.attrib["name"]: element.text for element in joint.findall("param")}
    assert params["ec_module.plugin"] == "ethercat_generic_plugins/EcCiA402Drive"
    assert params["ec_module.position"] == "1"
    assert params["ec_module.mode_of_operation"] == "8"
    assert result["actuator_groups"] == {"csp": ["updown"]}
    assert result["parameter_exports"]["motion_command_interfaces"] == ["position"]

    config = yaml.safe_load(result["runtime_files"]["updown.yaml"])
    assert config["use_slave_pdo_defaults"] is True
    assert config["auto_fault_reset"] is False and config["auto_state_transitions"] is False
    rpdo = {channel["index"]: channel for channel in config["rpdo"][0]["channels"]}
    tpdo = {channel["index"]: channel for channel in config["tpdo"][0]["channels"]}
    # Joint metres <-> raw counts with the zero at joint 0 (the top); +q up is motor positive.
    assert rpdo[0x607A]["command_interface"] == "position"
    assert rpdo[0x607A]["factor"] == pytest.approx(COUNTS_PER_METRE)
    assert rpdo[0x607A]["offset"] == ZERO
    assert tpdo[0x6064]["state_interface"] == "position"
    assert tpdo[0x6064]["factor"] * (-12503) + tpdo[0x6064]["offset"] == pytest.approx(-0.679)
    assert rpdo[0x6040]["command_interface"] == "control_word"
    # The drive's own speed limit is the module maximum, 0.1666 m/s = 19992 x 0.1 r/min.
    assert rpdo[0x607F]["default"] == 19992
    assert all(rpdo[index]["default"] == 0 for index in (0x6071, 0x60FF, 0x6081))
    assert tpdo[0x6061]["state_interface"] == "mode_of_operation_display"

    topology = result["expected_topology"]
    assert [item["ring_position"] for item in topology["responders"]] == [1]
    # A start outside the hard stops (2 cm beyond -1.033..+0.017 m) means a wrong zero.
    (window,) = topology["position_windows"]
    assert window["raw_min"] == pytest.approx(ZERO - 1.053 * COUNTS_PER_METRE)
    assert window["raw_max"] == pytest.approx(ZERO + 0.037 * COUNTS_PER_METRE)
    assert -12503 > window["raw_min"] and -12503 < window["raw_max"]


def test_mock_lift_reports_the_csp_startup_mode(render):
    result = render(mock=True)
    (joint,) = result["joints"]
    initial = {
        element.attrib["name"]: element.find("param").text
        for element in joint.findall("state_interface")
    }
    assert initial["mode_of_operation_display"] == "8"
    assert initial["status_word"] == "64"
    ET.tostring(joint)
