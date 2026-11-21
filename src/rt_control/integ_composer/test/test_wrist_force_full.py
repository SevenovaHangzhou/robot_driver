"""Advantech wrist force sensors on the arms' master (ELECTRI-238): raw values in the Robot Model
frames, saved-PDO and identity checks in the shared CiA402 topology, no second master health."""

from pathlib import Path
import sys

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


def request(tmp_path, mock, peers):
    manifest = read_with_parts(DATA / "machines/Alfa_V3/manifest.yaml")
    descriptor = manifest["modules"]["wrist_force_sensors"]
    machine = load_yaml(DATA / descriptor["config_file"])
    return {
        "module": descriptor,
        "machine": machine,
        "devices": {},
        "unit": None,
        "deployment": load_yaml(DATA / "units/Alfa_V3-Advantech-01/deployment.yaml"),
        "robot_model": {"joint_limits": {"joints": {}}, "joints": []},
        "use_mock_hardware": mock,
        "bus_cycle_hz": 1000,
        "runtime_dir": str(tmp_path),
        "render_mode": "normal",
        "hardware_system_id": "ecat_arms",
        "transport_layout": manifest["layout"]["ethercat"],
        "peer_machines": [peer(machine) for peer in peers],
    }


@pytest.fixture
def render(monkeypatch, tmp_path):
    monkeypatch.setattr(OWNER, "_share", lambda: RT / "hw_ethercat")

    def run(mock, peers):
        return render_checked(OWNER, request(tmp_path, mock, peers), PACKAGE / "schema", DATA)

    return run


def drives(_):
    # Another module on the same master: the lift drive (one axis).
    return load_yaml(DATA / "machines/Alfa_V3/updown.yaml")


def itself(machine):
    return machine


def test_full_machine_sensors_join_the_cia402_topology(render):
    result = render(mock=False, peers=[drives, itself])
    names = [sensor.attrib["name"] for sensor in result["sensors"]]
    # The first peer (the drives) renders the master health; the sensors add their AL states.
    assert "ethercat_master" not in names and "ethercat_domain" not in names
    assert {"ethercat_slave_18", "ethercat_slave_26"} <= set(names)
    topology = result["expected_topology"]
    assert topology["kind"] == "cia402_topology"
    assert [item["ring_position"] for item in topology["responders"]] == [18, 26]
    assert topology["responders"][0]["vendor_id"] == 161
    assert [item["ring_position"] for item in topology["pdo_checks"]] == [18, 26]
    assert topology["preflight_required"] is True
    assert not any(entry.get("bus") for entry in result["diagnostic_devices"])
    left = result["controller_parameters"]["left_wrist_force_sensor_broadcaster"]
    right = result["controller_parameters"]["right_wrist_force_sensor_broadcaster"]
    assert (left["frame_id"], right["frame_id"]) == ("left_ft_sensor_link", "right_ft_sensor_link")
    assert left["calibration_valid"] is False
    config = yaml.safe_load(result["runtime_files"]["bluepoint_p140000100_left.yaml"])
    assert config["assign_activate"] == 0x0300


def test_the_first_peer_counts_sensors_in_the_master_health(render):
    result = render(mock=True, peers=[itself, drives])
    master = next(
        sensor for sensor in result["sensors"] if sensor.attrib["name"] == "ethercat_master"
    )
    slaves = next(
        item
        for item in master.findall("state_interface")
        if item.attrib["name"] == "slaves_responding"
    )
    # Two sensors and the one lift drive.
    assert slaves.find("param").text == "3"
