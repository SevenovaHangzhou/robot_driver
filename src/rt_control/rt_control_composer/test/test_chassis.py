"""Chassis composition and synthetic sample B; no CAN or EtherCAT devices."""

from copy import deepcopy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

from rt_control_composer.compose import compose
from rt_control_composer.renderer_registry import render_checked
from rt_control_composer.schema.cross_checks import merge_systems
from rt_control_composer.schema.primitives import load_yaml

PACKAGE = Path(__file__).resolve().parents[1]
RT = PACKAGE.parent
for owner in ("robot_hw_ethercat", "robot_hw_canopen"):
    sys.path.insert(0, str(RT / owner))
from robot_hw_ethercat import rt_control_renderer as ETHERCAT
from robot_hw_ethercat.kinco_renderer import conversion_factors
from robot_hw_canopen import rt_control_renderer as CANOPEN


@pytest.fixture
def shares(monkeypatch):
    monkeypatch.setattr(ETHERCAT, "_share", lambda: RT / "robot_hw_ethercat")
    monkeypatch.setattr(CANOPEN, "_share", lambda: RT / "robot_hw_canopen")
    lookup = {path.name: path for path in RT.iterdir() if path.is_dir()}
    lookup["robot_description"] = RT.parent / "description/robot_description"
    return lookup


def arguments(shares):
    config = shares["rt_control_machine_config"]
    path = next((config / "units").glob("*/deployment.yaml"))
    return dict(
        robot_variant=load_yaml(path)["robot_variant"],
        machine_unit=path.parent.name,
        physical_profile="chassis_only",
        control_scope="chassis_only",
        package_share=lambda name: str(shares[name]),
    )


def test_chassis_mock_has_eight_axes_and_four_read_only_encoders(shares, tmp_path):
    result = compose(**arguments(shares), runtime_dir=tmp_path / "out")
    systems = ET.fromstring(result.robot_description).findall("ros2_control")
    assert len(systems) == 2
    assert [len(system.findall("joint")) for system in systems] == [8, 0]
    assert len(systems[1].findall("sensor")) == 4
    assert not systems[1].findall(".//command_interface")
    bus = yaml.safe_load((result.runtime_dir / "canopen_bus.yaml").read_text())
    manifest = load_yaml(
        next((shares["rt_control_machine_config"] / "machines").glob("*/manifest.yaml"))
    )
    assert manifest["layout"]["ethercat"]["cycle_hz"] == 1000
    assert bus["master"]["sync_period"] == manifest["layout"]["canopen"]["sync_period_us"]
    assert (
        result.controllers["controller_manager"]["ros__parameters"]["update_rate"]
        == manifest["layout"]["ethercat"]["cycle_hz"]
    )
    binding = load_yaml(
        next((shares["rt_control_machine_config"] / "machines").glob("*/controllers.yaml"))
    )["controllers"]["swerve_chassis"]
    assert binding["required_state_modules"] == ["swerve_encoders"]
    assert result.controller_order["inactive"] == [binding["name"]]
    assert all(
        "control_word" in [child.attrib["name"] for child in joint.findall("command_interface")]
        for joint in systems[0].findall("joint")
    )


def test_real_chassis_lists_missing_facts_before_any_output(shares, tmp_path):
    target = tmp_path / "real"
    with pytest.raises(ValueError, match="Missing real chassis facts") as error:
        compose(**arguments(shares), runtime_dir=target, use_mock_hardware=False)
    for required in (
        "motor_encoder_counts_per_revolution",
        "identity",
        "ring_position",
        "steering_zero_counts",
        "verified",
    ):
        assert required in str(error.value)
    assert not target.exists()


def test_kinco_position_and_velocity_roundtrips():
    dec = {"numerator": 512, "denominator": 1875}
    for counts in (4096, 65536):
        for ratio in (27.48, 140.0):
            for sign in (-1, 1):
                position, velocity = conversion_factors(counts, ratio, sign, dec)
                assert ((0.75 * position + 123) - 123) / position == pytest.approx(0.75)
                assert (-0.4 * velocity) / velocity == pytest.approx(-0.4)


def request_for_module(shares, name):
    config = shares["rt_control_machine_config"]
    manifest = load_yaml(next((config / "machines").glob("*/manifest.yaml")))
    entry = manifest["modules"][name]
    deployment = load_yaml(next((config / "units").glob("*/deployment.yaml")))
    unit_dir = next((config / "units").glob("*/deployment.yaml")).parent
    return {
        "module": entry,
        "machine": load_yaml(config / entry["config_file"]),
        "devices": {reference: load_yaml(config / reference) for reference in entry["device_refs"]},
        "unit": load_yaml(unit_dir / entry.get("unit_file", f"{name}_zero.yaml")),
        "deployment": deployment,
        "robot_model": {"joint_limits": {}},
        "use_mock_hardware": True,
        "bus_cycle_hz": manifest["layout"]["ethercat"]["cycle_hz"],
        "runtime_dir": "/synthetic/output",
        "render_mode": "normal",
        "transport_layout": manifest["layout"][entry["transport"]],
    }


def test_sample_b_owner_adapter_and_read_only_contract(shares, monkeypatch):
    request = request_for_module(shares, "swerve_chassis")
    sample = load_yaml(PACKAGE / "test/fixtures/sample_b/machines/SyntheticB/mobile_fixture.yaml")
    template = request["machine"]["spec"]["axes"][0]
    request["machine"]["module"] = sample["module"]
    request["machine"]["spec"]["axes"] = [
        {
            **template,
            "role": axis["role"],
            "module_position": axis["physical_joint"].split("_")[-1],
            "robot_model_joint": axis["robot_model_joint"],
            "ring_position": axis["ring_position"],
            "direction": axis["direction"],
            "mode_of_operation": axis["mode_of_operation"],
            "interface_contract": "swerve_steering_csp"
            if axis["role"] == "steering"
            else "swerve_drive_csv",
        }
        for axis in sample["spec"]["axes"]
    ]
    request["module"]["actuator_count"] = len(sample["spec"]["axes"])

    def denied(*args, **kwargs):
        pytest.fail("Renderer performed a write or device command")

    monkeypatch.setattr(Path, "write_text", denied)
    result = render_checked(
        ETHERCAT, request, PACKAGE / "schema", shares["rt_control_machine_config"]
    )
    assert len(result["joints"]) == len(sample["spec"]["axes"])
    assert result["managed_joints"] == [
        axis["robot_model_joint"] for axis in sample["spec"]["axes"]
    ]
    encoders = request_for_module(shares, "swerve_encoders")
    sensor_result = render_checked(
        CANOPEN, encoders, PACKAGE / "schema", shares["rt_control_machine_config"]
    )
    assert not sensor_result["managed_joints"] and not sensor_result["joints"]
    encoders["use_mock_hardware"] = False
    with pytest.raises(ValueError, match="Missing real encoder facts"):
        CANOPEN.render(encoders)


def test_x9_merges_two_modules_and_rejects_parameter_conflict(shares):
    arm_request = request_for_module(shares, "arms")
    chassis_request = request_for_module(shares, "swerve_chassis")
    for position, axis in enumerate(chassis_request["machine"]["spec"]["axes"], start=18):
        axis["ring_position"] = position
    peers = [arm_request["machine"], chassis_request["machine"]]
    arm_request["peer_machines"] = chassis_request["peer_machines"] = peers
    combined = merge_systems(
        [
            ("synthetic_owner", ETHERCAT.render(arm_request)),
            ("synthetic_owner", ETHERCAT.render(chassis_request)),
        ]
    )
    assert len(combined) == 1 and len(combined[0]["result"]["joints"]) == 24
    assert len(combined[0]["result"]["expected_topology"]["responders"]) == 26
    health = [
        element
        for element in combined[0]["result"]["sensors"]
        if element.attrib["name"] == "ethercat_master"
    ]
    assert len(health) == 1
    assert health[0].find("state_interface[@name='slaves_responding']/param").text == "26"
    request = request_for_module(shares, "swerve_chassis")
    left, right = deepcopy(request), deepcopy(request)
    for current, suffix, positions in (
        (left, "first", range(1, 5)),
        (right, "second", range(5, 9)),
    ):
        current["machine"]["module"] = suffix
        current["module"]["actuator_count"] = 4
        current["machine"]["spec"]["axes"] = current["machine"]["spec"]["axes"][:4]
        for axis, position in zip(current["machine"]["spec"]["axes"], positions):
            axis["ring_position"] = position
            axis["mock_joint_name"] += "_" + suffix
    peers = [left["machine"], right["machine"]]
    left["peer_machines"] = right["peer_machines"] = peers
    a, b = ETHERCAT.render(left), ETHERCAT.render(right)
    merged = merge_systems([("synthetic_owner", a), ("synthetic_owner", b)])
    assert len(merged) == 1
    assert len(merged[0]["result"]["joints"]) == 8
    assert len(merged[0]["result"]["expected_topology"]["responders"]) == 8
    names = [element.attrib["name"] for element in merged[0]["result"]["sensors"]]
    assert len(names) == len(set(names))
    conflict = deepcopy(b)
    conflict["hardware_params"]["control_frequency"] += 1
    with pytest.raises(ValueError, match="Conflicting"):
        merge_systems([("synthetic_owner", a), ("synthetic_owner", conflict)])
