"""Advantech full machine (ELECTRI-239): one controller_manager with the chassis, the lift, both
arms, the wrist force sensors, the steering encoders and the head; the arms and chassis groups
switch modes independently."""

from pathlib import Path
import sys

import pytest

from integ_composer.compose import compose

PACKAGE = Path(__file__).resolve().parents[1]
RT = PACKAGE.parent
for owner in ("hw_ethercat", "hw_canopen", "hw_can_damiao"):
    sys.path.insert(0, str(RT / owner))
from hw_can_damiao import rt_control_renderer as HEAD
from hw_canopen import rt_control_renderer as CANOPEN
from hw_ethercat import rt_control_renderer as ETHERCAT

ARMS = [f"{side}_joint{index}" for side in ("right", "left") for index in range(1, 8)]
CHASSIS = [joint for n in range(1, 5) for joint in (f"wheel{n}_joint", f"steer{n}_joint")]


@pytest.fixture
def shares(monkeypatch):
    monkeypatch.setattr(ETHERCAT, "_share", lambda: RT / "hw_ethercat")
    monkeypatch.setattr(CANOPEN, "_share", lambda: RT / "hw_canopen")
    monkeypatch.setattr(HEAD, "_share", lambda: RT / "hw_can_damiao")
    lookup = {path.name: path for path in RT.iterdir() if path.is_dir()}
    lookup["robot_description"] = RT.parent / "description/robot_description"
    return lookup


def full(shares, tmp_path, mock):
    return compose(
        robot_variant="Alfa_V3",
        machine_unit="Alfa_V3-Advantech-01",
        physical_profile="advantech_full",
        control_scope="advantech_full",
        package_share=lambda name: str(shares[name]),
        runtime_dir=tmp_path / ("mock" if mock else "real"),
        use_mock_hardware=mock,
    )


@pytest.mark.parametrize("mock", [True, False])
def test_two_motion_groups_and_one_enable_state(shares, tmp_path, mock):
    result = full(shares, tmp_path, mock)
    manager = result.controllers["enable_manager"]["ros__parameters"]
    assert manager["motion_groups"] == ["arms", "base"]
    arms, base = manager["arms"], manager["base"]
    assert (arms["motion_controller"], arms["rolling_controller"]) == ("arms_jtc", "arms_rolling")
    assert arms["motion_joints"] == ARMS + ["updown"]
    assert arms["mode_service"] == "/rt/arms/set_control_mode"
    assert (base["motion_controller"], base["trajectory_controller"]) == (
        "swerve_controller",
        "base_wheel_jtc",
    )
    assert base["mode_service"] == "/rt/base/set_control_mode"
    assert base["mode_switch_joints"] == [f"wheel{n}_joint" for n in range(1, 5)]
    assert manager["companion_controllers"] == ["head_position_controller"]
    assert manager["handshake_devices"] == ["damiao_head_control"]
    # Chassis (two per batch), the lift, then the arms joint by joint; the head last.
    assert manager["enable_batch_sizes"] == [2, 2, 2, 2, 1] + [1] * 14
    # The arm drives have no mode display; the chassis and the lift confirm their startup mode.
    modes = dict(zip(manager["managed_joints"], manager["expected_modes"]))
    assert {joint: modes[joint] for joint in ARMS} == dict.fromkeys(ARMS, 0)
    assert modes["updown"] == 8
    assert {modes[f"wheel{n}_joint"] for n in range(1, 5)} == {9}
    assert {modes[f"steer{n}_joint"] for n in range(1, 5)} == {8}
    assert manager["emergency_stop_response"] is False
    # The Tongyi lift is stopped with Disable Voltage (0x0005), never Quick Stop (vendor 2026-11-22).
    assert manager["emergency_disable_voltage_joints"] == ["updown"]
    assert result.lifecycle["mode_controller_groups"] == [
        ["arms_jtc", "arms_rolling"],
        ["swerve_controller", "base_wheel_jtc"],
    ]


def test_trajectory_rolling_and_joint_states(shares, tmp_path):
    result = full(shares, tmp_path, mock=True)
    jtc = result.controllers["arms_jtc"]["ros__parameters"]
    assert jtc["joints"] == ARMS + ["updown"]
    assert jtc["joint_velocity_limits"][-1] == pytest.approx(0.1666)
    assert jtc["joint_acceleration_limits"][-1] == pytest.approx(0.4)
    assert jtc["action_name"] == "/rt/arms/follow_joint_trajectory"
    rolling = result.controllers["arms_rolling"]["ros__parameters"]
    assert rolling["joints"] == ARMS
    states = result.controllers["joint_state_broadcaster"]["ros__parameters"]["joints"]
    assert states == ARMS + ["updown"] + CHASSIS + ["H1_joint", "H2_joint"]
    assert result.controller_order["active"][-1] == "enable_manager"
    assert {
        "left_wrist_force_sensor_broadcaster",
        "right_wrist_force_sensor_broadcaster",
        "wheel_observation_broadcaster",
    } <= set(result.controller_order["active"])


def test_one_ethercat_system_covers_the_scanned_bus(shares, tmp_path):
    result = full(shares, tmp_path, mock=False)
    systems = {item["result"]["hardware_system_id"]: item["result"] for item in result.systems}
    assert set(systems) == {"ecat_arms", "canopen_swerve_encoders", "damiao_head"}
    topology = systems["ecat_arms"]["expected_topology"]
    assert topology["kind"] == "cia402_topology" and topology["preflight_required"] is True
    rings = sorted(item["ring_position"] for item in topology["responders"])
    assert rings == list(range(27))
    # Both arms hang off the sub-coupler (ring 10); it hangs off the main coupler's port 2.
    assert topology["branches"] == [
        {"main_esc_port": 3, "downstream": 11, "esc_ring_position": 10},
        {"main_esc_port": 1, "downstream": 19, "esc_ring_position": 10},
    ]
    assert topology["internal_ports"] == [{"main_esc_port": 2, "downstream": 10}]
    names = [sensor.attrib["name"] for sensor in systems["ecat_arms"]["sensors"]]
    assert names.count("ethercat_master") == 1 and names.count("ethercat_domain") == 1
    assert result.lifecycle["hardware_activation_order"] == [
        "canopen_swerve_encoders",
        "damiao_head",
        "ecat_arms",
    ]
