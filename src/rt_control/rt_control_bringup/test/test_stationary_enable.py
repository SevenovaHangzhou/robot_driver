"""Stationary commissioning never exposes motion commands or infers calibration."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

BRINGUP = Path(__file__).resolve().parents[1]
HARDWARE = BRINGUP.parent / "robot_hw_ethercat"
sys.path.insert(0, str(BRINGUP))


def build(tmp_path):
    from rt_control_bringup.stationary_enable import build_stationary_enable

    return build_stationary_enable(HARDWARE, tmp_path, use_mock_hardware=False)


def test_stationary_entry_has_sixteen_controlword_owners_and_no_motion_interfaces(tmp_path):
    result = build(tmp_path)
    system = ET.fromstring(result.robot_description).find("ros2_control")
    assert system.attrib["name"] == "ecat_arms"
    axes = system.findall("joint")
    assert len(axes) == 16
    assert all([x.attrib["name"] for x in axis.findall("command_interface")] == ["control_word"]
               for axis in axes)
    names = [axis.attrib["name"] for axis in axes]
    manager = result.controllers["enable_manager"]["ros__parameters"]
    assert manager["managed_joints"] == names
    assert manager["enable_batch_joint_names"] == names
    assert sum(manager["enable_batch_sizes"]) == 16
    assert manager["enable_only"] is True
    assert manager["jtc_name"] == ""
    assert "ready_to_switch_on_disable_terminal_joints" not in manager
    assert manager["disable_terminal_policy"] == "switch_on_disabled"
    assert not any(isinstance(token, yaml.tokens.AliasToken)
                   for token in yaml.scan(yaml.safe_dump(result.controllers)))
    assert set(result.controllers["controller_manager"]["ros__parameters"]) == {
        "update_rate", "thread_priority", "enable_manager", "joint_state_broadcaster",
        "rt_internal_state_broadcaster",
    }


def test_stationary_profiles_hold_raw_position_in_csp_and_pp_without_triggering(tmp_path):
    result = build(tmp_path)
    system = ET.fromstring(result.robot_description).find("ros2_control")
    modes = []
    for axis in system.findall("joint"):
        params = {x.attrib["name"]: x.text for x in axis.findall("param")}
        config = yaml.safe_load(Path(params["ec_module.slave_config"]).read_text())
        modes.append(int(params["ec_module.mode_of_operation"]))
        assert config["auto_state_transitions"] is False
        assert config["auto_fault_reset"] is False
        assert config["sdo"] == [{"index": 0x6060, "sub_index": 0,
                                   "type": "int8", "value": modes[-1]}]
        target, word = config["rpdo"][0]["channels"]
        assert target["index"] == 0x607A and "command_interface" not in target
        assert all(channel["index"] != 0x60FE for pdo in config["rpdo"] for channel in pdo["channels"])
        assert word["command_interface"] == "control_word" and word["default"] == 0
        assert config["use_slave_pdo_defaults"] is False
        assert config["tpdo"][0]["channels"][0]["state_interface"] == "position_raw"
    assert modes == [8] * 7 + [1] + [8] * 7 + [1]
    assert len(system.findall("sensor[@name='ethercat_master']")) == 1


def test_stationary_zero_capture_requires_all_confirmed_axes(tmp_path):
    import shutil
    from rt_control_bringup.stationary_enable import build_stationary_enable

    hardware = tmp_path / "hardware"
    shutil.copytree(HARDWARE / "config/machines", hardware / "config/machines")
    zero_path = hardware / "config/machines/alfa_v3_mechanical_zero.yaml"
    zero = yaml.safe_load(zero_path.read_text())
    zero["axes"].pop()
    zero_path.write_text(yaml.safe_dump(zero))
    with pytest.raises(ValueError, match="sixteen|16|zero"):
        build_stationary_enable(hardware, tmp_path / "out", use_mock_hardware=False)
