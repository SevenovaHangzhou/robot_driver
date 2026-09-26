"""V3 dual-seven-axis motion runtime contract."""

from __future__ import annotations

from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


BRINGUP = Path(__file__).resolve().parents[1]
RT_CONTROL = BRINGUP.parent
HARDWARE = RT_CONTROL / "robot_hw_ethercat"
DESCRIPTION = RT_CONTROL.parent / "description/robot_description"
ROOT = RT_CONTROL.parents[1]
MOTION_JOINTS = tuple(
    [f"right_joint{index}" for index in range(1, 8)]
    + [f"left_joint{index}" for index in range(1, 8)]
)
PP_JOINTS = ("left_moving_jaw_joint", "right_moving_jaw_joint")

if str(BRINGUP) not in sys.path:
    sys.path.insert(0, str(BRINGUP))


def _build(tmp_path: Path, *, mock: bool = True, jtc_only: bool = False):
    from rt_control_bringup.arm_motion_runtime import build_arm_motion_runtime

    return build_arm_motion_runtime(
        hardware_share=HARDWARE,
        description_share=DESCRIPTION,
        runtime_dir=tmp_path,
        use_mock_hardware=mock,
        jtc_only=jtc_only,
    )


def _systems(robot_description: str):
    return ET.fromstring(robot_description).findall("ros2_control")


def _interfaces(joint: ET.Element, kind: str) -> list[str]:
    return [item.attrib["name"] for item in joint.findall(kind)]


def _initial_value(joint: ET.Element, kind: str, name: str) -> str | None:
    interface = joint.find(f"{kind}[@name='{name}']")
    if interface is None:
        return None
    parameter = interface.find("param[@name='initial_value']")
    return None if parameter is None else parameter.text


def test_mock_runtime_exposes_fourteen_csp_commands_and_holds_two_pp_axes(tmp_path):
    result = _build(tmp_path)
    systems = _systems(result.robot_description)
    assert len(systems) == 1
    assert systems[0].attrib["name"] == "ecat_arms"
    hardware_parameters = {
        parameter.attrib["name"]: parameter.text
        for parameter in systems[0].findall("hardware/param")
    }
    assert hardware_parameters["calculate_dynamics"] == "false"
    joints = {joint.attrib["name"]: joint for joint in systems[0].findall("joint")}
    assert set(joints) == set(MOTION_JOINTS + PP_JOINTS)

    for name in MOTION_JOINTS:
        assert _interfaces(joints[name], "command_interface") == [
            "position",
            "control_word",
        ]
        assert _interfaces(joints[name], "state_interface") == [
            "position",
            "digital_inputs",
            "status_word",
        ]
        assert _initial_value(joints[name], "state_interface", "status_word") == "64"
    for name in PP_JOINTS:
        assert _interfaces(joints[name], "command_interface") == ["control_word"]
        assert "position" not in _interfaces(joints[name], "state_interface")
        assert _initial_value(joints[name], "state_interface", "status_word") == "64"


def test_runtime_owns_motion_with_jtc_and_stages_rolling_inactive(tmp_path):
    config = _build(tmp_path).controllers
    manager = config["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 1000
    assert manager["whole_body_jtc"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert manager["rolling_trajectory_controller"]["type"] == (
        "rolling_trajectory_controller/RollingTrajectoryController"
    )
    assert "left_gripper_controller" not in manager
    assert "right_gripper_controller" not in manager

    jtc = config["whole_body_jtc"]["ros__parameters"]
    assert tuple(jtc["joints"]) == MOTION_JOINTS
    assert jtc["command_interfaces"] == ["position"]
    assert jtc["state_interfaces"] == ["position"]
    assert jtc["allow_partial_joints_goal"] is False
    assert len(jtc["trajectory_start_consistency_check"]["position_tolerances"]) == 14

    enable = config["enable_manager"]["ros__parameters"]
    assert enable["enable_only"] is False
    assert enable["motion_mode_switching"] is True
    assert tuple(enable["motion_joints"]) == MOTION_JOINTS
    assert enable["motion_controller_names"] == [
        "whole_body_jtc",
        "rolling_trajectory_controller",
    ]
    assert enable["default_motion_controller"] == "whole_body_jtc"
    assert enable["rolling_motion_controller"] == "rolling_trajectory_controller"
    assert set(enable["managed_joints"]) == set(MOTION_JOINTS + PP_JOINTS)


def test_jtc_only_runtime_exposes_fourteen_csp_axes_without_rolling(tmp_path):
    runtime = _build(tmp_path, jtc_only=True)
    manager = runtime.controllers["controller_manager"]["ros__parameters"]
    assert manager["whole_body_jtc"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert "rolling_trajectory_controller" not in manager
    assert "rolling_trajectory_controller" not in runtime.controllers
    assert not (tmp_path / "alfa_v3_rolling_envelope.mock.yaml").exists()
    jtc = runtime.controllers["whole_body_jtc"]["ros__parameters"]
    assert tuple(jtc["joints"]) == MOTION_JOINTS
    assert jtc["allow_partial_joints_goal"] is False
    enable = runtime.controllers["enable_manager"]["ros__parameters"]
    assert enable["enable_only"] is False
    assert enable["motion_mode_switching"] is False
    assert enable["jtc_name"] == "whole_body_jtc"
    assert set(enable["managed_joints"]) == set(MOTION_JOINTS + PP_JOINTS)


def test_verified_real_jtc_calibration_matches_confirmed_zero_and_direction(tmp_path):
    from rt_control_bringup.arm_motion_runtime import build_arm_motion_runtime

    hardware = tmp_path / "hardware"
    shutil.copytree(HARDWARE / "config/machines", hardware / "config/machines")
    calibration = hardware / "config/machines/alfa_v3_arm_motion_calibration.draft.yaml"
    values = yaml.safe_load(calibration.read_text())
    values["verified"] = True
    for index, entry in enumerate(values["axes"]):
        entry["direction"] = -1 if index % 2 else 1
    values["branch_mapping_verified"] = False
    calibration.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError, match="branch mapping"):
        build_arm_motion_runtime(
            hardware_share=hardware,
            description_share=DESCRIPTION,
            runtime_dir=tmp_path / "wrong-branch",
            use_mock_hardware=False,
            jtc_only=True,
            calibration_file=calibration,
        )
    values["branch_mapping_verified"] = True
    calibration.write_text(yaml.safe_dump(values))
    values["axes"][0]["direction"] = True
    calibration.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError, match="direction"):
        build_arm_motion_runtime(
            hardware_share=hardware,
            description_share=DESCRIPTION,
            runtime_dir=tmp_path / "boolean-sign",
            use_mock_hardware=False,
            jtc_only=True,
            calibration_file=calibration,
        )
    values["axes"][0]["direction"] = 1
    calibration.write_text(yaml.safe_dump(values))
    result = build_arm_motion_runtime(
        hardware_share=hardware,
        description_share=DESCRIPTION,
        runtime_dir=tmp_path / "real",
        use_mock_hardware=False,
        jtc_only=True,
        calibration_file=calibration,
    )
    system = ET.fromstring(result.robot_description).find("ros2_control")
    for entry in values["axes"]:
        joint = system.find(f"joint[@name='{entry['joint_name']}']")
        assert joint is not None
        path = next(
            item.text for item in joint.findall("param")
            if item.attrib["name"] == "ec_module.slave_config"
        )
        profile = yaml.safe_load(Path(path).read_text())
        target = profile["rpdo"][0]["channels"][0]
        feedback = profile["tpdo"][0]["channels"][0]
        assert target["offset"] == entry["zero_counts"]
        assert target["factor"] * entry["direction"] > 0
        assert feedback["factor"] * entry["direction"] > 0
        assert feedback["offset"] == pytest.approx(
            -entry["zero_counts"] * feedback["factor"]
        )

    values["axes"][0]["zero_counts"] += 1
    calibration.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError, match="zero_counts"):
        build_arm_motion_runtime(
            hardware_share=hardware,
            description_share=DESCRIPTION,
            runtime_dir=tmp_path / "mismatched",
            use_mock_hardware=False,
            jtc_only=True,
            calibration_file=calibration,
        )


def test_v3_rolling_runtime_uses_one_kilohertz_guards_and_v3_axis_order(tmp_path):
    config = _build(tmp_path).controllers
    rolling = config["rolling_trajectory_controller"]["ros__parameters"]
    assert rolling["nominal_controller_period_ms"] == 1
    assert rolling["maximum_controller_period_ms"] == 2
    assert rolling["configuration_source"] == "provisional"
    assert rolling["allow_test_only_configuration"] is False
    assert rolling["allow_provisional_configuration"] is True

    envelope = yaml.safe_load(Path(rolling["envelope_file"]).read_text(encoding="utf-8"))
    assert envelope["metadata"]["limits_source"] == "provisional"
    assert "V3" in envelope["metadata"]["estimation_method"]
    assert tuple(axis["name"] for axis in envelope["axes"]) == MOTION_JOINTS


def test_real_runtime_rejects_unverified_direction_calibration(tmp_path):
    with pytest.raises(ValueError, match="direction|calibration|verified"):
        _build(tmp_path, mock=False)


def test_runtime_launch_and_start_entry_are_installed_explicitly():
    cmake = (BRINGUP / "CMakeLists.txt").read_text(encoding="utf-8")
    package = (BRINGUP / "package.xml").read_text(encoding="utf-8")
    start = (BRINGUP / "scripts/rt_control_start").read_text(encoding="utf-8")

    assert "launch/rt_control_arm_runtime.launch.py" in cmake
    assert "--arm-runtime" in start
    assert "launch_file=rt_control_arm_runtime.launch.py" in start
    assert "<exec_depend>joint_trajectory_controller</exec_depend>" in package
    assert "<exec_depend>rolling_trajectory_controller</exec_depend>" in package
    assert "<exec_depend>gripper_controllers</exec_depend>" not in package


def test_calibration_draft_uses_user_provisional_plus_one_without_motion_admission(tmp_path):
    calibration = yaml.safe_load(
        (
            HARDWARE
            / "config/machines/alfa_v3_arm_motion_calibration.draft.yaml"
        ).read_text(encoding="utf-8")
    )
    assert calibration["verified"] is False
    assert calibration["branch_mapping_verified"] is True
    assert calibration["direction_status"] == "user_provisional_plus_one"
    assert calibration["output_counts_per_revolution"] == 524288
    assert calibration["external_transmission_ratio"] == 1.0
    axes = calibration["axes"]
    assert len(axes) == 14
    assert {axis["joint_name"] for axis in axes} == set(MOTION_JOINTS)
    assert all(type(axis["direction"]) is int and axis["direction"] == 1 for axis in axes)
    assert all(type(axis["zero_counts"]) is int for axis in axes)
    with pytest.raises(ValueError, match="calibration is not verified"):
        _build(tmp_path, mock=False, jtc_only=True)


def test_arm_runtime_declares_bq154_functional_modules(tmp_path):
    from rt_control_bringup.arm_motion_runtime import ARM_RUNTIME_MODULES

    manifest = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "config/machines/alfa_v3.yaml").read_text()
    )
    functional = set(manifest["functional_modules"])
    assert set(ARM_RUNTIME_MODULES["owned_modules"]) <= functional
    assert ARM_RUNTIME_MODULES["remote_module_name"] in functional
    assert ARM_RUNTIME_MODULES["remote_module_name"] not in ARM_RUNTIME_MODULES["owned_modules"]
    assert ARM_RUNTIME_MODULES["remote_service_prefix"] == "/rt/head"
    build = _build(tmp_path)
    enable = build.controllers["enable_manager"]["ros__parameters"]
    for key, value in ARM_RUNTIME_MODULES.items():
        assert enable[key] == value
