"""Compose the V3 14-CSP motion runtime while PP grippers remain hold-only."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import xacro
import yaml


MOTION_JOINTS = tuple(
    [f"right_joint{index}" for index in range(1, 8)]
    + [f"left_joint{index}" for index in range(1, 8)]
)
PP_JOINTS = ("left_moving_jaw_joint", "right_moving_jaw_joint")
COUNTS_PER_REVOLUTION = 524288
COUNTS_PER_RADIAN = COUNTS_PER_REVOLUTION / (2.0 * math.pi)
RADIANS_PER_COUNT = (2.0 * math.pi) / COUNTS_PER_REVOLUTION


@dataclass(frozen=True)
class ArmMotionRuntime:
    robot_description: str
    controllers: dict
    motion_joints: tuple[str, ...] = MOTION_JOINTS
    pp_joints: tuple[str, ...] = PP_JOINTS


def _param(parent: ET.Element, name: str, value) -> None:
    ET.SubElement(parent, "param", name=name).text = str(value)


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _load_axes(hardware_share: Path) -> tuple[list[dict], dict[int, dict]]:
    machines = hardware_share / "config/machines"
    inventory = _load_yaml(machines / "alfa_v3_arms_only.draft.yaml")
    zero = _load_yaml(machines / "alfa_v3_mechanical_zero.yaml")
    axes = [
        axis
        for side in ("left", "right")
        for axis in inventory["branches"][side]["axes"]
    ]
    zeros = {axis["ring_position"]: axis for axis in zero["axes"]}
    if [axis["ring_position"] for axis in axes] != list(range(1, 17)):
        raise ValueError("V3 arm inventory must cover ring positions 1..16")
    if [axis["mode_of_operation"] for axis in axes] != [8] * 7 + [1] + [8] * 7 + [1]:
        raise ValueError("V3 arm inventory must contain fourteen CSP and two PP axes")
    if set(zeros) != set(range(1, 17)):
        raise ValueError("V3 mechanical zero must cover all sixteen axes")
    return axes, zeros


def _load_calibration(hardware_share: Path, *, require_verified: bool) -> dict[str, dict]:
    calibration = _load_yaml(
        hardware_share
        / "config/machines/alfa_v3_arm_motion_calibration.draft.yaml"
    )
    if calibration["output_counts_per_revolution"] != COUNTS_PER_REVOLUTION:
        raise ValueError("Unexpected V3 output encoder resolution")
    entries = calibration["axes"]
    by_name = {entry["joint_name"]: entry for entry in entries}
    if set(by_name) != set(MOTION_JOINTS) or len(entries) != len(by_name):
        raise ValueError("V3 calibration must cover fourteen unique CSP joints")
    if require_verified:
        if calibration.get("verified") is not True:
            raise ValueError("V3 arm motion calibration is not verified")
        if any(entry.get("direction") not in (-1, 1) for entry in entries):
            raise ValueError("Every V3 CSP direction must be verified as +1 or -1")
    return by_name


def _append_interface(parent: ET.Element, kind: str, name: str, initial=None) -> None:
    interface = ET.SubElement(parent, kind, name=name)
    if initial is not None:
        _param(interface, "initial_value", initial)


def _csp_profile(template: dict, *, direction: int, zero_counts: int) -> dict:
    profile = deepcopy(template)
    profile["sdo"][0]["value"] = 8
    target = profile["rpdo"][0]["channels"][0]
    target["command_interface"] = "position"
    target["factor"] = direction * COUNTS_PER_RADIAN
    target["offset"] = zero_counts
    actual = profile["tpdo"][0]["channels"][0]
    actual["state_interface"] = "position"
    actual["factor"] = direction * RADIANS_PER_COUNT
    actual["offset"] = -direction * zero_counts * RADIANS_PER_COUNT
    return profile


def _pp_hold_profile(template: dict) -> dict:
    profile = deepcopy(template)
    profile["sdo"][0]["value"] = 1
    return profile


def _controller_config(envelope_path: Path, managed_joints: list[str]) -> dict:
    one_degree = math.pi / 180.0
    half_degree = math.pi / 360.0
    manager = {
        "update_rate": 1000,
        "thread_priority": 80,
        "joint_state_broadcaster": {
            "type": "joint_state_broadcaster/JointStateBroadcaster"
        },
        "rt_internal_state_broadcaster": {
            "type": "joint_state_broadcaster/JointStateBroadcaster"
        },
        "whole_body_jtc": {
            "type": "joint_trajectory_controller/JointTrajectoryController"
        },
        "rolling_trajectory_controller": {
            "type": "rolling_trajectory_controller/RollingTrajectoryController"
        },
        "enable_manager": {"type": "enable_manager/EnableManagerController"},
    }
    return {
        "controller_manager": {"ros__parameters": manager},
        "joint_state_broadcaster": {
            "ros__parameters": {
                "update_rate": 125,
                "publish_dynamic_joint_states": False,
                "joints": list(MOTION_JOINTS),
                "interfaces": ["position"],
            }
        },
        "rt_internal_state_broadcaster": {
            "ros__parameters": {
                "update_rate": 50,
                "use_local_topics": True,
                "publish_dynamic_joint_states": True,
            }
        },
        "whole_body_jtc": {
            "ros__parameters": {
                "joints": list(MOTION_JOINTS),
                "command_interfaces": ["position"],
                "state_interfaces": ["position"],
                "allow_partial_joints_goal": False,
                "open_loop_control": False,
                "set_last_command_interface_value_as_state_on_activation": False,
                "trajectory_start_consistency_check": {
                    "enabled": True,
                    "position_tolerances": [one_degree] * 14,
                    "feedback_age_state_interface": "ethercat_domain/process_data_age_ms",
                    "max_feedback_age_ms": 500.0,
                },
                "state_publish_rate": 50.0,
                "constraints": {
                    "goal_time": 0.5,
                    "stopped_velocity_tolerance": 0.01,
                },
            }
        },
        "rolling_trajectory_controller": {
            "ros__parameters": {
                "configuration_source": "provisional",
                "allow_test_only_configuration": False,
                "allow_provisional_configuration": True,
                "envelope_file": str(envelope_path),
                "buffer_capacity": 64,
                "max_horizon_ms": 600,
                "required_initial_horizon_ms": 500,
                "update_timeout_ms": 200,
                "replace_lead_ms": 4,
                "state_publish_period_ms": 20,
                "prime_timeout_ms": 100,
                "nominal_controller_period_ms": 1,
                "maximum_controller_period_ms": 2,
                "one_cycle_detection_guard_ms": 1,
                "stop_time_growth_guard_ms": 1,
                "non_rt_to_rt_visibility_guard_ms": 1,
                "period_quantization_guard_ms": 1,
                "open_feedback_age_limit_ms": 500.0,
                "takeover_tolerances": [half_degree] * 14,
                "splice_position_tolerances": [0.01] * 14,
                "splice_velocity_tolerances": [0.03] * 14,
            }
        },
        "enable_manager": {
            "ros__parameters": {
                "managed_joints": managed_joints,
                "enable_batch_joint_names": list(managed_joints),
                "enable_batch_sizes": [1] * 16,
                "disable_terminal_policy": "switch_on_disabled",
                "enable_only": False,
                "jtc_name": "whole_body_jtc",
                "motion_mode_switching": True,
                "motion_controller_names": [
                    "whole_body_jtc",
                    "rolling_trajectory_controller",
                ],
                "default_motion_controller": "whole_body_jtc",
                "rolling_motion_controller": "rolling_trajectory_controller",
                "batch_timeout": 4.0,
                "disable_stage_timeout": 4.0,
                "fault_reset_timeout": 4.0,
                "inter_batch_delay": 0.2,
                "controller_switch_timeout": 4.0,
                "service_result_timeout_ms": 90000,
                "mode_switch_source_state_max_age_ms": 100,
                "mode_switch_stable_interval_count": 5,
                "mode_switch_maximum_sample_period_ms": 2,
                "mode_switch_timeout_ms": 500,
                "mode_switch_feedback_age_limit_ms": 500.0,
                "mode_switch_stable_velocity_thresholds": [half_degree] * 14,
                "mode_switch_takeover_tolerances": [half_degree] * 14,
                "motion_joints": list(MOTION_JOINTS),
            }
        },
    }


def _expand_robot_model(description_share: Path, runtime_dir: Path) -> ET.Element:
    """Expand the source or installed model without mutating the package index."""
    model_path = description_share / "urdf/robot_dual_gripper.urdf.xacro"
    prefix = runtime_dir / "source_description_prefix"
    resource = prefix / "share/ament_index/resource_index/packages/robot_description"
    resource.parent.mkdir(parents=True, exist_ok=True)
    resource.write_text("", encoding="utf-8")
    package_link = prefix / "share/robot_description"
    if not package_link.exists():
        package_link.symlink_to(description_share, target_is_directory=True)
    previous_prefix = os.environ.get("AMENT_PREFIX_PATH")
    os.environ["AMENT_PREFIX_PATH"] = (
        str(prefix)
        if not previous_prefix
        else str(prefix) + os.pathsep + previous_prefix
    )
    try:
        return ET.fromstring(xacro.process_file(str(model_path)).toxml())
    finally:
        if previous_prefix is None:
            os.environ.pop("AMENT_PREFIX_PATH", None)
        else:
            os.environ["AMENT_PREFIX_PATH"] = previous_prefix


def build_arm_motion_runtime(
    hardware_share,
    description_share,
    runtime_dir,
    *,
    use_mock_hardware: bool = True,
    bringup_share=None,
) -> ArmMotionRuntime:
    hardware_share = Path(hardware_share)
    description_share = Path(description_share)
    runtime_dir = Path(runtime_dir)
    axes, zeros = _load_axes(hardware_share)
    calibration = _load_calibration(
        hardware_share, require_verified=not use_mock_hardware
    )
    runtime_dir.mkdir(parents=True, exist_ok=True)

    robot = _expand_robot_model(description_share, runtime_dir)
    system = ET.SubElement(robot, "ros2_control", name="ecat_arms", type="system")
    hardware = ET.SubElement(system, "hardware")
    ET.SubElement(hardware, "plugin").text = (
        "mock_components/GenericSystem"
        if use_mock_hardware
        else "ethercat_driver/EthercatDriver"
    )
    if use_mock_hardware:
        _param(hardware, "calculate_dynamics", "false")
    for name, value in {
        "master_id": 0,
        "control_frequency": 1000,
        "startup_bus_timeout_ms": 70000,
        "preload_timeout_ms": 5000,
    }.items():
        _param(hardware, name, value)

    template = _load_yaml(
        hardware_share / "config/machines/zeroerr_stationary.yaml"
    )
    managed_joints: list[str] = []
    for axis in axes:
        position = axis["ring_position"]
        mode = axis["mode_of_operation"]
        joint_name = axis["robot_model_joint"]
        managed_joints.append(joint_name)
        joint = ET.SubElement(system, "joint", name=joint_name)
        if mode == 8:
            _append_interface(joint, "command_interface", "position")
            _append_interface(joint, "command_interface", "control_word", 0)
            _append_interface(joint, "state_interface", "position", 0.0 if use_mock_hardware else None)
            _append_interface(joint, "state_interface", "digital_inputs", 0 if use_mock_hardware else None)
            _append_interface(joint, "state_interface", "status_word", 0x40 if use_mock_hardware else None)
            entry = calibration[joint_name]
            direction = 1 if use_mock_hardware else entry["direction"]
            profile = _csp_profile(
                template,
                direction=direction,
                zero_counts=entry["zero_counts"],
            )
        else:
            _append_interface(joint, "command_interface", "control_word", 0)
            _append_interface(joint, "state_interface", "position_raw", zeros[position]["zero_counts"] if use_mock_hardware else None)
            _append_interface(joint, "state_interface", "digital_inputs", 0 if use_mock_hardware else None)
            _append_interface(joint, "state_interface", "status_word", 0x40 if use_mock_hardware else None)
            profile = _pp_hold_profile(template)

        profile_path = runtime_dir / f"{joint_name}.yaml"
        profile_path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")
        if not use_mock_hardware:
            for key, value in {
                "name": joint_name + "_drive",
                "plugin": "ethercat_generic_plugins/EcCiA402Drive",
                "alias": 0,
                "position": position,
                "mode_of_operation": mode,
                "slave_config": profile_path,
            }.items():
                _param(joint, "ec_module." + key, value)

    sensors = {
        "ethercat_domain": {"process_data_age_ms": 0},
        "ethercat_master": {
            "link_up": 1,
            "slaves_responding": 18,
            "wc_error_count": 0,
        },
    }
    sensors.update(
        {f"ethercat_slave_{position}": {"al_state": 8} for position in range(1, 17)}
    )
    for sensor_name, interfaces in sensors.items():
        sensor = ET.SubElement(system, "sensor", name=sensor_name)
        for interface_name, initial in interfaces.items():
            _append_interface(
                sensor,
                "state_interface",
                interface_name,
                initial if use_mock_hardware else None,
            )

    source_envelope = (
        Path(bringup_share)
        if bringup_share is not None
        else Path(__file__).resolve().parents[1]
    ) / "config/alfa_v3_rolling_envelope.mock.yaml"
    envelope_path = runtime_dir / source_envelope.name
    shutil.copyfile(source_envelope, envelope_path)
    controllers = _controller_config(envelope_path, managed_joints)
    return ArmMotionRuntime(
        robot_description=ET.tostring(robot, encoding="unicode"),
        controllers=controllers,
    )
