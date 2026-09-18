"""ELECTRI-118 launch-time machine profile selection tests."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

from launch import LaunchContext
from launch.actions import LogInfo
import pytest


BRINGUP_DIR = Path(__file__).resolve().parents[1]
LAUNCH_PATH = BRINGUP_DIR / "launch/rt_control_module.launch.py"
MACHINE_PATH = BRINGUP_DIR / "config/machines/alfa_v3.yaml"
CMAKE_PATH = BRINGUP_DIR / "CMakeLists.txt"
PACKAGE_PATH = BRINGUP_DIR / "package.xml"

if str(BRINGUP_DIR) not in sys.path:
    sys.path.insert(0, str(BRINGUP_DIR))


def _launch_module():
    spec = importlib.util.spec_from_file_location("rt_control_module_launch", LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(**values: str) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "robot_variant": "alfa_v3",
            "physical_profile": "arms_only",
            "control_scope": "arms_only",
            "validation_only": "true",
            **values,
        }
    )
    return context


def test_module_launch_declares_profile_scope_and_validation_arguments():
    module = _launch_module()

    description = module.generate_launch_description()
    declared = {
        action.name
        for action in description.entities
        if hasattr(action, "name")
    }

    assert {
        "robot_variant",
        "physical_profile",
        "control_scope",
        "validation_only",
        "force_sensor_option",
    } <= declared


def test_static_arms_validation_returns_a_summary_without_creating_nodes(monkeypatch):
    module = _launch_module()
    observed = []

    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(BRINGUP_DIR))

    class RecordingNode:
        def __init__(self, **_kwargs):
            observed.append("node")

    monkeypatch.setattr(module, "Node", RecordingNode)

    actions = module._launch_setup(_context())

    assert observed == []
    assert len(actions) == 1
    assert isinstance(actions[0], LogInfo)
    message = actions[0].msg[0].text
    assert "actuators=16" in message
    assert "CSP=14" in message
    assert "PP=2" in message
    assert "arms.gripper_pp=2" in message
    assert "ethercat_ring=" + ",".join(str(position) for position in range(18)) in message
    assert "options=force_sensors:none" in message
    assert "model=robot_description:urdf/robot_dual_gripper.urdf.xacro" in message
    assert "end_effector=gripper" in message
    assert "buses=ethercat:required,canopen:not_required,damiao_can:not_required" in message
    assert "fault_dependencies=none" in message


def test_dual_bluepoint_option_is_visible_and_keeps_ring_tbd(monkeypatch):
    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(BRINGUP_DIR))

    actions = module._launch_setup(_context(force_sensor_option="bluepoint_dual"))

    message = actions[0].msg[0].text
    assert "options=force_sensors:bluepoint_dual" in message
    assert "active=arms,wrist_force_sensors" in message
    assert "state_sensors=2" in message
    assert "ethercat_ring=TBD" in message


def test_full_physical_profile_rejects_partial_control_scope(monkeypatch):
    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(BRINGUP_DIR))

    with pytest.raises(RuntimeError, match="not allowed"):
        module._launch_setup(
            _context(physical_profile="full_robot", control_scope="arms_only")
        )


def test_chassis_only_summary_lists_scoped_swerve_controller(monkeypatch):
    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(BRINGUP_DIR))

    actions = module._launch_setup(
        _context(physical_profile="chassis_only", control_scope="chassis_only")
    )
    message = actions[0].msg[0].text
    assert "controllers=swerve_controller:swerve_driver/SwerveController" in message
    assert "state_sensors=4" in message
    assert "active=swerve_chassis,swerve_encoders,active_suspension" in message
    assert "buses=ethercat:required,canopen:required,damiao_can:not_required" in message
    assert (
        "fault_dependencies=active_suspension->swerve_chassis:stop_and_inhibit"
        in message
    )
    assert "status=draft" in message


@pytest.mark.parametrize(
    "values",
    [
        {"validation_only": "false"},
        {"validation_only": "false", "control_scope": "full"},
        {
            "validation_only": "false",
            "physical_profile": "full_robot",
            "control_scope": "full",
        },
    ],
)
def test_non_static_or_not_ready_selection_fails_before_node_creation(monkeypatch, values):
    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(BRINGUP_DIR))

    with pytest.raises(RuntimeError, match="not runtime-ready|static validation only|invalid"):
        module._launch_setup(_context(**values))


def test_invalid_boolean_is_rejected_before_manifest_lookup(monkeypatch):
    module = _launch_module()
    looked_up = []
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda package: looked_up.append(package) or str(BRINGUP_DIR),
    )

    with pytest.raises(ValueError, match="validation_only must be true or false"):
        module._launch_setup(_context(validation_only="yes"))
    assert looked_up == []


def test_launch_uses_package_owned_manifest_path(monkeypatch):
    module = _launch_module()
    paths = []

    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(BRINGUP_DIR))
    original_loader = module.load_machine_manifest

    def recording_loader(path):
        paths.append(Path(path))
        return original_loader(path)

    monkeypatch.setattr(module, "load_machine_manifest", recording_loader)
    module._launch_setup(_context())

    assert paths == [MACHINE_PATH]


def test_package_installs_machine_config_and_declares_launch_runtime_dependencies():
    cmake = CMAKE_PATH.read_text(encoding="utf-8")
    package = PACKAGE_PATH.read_text(encoding="utf-8")

    assert "launch/rt_control_module.launch.py" in cmake
    assert "launch/rt_control_enable_only.launch.py" in cmake
    assert "config/machines" in cmake
    assert "launch/rt_control.launch.py" not in cmake
    assert "test_preop_snapshot_launch" not in cmake
    assert "test_mock_contract" not in cmake
    assert "<exec_depend>launch</exec_depend>" in package
    assert "<exec_depend>launch_ros</exec_depend>" in package
    assert "<exec_depend>x503_force_sensor</exec_depend>" not in package
    assert "<exec_depend>diff_drive_controller</exec_depend>" not in package
    for independently_deployed_module in (
        "bms_node",
        "lpms_nav3_can",
        "modbus_tcp_rtu485_led",
        "robot_hw_canopen",
        "swerve_driver",
        "plc_io_modbus",
        "rt_diagnostics",
        "rt_force_torque_broadcaster",
    ):
        assert f"<exec_depend>{independently_deployed_module}</exec_depend>" not in package


def test_installed_start_defaults_to_v3_validation_and_keeps_enable_explicit():
    start = (BRINGUP_DIR / "scripts/rt_control_start").read_text(encoding="utf-8")

    assert "launch_file=rt_control_module.launch.py" in start
    assert 'if [[ "${1:-}" == "--enable-only" ]]' in start
    assert "launch_file=rt_control_enable_only.launch.py" in start
    assert "orderly_disable_required=1" in start
    assert "launch_file=rt_control.launch.py" not in start
