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


def test_full_physical_profile_arms_scope_reports_inactive_modules(monkeypatch):
    module = _launch_module()
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(BRINGUP_DIR))

    actions = module._launch_setup(
        _context(physical_profile="full_robot", control_scope="arms_only")
    )

    message = actions[0].msg[0].text
    assert "physical_profile=full_robot" in message
    assert "control_scope=arms_only" in message
    assert "inactive=updown,swerve_chassis,swerve_encoders,head_gimbal" in message


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

    assert "install(\n  DIRECTORY launch config urdf" in cmake
    assert "<exec_depend>launch</exec_depend>" in package
    assert "<exec_depend>launch_ros</exec_depend>" in package
