"""Repository closure for the V3 FJT and rolling runtime."""

from pathlib import Path
import hashlib

import yaml
import pytest


ROOT = Path(__file__).resolve().parents[2]
ROLLING = ROOT / "src/rt_control/ctrl_rolling_trajectory"
EXPECTED_INTERFACE_SHA = "ea582c8a711874813d2ad7f0801b366bd32ab67e"
EXPECTED_JOINTS = tuple(
    [f"right_joint{index}" for index in range(1, 8)]
    + [f"left_joint{index}" for index in range(1, 8)]
)


def test_public_interface_pin_contains_remote_rolling_schema():
    dependencies = yaml.safe_load((ROOT / "deps.repos").read_text(encoding="utf-8"))
    source_lock = yaml.safe_load(
        (ROOT / "src/interfaces/source-lock.yaml").read_text(encoding="utf-8")
    )
    pin = dependencies["repositories"]["src/vendor/robot_interfaces"]["version"]
    assert pin == EXPECTED_INTERFACE_SHA
    assert source_lock["commit"] == EXPECTED_INTERFACE_SHA


def test_rolling_package_is_in_source_and_runtime_closure():
    assert (ROLLING / "package.xml").is_file()
    assert (ROLLING / "rolling_trajectory_controller_plugins.xml").is_file()
    bootstrap = (ROOT / "tools/bootstrap_native_dev.sh").read_text(encoding="utf-8")
    workflow = (ROOT / ".github/workflows/rt-control-ci.yml").read_text(encoding="utf-8")
    from tools.repository_gate import domain_packages
    assert "ctrl_rolling_trajectory" in bootstrap
    assert workflow.count("ctrl_rolling_trajectory") >= 3
    assert "ctrl_rolling_trajectory" in domain_packages(ROOT)


def _machine_motion_joints():
    config = ROOT / "src/rt_control/hwcfg_machines"
    manifest = yaml.safe_load((config / "machines/Alfa_V3/manifest.yaml").read_text())
    policy = yaml.safe_load((config / manifest["policy_refs"]["motion"]["file"]).read_text())
    return tuple(policy["joints"])


def test_machine_motion_joints_are_the_v3_arm_axes_without_turn_or_updown():
    # The rolling controller takes its joint names from this policy (arch-s6).
    joints = _machine_motion_joints()
    assert joints == EXPECTED_JOINTS
    assert "turn" not in joints and "updown" not in joints


def test_imported_public_axes_match_machine_joints_and_digest():
    registry = ROOT / "src/vendor/robot_interfaces/contract/endpoints.yaml"
    if not registry.is_file():
        pytest.skip("import deps.repos to verify the public/machine axis contract")
    axes = yaml.safe_load(registry.read_text())["axis_sets"]["v3_dual_arm"]
    assert tuple(axes["joint_names"]) == _machine_motion_joints()
    assert axes["position_unit"] == "rad"
    assert axes["velocity_unit"] == "rad/s"
    canonical = "".join(f"{name}:rad:rad/s\n" for name in _machine_motion_joints()).encode()
    assert axes["sha256"] == hashlib.sha256(canonical).hexdigest()


def test_ci_runs_upstream_jtc_once_after_other_package_tests():
    workflow = (ROOT / ".github/workflows/rt-control-ci.yml").read_text()
    step = workflow.split("- name: Test\n", 1)[1].split("- name:", 1)[0]
    assert step.count("colcon test --event-handlers") == 2
    first, second = step.split("colcon test --event-handlers console_direct+ --executor sequential", 1)
    assert "--packages-skip joint_trajectory_controller" in first
    assert "--packages-select joint_trajectory_controller" in second
    assert second.index("--packages-select joint_trajectory_controller") < second.index("colcon test-result --verbose")
    assert "--retest" not in step and "--ctest-args" not in step


def test_ci_sources_overlay_before_v3_installed_config_validation():
    workflow = (ROOT / ".github/workflows/rt-control-ci.yml").read_text()
    step = workflow.split("- name: Enforce V3 source and runtime boundary", 1)[1]
    assert "shell: bash" in step
    assert step.index("source /opt/ros/humble/setup.bash") < step.index("source install/setup.bash")
    assert step.index("source install/setup.bash") < step.index("python3 tools/check_v3_branch_contract.py")


def test_ci_sources_the_built_overlay_before_expanding_the_v3_model():
    workflow = (ROOT / ".github/workflows/rt-control-ci.yml").read_text(
        encoding="utf-8"
    )
    validation = workflow.split("- name: Validate shared robot description", 1)[1]
    validation = validation.split("- name:", 1)[0]
    assert validation.index("source install/setup.bash") < validation.index(
        "xacro src/description/robot_description/urdf/robot_alfav3_mobile.urdf"
    )
