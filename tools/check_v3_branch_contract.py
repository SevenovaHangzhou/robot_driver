#!/usr/bin/env python3
"""Validate the V3-only source, launch, and build-closure contract."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import yaml


EXPECTED_DESCRIPTION_REVISION = "12a83dd0a008b2e1046b86053cd3a47592307acb"
EXPECTED_DESCRIPTION_TREE = "adc951282e0c6746ca21d55014d649481f3a000d"
EXPECTED_ROLLING_INTERFACE_REVISION = "ea582c8a711874813d2ad7f0801b366bd32ab67e"


def validate(repository_root: Path) -> list[str]:
    findings: list[str] = []
    source_lock_path = repository_root / "src/description/source-lock.yaml"
    machine_path = (
        repository_root / "src/rt_control/hwcfg_machines/machines/Alfa_V3/manifest.yaml"
    )
    cmake_path = repository_root / "src/rt_control/rt_control_bringup/CMakeLists.txt"
    package_path = repository_root / "src/rt_control/rt_control_bringup/package.xml"
    start_path = repository_root / "src/rt_control/rt_control_bringup/scripts/rt_control_start"
    dependency_path = repository_root / "deps.repos"
    interface_lock_path = repository_root / "src/interfaces/source-lock.yaml"
    arm_calibration_path = (
        repository_root / "src/rt_control/hwcfg_machines/machines/Alfa_V3/arms.yaml"
    )

    try:
        source_lock = yaml.safe_load(source_lock_path.read_text(encoding="utf-8"))
        machine = yaml.safe_load(machine_path.read_text(encoding="utf-8"))
        cmake = cmake_path.read_text(encoding="utf-8")
        package = package_path.read_text(encoding="utf-8")
        start = start_path.read_text(encoding="utf-8")
        dependencies = yaml.safe_load(dependency_path.read_text(encoding="utf-8"))
        interface_lock = yaml.safe_load(interface_lock_path.read_text(encoding="utf-8"))
        arm_calibration = yaml.safe_load(arm_calibration_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        return [f"V3 contract input could not be read: {error}"]

    expected_source = {
        "repository": "https://github.com/SevenovaHangzhou/robot_description.git",
        "source_branch": "main",
        "source_revision": EXPECTED_DESCRIPTION_REVISION,
        "source_tree": EXPECTED_DESCRIPTION_TREE,
        "model_family": "alfa_v3",
        "default_end_effector": "mobile_proxy",
    }
    for key, expected in expected_source.items():
        if source_lock.get(key) != expected:
            findings.append(f"source-lock {key} must be {expected!r}")

    model = machine.get("robot_model", {})
    expected_model = {
        "package": "robot_description",
        "source_revision": EXPECTED_DESCRIPTION_REVISION,
    }
    for key, expected in expected_model.items():
        if model.get(key) != expected:
            findings.append(f"alfa_v3 robot_model.{key} must be {expected!r}")

    deployment = yaml.safe_load(
        (
            repository_root
            / "src/rt_control/hwcfg_machines/units/Alfa_V3-01/deployment.yaml"
        ).read_text()
    )
    # ELECTRI-127: the unit deploys the Alfav3 dual arms on the Advantech mobile base; no gripper joints.
    if (
        deployment["end_effector"] != "mobile_proxy"
        or model["variants"].get("mobile_proxy", {}).get("xacro_file")
        != "urdf/robot_alfav3_mobile.urdf"
    ):
        findings.append("V3 deployment must select the deployed mobile model")
    if "launch/rt_control_module.launch.py" not in cmake:
        findings.append("V3 bringup must install the machine-profile launch")
    if "launch/rt_control_scope_runtime.launch.py" not in cmake:
        findings.append("V3 bringup must install the staged scope runtime launch")
    for retired_install in (
        "launch/rt_control.launch.py",
        "launch/rt_control_arm_runtime.launch.py",
        "launch/rt_control_enable_only.launch.py",
        "launch/rt_control_force_sensors.launch.py",
        "launch/rt_control_head_runtime.launch.py",
        "config/controllers.yaml",
        "test_preop_snapshot_launch",
        "test_mock_contract",
    ):
        if retired_install in cmake:
            findings.append(f"V3 bringup must not install/test {retired_install}")
    for retired_dependency in ("x503_force_sensor", "diff_drive_controller"):
        if f"<exec_depend>{retired_dependency}</exec_depend>" in package:
            findings.append(f"V3 bringup must not depend on {retired_dependency}")
    if "launch_file=rt_control_module.launch.py" not in start:
        findings.append("V3 rt_control_start must default to static module validation")
    if "launch_file=rt_control.launch.py" in start:
        findings.append("V3 rt_control_start must not select the V2 launch")
    if 'exec "$0" --scope arms_only "$@"' not in start:
        findings.append("V3 rt_control_start must expose the explicit arm runtime")
    if "<exec_depend>ctrl_rolling_trajectory</exec_depend>" not in package:
        findings.append("V3 bringup must depend on ctrl_rolling_trajectory")
    interface_pin = (
        dependencies.get("repositories", {}).get("src/vendor/robot_interfaces", {}).get("version")
    )
    if interface_pin != EXPECTED_ROLLING_INTERFACE_REVISION:
        findings.append("V3 deps.repos must pin the rolling public interface revision")
    if interface_lock.get("commit") != EXPECTED_ROLLING_INTERFACE_REVISION:
        findings.append("V3 source-lock must match the rolling public interface revision")
    calibration_axes = [
        axis
        for branch in arm_calibration["spec"]["branches"].values()
        for axis in branch["axes"]
        if axis["mode_of_operation"] == 8
    ]
    if len(calibration_axes) != 14:
        findings.append("V3 arm runtime must declare exactly 14 CSP calibration axes")

    safety = yaml.safe_load(
        (
            repository_root
            / "src/rt_control/hwcfg_machines/machines/Alfa_V3/safety_policy.yaml"
        ).read_text()
    )["fault_dependencies"]
    expected_safety = [
        {
            "source_module": "active_suspension",
            "affected_modules": ["swerve_chassis"],
            "reaction": "stop_and_inhibit",
        },
        {
            "source_module": "swerve_chassis",
            "affected_modules": ["arms"],
            "reaction": "stop",
        },
    ]
    if safety != expected_safety:
        findings.append("V3 fault dependency policy does not match BQ-151")
    if machine.get("profiles", {}).get("full_robot", {}).get("allowed_scopes") != ["full"]:
        findings.append("full_robot must allow only the full control scope")
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repository-root",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
    )
    args = parser.parse_args()
    root = args.repository_root.resolve()
    findings = validate(root)
    validator = root / "src/rt_control/rt_control_bringup/scripts/validate_machine_profile.py"
    manifest = root / "src/rt_control/hwcfg_machines/machines/Alfa_V3/manifest.yaml"
    result = subprocess.run(
        [
            sys.executable,
            str(validator),
            "--manifest",
            str(manifest),
            "--unit",
            "Alfa_V3-01",
            "--all",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        findings.append(f"V3 machine profile validation failed: {result.stderr.strip()}")
    if findings:
        print("V3 branch contract failed:", file=sys.stderr)
        for finding in findings:
            print(f"- {finding}", file=sys.stderr)
        return 1
    print("PASS: V3 description identity, machine policy, and runtime boundary")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
