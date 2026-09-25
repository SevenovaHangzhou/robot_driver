#!/usr/bin/env python3
"""Validate the V3-only source, launch, and build-closure contract."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import yaml


EXPECTED_DESCRIPTION_REVISION = "ec69ca04297896c1296720324d23cdb8f80d9e63"
EXPECTED_DESCRIPTION_TREE = "864dfff98fdd4cf54df35095232b38728c073c01"


def validate(repository_root: Path) -> list[str]:
    findings: list[str] = []
    source_lock_path = repository_root / "src/description/source-lock.yaml"
    machine_path = (
        repository_root
        / "src/rt_control/rt_control_bringup/config/machines/alfa_v3.yaml"
    )
    cmake_path = repository_root / "src/rt_control/rt_control_bringup/CMakeLists.txt"
    package_path = repository_root / "src/rt_control/rt_control_bringup/package.xml"
    start_path = (
        repository_root
        / "src/rt_control/rt_control_bringup/scripts/rt_control_start"
    )

    try:
        source_lock = yaml.safe_load(source_lock_path.read_text(encoding="utf-8"))
        machine = yaml.safe_load(machine_path.read_text(encoding="utf-8"))
        cmake = cmake_path.read_text(encoding="utf-8")
        package = package_path.read_text(encoding="utf-8")
        start = start_path.read_text(encoding="utf-8")
    except (OSError, yaml.YAMLError) as error:
        return [f"V3 contract input could not be read: {error}"]

    expected_source = {
        "repository": "https://github.com/SevenovaHangzhou/robot_description.git",
        "source_branch": "robot_v3_suction_chassis",
        "source_revision": EXPECTED_DESCRIPTION_REVISION,
        "source_tree": EXPECTED_DESCRIPTION_TREE,
        "model_family": "alfa_v3",
        "default_end_effector": "suction",
    }
    for key, expected in expected_source.items():
        if source_lock.get(key) != expected:
            findings.append(f"source-lock {key} must be {expected!r}")

    model = machine.get("robot_model", {})
    expected_model = {
        "package": "robot_description",
        "xacro_file": "urdf/robot_dual_gripper.urdf.xacro",
        "end_effector": "gripper",
        "source_revision": EXPECTED_DESCRIPTION_REVISION,
    }
    for key, expected in expected_model.items():
        if model.get(key) != expected:
            findings.append(f"alfa_v3 robot_model.{key} must be {expected!r}")

    if "launch/rt_control_module.launch.py" not in cmake:
        findings.append("V3 bringup must install the machine-profile launch")
    for retired_install in (
        "launch/rt_control.launch.py",
        "config/controllers.yaml",
        "test_preop_snapshot_launch",
        "test_mock_contract",
    ):
        if retired_install in cmake:
            findings.append(f"V3 bringup must not install/test {retired_install}")
    for retired_dependency in ("x503_force_sensor", "diff_drive_controller"):
        if f"<exec_depend>{retired_dependency}</exec_depend>" in package:
            findings.append(
                f"V3 bringup must not depend on {retired_dependency}"
            )
    if "launch_file=rt_control_module.launch.py" not in start:
        findings.append("V3 rt_control_start must default to static module validation")
    if "launch_file=rt_control.launch.py" in start:
        findings.append("V3 rt_control_start must not select the V2 launch")

    safety = machine.get("safety_policy", {}).get("fault_dependencies", [])
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
    if machine.get("profiles", {}).get("full_robot", {}).get("allowed_scopes") != [
        "full"
    ]:
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
    manifest = root / "src/rt_control/rt_control_bringup/config/machines/alfa_v3.yaml"
    result = subprocess.run(
        [sys.executable, str(validator), "--manifest", str(manifest), "--all"],
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
