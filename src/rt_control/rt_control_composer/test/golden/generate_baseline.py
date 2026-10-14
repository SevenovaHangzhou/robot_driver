"""Generate frozen outputs only from the pinned old checkout, in a separate process."""

import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml

PIN = "b7650e1e7e88f0ed53794fa8e772b443d21d996a"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = args.baseline.resolve()
    assert (
        subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
        == PIN
    )
    assert not subprocess.check_output(
        ["git", "-C", str(source), "status", "--porcelain"], text=True
    )
    bringup = source / "src/rt_control/rt_control_bringup"
    sys.path.insert(0, str(bringup))
    from rt_control_bringup.arm_motion_runtime import build_arm_motion_runtime
    from rt_control_bringup.stationary_enable import build_stationary_enable

    hardware = source / "src/rt_control/robot_hw_ethercat"
    description = source / "src/description/robot_description"
    args.output.mkdir(parents=True, exist_ok=True)
    calibration = yaml.safe_load(
        (hardware / "config/machines/alfa_v3_arm_motion_calibration.draft.yaml").read_text()
    )
    calibration["verified"] = True
    for index, entry in enumerate(calibration["axes"]):
        entry["direction"] = -1 if index % 2 else 1
    test_calibration = args.output / "verified_calibration.yaml"
    test_calibration.write_text(yaml.safe_dump(calibration, sort_keys=False))
    cases = {}
    for name in (
        "arm-mock",
        "arm-real",
        "arm-mock-jtc",
        "arm-real-jtc",
        "stationary-mock",
        "stationary-real",
    ):
        target = args.output / name
        assert not target.exists(), "Frozen case already exists; do not overwrite it"
        target.mkdir()
        mock = "mock" in name
        jtc = name.endswith("-jtc")
        with tempfile.TemporaryDirectory(prefix="old-composer-") as temporary:
            runtime = Path(temporary)
            if name.startswith("arm-"):
                result = build_arm_motion_runtime(
                    hardware,
                    description,
                    runtime,
                    use_mock_hardware=mock,
                    bringup_share=bringup,
                    jtc_only=jtc,
                    calibration_file=test_calibration,
                )
                order = {
                    "inactive": ["whole_body_jtc"]
                    + ([] if jtc else ["rolling_trajectory_controller"]),
                    "active": [
                        "joint_state_broadcaster",
                        "rt_internal_state_broadcaster",
                        "enable_manager",
                    ],
                }
            else:
                result = build_stationary_enable(hardware, runtime, use_mock_hardware=mock)
                order = {
                    "inactive": [],
                    "active": [
                        "joint_state_broadcaster",
                        "rt_internal_state_broadcaster",
                        "enable_manager",
                    ],
                }
            (target / "robot_description.xml").write_text(
                result.robot_description.replace(str(runtime), "&lt;RUNTIME_DIR&gt;") + "\n"
            )
            (target / "controllers.yaml").write_text(
                yaml.safe_dump(result.controllers, sort_keys=False).replace(
                    str(runtime), "<RUNTIME_DIR>"
                )
            )
            (target / "controller_order.yaml").write_text(yaml.safe_dump(order, sort_keys=False))
            (target / "runtime").mkdir()
            for path in sorted(runtime.glob("*.yaml")):
                (target / "runtime" / path.name).write_bytes(path.read_bytes())
        cases[name] = {
            "use_mock_hardware": mock,
            "jtc_only": jtc,
            "outputs": {
                str(path.relative_to(target)): sha256(path.read_bytes()).hexdigest()
                for path in sorted(target.rglob("*"))
                if path.is_file()
            },
        }
    inputs = [
        path
        for directory in (
            hardware / "config/machines",
            description / "urdf",
            description / "config",
        )
        for path in sorted(directory.rglob("*"))
        if path.is_file()
    ]
    inputs += [
        bringup / "rt_control_bringup/arm_motion_runtime.py",
        bringup / "rt_control_bringup/stationary_enable.py",
        bringup / "launch/rt_control_arm_runtime.launch.py",
        bringup / "launch/rt_control_enable_only.launch.py",
        bringup / "config/alfa_v3_rolling_envelope.mock.yaml",
    ]
    manifest = {
        "source_commit": PIN,
        "generation_command": "python3 test/golden/generate_baseline.py --baseline <READ_ONLY_BASELINE> --output test/golden",
        "cases": cases,
        "inputs": {
            str(path.relative_to(source)): sha256(path.read_bytes()).hexdigest() for path in inputs
        },
        "test_calibration_sha256": sha256(test_calibration.read_bytes()).hexdigest(),
        "rename_map": {
            "alfa_v3_stationary_enable": "Alfa_V3_stationary_enable",
            "alfa_v3_rolling_envelope.mock.yaml": "Alfa_V3_arm_envelope.mock.yaml",
            "<RUNTIME_DIR>/alfa_v3_rolling_envelope.mock.yaml": "<RUNTIME_DIR>/Alfa_V3_arm_envelope.mock.yaml",
            "runtime/alfa_v3_rolling_envelope.mock.yaml": "runtime/Alfa_V3_arm_envelope.mock.yaml",
        },
    }
    (args.output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    print(json.dumps({"source_commit": PIN, "cases": list(cases)}))


if __name__ == "__main__":
    main()
