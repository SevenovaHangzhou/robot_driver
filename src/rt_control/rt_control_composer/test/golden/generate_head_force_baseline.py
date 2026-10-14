"""Generate only four additional cases from the pinned old checkout."""

import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml

PIN = "b7650e1e7e88f0ed53794fa8e772b443d21d996a"
STARTUP_ID = "force-bench-synthetic-golden"


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
    rt = source / "src/rt_control"
    sys.path.insert(0, str(rt / "rt_control_bringup"))
    from rt_control_bringup.head_runtime import build_head_runtime
    from rt_control_bringup import force_sensor_runtime as force

    namespace = {}
    text = (rt / "rt_control_bringup/test/test_v3_head_can_integration.py").read_text()
    function = text[text.index("def _config():") : text.index("\ndef _write")]
    exec(function, namespace)
    config = namespace["_config"]()
    profiles = [
        yaml.safe_load(
            (
                rt / "robot_hw_ethercat/config/slaves" / ("bluepoint_p140000100_" + side + ".yaml")
            ).read_text()
        )
        for side in ("left", "right")
    ]
    cli = {}
    cli["ethercat master -m 0"] = "Phase: Idle\nActive: no\nLink: UP\nSlaves: 6\n"
    for position, profile in zip((2, 4), profiles):
        cli[f"ethercat slaves -m 0 -p {position} -v"] = (
            f"Vendor Id: {hex(profile['vendor_id'])}\nProduct code: {hex(profile['product_id'])}\nRevision number: {hex(profile['metadata']['revision_id'])}\nState: PREOP\nFlag: +\n"
        )
        lines = []
        for direction, label in (("rpdo", "RxPDO"), ("tpdo", "TxPDO")):
            for pdo in profile[direction]:
                lines.append(f"{label} 0x{pdo['index']:04x}")
                lines += [
                    f"PDO entry 0x{channel['index']:04x}:{channel['sub_index']:02x}, 32 bit"
                    for channel in pdo["channels"]
                ]
        cli[f"ethercat pdos -m 0 -p {position}"] = "\n".join(lines)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "head_config.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
    (args.output / "force_cli.yaml").write_text(yaml.safe_dump(cli, sort_keys=False))
    force.uuid.uuid4 = lambda: "synthetic-golden"
    manifest = yaml.safe_load((args.output / "manifest.yaml").read_text())
    for name in ("head-mock", "head-real", "force-mock", "force-real"):
        target = args.output / name
        assert not target.exists(), "Frozen case exists; do not overwrite"
        target.mkdir()
        (target / "runtime").mkdir()
        mock = "mock" in name
        with tempfile.TemporaryDirectory(prefix="old-head-force-") as temporary:
            runtime = Path(temporary)
            if name.startswith("head"):
                result = build_head_runtime(
                    config=config,
                    description_share=source / "src/description/robot_description",
                    hardware_share=rt / "robot_hw_can",
                    controller_share=rt / "damiao_head_controller",
                    bringup_share=rt / "rt_control_bringup",
                    use_mock_hardware=mock,
                )
                order = {
                    "inactive": ["head_position_controller"],
                    "active": ["joint_state_broadcaster", "damiao_head_manager"],
                }
            else:
                result = force.build_force_sensor_runtime(
                    hardware_share=rt / "robot_hw_ethercat",
                    bringup_share=rt / "rt_control_bringup",
                    runtime_dir=runtime,
                    use_mock_hardware=mock,
                    sync_mode="dc",
                    run=lambda argv: cli[" ".join(argv)],
                )
                order = {"inactive": [], "active": list(result.controller_names)}
            xml = result.robot_description.replace(str(runtime), "&lt;RUNTIME_DIR&gt;")
            (target / "robot_description.xml").write_text(xml + "\n")
            (target / "controllers.yaml").write_text(
                yaml.safe_dump(result.controllers, sort_keys=False).replace(
                    str(runtime), "<RUNTIME_DIR>"
                )
            )
            (target / "controller_order.yaml").write_text(yaml.safe_dump(order, sort_keys=False))
            for path in runtime.glob("*.yaml"):
                (target / "runtime" / path.name).write_bytes(path.read_bytes())
        manifest["cases"][name] = {
            "use_mock_hardware": mock,
            "sync_mode": "dc" if name.startswith("force") else "",
            "startup_id": STARTUP_ID,
            "outputs": {
                path.relative_to(target).as_posix(): sha256(path.read_bytes()).hexdigest()
                for path in sorted(target.rglob("*"))
                if path.is_file()
            },
        }
    inputs = [
        rt / "rt_control_bringup" / "rt_control_bringup" / name
        for name in ("head_runtime.py", "head_can_config.py", "force_sensor_runtime.py")
    ]
    inputs += [
        rt / "rt_control_bringup/urdf/alfa_v3_head.ros2_control.xacro",
        rt / "robot_hw_can/urdf/damiao.ros2_control.xacro",
        rt / "damiao_head_controller/config/controllers.yaml",
        rt / "rt_control_bringup/test/test_v3_head_can_integration.py",
        rt / "rt_control_bringup/config/bluepoint_force_sensors.draft.yaml",
    ]
    inputs += [
        rt / "robot_hw_ethercat/config/slaves" / ("bluepoint_p140000100_" + side + ".yaml")
        for side in ("left", "right")
    ]
    manifest["inputs"].update(
        {str(path.relative_to(source)): sha256(path.read_bytes()).hexdigest() for path in inputs}
    )
    manifest["head_force_generation_command"] = (
        "python3 test/golden/generate_head_force_baseline.py --baseline <READ_ONLY_BASELINE> --output test/golden"
    )
    manifest["head_config_sha256"] = sha256(
        (args.output / "head_config.yaml").read_bytes()
    ).hexdigest()
    manifest["force_cli_sha256"] = sha256((args.output / "force_cli.yaml").read_bytes()).hexdigest()
    (args.output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    print(
        json.dumps(
            {"source_commit": PIN, "added": ["head-mock", "head-real", "force-mock", "force-real"]}
        )
    )


if __name__ == "__main__":
    main()
