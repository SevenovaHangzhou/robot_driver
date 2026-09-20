import importlib.util
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _module():
    spec = importlib.util.spec_from_file_location("ci_scope", ROOT / "tools/ci_scope.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_head_scope_accepts_only_head_owned_paths():
    paths = [
        "src/rt_control/robot_hw_can/src/protocol.cpp",
        "src/rt_control/damiao_head_controller/src/head_manager_controller.cpp",
        "src/rt_control/rt_control_bringup/launch/rt_control_head_runtime.launch.py",
        "src/rt_control/rt_control_bringup/rt_control_bringup/head_runtime.py",
    ]
    assert _module().is_head_only(paths)


def test_head_scope_rejects_empty_shared_and_ci_changes():
    module = _module()
    assert not module.is_head_only([])
    assert not module.is_head_only([
        "src/rt_control/robot_hw_can/src/protocol.cpp",
        "src/rt_control/rt_control_bringup/config/machines/alfa_v3.yaml",
    ])
    assert not module.is_head_only([".github/workflows/rt-control-ci.yml"])
    assert not module.is_head_only([
        "src/rt_control/rt_control_bringup/rt_control_bringup/machine_profile.py"
    ])


def test_cli_emits_github_output_boole():
    command = [sys.executable, str(ROOT / "tools/ci_scope.py")]
    accepted = subprocess.run(
        command,
        input="src/rt_control/robot_hw_can/src/protocol.cpp\n",
        text=True,
        capture_output=True,
        check=True,
    )
    rejected = subprocess.run(
        command,
        input="src/rt_control/rt_control_bringup/config/machines/alfa_v3.yaml\n",
        text=True,
        capture_output=True,
        check=True,
    )
    assert accepted.stdout == "true\n"
    assert rejected.stdout == "false\n"
