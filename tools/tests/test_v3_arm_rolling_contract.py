"""Repository closure for the V3 FJT and rolling runtime."""

from pathlib import Path
import hashlib
import re

import yaml


ROOT = Path(__file__).resolve().parents[2]
ROLLING = ROOT / "src/rt_control/rolling_trajectory_controller"
EXPECTED_INTERFACE_SHA = "9aa2693d7d3235958369272b7ce8c48592dd7e83"
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
    repository_gate = (ROOT / "tools/repository_gate.py").read_text(encoding="utf-8")
    assert "rolling_trajectory_controller" in bootstrap
    assert workflow.count("rolling_trajectory_controller") >= 3
    assert '"rolling_trajectory_controller"' in repository_gate


def test_rolling_core_uses_v3_fixed_axis_order_without_turn_or_updown():
    header = (
        ROLLING
        / "include/rolling_trajectory_controller/rolling_types.hpp"
    ).read_text(encoding="utf-8")
    match = re.search(r"kJointNames\s*=\s*\{(.*?)\};", header, re.S)
    assert match is not None
    names = tuple(re.findall(r'"([^"]+)"', match.group(1)))
    assert names == EXPECTED_JOINTS
    assert '"turn"' not in match.group(1)
    assert '"updown"' not in match.group(1)


def test_v3_axis_set_hash_is_derived_from_the_documented_canonical_lines():
    implementation = (
        ROLLING / "src/rolling_trajectory_controller.cpp"
    ).read_text(encoding="utf-8")
    block = re.search(r"kAxisSetHash\s*=\s*\{(.*?)\};", implementation, re.S)
    assert block is not None
    encoded = bytes(int(value, 16) for value in re.findall(r"0x([0-9a-fA-F]+)U", block.group(1)))
    canonical = "".join(f"{name}:rad:rad/s\n" for name in EXPECTED_JOINTS).encode()
    assert encoded == hashlib.sha256(canonical).digest()


def test_v3_branch_contract_requires_arm_runtime_not_only_validation():
    contract = (ROOT / "tools/check_v3_branch_contract.py").read_text(encoding="utf-8")
    assert "rt_control_arm_runtime.launch.py" in contract
    assert "rolling_trajectory_controller" in contract
    assert "14 CSP" in contract or "fourteen CSP" in contract
