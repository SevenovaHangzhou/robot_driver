"""The public rolling axis set matches the machine motion joints."""

from pathlib import Path
import hashlib

import yaml
import pytest


ROOT = Path(__file__).resolve().parents[2]


def _machine_motion_joints():
    config = ROOT / "src/rt_control/hwcfg_machines"
    manifest = yaml.safe_load((config / "machines/Alfa_V3/manifest.yaml").read_text())
    policy = yaml.safe_load((config / manifest["policy_refs"]["motion"]["file"]).read_text())
    return tuple(policy["joints"])


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
