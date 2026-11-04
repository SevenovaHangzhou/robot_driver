"""Device fault tables name company DREE codes owned by the public robot_interfaces contract."""

from pathlib import Path
import re

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
FAULTS = ROOT / "src/rt_control/hwcfg_machines/devices/faults"
ERROR_CODES = ROOT / "src/vendor/robot_interfaces/robot_system_interfaces/msg/ErrorCode.msg"


def _dree_names():
    return {
        fault["dree"]
        for path in FAULTS.glob("*.yaml")
        for fault in yaml.safe_load(path.read_text(encoding="utf-8"))["faults"]
    }


def test_fault_table_dree_names_exist_in_the_public_error_codes():
    if not ERROR_CODES.is_file():
        pytest.skip("import deps.repos to check DREE names against robot_interfaces")
    defined = set(re.findall(r"^uint32 (RT_[A-Z0-9_]+)=", ERROR_CODES.read_text(), re.M))
    if "RT_DEVICE_FAULT" not in defined:
        pytest.skip("pinned robot_interfaces predates the device-fault DREE codes")
    assert _dree_names() <= defined
