"""Frozen readback contract and retired executable behavior."""
import os
from pathlib import Path
import subprocess
import sys

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_only_confirmed_units_and_readback_indices_are_configured():
    config = yaml.safe_load((PACKAGE.parent / "robot_hw_ethercat/config/x503b_readback.yaml").read_text())
    assert config["engineering_unit_contract"] == "force_N_torque_Nm"
    assert config["validity_policy"] == "sample_codes_in_range"
    assert config["valid_sample_codes"] == []
    assert (config["sample_code_min"], config["sample_code_max"]) == (-999999, 999999)
    assert config["read_only_sdo"] == {
        "index": 0x8005, "decimal_subindices": list(range(6, 12)),
        "unit_subindices": list(range(12, 18)), "expected_unit_codes": [5, 5, 5, 7, 7, 7]}


def test_retired_runtime_reader_rejects_without_invoking_ethercat(tmp_path):
    marker = tmp_path / "device-accessed"
    fake = tmp_path / "ethercat"
    fake.write_text(f"#!/bin/sh\ntouch '{marker}'\n")
    fake.chmod(0o755)
    result = subprocess.run([sys.executable, str(PACKAGE / "scripts/x503_sdo_snapshot")],
                            capture_output=True, text=True, env={**os.environ, "PATH": str(tmp_path)})
    assert result.returncode == 2
    assert "PREOP" in result.stderr
    assert not marker.exists()
