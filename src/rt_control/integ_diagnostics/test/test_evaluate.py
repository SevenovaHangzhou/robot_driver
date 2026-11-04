"""Stage B and passive rules over one state sample, with the real ZeroErr fault table."""

from pathlib import Path

import pytest
import yaml

from integ_diagnostics.evaluate import complete, evaluate

RT = Path(__file__).resolve().parents[2]
TABLES = {
    "zeroerr_erob": yaml.safe_load(
        (RT / "hwcfg_machines/devices/faults/zeroerr_erob.yaml").read_text(encoding="utf-8")
    )
}
DRIVE = {
    "source": "arms/left_joint3/drive",
    "role": "drive",
    "state": {
        "al_state": "ethercat_slave_3/al_state",
        "status_word": "left_joint3/status_word",
        "error_code": "left_joint3/error_code",
    },
    "allowed_startup_codes": ["0xA000"],
    "fault_table": "zeroerr_erob",
}
MASTER = {
    "source": "ethercat/master0",
    "role": "ethercat_master",
    "state": {
        "link_up": "ethercat_master/link_up",
        "slaves_responding": "ethercat_master/slaves_responding",
    },
    "expected": {"slaves_responding": 2},
}
DEVICES = [DRIVE, MASTER]


def sample(al_state=8, status_word=0x40, error_code=0, link_up=1, slaves=2):
    return {
        "ethercat_slave_3/al_state": al_state,
        "left_joint3/status_word": status_word,
        "left_joint3/error_code": error_code,
        "ethercat_master/link_up": link_up,
        "ethercat_master/slaves_responding": slaves,
    }


def test_stage_b_waits_for_every_declared_interface():
    values = sample()
    assert complete(DEVICES, values)
    assert not complete(DEVICES, {**values, "left_joint3/error_code": float("nan")})
    del values["ethercat_master/link_up"]
    assert not complete(DEVICES, values)


def test_healthy_sample_has_no_entries():
    assert evaluate(DEVICES, TABLES, sample()) == []


def test_declared_startup_code_warns_without_blocking():
    [entry] = evaluate(DEVICES, TABLES, sample(status_word=0x0218, error_code=0xA000))
    assert entry["dree"] == "RT_DEVICE_FAULT_RESETTABLE" and not entry["blocking"]
    assert entry["severity"] == "WARN" and "0xA000" in entry["detail"]


@pytest.mark.parametrize(
    "code, dree, text",
    [
        (0x2214, "RT_DEVICE_FAULT", "电机电流过流"),
        (0xF005, "RT_STO_ACTIVE", "STO"),
        (0x1234, "RT_DEVICE_FAULT", "未登记故障码 0x1234"),
    ],
)
def test_vendor_codes_are_located_and_mapped_through_the_fault_table(code, dree, text):
    [entry] = evaluate(DEVICES, TABLES, sample(status_word=0x0218, error_code=code))
    assert (entry["source"], entry["dree"], entry["blocking"]) == (
        "arms/left_joint3/drive",
        dree,
        True,
    )
    assert text in entry["detail"]


def test_offline_slave_and_master_problems_block():
    entries = evaluate(DEVICES, TABLES, sample(al_state=2, slaves=1))
    assert [(e["source"], e["dree"]) for e in entries] == [
        ("arms/left_joint3/drive", "RT_DEVICE_OFFLINE"),
        ("ethercat/master0", "RT_DEVICE_OFFLINE"),
    ]
    assert "1 of 2 slaves responding" in entries[1]["detail"]
    [link] = evaluate([MASTER], TABLES, sample(link_up=0))
    assert link["detail"] == "EtherCAT link down"


def test_fault_bit_without_an_error_code_interface_still_blocks():
    drive = {**DRIVE, "state": {k: v for k, v in DRIVE["state"].items() if k != "error_code"}}
    [entry] = evaluate([drive], TABLES, sample(status_word=0x0218))
    assert entry["dree"] == "RT_DEVICE_FAULT" and entry["blocking"]
