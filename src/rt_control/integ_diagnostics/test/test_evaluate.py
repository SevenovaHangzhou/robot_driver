"""Stage B and passive rules over one state sample, with the real ZeroErr fault table."""

from pathlib import Path

import pytest
import yaml

from integ_diagnostics.evaluate import complete, evaluate, readiness

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


def test_healthy_sample_has_no_entries_and_is_ready():
    assert evaluate(DEVICES, TABLES, sample()) == []
    assert readiness(False, [], [])["status"] == "HEALTHY"


def test_declared_startup_code_warns_without_blocking():
    [entry] = evaluate(DEVICES, TABLES, sample(status_word=0x0218, error_code=0xA000))
    assert entry["dree"] == "RT_DEVICE_FAULT_RESETTABLE" and not entry["blocking"]
    assert entry["severity"] == "WARN" and "0xA000" in entry["detail"]
    fields = readiness(False, [], [entry])
    assert fields["ready"] and fields["status"] == "DEGRADED" and fields["blockers"] == []


@pytest.mark.parametrize(
    "code, dree, blocker, text",
    [
        (0x2214, "RT_DEVICE_FAULT", "device_fault", "电机电流过流"),
        (0xF005, "RT_STO_ACTIVE", "sto_active", "STO"),
        (0x1234, "RT_DEVICE_FAULT", "device_fault", "未登记故障码 0x1234"),
    ],
)
def test_vendor_codes_are_located_and_mapped_through_the_fault_table(code, dree, blocker, text):
    [entry] = evaluate(DEVICES, TABLES, sample(status_word=0x0218, error_code=code))
    assert (entry["source"], entry["dree"], entry["blocking"]) == (
        "arms/left_joint3/drive",
        dree,
        True,
    )
    assert text in entry["detail"]
    fields = readiness(False, [], [entry])
    assert (
        not fields["ready"]
        and fields["status"] == "UNAVAILABLE"
        and fields["blockers"] == [blocker]
    )


def test_offline_slave_and_master_problems_block():
    entries = evaluate(DEVICES, TABLES, sample(al_state=2, slaves=1))
    assert [(e["source"], e["dree"]) for e in entries] == [
        ("arms/left_joint3/drive", "RT_DEVICE_OFFLINE"),
        ("ethercat/master0", "RT_DEVICE_OFFLINE"),
    ]
    assert "1 of 2 slaves responding" in entries[1]["detail"]
    [link] = evaluate([MASTER], TABLES, sample(link_up=0))
    assert link["detail"] == "EtherCAT link down"
    assert readiness(False, [], entries)["blockers"] == ["device_offline"]


def test_fault_bit_without_an_error_code_interface_still_blocks():
    drive = {**DRIVE, "state": {k: v for k, v in DRIVE["state"].items() if k != "error_code"}}
    [entry] = evaluate([drive], TABLES, sample(status_word=0x0218))
    assert entry["dree"] == "RT_DEVICE_FAULT" and entry["blocking"]


def test_pending_readiness_carries_stage_a_warnings_only():
    stage_a = [
        {
            "source": "x",
            "dree": "RT_DEVICE_FAULT_RESETTABLE",
            "severity": "WARN",
            "blocking": False,
            "detail": "d",
        }
    ]
    fields = readiness(True, stage_a, [])
    assert not fields["ready"] and fields["blockers"] == ["self_check_pending"]
    assert fields["errors"] == stage_a and fields["operational_state"] == "self_check_pending"


HEAD = {
    "source": "head_gimbal/H1_joint/head_motor",
    "role": "head_motor",
    "state": {"fault_code": "H1_joint/fault_code", "feedback_age_ms": "H1_joint/feedback_age_ms"},
    "expected": {"max_feedback_age_ms": 50},
    "fault_table": "damiao_dm",
}
ENCODER = {
    "source": "swerve_encoders/steering_encoder_1/encoder",
    "role": "encoder",
    "state": {
        "ready": "steering_encoder_1/ready",
        "feedback_age_ms": "steering_encoder_1/feedback_age_ms",
    },
}
HEAD_TABLES = {
    "damiao_dm": yaml.safe_load(
        (RT / "hwcfg_machines/devices/faults/damiao_dm.yaml").read_text(encoding="utf-8")
    )
}


@pytest.mark.parametrize(
    ("fault_code", "age", "dree", "text"),
    [
        (0, 12.0, None, None),
        (9, 12.0, "RT_DEVICE_FAULT", "欠压"),
        (0, 80.0, "RT_DEVICE_FEEDBACK_STALE", "80 ms > 50 ms"),
    ],
)
def test_head_motor_reports_vendor_errors_and_stale_feedback(fault_code, age, dree, text):
    values = {"H1_joint/fault_code": fault_code, "H1_joint/feedback_age_ms": age}
    entries = evaluate([HEAD], HEAD_TABLES, values)
    if dree is None:
        assert entries == []
    else:
        assert [(e["dree"], e["blocking"]) for e in entries] == [(dree, True)]
        assert text in entries[0]["detail"]


def test_encoder_that_is_not_ready_blocks_as_stale_feedback():
    ready = {"steering_encoder_1/ready": 1, "steering_encoder_1/feedback_age_ms": 4.0}
    assert evaluate([ENCODER], {}, ready) == []
    stale = {"steering_encoder_1/ready": 0, "steering_encoder_1/feedback_age_ms": 40.0}
    (entry,) = evaluate([ENCODER], {}, stale)
    assert entry["dree"] == "RT_DEVICE_FEEDBACK_STALE" and entry["blocking"]
