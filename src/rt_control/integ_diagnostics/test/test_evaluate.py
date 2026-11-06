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


KINCO_TABLES = {
    "kinco_fd": yaml.safe_load(
        (RT / "hwcfg_machines/devices/faults/kinco_fd.yaml").read_text(encoding="utf-8")
    )
}


def kinco(encoder):
    return {
        "source": "swerve_chassis/steer1_joint/drive",
        "role": "drive",
        "state": {
            "al_state": "ethercat_slave_0/al_state",
            "status_word": "steer1_joint/status_word",
            "error_word_1": "steer1_joint/error_word_1",
            "error_word_2": "steer1_joint/error_word_2",
        },
        "fault_table": "kinco_fd",
        "motor_encoder_type": encoder,
    }


def kinco_sample(word_1=0, word_2=0, status_word=0x0231):
    return {
        "ethercat_slave_0/al_state": 8,
        "steer1_joint/status_word": status_word,
        "steer1_joint/error_word_1": word_1,
        "steer1_joint/error_word_2": word_2,
    }


def test_kinco_without_error_bits_is_healthy():
    assert evaluate([kinco("magnetic")], KINCO_TABLES, kinco_sample()) == []


@pytest.mark.parametrize(
    "encoder, message",
    [("magnetic", "编码器通讯错误"), ("incremental", "编码器 ABZ 信号错误")],
)
def test_kinco_error_bits_follow_the_motor_encoder_type(encoder, message):
    # Word 1 bit 0 only points at word 2; bit 1 is encoder-specific; word 2 bit 1 is the watchdog.
    entries = evaluate(
        [kinco(encoder)],
        KINCO_TABLES,
        kinco_sample(word_1=0x0003, word_2=0x0002, status_word=0x0238),
    )
    assert [entry["detail"].split("（")[0] for entry in entries] == [
        f"0x2601 bit1 {message}",
        "0x2602 bit1 看门狗报错",
    ]
    assert all(entry["dree"] == "RT_DEVICE_FAULT" and entry["blocking"] for entry in entries)
    assert "Kinco 手册 p100" in entries[0]["detail"]


def test_kinco_unlisted_bit_is_still_a_fault():
    [entry] = evaluate([kinco("magnetic")], KINCO_TABLES, kinco_sample(word_2=1 << 5))
    assert (entry["dree"], entry["detail"]) == ("RT_DEVICE_FAULT", "0x2602 bit5 未登记")


def test_kinco_fault_bit_without_error_words_falls_back_to_the_status_word():
    [entry] = evaluate([kinco("magnetic")], KINCO_TABLES, kinco_sample(status_word=0x0238))
    assert entry["detail"] == "CiA402 fault bit set, no error code read"
