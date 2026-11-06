"""Readiness from controller state: one row per case of the 2026-11-04 table."""

from pathlib import Path

import pytest
import yaml

from integ_diagnostics.domain import (
    domain_readiness,
    failure_entry,
    offline_entry,
    readiness_line,
)

RT = Path(__file__).resolve().parents[2]
TABLES = {
    "zeroerr_erob": yaml.safe_load(
        (RT / "hwcfg_machines/devices/faults/zeroerr_erob.yaml").read_text(encoding="utf-8")
    )
}
DRIVE = {
    "source": "arms/left_joint3/drive",
    "role": "drive",
    "state": {"al_state": "ethercat_slave_3/al_state", "error_code": "left_joint3/error_code"},
    "fault_table": "zeroerr_erob",
}
WARN = {
    "source": "arms/left_joint3/drive",
    "dree": "RT_DEVICE_FAULT_RESETTABLE",
    "severity": "WARN",
    "blocking": False,
    "detail": "0x603F: 0xA000",
}
DONE = {"pending": False, "entries": []}
UP = {"dual_arm_jtc": "active"}
HW = {"ecat_arms": "active"}


def enable(state, **values):
    return {"state": state, "stage": "success", "failed_joint": "", **values}


def test_idle_after_self_check_is_not_ready_and_keeps_its_warnings():
    fields = domain_readiness({"pending": False, "entries": [WARN]}, enable("IDLE"), UP, HW)
    assert not fields["ready"] and fields["status"] == "UNAVAILABLE"
    assert fields["blockers"] == ["not_enabled"] and fields["errors"] == [WARN]
    assert fields["operational_state"] == "idle"


def test_enabled_with_active_controllers_is_healthy():
    fields = domain_readiness({"pending": False, "entries": [WARN]}, enable("ENABLED"), UP, HW)
    assert fields["ready"] and fields["status"] == "HEALTHY" and fields["errors"] == []


@pytest.mark.parametrize(
    ("controllers", "hardware", "blocker"),
    [
        ({"dual_arm_jtc": "inactive"}, HW, "controller_inactive"),
        (UP, {"ecat_arms": "inactive"}, "hardware_inactive"),
    ],
)
def test_inactive_controllers_or_hardware_block(controllers, hardware, blocker):
    fields = domain_readiness(DONE, enable("ENABLED"), controllers, hardware)
    assert not fields["ready"] and fields["blockers"] == [blocker]


def test_pending_self_check_and_missing_enable_manager_block():
    fields = domain_readiness({"pending": True, "entries": []}, None, None, None)
    assert fields["blockers"] == ["enable_manager_unavailable", "self_check_pending"]


@pytest.mark.parametrize(
    ("stage", "code", "token", "dree", "text"),
    [
        ("device_offline", 0, "device_offline", "RT_DEVICE_OFFLINE", "stale ethercat_domain"),
        ("fault_detected", 0x2214, "device_fault", "RT_DEVICE_FAULT", "电机电流过流"),
        ("enable_batch_timeout", 0, "enable_failed", "RT_ENABLE_FAILED", "enable_batch_timeout"),
    ],
)
def test_latched_failure_names_the_device_and_the_vendor_code(stage, code, token, dree, text):
    latched = enable("FAILED", stage=stage, failed_joint="left_joint3")
    if stage == "device_offline":
        latched["offline_feedback"] = "ethercat_domain/process_data_age_ms"
    failure = failure_entry(latched, [DRIVE], TABLES, {"left_joint3/error_code": code})
    fields = domain_readiness(DONE, latched, UP, HW, failure)
    assert fields["blockers"] == [token]
    [error] = fields["errors"]
    assert (error["source"], error["dree"], error["severity"]) == (
        "arms/left_joint3/drive",
        dree,
        "FAULT",
    )
    assert text in error["detail"]


def test_stale_feedback_at_self_check_blocks_as_offline():
    stale = {**WARN, "dree": "RT_DEVICE_FEEDBACK_STALE", "severity": "FAULT", "blocking": True}
    fields = domain_readiness({"pending": False, "entries": [stale]}, enable("ENABLED"), UP, HW)
    assert fields["blockers"] == ["device_offline"] and fields["errors"] == [stale]


def test_log_line_ignores_enable_phase_changes():
    enabling = domain_readiness(DONE, enable("ENABLING"), UP, HW)
    waiting = domain_readiness(DONE, enable("INTER_BATCH_DELAY"), UP, HW)
    assert enabling != waiting
    assert (
        readiness_line(enabling)
        == readiness_line(waiting)
        == ("RT_READINESS ready=false status=UNAVAILABLE blockers=not_enabled")
    )


def test_slave_outside_op_blocks_while_idle_and_names_the_drive():
    idle = enable("IDLE", offline_now="ethercat_slave_3/al_state")
    offline = offline_entry(idle, [DRIVE])
    fields = domain_readiness(DONE, idle, UP, HW, offline=offline)
    assert fields["blockers"] == ["device_offline", "not_enabled"]
    [error] = fields["errors"]
    assert (error["source"], error["dree"]) == ("arms/left_joint3/drive", "RT_DEVICE_OFFLINE")
    assert "not in OP" in error["detail"]


def test_restart_latch_is_a_blocker_and_the_recorded_source_is_not_repeated():
    latched = enable(
        "FAILED",
        stage="device_offline",
        offline_feedback="ethercat_slave_3/al_state",
        offline_now="ethercat_slave_3/al_state",
        restart_required="true",
    )
    assert offline_entry(latched, [DRIVE]) is None
    failure = failure_entry(latched, [DRIVE], TABLES, {})
    fields = domain_readiness(DONE, latched, UP, HW, failure)
    assert fields["blockers"] == ["device_offline", "restart_required"]
    assert [e["source"] for e in fields["errors"]] == ["arms/left_joint3/drive"]


def test_latched_kinco_failure_reports_the_error_word_bits():
    tables = {
        "kinco_fd": yaml.safe_load(
            (RT / "hwcfg_machines/devices/faults/kinco_fd.yaml").read_text(encoding="utf-8")
        )
    }
    drive = {
        "source": "swerve_chassis/wheel1_joint/drive",
        "role": "drive",
        "state": {
            "al_state": "ethercat_slave_1/al_state",
            "error_word_1": "wheel1_joint/error_word_1",
            "error_word_2": "wheel1_joint/error_word_2",
        },
        "fault_table": "kinco_fd",
        "motor_encoder_type": "incremental",
    }
    latched = enable("FAILED", stage="fault_detected", failed_joint="wheel1_joint")
    values = {"wheel1_joint/error_word_1": 1 << 13, "wheel1_joint/error_word_2": 0}
    token, entry = failure_entry(latched, [drive], tables, values)
    assert (token, entry["source"], entry["dree"]) == (
        "device_fault",
        "swerve_chassis/wheel1_joint/drive",
        "RT_DEVICE_FAULT",
    )
    assert "0x2601 bit13 电机温度过高" in entry["detail"]
