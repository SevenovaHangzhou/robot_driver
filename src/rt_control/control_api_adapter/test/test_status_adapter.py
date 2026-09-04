import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from control_api_adapter.status_adapter import (  # noqa: E402
    BatteryHealthSnapshot,
    CANOPEN_SUMMARY_NAME,
    ComponentSnapshot,
    ETHERCAT_SUMMARY_NAME,
    PlcHealthSnapshot,
    _diagnostic_level,
    build_safety_summary,
    populate_domain_readiness,
)


def ok_component(name: str, **values: str) -> ComponentSnapshot:
    return ComponentSnapshot(
        name=name,
        level=0,
        fresh=True,
        message="healthy",
        values=values,
    )


def test_safety_summary_requires_enabled_control_and_all_approved_sources() -> None:
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager",
            state="ENABLED",
            stage="success",
        ),
        ethercat=ok_component(ETHERCAT_SUMMARY_NAME),
        canopen=ok_component(CANOPEN_SUMMARY_NAME),
        plc=PlcHealthSnapshot(connected=True, data_fresh=True),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )

    assert summary.safe_to_start_motion
    assert summary.control_enabled
    assert summary.state == "READY"


def test_safety_summary_rejects_plc_remote_control_error() -> None:
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager",
            state="ENABLED",
            stage="success",
        ),
        ethercat=ok_component(ETHERCAT_SUMMARY_NAME),
        canopen=ok_component(CANOPEN_SUMMARY_NAME),
        plc=PlcHealthSnapshot(
            connected=True,
            data_fresh=True,
            error="remote control unavailable: injected failure",
        ),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )

    assert not summary.safe_to_start_motion
    assert not summary.plc_ok
    assert summary.state == "NOT_READY"
    assert summary.active_faults == (
        "plc: remote control unavailable: injected failure",
    )


@pytest.mark.parametrize(
    ("enable_state", "enable_stage"),
    [
        ("ENABLED", ""),
        ("ENABLED", "fault_detected"),
        ("ENABLED", "operation_in_progress"),
        ("ENABLED", "jtc_activate_failed"),
        ("ENABLED", "fault_requires_reset"),
        ("UNKNOWN", "success"),
    ],
)
def test_safety_summary_rejects_invalid_enable_manager_stable_state(
    enable_state: str,
    enable_stage: str,
) -> None:
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager",
            state=enable_state,
            stage=enable_stage,
        ),
        ethercat=ok_component(ETHERCAT_SUMMARY_NAME),
        canopen=ok_component(CANOPEN_SUMMARY_NAME),
        plc=PlcHealthSnapshot(connected=True, data_fresh=True),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )

    assert not summary.safe_to_start_motion
    assert not summary.control_enabled
    assert not summary.enable_manager_ok
    assert summary.state == "NOT_READY"
    assert summary.active_faults == (
        f"enable_manager: invalid state/stage {enable_state!r}/{enable_stage!r}",
    )


def test_safety_summary_reports_canopen_fault() -> None:
    canopen = ComponentSnapshot(
        name=CANOPEN_SUMMARY_NAME,
        level=2,
        fresh=True,
        message="Fault",
        values={},
    )
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager",
            state="ENABLED",
            stage="success",
        ),
        ethercat=ok_component(ETHERCAT_SUMMARY_NAME),
        canopen=canopen,
        plc=PlcHealthSnapshot(connected=True, data_fresh=True),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )

    assert not summary.safe_to_start_motion
    assert not summary.canopen_ok
    assert f"{CANOPEN_SUMMARY_NAME}: Fault" in summary.active_faults


def test_diagnostic_level_accepts_ros_python_byte_field() -> None:
    assert _diagnostic_level(0) == 0
    assert _diagnostic_level(b"\x00") == 0
    assert _diagnostic_level(b"\x02") == 2
    assert _diagnostic_level(b"\x03") == 3
    assert _diagnostic_level(b"") == 2


def test_domain_readiness_uses_vendored_schema() -> None:
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager",
            state="ENABLED",
            stage="success",
        ),
        ethercat=ok_component(ETHERCAT_SUMMARY_NAME),
        canopen=ComponentSnapshot(
            CANOPEN_SUMMARY_NAME, 2, True, "Fault", {}
        ),
        plc=PlcHealthSnapshot(connected=True, data_fresh=True),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )
    message = SimpleNamespace(
        STATUS_HEALTHY="HEALTHY",
        STATUS_DEGRADED="DEGRADED",
        STATUS_UNAVAILABLE="UNAVAILABLE",
        header=None,
        domain="",
        readiness_name="",
        ready=True,
        status="",
        operational_state="",
        map_version="unexpected",
        producer_instance_id="",
        blockers=[],
        errors=["unexpected"],
    )

    populate_domain_readiness(
        message,
        header="header",
        summary=summary,
        producer_instance_id="instance-1",
    )

    assert message.header == "header"
    assert message.domain == "rt_control"
    assert message.readiness_name == "rt_control"
    assert not message.ready
    assert message.status == message.STATUS_UNAVAILABLE
    assert message.operational_state == "NOT_READY"
    assert message.map_version == ""
    assert message.producer_instance_id == "instance-1"
    assert message.blockers == ["canopen_unavailable"]
    assert message.errors == []


def test_domain_readiness_healthy_state_has_no_blockers_or_errors() -> None:
    summary = build_safety_summary(
        enable_manager=ok_component(
            "/robot/rt_control/enable_manager",
            state="ENABLED",
            stage="success",
        ),
        ethercat=ok_component(ETHERCAT_SUMMARY_NAME),
        canopen=ok_component(CANOPEN_SUMMARY_NAME),
        plc=PlcHealthSnapshot(connected=True, data_fresh=True),
        bms=BatteryHealthSnapshot(present=True, fresh=True),
    )
    message = SimpleNamespace(
        STATUS_HEALTHY="HEALTHY",
        STATUS_DEGRADED="DEGRADED",
        STATUS_UNAVAILABLE="UNAVAILABLE",
        blockers=["unexpected"],
        errors=["unexpected"],
    )

    populate_domain_readiness(
        message,
        header="header",
        summary=summary,
        producer_instance_id="instance-1",
    )

    assert message.ready
    assert message.status == message.STATUS_HEALTHY
    assert message.map_version == ""
    assert message.blockers == []
    assert message.errors == []
