import sys
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rt_control_operator_ui.diagnostic_adapter import DiagnosticFaultProjector  # noqa: E402
from rt_control_operator_ui.fault_catalog import FaultCatalog  # noqa: E402


def status(
    *,
    name: str = "/robot/rt_control/ethercat/slave_8",
    level: int = 2,
    message: str = "CiA402 fault",
    values: dict[str, str] | None = None,
):
    return SimpleNamespace(
        name=name,
        level=level,
        message=message,
        values=[
            SimpleNamespace(key=key, value=value)
            for key, value in (values or {}).items()
        ],
    )


def projector() -> DiagnosticFaultProjector:
    catalog = FaultCatalog.load_directory(PACKAGE_ROOT / "config" / "fault_catalog")
    return DiagnosticFaultProjector(catalog)


def test_ti5_diagnostic_projects_raw_values_and_manual_explanation() -> None:
    event = projector().project(
        status(
            values={
                "joint": "left_joint2",
                "position": "8",
                "vendor": "TI5",
                "status_word_hex": "0x1218",
                "error_code_hex": "0x7500",
            }
        ),
        timestamp_s=10.0,
    )

    assert event is not None
    assert event.joint == "left_joint2"
    assert event.address == 8
    assert event.status_word == 0x1218
    assert event.error_code == 0x7500
    assert event.message == "通信错误"
    assert event.reset_policy == "reset_once_after_link_recovered"
    assert "EtherCAT 异常从 OP 切换到非 OP" in event.possible_causes
    assert "确认 EtherCAT Link UP" in event.operator_actions
    assert event.manual_version == "1.1.1"
    assert event.manual_pages == "77-78"


def test_enable_manager_failed_statusword_is_preserved_without_guessing_vendor_code() -> None:
    event = projector().project(
        status(
            name="/robot/rt_control/enable_manager",
            message="failed",
            values={"failed_joint": "right_joint2", "failed_status_word": "4616"},
        ),
        timestamp_s=11.0,
    )

    assert event is not None
    assert event.joint == "right_joint2"
    assert event.status_word == 0x1208
    assert event.error_code == 0
    assert "failed" in event.message
    assert event.reset_policy == "do_not_reset"


def test_ok_status_emits_recovery_only_for_a_previously_active_source() -> None:
    adapter = projector()
    source = "/robot/rt_control/ethercat/slave_8"
    assert adapter.project(status(name=source, level=0, message="healthy"), 1.0) is None

    active = adapter.project(
        status(
            name=source,
            values={
                "joint": "left_joint2",
                "position": "8",
                "vendor": "TI5",
                "status_word_hex": "0x1218",
                "error_code_hex": "0x7500",
            },
        ),
        2.0,
    )
    recovered = adapter.project(status(name=source, level=0, message="healthy"), 3.0)

    assert active is not None and active.active
    assert recovered is not None and not recovered.active
    assert recovered.fingerprint == active.fingerprint


def test_master_link_failure_uses_diagnostic_text_when_no_vendor_code_exists() -> None:
    event = projector().project(
        status(
            name="/robot/rt_control/ethercat/master",
            message="EtherCAT link is down",
            values={"link": "0", "slaves_responding": "0", "wc_error_count": "42"},
        ),
        4.0,
    )

    assert event is not None
    assert event.source == "/robot/rt_control/ethercat/master"
    assert event.error_code == 0
    assert event.message == "EtherCAT link is down"
