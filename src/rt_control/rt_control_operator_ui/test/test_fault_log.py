import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rt_control_operator_ui.fault_log import FaultEvent, FaultIncidentModel  # noqa: E402


def event(
    timestamp_s: float,
    *,
    source: str = "ethercat/slave_8",
    joint: str = "left_joint2",
    code: int = 0x7500,
    message: str = "通信错误",
    active: bool = True,
) -> FaultEvent:
    return FaultEvent(
        timestamp_s=timestamp_s,
        source=source,
        joint=joint,
        address=8,
        vendor="TI5",
        status_word=0x1218,
        error_code=code,
        message=message,
        severity="error",
        reset_policy="reset_once_after_link_recovered",
        active=active,
    )


def test_first_fault_remains_pinned_when_cascaded_faults_arrive() -> None:
    model = FaultIncidentModel(max_rows=100)
    first = event(1.0, source="ethercat/master", joint="", code=0, message="AL超时")
    cascade = event(1.1)

    model.ingest(first)
    model.ingest(cascade)

    assert model.first_fault == first
    assert [row.event for row in model.rows] == [first, cascade]


def test_identical_faults_update_count_and_timestamps_without_adding_rows() -> None:
    model = FaultIncidentModel(max_rows=100)

    for index in range(10_000):
        model.ingest(event(float(index)))

    assert len(model.rows) == 1
    row = model.rows[0]
    assert row.repeat_count == 10_000
    assert row.first_timestamp_s == 0.0
    assert row.last_timestamp_s == 9_999.0


def test_recovery_marks_existing_row_without_deleting_history() -> None:
    model = FaultIncidentModel(max_rows=100)
    model.ingest(event(1.0))

    model.ingest(event(2.0, active=False))

    assert len(model.rows) == 1
    assert model.rows[0].active is False
    assert model.rows[0].repeat_count == 1
    assert model.rows[0].last_timestamp_s == 2.0


def test_row_limit_evicts_old_non_first_fault_but_never_the_first_fault() -> None:
    model = FaultIncidentModel(max_rows=3)
    first = event(1.0, source="ethercat/master", joint="", code=0, message="Link DOWN")
    model.ingest(first)
    for index in range(2, 7):
        model.ingest(
            event(
                float(index),
                source=f"ethercat/slave_{index}",
                joint=f"joint_{index}",
                code=0xA000 + index,
            )
        )

    assert len(model.rows) == 3
    assert model.rows[0].event == first
    assert model.first_fault == first
