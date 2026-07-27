import math

from bms_node.bms_protocol import (
    BASIC_STATUS_FRAME_ID,
    BmsSample,
    ingest_basic_frame,
    project_battery_values,
)


def test_basic_frame_decodes_voltage_and_soc_fraction() -> None:
    sample = BmsSample()

    assert ingest_basic_frame(
        sample,
        BASIC_STATUS_FRAME_ID,
        bytes([0x01, 0xF4, 0xFF, 0x9C, 80, 95, 0x00, 0x0A]),
        now_s=10.0,
    )

    assert sample.voltage_v == 50.0
    assert sample.soc_fraction == 0.8
    assert sample.last_frame_s == 10.0


def test_basic_frame_accepts_exact_soc_boundaries() -> None:
    empty = BmsSample()
    full = BmsSample()

    assert ingest_basic_frame(empty, BASIC_STATUS_FRAME_ID, b"\x00\x64\x00\x00\x00", 1.0)
    assert ingest_basic_frame(full, BASIC_STATUS_FRAME_ID, b"\x00\x64\x00\x00\x64", 1.0)

    assert empty.soc_fraction == 0.0
    assert full.soc_fraction == 1.0


def test_invalid_frame_does_not_mutate_sample() -> None:
    sample = BmsSample(voltage_v=48.0, soc_fraction=0.5, last_frame_s=2.0)

    assert not ingest_basic_frame(sample, 0x5FC, b"\x00" * 8, 3.0)
    assert not ingest_basic_frame(sample, BASIC_STATUS_FRAME_ID, b"\x00" * 4, 3.0)

    assert sample == BmsSample(voltage_v=48.0, soc_fraction=0.5, last_frame_s=2.0)


def test_out_of_range_soc_is_rejected_without_mutation() -> None:
    sample = BmsSample(voltage_v=48.0, soc_fraction=0.5, last_frame_s=2.0)

    assert not ingest_basic_frame(
        sample,
        BASIC_STATUS_FRAME_ID,
        bytes([0x01, 0xE0, 0x00, 0x00, 101]),
        3.0,
    )

    assert sample == BmsSample(voltage_v=48.0, soc_fraction=0.5, last_frame_s=2.0)


def test_non_finite_receive_timestamp_is_rejected_without_mutation() -> None:
    sample = BmsSample(voltage_v=48.0, soc_fraction=0.5, last_frame_s=2.0)

    assert not ingest_basic_frame(
        sample,
        BASIC_STATUS_FRAME_ID,
        bytes([0x01, 0xE0, 0x00, 0x00, 80]),
        math.nan,
    )

    assert sample == BmsSample(voltage_v=48.0, soc_fraction=0.5, last_frame_s=2.0)


def test_projection_returns_nan_when_sample_is_missing_or_stale() -> None:
    missing_voltage, missing_soc = project_battery_values(BmsSample(), 10.0, 3.0)
    stale_voltage, stale_soc = project_battery_values(
        BmsSample(voltage_v=50.0, soc_fraction=0.8, last_frame_s=5.0),
        10.0,
        3.0,
    )

    assert math.isnan(missing_voltage)
    assert math.isnan(missing_soc)
    assert math.isnan(stale_voltage)
    assert math.isnan(stale_soc)


def test_projection_rejects_future_timestamp_and_invalid_timeout() -> None:
    sample = BmsSample(voltage_v=50.0, soc_fraction=0.8, last_frame_s=11.0)

    for timeout_s in (0.0, -1.0, math.nan):
        voltage, soc = project_battery_values(sample, 10.0, timeout_s)
        assert math.isnan(voltage)
        assert math.isnan(soc)


def test_projection_rejects_invalid_values_as_one_atomic_sample() -> None:
    for sample in (
        BmsSample(voltage_v=math.nan, soc_fraction=0.8, last_frame_s=9.0),
        BmsSample(voltage_v=50.0, soc_fraction=1.1, last_frame_s=9.0),
    ):
        voltage, soc = project_battery_values(sample, 10.0, 3.0)
        assert math.isnan(voltage)
        assert math.isnan(soc)
