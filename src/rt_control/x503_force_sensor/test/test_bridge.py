from types import SimpleNamespace

from x503_force_sensor.bridge import (
    CalibrationSnapshot,
    CONFIRMED_ENGINEERING_UNIT_CONTRACT,
    calibration_from_values,
    convert_to_wrench,
    extract_sensor_frame,
)


def _message(values):
    names = [
        *(f"channel_{index}_raw" for index in range(1, 7)),
        *(f"sample_code_{index}_raw" for index in range(1, 7)),
    ]
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=3)),
        joint_names=["right_force_sensor"],
        interface_values=[
            SimpleNamespace(interface_names=names, values=values)
        ],
    )


def test_extracts_all_channels_from_one_complete_frame():
    frame = extract_sensor_frame(
        _message(list(range(1, 13))), "right_force_sensor"
    )

    assert frame is not None
    assert frame.raw_values == (1, 2, 3, 4, 5, 6)
    assert frame.sample_codes == (7, 8, 9, 10, 11, 12)
    assert frame.stamp.sec == 12


def test_rejects_partial_or_non_integral_frame():
    values = list(range(1, 12))
    assert extract_sensor_frame(_message(values), "right_force_sensor") is None
    values = list(range(1, 13))
    values[2] = 1.5
    assert extract_sensor_frame(_message(values), "right_force_sensor") is None
    values[2] = 10**400
    assert extract_sensor_frame(_message(values), "right_force_sensor") is None


def test_accepts_signed_dint_boundaries():
    values = [
        -2147483648,
        -1,
        0,
        1,
        2147483647,
        -42,
        1,
        2,
        3,
        4,
        5,
        6,
    ]
    frame = extract_sensor_frame(_message(values), "right_force_sensor")

    assert frame is not None
    assert frame.raw_values == tuple(values[:6])


def test_raw_values_do_not_become_wrench_without_readback():
    frame = extract_sensor_frame(
        _message(list(range(1, 13))), "right_force_sensor"
    )
    assert frame is not None
    calibration = CalibrationSnapshot(
        False, (1,) * 6, (100,) * 6, "unresolved"
    )
    assert convert_to_wrench(frame, calibration) is None


def test_converts_only_validated_units_and_decimals():
    frame = extract_sensor_frame(
        _message([100, 200, 300, 400, 500, 600, 1, 2, 3, 4, 5, 6]),
        "right_force_sensor",
    )
    assert frame is not None
    calibration = CalibrationSnapshot(
        True,
        (1, 1, 1, 2, 2, 2),
        (10, 11, 12, 13, 14, 15),
        "sample_codes_equal",
        (1, 2, 3, 4, 5, 6),
        CONFIRMED_ENGINEERING_UNIT_CONTRACT,
    )
    assert convert_to_wrench(frame, calibration) == (
        10.0,
        20.0,
        30.0,
        4.0,
        5.0,
        6.0,
    )


def test_accepts_the_manual_decimal_upper_bound():
    frame = extract_sensor_frame(
        _message([1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6]),
        "right_force_sensor",
    )
    assert frame is not None
    calibration = CalibrationSnapshot(
        True,
        (10, 10, 10, 10, 10, 10),
        (5, 5, 5, 7, 7, 7),
        "sample_codes_equal",
        (1, 2, 3, 4, 5, 6),
        CONFIRMED_ENGINEERING_UNIT_CONTRACT,
    )
    assert convert_to_wrench(frame, calibration) == (
        1e-10,
        2e-10,
        3e-10,
        4e-10,
        5e-10,
        6e-10,
    )


def test_accepts_sample_codes_in_the_manual_range():
    frame = extract_sensor_frame(
        _message([100, 200, 300, 400, 500, 600, 7, 8, 9, 10, 11, 12]),
        "right_force_sensor",
    )
    assert frame is not None
    calibration = CalibrationSnapshot(
        True,
        (1, 1, 1, 2, 2, 2),
        (5, 5, 5, 7, 7, 7),
        "sample_codes_in_range",
        (),
        CONFIRMED_ENGINEERING_UNIT_CONTRACT,
        -999999,
        999999,
    )
    assert convert_to_wrench(frame, calibration) == (
        10.0,
        20.0,
        30.0,
        4.0,
        5.0,
        6.0,
    )


def test_rejects_sample_codes_outside_the_manual_range():
    frame = extract_sensor_frame(
        _message([100, 200, 300, 400, 500, 600, 7, 8, 9, 10, 11, 12]),
        "right_force_sensor",
    )
    assert frame is not None
    calibration = CalibrationSnapshot(
        True,
        (1, 1, 1, 2, 2, 2),
        (5, 5, 5, 7, 7, 7),
        "sample_codes_in_range",
        (),
        CONFIRMED_ENGINEERING_UNIT_CONTRACT,
        -5,
        5,
    )
    assert convert_to_wrench(frame, calibration) is None


def test_rejects_unconfirmed_engineering_unit_contract():
    frame = extract_sensor_frame(
        _message(list(range(1, 13))), "right_force_sensor"
    )
    assert frame is not None
    calibration = CalibrationSnapshot(
        True,
        (0,) * 6,
        (5, 5, 5, 7, 7, 7),
        "sample_codes_equal",
        (1, 2, 3, 4, 5, 6),
    )
    assert convert_to_wrench(frame, calibration) is None


def test_parses_calibration_snapshot_values():
    values = {
        **{f"decimal_{index}": "1" for index in range(1, 7)},
        **{f"unit_{index}": str(index) for index in range(1, 7)},
        "snapshot_valid": "true",
        "engineering_unit_contract": CONFIRMED_ENGINEERING_UNIT_CONTRACT,
        "validity_policy": "sample_codes_equal",
        **{f"valid_sample_code_{index}": str(index) for index in range(1, 7)},
    }
    snapshot = calibration_from_values(values)
    assert snapshot.engineering_units_valid
    assert snapshot.sample_validity_confirmed


def test_malformed_sample_code_snapshot_fails_closed():
    values = {
        **{f"decimal_{index}": "1" for index in range(1, 7)},
        **{f"unit_{index}": str(index) for index in range(1, 7)},
        "snapshot_valid": "true",
        "engineering_unit_contract": CONFIRMED_ENGINEERING_UNIT_CONTRACT,
        "validity_policy": "sample_codes_equal",
        **{
            f"valid_sample_code_{index}": "bad" for index in range(1, 7)
        },
    }
    snapshot = calibration_from_values(values)
    assert not snapshot.sample_validity_confirmed


def test_out_of_range_unit_code_fails_closed():
    values = {
        **{f"decimal_{index}": "1" for index in range(1, 7)},
        **{f"unit_{index}": "1" for index in range(1, 6)},
        "unit_6": str(2**32),
        "snapshot_valid": "true",
        "engineering_unit_contract": CONFIRMED_ENGINEERING_UNIT_CONTRACT,
        "validity_policy": "sample_codes_equal",
        **{f"valid_sample_code_{index}": str(index) for index in range(1, 7)},
    }
    snapshot = calibration_from_values(values)
    assert not snapshot.engineering_units_valid
