from x503_force_sensor.calibration import (
    CONFIRMED_ENGINEERING_UNIT_CONTRACT,
    calibration_from_values,
)


def values():
    return {
        **{f"decimal_{index}": "1" for index in range(1, 7)},
        **{f"unit_{index}": str(5 if index <= 3 else 7) for index in range(1, 7)},
        "snapshot_valid": "true",
        "engineering_unit_contract": CONFIRMED_ENGINEERING_UNIT_CONTRACT,
        "validity_policy": "sample_codes_in_range",
        "sample_code_min": "-999999",
        "sample_code_max": "999999",
    }


def test_parses_verified_engineering_metadata():
    snapshot = calibration_from_values(values())

    assert snapshot.engineering_units_valid
    assert snapshot.sample_validity_confirmed
    assert snapshot.units == (5, 5, 5, 7, 7, 7)


def test_missing_or_out_of_range_metadata_fails_closed():
    missing = values()
    missing.pop("decimal_1")
    assert not calibration_from_values(missing).engineering_units_valid

    invalid = values()
    invalid["sample_code_max"] = str(2**31)
    assert not calibration_from_values(invalid).sample_validity_confirmed


def test_unconfirmed_unit_contract_fails_closed():
    unconfirmed = values()
    unconfirmed["engineering_unit_contract"] = "unresolved"

    assert not calibration_from_values(unconfirmed).engineering_units_valid
