from pathlib import Path


SOURCE = (Path(__file__).resolve().parents[1] / "src" / "rt_diagnostics_node.cpp").read_text(
    encoding="utf-8"
)


def test_ethercat_axis_diagnostics_preserve_operator_fault_identity() -> None:
    for key in (
        'makeKeyValue("joint"',
        'makeKeyValue("ring_position"',
        'makeKeyValue("vendor"',
        'makeKeyValue("al_state_raw"',
        'makeKeyValue("status_word_raw"',
        'makeKeyValue("status_word_hex"',
    ):
        assert key in SOURCE


def test_existing_diagnostic_keys_remain_for_compatibility() -> None:
    for key in (
        'makeKeyValue("al_state"',
        'makeKeyValue("cia402_state"',
        'makeKeyValue("position"',
    ):
        assert key in SOURCE


def test_x503_rows_identify_sensor_without_pretending_it_is_a_joint() -> None:
    assert 'makeKeyValue("sensor", kSensorNames[sensor])' in SOURCE
    assert 'makeKeyValue("vendor", "X503")' in SOURCE
    assert 'makeKeyValue("ring_position", std::to_string(position))' in SOURCE
