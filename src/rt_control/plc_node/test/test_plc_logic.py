import pytest

from plc_node.plc_logic import (
    LEFT_SOLENOID_BIT,
    RIGHT_SOLENOID_BIT,
    VACUUM_PUMP_BIT,
    decode_io_snapshot,
    enable_remote_control_word,
    output_matches,
    set_output_bit,
)


def test_enable_remote_control_sets_only_bit_zero() -> None:
    assert enable_remote_control_word(0xA500) == 0xA501
    assert enable_remote_control_word(0xA501) == 0xA501


@pytest.mark.parametrize(
    ("bit", "enabled", "expected"),
    [
        (LEFT_SOLENOID_BIT, True, 0b101),
        (RIGHT_SOLENOID_BIT, True, 0b111),
        (VACUUM_PUMP_BIT, False, 0b001),
    ],
)
def test_set_output_bit_preserves_every_other_bit(bit: int, enabled: bool, expected: int) -> None:
    assert set_output_bit(0b101, bit, enabled) == expected


def test_set_output_bit_preserves_non_io_bits() -> None:
    assert set_output_bit(0xA505, RIGHT_SOLENOID_BIT, True) == 0xA507


def test_set_output_bit_rejects_unknown_output() -> None:
    with pytest.raises(ValueError, match="unsupported PLC output bit"):
        set_output_bit(0, 3, True)


def test_decode_snapshot_uses_confirmed_input_and_output_mapping() -> None:
    snapshot = decode_io_snapshot(di_status=0b01, do_status=0b110, io_alarm=2)

    assert snapshot.left_vacuum_established is True
    assert snapshot.right_vacuum_established is False
    assert snapshot.left_solenoid_on is False
    assert snapshot.right_solenoid_on is True
    assert snapshot.vacuum_pump_on is True
    assert snapshot.io_alarm == 2


@pytest.mark.parametrize("enabled", [False, True])
def test_output_matches_requires_command_and_actual_status(enabled: bool) -> None:
    value = 1 << LEFT_SOLENOID_BIT if enabled else 0
    opposite = 0 if enabled else 1 << LEFT_SOLENOID_BIT

    assert output_matches(value, value, LEFT_SOLENOID_BIT, enabled)
    assert not output_matches(opposite, value, LEFT_SOLENOID_BIT, enabled)
    assert not output_matches(value, opposite, LEFT_SOLENOID_BIT, enabled)
