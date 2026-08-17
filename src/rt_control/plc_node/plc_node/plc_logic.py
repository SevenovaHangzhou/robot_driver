from __future__ import annotations

from dataclasses import dataclass


LEFT_SOLENOID_BIT = 1
RIGHT_SOLENOID_BIT = 0
VACUUM_PUMP_BIT = 2
SUPPORTED_OUTPUT_BITS = frozenset(
    (LEFT_SOLENOID_BIT, RIGHT_SOLENOID_BIT, VACUUM_PUMP_BIT)
)
REMOTE_CONTROL_ENABLE_MASK = 1


@dataclass(frozen=True)
class PlcIoSnapshot:
    left_vacuum_established: bool
    right_vacuum_established: bool
    left_solenoid_on: bool
    right_solenoid_on: bool
    vacuum_pump_on: bool
    io_alarm: int


def enable_remote_control_word(control_word: int) -> int:
    return (int(control_word) | REMOTE_CONTROL_ENABLE_MASK) & 0xFFFF


def set_output_bit(current_word: int, bit_index: int, enabled: bool) -> int:
    if bit_index not in SUPPORTED_OUTPUT_BITS:
        raise ValueError(f"unsupported PLC output bit: {bit_index}")
    bit_mask = 1 << bit_index
    if enabled:
        return (int(current_word) | bit_mask) & 0xFFFF
    return int(current_word) & ~bit_mask & 0xFFFF


def output_matches(
    command_word: int,
    status_word: int,
    bit_index: int,
    expected: bool,
) -> bool:
    if bit_index not in SUPPORTED_OUTPUT_BITS:
        raise ValueError(f"unsupported PLC output bit: {bit_index}")
    bit_mask = 1 << bit_index
    expected_bit = bit_mask if expected else 0
    return (
        int(command_word) & bit_mask == expected_bit
        and int(status_word) & bit_mask == expected_bit
    )


def decode_io_snapshot(di_status: int, do_status: int, io_alarm: int) -> PlcIoSnapshot:
    return PlcIoSnapshot(
        left_vacuum_established=bool(int(di_status) & (1 << 0)),
        right_vacuum_established=bool(int(di_status) & (1 << 1)),
        left_solenoid_on=bool(int(do_status) & (1 << LEFT_SOLENOID_BIT)),
        right_solenoid_on=bool(int(do_status) & (1 << RIGHT_SOLENOID_BIT)),
        vacuum_pump_on=bool(int(do_status) & (1 << VACUUM_PUMP_BIT)),
        io_alarm=int(io_alarm) & 0xFFFF,
    )
