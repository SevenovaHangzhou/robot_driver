from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Protocol

from plc_node.plc_logic import (
    REMOTE_CONTROL_ENABLE_MASK,
    PlcIoSnapshot,
    decode_io_snapshot,
    enable_remote_control_word,
    output_matches,
    set_output_bit,
)


class RegisterClient(Protocol):
    def read_holding_registers(self, start: int, count: int) -> list[int]: ...

    def write_single_register(self, address: int, value: int) -> None: ...


class PlcControlError(RuntimeError):
    pass


@dataclass(frozen=True)
class OutputWriteResult:
    command_word: int
    status_word: int


def repair_remote_control(client: RegisterClient, control_register: int) -> bool:
    """Ensure MW201 bit0 is set without changing any other control bit."""
    current_word = client.read_holding_registers(control_register, 1)[0]
    desired_word = enable_remote_control_word(current_word)
    if desired_word == current_word:
        return False

    client.write_single_register(control_register, desired_word)
    readback = client.read_holding_registers(control_register, 1)[0]
    if readback & REMOTE_CONTROL_ENABLE_MASK != REMOTE_CONTROL_ENABLE_MASK:
        raise PlcControlError(
            "remote control enable readback failed: "
            f"MW{control_register}=0x{readback:04x}"
        )
    if readback != desired_word:
        raise PlcControlError(
            "remote control readback mismatch: "
            f"MW{control_register}=0x{readback:04x}, "
            f"expected=0x{desired_word:04x}"
        )
    return True


def write_output_bit(
    client: RegisterClient,
    *,
    bit_index: int,
    enabled: bool,
    command_register: int,
    status_register: int,
    verify_timeout_s: float,
    verify_period_s: float,
) -> OutputWriteResult:
    """Modify one output bit and require command and physical-state agreement."""
    timeout_s = float(verify_timeout_s)
    period_s = float(verify_period_s)
    if not math.isfinite(timeout_s) or timeout_s < 0.0:
        raise ValueError("verify_timeout_s must be finite and non-negative")
    if not math.isfinite(period_s) or period_s <= 0.0:
        raise ValueError("verify_period_s must be finite and greater than zero")

    current_word = client.read_holding_registers(command_register, 1)[0]
    desired_word = set_output_bit(current_word, bit_index, enabled)
    client.write_single_register(command_register, desired_word)

    deadline_s = time.monotonic() + timeout_s
    command_readback = current_word
    status_readback = 0
    while True:
        command_readback = client.read_holding_registers(command_register, 1)[0]
        status_readback = client.read_holding_registers(status_register, 1)[0]
        if output_matches(command_readback, status_readback, bit_index, enabled):
            return OutputWriteResult(command_readback, status_readback)

        remaining_s = deadline_s - time.monotonic()
        if remaining_s <= 0.0:
            break
        time.sleep(min(period_s, remaining_s))

    raise PlcControlError(
        "output readback timed out: "
        f"bit={bit_index} expected={int(enabled)} "
        f"command=0x{command_readback:04x} status=0x{status_readback:04x}"
    )


def read_io_snapshot(
    client: RegisterClient,
    *,
    input_register: int,
    output_status_register: int,
    alarm_register: int,
) -> PlcIoSnapshot:
    if (
        output_status_register == input_register + 1
        and alarm_register == input_register + 2
    ):
        di_status, do_status, io_alarm = client.read_holding_registers(
            input_register, 3
        )
    else:
        di_status = client.read_holding_registers(input_register, 1)[0]
        do_status = client.read_holding_registers(output_status_register, 1)[0]
        io_alarm = client.read_holding_registers(alarm_register, 1)[0]
    return decode_io_snapshot(di_status, do_status, io_alarm)
