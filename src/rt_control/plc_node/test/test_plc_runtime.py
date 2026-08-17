import pytest

from plc_node.plc_logic import LEFT_SOLENOID_BIT, RIGHT_SOLENOID_BIT
from plc_node.plc_runtime import (
    PlcControlError,
    read_io_snapshot,
    repair_remote_control,
    write_output_bit,
)


class FakeRegisterClient:
    def __init__(self, registers=None, mirror_output=True, ignore_control_write=False):
        self.registers = dict(registers or {})
        self.mirror_output = mirror_output
        self.ignore_control_write = ignore_control_write
        self.writes = []

    def read_holding_registers(self, start, count):
        return [self.registers.get(address, 0) for address in range(start, start + count)]

    def write_single_register(self, address, value):
        self.writes.append((address, value))
        if address == 201 and self.ignore_control_write:
            return
        self.registers[address] = value
        if address == 200 and self.mirror_output:
            self.registers[211] = value


def test_repair_remote_control_changes_only_bit_zero_and_verifies_readback() -> None:
    client = FakeRegisterClient({201: 0xA500})

    changed = repair_remote_control(client, control_register=201)

    assert changed is True
    assert client.registers[201] == 0xA501
    assert client.writes == [(201, 0xA501)]


def test_repair_remote_control_does_not_write_when_already_enabled() -> None:
    client = FakeRegisterClient({201: 0xA501})

    assert repair_remote_control(client, control_register=201) is False
    assert client.writes == []


def test_repair_remote_control_rejects_failed_readback() -> None:
    client = FakeRegisterClient({201: 0xA500}, ignore_control_write=True)

    with pytest.raises(PlcControlError, match="remote control enable readback failed"):
        repair_remote_control(client, control_register=201)


def test_repair_remote_control_rejects_unrelated_bit_readback_change() -> None:
    class MutatingControlClient(FakeRegisterClient):
        def write_single_register(self, address, value):
            super().write_single_register(address, value)
            if address == 201:
                self.registers[address] ^= 1 << 1

    client = MutatingControlClient({201: 0xA500})

    with pytest.raises(PlcControlError, match="remote control readback mismatch"):
        repair_remote_control(client, control_register=201)


def test_write_output_changes_only_requested_bit_and_requires_both_readbacks() -> None:
    client = FakeRegisterClient({200: 0xA507, 201: 1, 211: 0xA507})

    result = write_output_bit(
        client,
        bit_index=RIGHT_SOLENOID_BIT,
        enabled=False,
        command_register=200,
        status_register=211,
        verify_timeout_s=0.0,
        verify_period_s=0.01,
    )

    assert result.command_word == 0xA506
    assert result.status_word == 0xA506
    assert client.writes == [(200, 0xA506)]


def test_write_output_fails_without_compensating_write_when_status_disagrees() -> None:
    client = FakeRegisterClient(
        {200: 0xA500, 201: 1, 211: 0xA500},
        mirror_output=False,
    )

    with pytest.raises(PlcControlError, match="output readback timed out"):
        write_output_bit(
            client,
            bit_index=LEFT_SOLENOID_BIT,
            enabled=True,
            command_register=200,
            status_register=211,
            verify_timeout_s=0.0,
            verify_period_s=0.01,
        )

    assert client.writes == [(200, 0xA502)]


def test_read_io_snapshot_uses_three_confirmed_status_registers() -> None:
    client = FakeRegisterClient({210: 0b10, 211: 0b101, 212: 7})

    snapshot = read_io_snapshot(
        client,
        input_register=210,
        output_status_register=211,
        alarm_register=212,
    )

    assert not snapshot.left_vacuum_established
    assert snapshot.right_vacuum_established
    assert not snapshot.left_solenoid_on
    assert snapshot.right_solenoid_on
    assert snapshot.vacuum_pump_on
    assert snapshot.io_alarm == 7
