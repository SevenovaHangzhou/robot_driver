import struct

import pytest

from plc_node.modbus_tcp import ModbusError, ModbusTcpClient


class FakeSocket:
    def __init__(self, response: bytes):
        self.response = bytearray(response)
        self.sent = b""

    def sendall(self, data: bytes) -> None:
        self.sent += data

    def recv(self, length: int) -> bytes:
        chunk = bytes(self.response[:length])
        del self.response[:length]
        return chunk

    def close(self) -> None:
        pass


def _response(function: int, payload: bytes, *, transaction: int = 1) -> bytes:
    pdu = bytes([function]) + payload
    return struct.pack(">HHHB", transaction, 0, len(pdu) + 1, 1) + pdu


def test_read_holding_registers_parses_exact_response() -> None:
    client = ModbusTcpClient("127.0.0.1", 502, 1, 1.0)
    client._sock = FakeSocket(_response(0x03, b"\x04\x12\x34\xab\xcd"))

    assert client.read_holding_registers(210, 2) == [0x1234, 0xABCD]


def test_read_holding_registers_rejects_malformed_payload_as_modbus_error() -> None:
    client = ModbusTcpClient("127.0.0.1", 502, 1, 1.0)
    client._sock = FakeSocket(_response(0x03, b""))

    with pytest.raises(ModbusError, match="missing byte count"):
        client.read_holding_registers(210, 1)


def test_request_rejects_invalid_mbap_length_before_receiving_payload() -> None:
    client = ModbusTcpClient("127.0.0.1", 502, 1, 1.0)
    client._sock = FakeSocket(struct.pack(">HHHB", 1, 0, 1, 1))

    with pytest.raises(ModbusError, match="invalid MBAP length"):
        client.read_holding_registers(210, 1)


def test_write_register_requires_exact_echo() -> None:
    client = ModbusTcpClient("127.0.0.1", 502, 1, 1.0)
    client._sock = FakeSocket(_response(0x06, struct.pack(">HH", 200, 2)))

    with pytest.raises(ModbusError, match="write echo mismatch"):
        client.write_single_register(200, 1)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"host": "", "port": 502, "unit_id": 1, "timeout": 1.0},
        {"host": "plc", "port": 0, "unit_id": 1, "timeout": 1.0},
        {"host": "plc", "port": 502, "unit_id": 256, "timeout": 1.0},
        {"host": "plc", "port": 502, "unit_id": 1, "timeout": 0.0},
    ],
)
def test_client_rejects_invalid_connection_parameters(kwargs) -> None:
    with pytest.raises(ValueError):
        ModbusTcpClient(**kwargs)
