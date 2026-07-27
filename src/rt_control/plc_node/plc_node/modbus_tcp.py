from __future__ import annotations

import math
import socket
import struct


class ModbusError(RuntimeError):
    pass


class ModbusTcpClient:
    def __init__(
        self,
        host: str,
        port: int,
        unit_id: int,
        timeout: float,
        interface: str | None = None,
    ) -> None:
        if not str(host).strip():
            raise ValueError("host must not be empty")
        if not 1 <= int(port) <= 65535:
            raise ValueError("port must be in range 1..65535")
        if not 0 <= int(unit_id) <= 255:
            raise ValueError("unit_id must be in range 0..255")
        if not math.isfinite(float(timeout)) or float(timeout) <= 0.0:
            raise ValueError("timeout must be finite and greater than zero")

        self.host = str(host).strip()
        self.port = int(port)
        self.unit_id = int(unit_id)
        self.timeout = float(timeout)
        self.interface = str(interface).strip() if interface else None
        self._sock: socket.socket | None = None
        self._transaction_id = 0

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.settimeout(self.timeout)
            if self.interface:
                bind_option = getattr(socket, "SO_BINDTODEVICE", None)
                if bind_option is None:
                    raise ModbusError("current OS does not support SO_BINDTODEVICE")
                try:
                    sock.setsockopt(
                        socket.SOL_SOCKET,
                        bind_option,
                        self.interface.encode() + b"\0",
                    )
                except PermissionError as exc:
                    raise ModbusError(
                        f"binding to interface {self.interface} requires CAP_NET_RAW"
                    ) from exc
            sock.connect((self.host, self.port))
        except Exception:
            sock.close()
            raise
        self._sock = sock

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def read_holding_registers(self, start: int, count: int) -> list[int]:
        _validate_register_range(start, count)
        response = self._request(0x03, struct.pack(">HH", start, count))
        return _unpack_registers(response, count)

    def write_single_register(self, address: int, value: int) -> None:
        _validate_register_range(address, 1)
        if not 0 <= int(value) <= 0xFFFF:
            raise ValueError(f"register value out of uint16 range: {value}")
        response = self._request(0x06, struct.pack(">HH", address, value))
        if len(response) != 4:
            raise ModbusError(
                f"register write response length mismatch: expected=4 actual={len(response)}"
            )
        echoed_address, echoed_value = struct.unpack(">HH", response)
        if echoed_address != address or echoed_value != value:
            raise ModbusError(
                "register write echo mismatch: "
                f"address={echoed_address}, value=0x{echoed_value:04x}"
            )

    def _request(self, function_code: int, payload: bytes) -> bytes:
        if self._sock is None:
            raise ModbusError("client is not connected")

        self._transaction_id = (self._transaction_id + 1) & 0xFFFF
        request = struct.pack(
            ">HHHB",
            self._transaction_id,
            0,
            len(payload) + 2,
            self.unit_id,
        ) + bytes([function_code]) + payload
        self._sock.sendall(request)

        header = _recv_exact(self._sock, 7)
        transaction_id, protocol_id, length, unit_id = struct.unpack(">HHHB", header)
        if transaction_id != self._transaction_id:
            raise ModbusError(
                f"transaction mismatch: got {transaction_id}, expected {self._transaction_id}"
            )
        if protocol_id != 0:
            raise ModbusError(f"unexpected protocol id: {protocol_id}")
        if unit_id != self.unit_id:
            raise ModbusError(f"unit id mismatch: got {unit_id}, expected {self.unit_id}")
        if not 2 <= length <= 254:
            raise ModbusError(f"invalid MBAP length: {length}")

        pdu = _recv_exact(self._sock, length - 1)
        response_function = pdu[0]
        response_payload = pdu[1:]
        if response_function == (function_code | 0x80):
            exception_code = response_payload[0] if response_payload else -1
            raise ModbusError(
                f"Modbus exception for function 0x{function_code:02x}: "
                f"code {exception_code}"
            )
        if response_function != function_code:
            raise ModbusError(
                f"function mismatch: got 0x{response_function:02x}, "
                f"expected 0x{function_code:02x}"
            )
        return response_payload


def _validate_register_range(start: int, count: int) -> None:
    if not 0 <= int(start) <= 0xFFFF:
        raise ValueError(f"register address out of uint16 range: {start}")
    if not 1 <= int(count) <= 125:
        raise ValueError(f"register count must be in range 1..125: {count}")
    if int(start) + int(count) > 0x10000:
        raise ValueError("register range exceeds uint16 address space")


def _recv_exact(sock, length: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < length:
        chunk = sock.recv(length - len(chunks))
        if not chunk:
            raise ModbusError("connection closed by remote host")
        chunks.extend(chunk)
    return bytes(chunks)


def _unpack_registers(response: bytes, count: int) -> list[int]:
    if not response:
        raise ModbusError("register response is missing byte count")
    byte_count = response[0]
    data = response[1:]
    expected = count * 2
    if byte_count != expected or len(data) != expected:
        raise ModbusError(
            "register response length mismatch: "
            f"byte_count={byte_count}, data={len(data)}, expected={expected}"
        )
    try:
        return list(struct.unpack(f">{count}H", data))
    except struct.error as exc:
        raise ModbusError(f"invalid register response: {exc}") from exc
