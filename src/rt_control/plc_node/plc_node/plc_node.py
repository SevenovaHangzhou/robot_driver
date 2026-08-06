from __future__ import annotations

import math
import threading
import time
from typing import Callable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rt_control_interfaces.msg import PlcIoState
from std_srvs.srv import SetBool

from plc_node.modbus_tcp import ModbusError, ModbusTcpClient
from plc_node.plc_logic import (
    LEFT_SOLENOID_BIT,
    RIGHT_SOLENOID_BIT,
    VACUUM_PUMP_BIT,
    PlcIoSnapshot,
)
from plc_node.plc_runtime import (
    PlcControlError,
    read_io_snapshot,
    repair_remote_control,
    write_output_bit,
)


OUTPUT_SERVICE_BINDINGS = (
    ("/plc/left_solenoid", LEFT_SOLENOID_BIT),
    ("/plc/right_solenoid", RIGHT_SOLENOID_BIT),
    ("/plc/vacuum_pump", VACUUM_PUMP_BIT),
)
_EMPTY_SNAPSHOT = PlcIoSnapshot(False, False, False, False, False, 0)


def state_is_fresh(
    last_update_s: float | None,
    *,
    now_s: float,
    timeout_s: float,
) -> bool:
    if last_update_s is None:
        return False
    values = (float(last_update_s), float(now_s), float(timeout_s))
    if not all(math.isfinite(value) for value in values) or timeout_s <= 0.0:
        return False
    age_s = now_s - last_update_s
    return 0.0 <= age_s <= timeout_s


def plc_state_message(
    snapshot: PlcIoSnapshot,
    *,
    connected: bool,
    data_fresh: bool,
    error: str,
    stamp,
) -> PlcIoState:
    message = PlcIoState()
    message.header.stamp = stamp
    message.header.frame_id = "plc"
    message.connected = connected
    message.data_fresh = data_fresh
    message.left_vacuum_established = snapshot.left_vacuum_established
    message.right_vacuum_established = snapshot.right_vacuum_established
    message.left_solenoid_on = snapshot.left_solenoid_on
    message.right_solenoid_on = snapshot.right_solenoid_on
    message.vacuum_pump_on = snapshot.vacuum_pump_on
    message.io_alarm = snapshot.io_alarm
    message.error = error
    return message


class PlcNode(Node):
    def __init__(self) -> None:
        super().__init__("plc_node")
        self.declare_parameter("host", "192.168.1.88")
        self.declare_parameter("port", 502)
        self.declare_parameter("unit_id", 1)
        self.declare_parameter("timeout_s", 1.0)
        self.declare_parameter("interface", "enp4s0")
        self.declare_parameter("poll_period_s", 0.5)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter("data_timeout_s", 1.5)
        self.declare_parameter("do_command_register", 200)
        self.declare_parameter("io_control_register", 201)
        self.declare_parameter("di_status_register", 210)
        self.declare_parameter("do_status_register", 211)
        self.declare_parameter("io_alarm_register", 212)
        self.declare_parameter("write_verify_timeout_s", 1.0)
        self.declare_parameter("write_verify_period_s", 0.05)
        self.declare_parameter("state_topic", "/plc/io_state")

        self._host = str(self.get_parameter("host").value).strip()
        self._port = int(self.get_parameter("port").value)
        self._unit_id = int(self.get_parameter("unit_id").value)
        if not self._host:
            raise ValueError("host must not be empty")
        if not 1 <= self._port <= 65535:
            raise ValueError("port must be in range 1..65535")
        if not 0 <= self._unit_id <= 255:
            raise ValueError("unit_id must be in range 0..255")
        self._timeout_s = self._positive_parameter("timeout_s")
        raw_interface = str(self.get_parameter("interface").value).strip()
        self._interface = raw_interface or None
        poll_period_s = self._positive_parameter("poll_period_s")
        self._reconnect_period_s = self._positive_parameter("reconnect_period_s")
        self._data_timeout_s = self._positive_parameter("data_timeout_s")
        self._verify_timeout_s = self._non_negative_parameter(
            "write_verify_timeout_s"
        )
        self._verify_period_s = self._positive_parameter("write_verify_period_s")
        self._do_command_register = self._register_parameter("do_command_register")
        self._io_control_register = self._register_parameter("io_control_register")
        self._di_status_register = self._register_parameter("di_status_register")
        self._do_status_register = self._register_parameter("do_status_register")
        self._io_alarm_register = self._register_parameter("io_alarm_register")

        state_topic = str(self.get_parameter("state_topic").value).strip()
        if not state_topic:
            raise ValueError("state_topic must not be empty")
        self._publisher = self.create_publisher(PlcIoState, state_topic, 10)
        self._services = [
            self.create_service(SetBool, name, self._service_callback(bit, name))
            for name, bit in OUTPUT_SERVICE_BINDINGS
        ]

        self._lock = threading.Lock()
        self._client: ModbusTcpClient | None = None
        self._last_connect_attempt_s = -math.inf
        self._snapshot = _EMPTY_SNAPSHOT
        self._last_update_s: float | None = None
        self._last_error = ""
        self._timer = self.create_timer(poll_period_s, self._poll_once)

        self.get_logger().info(
            "PLC bridge ready: %s:%d unit=%d interface=%s poll=%.1fs"
            % (
                self._host,
                self._port,
                self._unit_id,
                self._interface or "route",
                poll_period_s,
            )
        )
        if self._interface:
            self.get_logger().info(
                f"PLC socket is bound to {self._interface}; CAP_NET_RAW is required"
            )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and greater than zero")
        return value

    def _non_negative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def _register_parameter(self, name: str) -> int:
        value = int(self.get_parameter(name).value)
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f"{name} must be in range 0..65535")
        return value

    def destroy_node(self) -> bool:
        with self._lock:
            self._close_client_locked()
        return super().destroy_node()

    def _service_callback(self, bit_index: int, service_name: str) -> Callable:
        def callback(
            request: SetBool.Request, response: SetBool.Response
        ) -> SetBool.Response:
            enabled = bool(request.data)
            with self._lock:
                try:
                    client = self._ensure_client_locked(force=True)
                    if client is None:
                        raise PlcControlError("PLC connection is unavailable")
                    repair_remote_control(client, self._io_control_register)
                    result = write_output_bit(
                        client,
                        bit_index=bit_index,
                        enabled=enabled,
                        command_register=self._do_command_register,
                        status_register=self._do_status_register,
                        verify_timeout_s=self._verify_timeout_s,
                        verify_period_s=self._verify_period_s,
                    )
                    self._last_error = ""
                    response.success = True
                    response.message = (
                        f"{service_name}={'ON' if enabled else 'OFF'} verified "
                        f"command=0x{result.command_word:04x} "
                        f"status=0x{result.status_word:04x}"
                    )
                except PlcControlError as exc:
                    self._last_error = str(exc)
                    response.success = False
                    response.message = f"{service_name} failed: {exc}"
                except (OSError, ModbusError, ValueError) as exc:
                    self._last_error = str(exc)
                    self._close_client_locked()
                    response.success = False
                    response.message = f"{service_name} failed: {exc}"
                self._publish_state_locked()
            return response

        return callback

    def _poll_once(self) -> None:
        with self._lock:
            client = self._ensure_client_locked(force=False)
            if client is None:
                self._publish_state_locked()
                return

            remote_error = ""
            try:
                changed = repair_remote_control(client, self._io_control_register)
                if changed:
                    self.get_logger().warning(
                        f"repaired PLC remote enable MW{self._io_control_register} bit0"
                    )
            except (OSError, ModbusError, PlcControlError) as exc:
                remote_error = f"remote control unavailable: {exc}"

            try:
                self._snapshot = read_io_snapshot(
                    client,
                    input_register=self._di_status_register,
                    output_status_register=self._do_status_register,
                    alarm_register=self._io_alarm_register,
                )
                self._last_update_s = time.monotonic()
                self._last_error = remote_error
            except (OSError, ModbusError, ValueError) as exc:
                self._last_error = str(exc)
                self.get_logger().warning(f"PLC status poll failed: {exc}")
                self._close_client_locked()
            self._publish_state_locked()

    def _ensure_client_locked(self, *, force: bool) -> ModbusTcpClient | None:
        if self._client is not None and self._client.connected:
            return self._client

        now_s = time.monotonic()
        if not force and now_s - self._last_connect_attempt_s < self._reconnect_period_s:
            return None
        self._last_connect_attempt_s = now_s
        client = ModbusTcpClient(
            self._host,
            self._port,
            self._unit_id,
            self._timeout_s,
            self._interface,
        )
        try:
            client.connect()
        except (OSError, ModbusError) as exc:
            client.close()
            self._last_error = str(exc)
            if force:
                raise
            self.get_logger().warning(f"PLC connect failed: {exc}")
            return None

        self._client = client
        self._last_error = ""
        self.get_logger().info(f"connected to PLC {self._host}:{self._port}")
        return client

    def _close_client_locked(self) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None

    def _publish_state_locked(self) -> None:
        now_s = time.monotonic()
        connected = self._client is not None and self._client.connected
        fresh = connected and state_is_fresh(
            self._last_update_s,
            now_s=now_s,
            timeout_s=self._data_timeout_s,
        )
        self._publisher.publish(
            plc_state_message(
                self._snapshot,
                connected=connected,
                data_fresh=fresh,
                error=self._last_error,
                stamp=self.get_clock().now().to_msg(),
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: PlcNode | None = None
    try:
        node = PlcNode()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
