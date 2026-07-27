from __future__ import annotations

import math
import socket
import struct
import threading
import time
from dataclasses import replace

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from bms_node.bms_protocol import (
    BASIC_STATUS_FRAME_ID,
    BmsSample,
    ingest_basic_frame,
    project_battery_values,
)


_CAN_FRAME_STRUCT = struct.Struct("=IB3x8s")
_CAN_FILTER_STRUCT = struct.Struct("=II")
CAN_FRAME_SIZE = _CAN_FRAME_STRUCT.size
_CAN_EFF_FLAG = getattr(socket, "CAN_EFF_FLAG", 0x80000000)
_CAN_RTR_FLAG = getattr(socket, "CAN_RTR_FLAG", 0x40000000)
_CAN_ERR_FLAG = getattr(socket, "CAN_ERR_FLAG", 0x20000000)
_CAN_SFF_MASK = getattr(socket, "CAN_SFF_MASK", 0x000007FF)
_CAN_FRAME_FLAG_MASK = _CAN_EFF_FLAG | _CAN_RTR_FLAG | _CAN_ERR_FLAG


def build_can_filter_blob() -> bytes:
    return _CAN_FILTER_STRUCT.pack(BASIC_STATUS_FRAME_ID, _CAN_SFF_MASK)


def decode_can_frame(raw_frame: bytes) -> tuple[int, bytes] | None:
    if len(raw_frame) != CAN_FRAME_SIZE:
        return None
    raw_can_id, can_dlc, data = _CAN_FRAME_STRUCT.unpack(raw_frame)
    if raw_can_id & _CAN_FRAME_FLAG_MASK or can_dlc > 8:
        return None
    can_id = raw_can_id & _CAN_SFF_MASK
    if can_id != BASIC_STATUS_FRAME_ID:
        return None
    return can_id, data[: int(can_dlc)]


def battery_state_message(
    sample: BmsSample,
    *,
    now_s: float,
    timeout_s: float,
    stamp,
    frame_id: str,
) -> BatteryState:
    voltage_v, soc_fraction = project_battery_values(sample, now_s, timeout_s)
    fresh = not math.isnan(voltage_v) and not math.isnan(soc_fraction)

    message = BatteryState()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.voltage = voltage_v
    message.temperature = math.nan
    message.current = math.nan
    message.charge = math.nan
    message.capacity = math.nan
    message.design_capacity = math.nan
    message.percentage = soc_fraction
    message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    message.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
    message.present = fresh
    return message


class BmsNode(Node):
    def __init__(self) -> None:
        super().__init__("bms_node")
        self.declare_parameter("can_interface", "can1")
        self.declare_parameter("receive_timeout_s", 0.5)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter("publish_period_s", 5.0)
        self.declare_parameter("frame_timeout_s", 3.0)
        self.declare_parameter("battery_state_topic", "/bms/battery_state")

        self._can_interface = str(self.get_parameter("can_interface").value).strip()
        self._receive_timeout_s = self._positive_parameter("receive_timeout_s")
        self._reconnect_period_s = self._positive_parameter("reconnect_period_s")
        publish_period_s = self._positive_parameter("publish_period_s")
        self._frame_timeout_s = self._positive_parameter("frame_timeout_s")
        if not self._can_interface:
            raise ValueError("can_interface must not be empty")

        topic = str(self.get_parameter("battery_state_topic").value).strip()
        if not topic:
            raise ValueError("battery_state_topic must not be empty")
        self._publisher = self.create_publisher(BatteryState, topic, 10)

        self._lock = threading.Lock()
        self._sample = BmsSample()
        self._socket: socket.socket | None = None
        self._stop_event = threading.Event()
        self._publish_timer = self.create_timer(publish_period_s, self._publish)
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="bms-can-reader",
            daemon=True,
        )
        self._reader_thread.start()
        self.get_logger().info(
            "BMS monitor ready: interface=%s frame=0x%03x publish=%.1fs"
            % (self._can_interface, BASIC_STATUS_FRAME_ID, publish_period_s)
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and greater than zero")
        return value

    def destroy_node(self) -> bool:
        self._stop_event.set()
        with self._lock:
            self._close_socket_locked()
        self._reader_thread.join(timeout=max(1.0, self._receive_timeout_s + 0.5))
        if self._reader_thread.is_alive():
            self.get_logger().error("BMS CAN reader did not stop within timeout")
        return super().destroy_node()

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            sock = self._ensure_socket()
            if sock is None:
                self._stop_event.wait(self._reconnect_period_s)
                continue
            try:
                decoded = decode_can_frame(sock.recv(CAN_FRAME_SIZE))
                if decoded is None:
                    continue
                can_id, payload = decoded
                with self._lock:
                    ingest_basic_frame(
                        self._sample, can_id, payload, time.monotonic()
                    )
            except socket.timeout:
                continue
            except OSError as exc:
                with self._lock:
                    self._close_socket_locked()
                if not self._stop_event.is_set():
                    self.get_logger().warning(f"BMS CAN read failed: {exc}")
                self._stop_event.wait(self._reconnect_period_s)

    def _ensure_socket(self) -> socket.socket | None:
        with self._lock:
            if self._socket is not None:
                return self._socket
            sock: socket.socket | None = None
            try:
                sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                sock.settimeout(self._receive_timeout_s)
                sock.setsockopt(
                    socket.SOL_CAN_RAW,
                    socket.CAN_RAW_FILTER,
                    build_can_filter_blob(),
                )
                sock.bind((self._can_interface,))
            except OSError as exc:
                if sock is not None:
                    sock.close()
                self.get_logger().warning(f"BMS CAN connect failed: {exc}")
                return None
            self._socket = sock
            self.get_logger().info(f"connected to BMS CAN interface {self._can_interface}")
            return sock

    def _close_socket_locked(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def _publish(self) -> None:
        with self._lock:
            sample = replace(self._sample)
        self._publisher.publish(
            battery_state_message(
                sample,
                now_s=time.monotonic(),
                timeout_s=self._frame_timeout_s,
                stamp=self.get_clock().now().to_msg(),
                frame_id=self._can_interface,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: BmsNode | None = None
    try:
        node = BmsNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
