import math
import socket
import struct

from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import BatteryState

import bms_node.bms_node as bms_node_module
from bms_node.bms_node import (
    CAN_FRAME_SIZE,
    battery_state_message,
    build_can_filter_blob,
    decode_can_frame,
)
from bms_node.bms_protocol import BASIC_STATUS_FRAME_ID, BmsSample


def _raw_frame(can_id: int, payload: bytes) -> bytes:
    return struct.pack("=IB3x8s", can_id, len(payload), payload.ljust(8, b"\x00"))


def test_socket_filter_accepts_only_the_basic_status_identifier() -> None:
    expected = struct.pack(
        "=II",
        BASIC_STATUS_FRAME_ID,
        getattr(socket, "CAN_SFF_MASK", 0x7FF),
    )

    assert build_can_filter_blob() == expected


def test_decode_can_frame_accepts_only_standard_data_frame_0x3fc() -> None:
    payload = b"\x01\xf4\x00\x00\x50"

    assert decode_can_frame(_raw_frame(BASIC_STATUS_FRAME_ID, payload)) == (
        BASIC_STATUS_FRAME_ID,
        payload,
    )
    assert decode_can_frame(_raw_frame(0x3FB, payload)) is None
    assert decode_can_frame(_raw_frame(0x80000000 | BASIC_STATUS_FRAME_ID, payload)) is None
    assert decode_can_frame(b"\x00" * (CAN_FRAME_SIZE - 1)) is None


def test_battery_state_message_exposes_only_voltage_and_soc() -> None:
    message = battery_state_message(
        BmsSample(voltage_v=50.0, soc_fraction=0.8, last_frame_s=8.0),
        now_s=10.0,
        timeout_s=3.0,
        stamp=Time(sec=10),
        frame_id="can1",
    )

    assert message.voltage == 50.0
    assert message.percentage == 0.8
    assert message.present is True
    assert message.header.frame_id == "can1"
    assert math.isnan(message.temperature)
    assert math.isnan(message.current)
    assert math.isnan(message.charge)
    assert math.isnan(message.capacity)
    assert math.isnan(message.design_capacity)
    assert message.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    assert message.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    assert message.power_supply_technology == BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN


def test_battery_state_message_publishes_nan_when_frame_is_stale() -> None:
    message = battery_state_message(
        BmsSample(voltage_v=50.0, soc_fraction=0.8, last_frame_s=5.0),
        now_s=10.0,
        timeout_s=3.0,
        stamp=Time(sec=10),
        frame_id="can1",
    )

    assert math.isnan(message.voltage)
    assert math.isnan(message.percentage)
    assert message.present is False


def test_main_treats_context_shutdown_as_clean_exit(monkeypatch) -> None:
    class FakeNode:
        destroyed = False

        def destroy_node(self):
            self.destroyed = True

    node = FakeNode()
    monkeypatch.setattr(bms_node_module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(bms_node_module, "BmsNode", lambda: node)
    monkeypatch.setattr(
        bms_node_module.rclpy,
        "spin",
        lambda _node: (_ for _ in ()).throw(ExternalShutdownException()),
    )
    monkeypatch.setattr(bms_node_module.rclpy, "ok", lambda: False)

    bms_node_module.main()

    assert node.destroyed
