#!/usr/bin/env python3
"""ELECTRI-109 Mock-only battery and ultrasonic publishers for the operator UI.

Synthetic data for UI development. Not installed, not a driver, never used on
hardware. Topic names and message types match the real producers
(R-OUT-04 ``/battery_state`` and ``modbus_tcp_rtu485`` ``ultrasonic/channelN/range``).
"""

import math
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Range

import robot_interfaces_qos

CHANNELS = 8
ULTRASONIC_PERIOD_S = 0.3
BATTERY_PERIOD_S = 1.0
FIELD_OF_VIEW_RAD = 0.6981317008
MIN_RANGE_M = 0.01
MAX_RANGE_M = 3.5
# Channel 8 goes silent for 10 s out of every 20 s to exercise the stale display.
STALE_CHANNEL = 7
STALE_WINDOW_S = 10.0


# Demo scenario: an obstacle approaches the front (channels 1 and 3) from 3.0 m
# to 0.15 m and retreats, then the same happens at the rear (channels 5 and 7).
# All other channels stay beyond the radar range so the beeper follows one
# clearly approaching obstacle.
CYCLE_S = 40.0
APPROACH_S = 10.0
NEAR_M = 0.15
FAR_M = 3.0


def _approach(phase: float) -> float:
    """3.0 m -> 0.15 m -> 3.0 m over 2 * APPROACH_S, far otherwise."""
    if phase >= 2 * APPROACH_S:
        return FAR_M
    ratio = phase / APPROACH_S if phase < APPROACH_S else 2 - phase / APPROACH_S
    return FAR_M - (FAR_M - NEAR_M) * ratio


def synthetic_range(channel: int, t: float) -> float:
    phase = t % CYCLE_S
    noise = 0.02 * math.sin(7.0 * t + channel)
    if channel in (0, 2):
        return _approach(phase) + noise
    if channel in (4, 6):
        return _approach((phase - CYCLE_S / 2) % CYCLE_S) + noise
    return 2.9 + 0.3 * math.sin(0.3 * t + channel) + noise


class MockSensors(Node):
    def __init__(self) -> None:
        super().__init__("operator_web_mock_sensors")
        self._start = self.get_clock().now()
        self._battery = self.create_publisher(
            BatteryState, "/battery_state", robot_interfaces_qos.state()
        )
        self._ranges = [
            self.create_publisher(
                Range, f"/ultrasonic/channel{i + 1}/range", qos_profile_sensor_data
            )
            for i in range(CHANNELS)
        ]
        self.create_timer(ULTRASONIC_PERIOD_S, self._publish_ranges)
        self.create_timer(BATTERY_PERIOD_S, self._publish_battery)

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._start).nanoseconds * 1e-9

    def _publish_ranges(self) -> None:
        t = self._elapsed()
        stamp = self.get_clock().now().to_msg()
        for i, publisher in enumerate(self._ranges):
            if i == STALE_CHANNEL and int(t // STALE_WINDOW_S) % 2 == 1:
                continue
            message = Range()
            message.header.stamp = stamp
            message.header.frame_id = f"ultrasonic_channel_{i + 1}_link"
            message.radiation_type = Range.ULTRASOUND
            message.field_of_view = FIELD_OF_VIEW_RAD
            message.min_range = MIN_RANGE_M
            message.max_range = MAX_RANGE_M
            message.range = synthetic_range(i, t)
            publisher.publish(message)

    def _publish_battery(self) -> None:
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.present = True
        message.voltage = 51.2
        message.percentage = max(0.05, 0.82 - 0.0005 * self._elapsed())
        self._battery.publish(message)


def main() -> None:
    rclpy.init()
    node = MockSensors()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch forwards SIGINT after the terminal already sent one.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
