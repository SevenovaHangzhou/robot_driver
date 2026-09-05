"""Non-real-time X503B raw shadow and gated WrenchStamped bridge."""

from __future__ import annotations

from typing import Any

from .bridge import (
    CalibrationSnapshot,
    SensorConfig,
    calibration_from_values,
    convert_to_wrench,
    extract_sensor_frame,
)


def _key_values(status: Any) -> dict[str, str]:
    return {item.key: item.value for item in status.values}


def main(args=None) -> int:
    import rclpy
    from control_msgs.msg import DynamicJointState
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import WrenchStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Int32MultiArray

    class X503WrenchBridge(Node):
        def __init__(self) -> None:
            super().__init__("x503_force_sensor_bridge")
            sensor_names = list(
                self.declare_parameter(
                    "sensor_names", ["right_force_sensor", "left_force_sensor"]
                ).value
            )
            wrench_topics = list(
                self.declare_parameter(
                    "wrench_topics",
                    [
                        "/rt_control/right_x503b/wrench",
                        "/rt_control/left_x503b/wrench",
                    ],
                ).value
            )
            raw_topics = list(
                self.declare_parameter(
                    "raw_topics",
                    [
                        "/rt_control/right_x503b/raw",
                        "/rt_control/left_x503b/raw",
                    ],
                ).value
            )
            frame_ids = list(
                self.declare_parameter(
                    "frame_ids",
                    ["right_ft_sensor_link", "left_ft_sensor_link"],
                ).value
            )
            if not (
                len(sensor_names)
                == len(wrench_topics)
                == len(raw_topics)
                == len(frame_ids)
            ):
                raise ValueError(
                    "sensor_names/topics/raw_topics/frame_ids must have "
                    "equal lengths"
                )
            self._configs = tuple(
                SensorConfig(name, wrench, raw, frame)
                for name, wrench, raw, frame in zip(
                    sensor_names, wrench_topics, raw_topics, frame_ids
                )
            )
            self._calibration: dict[str, CalibrationSnapshot] = {}
            self._wrench_publishers = {
                config.sensor_name: self.create_publisher(
                    WrenchStamped, config.wrench_topic, 10
                )
                for config in self._configs
            }
            self._raw_publishers = {
                config.sensor_name: self.create_publisher(
                    Int32MultiArray, config.raw_topic, 10
                )
                for config in self._configs
            }
            calibration_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            dynamic_topic = self.declare_parameter(
                "dynamic_joint_states_topic",
                "/rt_internal_state_broadcaster/dynamic_joint_states",
            ).value
            calibration_topic = self.declare_parameter(
                "calibration_topic", "/rt_control/x503b/calibration"
            ).value
            self.create_subscription(
                DynamicJointState, dynamic_topic, self._on_dynamic_state, 10
            )
            self.create_subscription(
                DiagnosticArray,
                calibration_topic,
                self._on_calibration,
                calibration_qos,
            )

        def _on_calibration(self, message: Any) -> None:
            for status in message.status:
                snapshot = calibration_from_values(_key_values(status))
                if status.hardware_id in {
                    config.sensor_name for config in self._configs
                }:
                    self._calibration[status.hardware_id] = snapshot

        def _on_dynamic_state(self, message: Any) -> None:
            for config in self._configs:
                frame = extract_sensor_frame(message, config.sensor_name)
                if frame is None:
                    continue
                raw_message = Int32MultiArray()
                raw_message.data = list(frame.raw_values)
                self._raw_publishers[config.sensor_name].publish(raw_message)
                wrench = convert_to_wrench(
                    frame,
                    self._calibration.get(
                        config.sensor_name,
                        CalibrationSnapshot(False, (), (), "unresolved"),
                    ),
                )
                if wrench is None:
                    continue
                output = WrenchStamped()
                output.header.stamp = message.header.stamp
                output.header.frame_id = config.frame_id
                output.wrench.force.x = wrench[0]
                output.wrench.force.y = wrench[1]
                output.wrench.force.z = wrench[2]
                output.wrench.torque.x = wrench[3]
                output.wrench.torque.y = wrench[4]
                output.wrench.torque.z = wrench[5]
                self._wrench_publishers[config.sensor_name].publish(output)

    rclpy.init(args=args)
    node = X503WrenchBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
