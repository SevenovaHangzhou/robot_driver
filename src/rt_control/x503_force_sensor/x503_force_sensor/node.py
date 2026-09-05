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


def _key_values(status: Any) -> dict[str, str] | None:
    values: dict[str, str] = {}
    for item in status.values:
        if not item.key or item.key in values:
            return None
        values[item.key] = item.value
    return values


def main(args=None) -> int:
    import rclpy
    from control_msgs.msg import DynamicJointState
    from diagnostic_msgs.msg import DiagnosticArray
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import WrenchStamped
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Int32MultiArray

    class X503WrenchBridge(Node):
        def __init__(self) -> None:
            super().__init__("x503_force_sensor_bridge")
            string_array = ParameterDescriptor(
                type=Parameter.Type.STRING_ARRAY.value
            )
            sensor_names = list(
                self.declare_parameter(
                    "sensor_names", None, string_array
                ).value
                or []
            )
            wrench_topics = list(
                self.declare_parameter(
                    "wrench_topics", None, string_array
                ).value
                or []
            )
            raw_topics = list(
                self.declare_parameter("raw_topics", None, string_array).value
                or []
            )
            frame_ids = list(
                self.declare_parameter("frame_ids", None, string_array).value
                or []
            )
            if (
                not sensor_names
                or len(sensor_names) != len(wrench_topics)
                or len(sensor_names) != len(raw_topics)
                or len(sensor_names) != len(frame_ids)
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
            self._sensor_names = {
                config.sensor_name for config in self._configs
            }
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
                if status.hardware_id not in self._sensor_names:
                    continue
                if status.level == DiagnosticStatus.ERROR:
                    self._calibration[status.hardware_id] = (
                        CalibrationSnapshot(False, (), (), "unresolved")
                    )
                    continue
                values = _key_values(status)
                self._calibration[status.hardware_id] = (
                    calibration_from_values(values or {})
                )

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
                output.header.stamp = self.get_clock().now().to_msg()
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
