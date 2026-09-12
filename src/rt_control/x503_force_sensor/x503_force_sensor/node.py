"""PDO-only X503 bridge using the current launch's PREOP calibration."""
from __future__ import annotations

import math
from typing import Any

from control_msgs.msg import DynamicJointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_interfaces_qos import fast_state, latched
from std_msgs.msg import Int32MultiArray

from .bridge import SensorConfig, convert_to_wrench, extract_sensor_frame
from .snapshot import SnapshotState


def _state_value(message: Any, component: str, interface: str) -> int | None:
    if len(message.joint_names) != len(message.interface_values):
        return None
    matches = [v for name, v in zip(message.joint_names, message.interface_values) if name == component]
    if len(matches) != 1:
        return None
    item = matches[0]
    if len(item.interface_names) != len(item.values):
        return None
    values = [v for name, v in zip(item.interface_names, item.values) if name == interface]
    if len(values) != 1 or not math.isfinite(values[0]) or int(values[0]) != values[0]:
        return None
    return int(values[0])


class X503WrenchBridge(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("x503_force_sensor_bridge", **kwargs)
        strings = ParameterDescriptor(type=Parameter.Type.STRING_ARRAY.value, read_only=True)
        arrays = {
            key: list(self.declare_parameter(key, None, strings).value or [])
            for key in ("sensor_names", "wrench_topics", "raw_topics", "frame_ids")
        }
        names = arrays["sensor_names"]
        positions = list(self.declare_parameter(
            "slave_positions", None,
            ParameterDescriptor(type=Parameter.Type.INTEGER_ARRAY.value, read_only=True)).value or [])
        if (not names or len(names) != len(set(names)) or len(positions) != len(names)
                or len(positions) != len(set(positions))
                or any(type(p) is not int or not 0 <= p <= 65534 for p in positions)
                or any(len(values) != len(names) for values in arrays.values())
                or any(not isinstance(v, str) or not v or v != v.strip()
                       for values in arrays.values() for v in values)):
            raise ValueError("sensor names, positions, topics and frames must be complete and unique")
        self._positions = dict(zip(names, positions))
        self._configs = tuple(SensorConfig(*values) for values in zip(
            names, arrays["wrench_topics"], arrays["raw_topics"], arrays["frame_ids"]))
        readonly = ParameterDescriptor(read_only=True)
        startup_id = self.declare_parameter("startup_id", "", readonly).value
        payload = self.declare_parameter("preop_snapshot_json", "", readonly).value
        self._snapshot = SnapshotState(payload, startup_id, self._positions)
        self._wrench_publishers = {
            c.sensor_name: self.create_publisher(WrenchStamped, c.wrench_topic, fast_state())
            for c in self._configs}
        self._raw_publishers = {
            c.sensor_name: self.create_publisher(Int32MultiArray, c.raw_topic, fast_state())
            for c in self._configs}
        calibration_topic = self.declare_parameter(
            "calibration_topic", "/rt_control/x503b/calibration", readonly).value
        self._calibration_publisher = self.create_publisher(DiagnosticArray, calibration_topic, latched())
        dynamic_topic = self.declare_parameter(
            "dynamic_joint_states_topic", "/rt_internal_state_broadcaster/dynamic_joint_states", readonly).value
        self.create_subscription(DynamicJointState, dynamic_topic, self._on_dynamic_state, 10)
        # Retain metadata for late subscribers; never accept a replacement from
        # another node or a previous launch.
        self._publish_calibration()

    def _publish_calibration(self) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for record in self._snapshot.statuses():
            status = DiagnosticStatus()
            status.name = "/robot/rt_control/x503b/" + record["sensor_name"] + "/calibration"
            status.hardware_id = record["sensor_name"]
            valid = record["values"]["snapshot_valid"] == "true"
            status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.ERROR
            status.message = ("X503B unit/decimal snapshot verified in PREOP" if valid
                              else record.get("error") or "No valid PREOP snapshot")
            values = {**record["values"], "startup_id": self._snapshot.startup_id,
                      "snapshot_source": self._snapshot.source}
            status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
            message.status.append(status)
        self._calibration_publisher.publish(message)

    def _on_dynamic_state(self, message: DynamicJointState) -> None:
        link_up = _state_value(message, "ethercat_master", "link_up") == 1
        invalidated = False
        for config in self._configs:
            position = self._positions[config.sensor_name]
            al_state = _state_value(message, f"ethercat_slave_{position}", "al_state")
            invalidated |= self._snapshot.observe(config.sensor_name, al_state=al_state, link_up=link_up)
            frame = extract_sensor_frame(message, config.sensor_name)
            if frame is None:
                continue
            raw = Int32MultiArray()
            raw.data = list(frame.raw_values)
            self._raw_publishers[config.sensor_name].publish(raw)
            wrench = convert_to_wrench(frame, self._snapshot.calibration(config.sensor_name))
            if wrench is None:
                continue
            output = WrenchStamped()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = config.frame_id
            output.wrench.force.x, output.wrench.force.y, output.wrench.force.z = wrench[:3]
            output.wrench.torque.x, output.wrench.torque.y, output.wrench.torque.z = wrench[3:]
            self._wrench_publishers[config.sensor_name].publish(output)
        if invalidated:
            self._publish_calibration()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = X503WrenchBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
