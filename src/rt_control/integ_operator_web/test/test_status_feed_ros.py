"""Status feed against a real ROS graph: the chassis speed arrives from the swerve odometry."""

from pathlib import Path
import sys
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
import robot_interfaces_qos

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.status.feed import Feed, FeedNode  # noqa: E402

# Published by ctrl_swerve (R-OUT-02 in robot_interfaces contract/endpoints.yaml).
SWERVE_ODOMETRY = "/rt/base/odom"


def test_chassis_speed_comes_from_the_swerve_odometry():
    rclpy.init()
    try:
        feed = Feed()
        node = FeedNode(feed)
        source = rclpy.create_node("swerve_odometry_source")
        publisher = source.create_publisher(
            Odometry, SWERVE_ODOMETRY, robot_interfaces_qos.fast_state()
        )
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(source)
        message = Odometry()
        message.twist.twist.linear.x = 0.3
        message.twist.twist.angular.z = -0.2
        deadline = time.monotonic() + 5.0
        while feed.odometry is None and time.monotonic() < deadline:
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.05)
        assert feed.odometry is not None
        assert feed.odometry.value == {"vx": 0.3, "vy": 0.0, "wz": -0.2}
    finally:
        rclpy.shutdown()
