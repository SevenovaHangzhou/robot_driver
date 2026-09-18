"""Independent V3 E08 range acquisition; no actuator or bus-control nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument(
                "ultrasonic_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("modbus_tcp_rtu485"), "config", "ultrasonic.yaml"]
                ),
            ),
            Node(
                package="modbus_tcp_rtu485",
                executable="ultrasonic_node",
                name="ultrasonic_node",
                output="both",
                parameters=[LaunchConfiguration("ultrasonic_config")],
                condition=UnlessCondition(LaunchConfiguration("use_mock_hardware")),
            ),
        ]
    )
