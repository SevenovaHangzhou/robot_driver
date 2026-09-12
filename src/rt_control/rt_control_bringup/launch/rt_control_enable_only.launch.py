"""Stationary 14 CSP + 2 PP bringup; services never expose a motion goal."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from rt_control_bringup.stationary_enable import build_stationary_enable, verify_stationary_bus


def setup(context):
    mock = LaunchConfiguration("use_mock_hardware").perform(context)
    if mock not in ("true", "false"):
        raise ValueError("use_mock_hardware must be true or false")
    if mock == "false":
        verify_stationary_bus()
    runtime = Path(tempfile.mkdtemp(prefix="alfa-v3-enable-"))
    build = build_stationary_enable(get_package_share_directory("robot_hw_ethercat"), runtime,
                                    use_mock_hardware=mock == "true")
    config = runtime / "controllers.yaml"
    config.write_text(yaml.safe_dump(build.controllers, sort_keys=False), encoding="utf-8")
    manager = Node(package="controller_manager", executable="ros2_control_node",
                   parameters=[{"robot_description": build.robot_description}, str(config)], output="both")
    spawner = Node(package="controller_manager", executable="spawner", output="both",
                   arguments=["joint_state_broadcaster", "rt_internal_state_broadcaster", "enable_manager",
                              "--controller-manager-timeout", "90"])

    def stop_on_failure(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason="Stationary controller loading failed"))]
        return []

    return [RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=[
                EmitEvent(event=Shutdown(reason="Stationary controller manager exited"))])),
            manager, RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=stop_on_failure)), spawner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        OpaqueFunction(function=setup),
    ])
