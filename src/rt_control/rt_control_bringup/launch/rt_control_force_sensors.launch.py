"""Raw-only dual Blue Point bench; no motor plugins or enable controllers."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from rt_control_bringup.force_sensor_runtime import build_force_sensor_runtime


def setup(context):
    raw_mock = LaunchConfiguration("use_mock_hardware").perform(context)
    if raw_mock not in ("true", "false"):
        raise ValueError("use_mock_hardware must be true or false")
    runtime = Path(tempfile.mkdtemp(prefix="bluepoint-raw-bench-"))
    build = build_force_sensor_runtime(
        hardware_share=get_package_share_directory("robot_hw_ethercat"),
        bringup_share=get_package_share_directory("rt_control_bringup"),
        runtime_dir=runtime, use_mock_hardware=raw_mock == "true",
        sync_mode=LaunchConfiguration("sync_mode").perform(context))
    config = runtime / "controllers.yaml"
    config.write_text(yaml.safe_dump(build.controllers, sort_keys=False))
    manager = Node(package="controller_manager", executable="ros2_control_node",
                   parameters=[{"robot_description": build.robot_description}, str(config)], output="screen")
    loader = Node(package="controller_manager", executable="spawner",
                  arguments=[*build.controller_names, "--controller-manager", "/controller_manager",
                             "--controller-manager-timeout", "90"], output="screen")

    def loader_exit(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason="Force broadcaster activation failed"))]
        return []

    return [LogInfo(msg=f"BluePoint raw bench: mock={raw_mock}, left=2/right=4; wrench disabled"),
            RegisterEventHandler(OnProcessExit(target_action=manager,
                on_exit=[EmitEvent(event=Shutdown(reason="Force controller manager exited"))])),
            RegisterEventHandler(OnProcessExit(target_action=loader, on_exit=loader_exit)), manager, loader]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("sync_mode", default_value="",
                              description="Required for real hardware: sm or dc"),
        OpaqueFunction(function=setup),
    ])
