"""Explicit V3 DaMiao head-only runtime; motor motion remains service-gated."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from rt_control_bringup.head_can_config import load_head_can_config
from rt_control_bringup.head_runtime import build_head_runtime


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context)
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _start_if_succeeded(next_action, reason):
    def handler(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason=reason))]
        return [next_action]
    return handler


def setup(context):
    config_path = LaunchConfiguration("head_can_config").perform(context).strip()
    if not config_path:
        raise ValueError("head_can_config is required for the V3 head runtime")
    config = load_head_can_config(config_path)
    use_mock_hardware = _boolean(context, "use_mock_hardware")
    bringup_share = Path(get_package_share_directory("rt_control_bringup"))
    build = build_head_runtime(
        config=config,
        description_share=get_package_share_directory("robot_description"),
        hardware_share=get_package_share_directory("robot_hw_can"),
        controller_share=get_package_share_directory("damiao_head_controller"),
        bringup_share=bringup_share,
        use_mock_hardware=use_mock_hardware,
    )
    runtime = Path(tempfile.mkdtemp(prefix="alfa-v3-head-runtime-"))
    controller_config = runtime / "controllers.yaml"
    controller_config.write_text(
        yaml.safe_dump(build.controllers, sort_keys=False), encoding="utf-8"
    )
    manager = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[{"robot_description": build.robot_description}, str(controller_config)],
        output="both",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": build.robot_description}], output="both",
    )
    position_loader = Node(
        package="controller_manager", executable="spawner",
        arguments=["head_position_controller", "--inactive", "--controller-manager-timeout", "30"],
        output="both",
    )
    lifecycle_loader = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "damiao_head_manager",
                   "--controller-manager-timeout", "30"],
        output="both",
    )
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason="V3 head controller manager exited"))],
        )),
        manager,
        robot_state_publisher,
        position_loader,
        RegisterEventHandler(OnProcessExit(
            target_action=position_loader,
            on_exit=_start_if_succeeded(lifecycle_loader, "V3 head controller loading failed"),
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("head_can_config", default_value=""),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        OpaqueFunction(function=setup),
    ])
