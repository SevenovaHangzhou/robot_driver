"""Stationary 14 CSP + 2 PP bringup; services never expose a motion goal."""

from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from rt_control_composer.compose import compose, preflight


def setup(context):
    mock = LaunchConfiguration("use_mock_hardware").perform(context)
    if mock not in ("true", "false"):
        raise ValueError("use_mock_hardware must be true or false")
    runtime = Path(tempfile.mkdtemp(prefix="rt-control-enable-"))
    build = compose(
        robot_variant=LaunchConfiguration("robot_variant", default="Alfa_V3").perform(context),
        machine_unit=LaunchConfiguration("machine_unit", default="Alfa_V3-01").perform(context),
        physical_profile="arms_only",
        control_scope="arms_stationary_enable",
        runtime_dir=runtime,
        use_mock_hardware=mock == "true",
    )
    preflight(build)
    controller_order = build.controller_order["active"]
    config = runtime / "controllers.yaml"
    config.write_text(yaml.safe_dump(build.controllers, sort_keys=False), encoding="utf-8")
    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": build.robot_description}, str(config)],
        output="both",
    )
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        arguments=[*controller_order, "--controller-manager-timeout", "90"],
    )

    def stop_on_failure(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason="Stationary controller loading failed"))]
        return []

    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=manager,
                on_exit=[EmitEvent(event=Shutdown(reason="Stationary controller manager exited"))],
            )
        ),
        manager,
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=stop_on_failure)),
        spawner,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("robot_variant", default_value="Alfa_V3"),
            DeclareLaunchArgument("machine_unit", default_value="Alfa_V3-01"),
            OpaqueFunction(function=setup),
        ]
    )
