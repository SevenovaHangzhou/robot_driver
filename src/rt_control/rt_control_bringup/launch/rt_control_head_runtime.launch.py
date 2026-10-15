"""Explicit V3 DaMiao head-only runtime; motor motion remains service-gated."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from rt_control_composer.artifact import load_artifact, runtime_arguments
from rt_control_composer.compose import preflight


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
    use_mock_hardware = _boolean(context, "use_mock_hardware")
    publish_robot_state = (
        _boolean(context, "publish_robot_state")
        if "publish_robot_state" in context.launch_configurations
        else True
    )
    build = load_artifact(
        LaunchConfiguration("generated_dir").perform(context),
        robot_variant=LaunchConfiguration("robot_variant", default="Alfa_V3").perform(context),
        machine_unit=LaunchConfiguration("machine_unit", default="Alfa_V3-01").perform(context),
        physical_profile="head_only",
        control_scope="head_only",
        use_mock_hardware=use_mock_hardware,
    )
    preflight(build)
    order = build.controller_order
    controller_config = Path(build.runtime_dir) / "controllers.yaml"
    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": build.robot_description}, str(controller_config)],
        arguments=runtime_arguments(build, {}),
        output="both",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": build.robot_description}],
        output="both",
    )
    position_loader = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[*order["inactive"], "--inactive", "--controller-manager-timeout", "30"],
        output="both",
    )
    lifecycle_loader = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[*order["active"], "--controller-manager-timeout", "30"],
        output="both",
    )
    actions = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=manager,
                on_exit=[EmitEvent(event=Shutdown(reason="V3 head controller manager exited"))],
            )
        ),
        manager,
        position_loader,
        RegisterEventHandler(
            OnProcessExit(
                target_action=position_loader,
                on_exit=_start_if_succeeded(lifecycle_loader, "V3 head controller loading failed"),
            )
        ),
    ]
    if publish_robot_state:
        actions.insert(2, robot_state_publisher)
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("robot_variant", default_value="Alfa_V3"),
            DeclareLaunchArgument("machine_unit", default_value="Alfa_V3-01"),
            DeclareLaunchArgument("publish_robot_state", default_value="true"),
            DeclareLaunchArgument(
                "generated_dir",
                default_value=EnvironmentVariable("RT_CONTROL_GENERATED_DIR", default_value=""),
                description="Absolute artifact directory from compose_artifact",
            ),
            OpaqueFunction(function=setup),
        ]
    )
