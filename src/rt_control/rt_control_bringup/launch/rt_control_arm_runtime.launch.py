"""V3 dual-seven-axis FJT/rolling runtime; PP grippers remain hold-only."""

from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from rt_control_composer.compose import compose, preflight


def _boolean(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context)
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _shutdown_if_failed(reason: str):
    def handler(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason=reason))]
        return []

    return handler


def _start_if_succeeded(next_action, reason: str):
    def handler(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason=reason))]
        return [next_action]

    return handler


def setup(context):
    use_mock_hardware = _boolean(context, "use_mock_hardware")
    jtc_only = _boolean(context, "jtc_only")
    publish_robot_state = (
        _boolean(context, "publish_robot_state")
        if "publish_robot_state" in context.launch_configurations
        else True
    )
    runtime = Path(tempfile.mkdtemp(prefix="rt-control-arm-runtime-"))
    build = compose(
        robot_variant=LaunchConfiguration("robot_variant", default="Alfa_V3").perform(context),
        machine_unit=LaunchConfiguration("machine_unit", default="Alfa_V3-01").perform(context),
        physical_profile="arms_only",
        control_scope="arms_only",
        runtime_dir=runtime,
        use_mock_hardware=use_mock_hardware,
        jtc_only=jtc_only,
    )
    preflight(build)
    controller_order = build.controller_order
    controller_config = runtime / "controllers.yaml"
    controller_config.write_text(
        yaml.safe_dump(build.controllers, sort_keys=False), encoding="utf-8"
    )

    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": build.robot_description},
            str(controller_config),
        ],
        output="both",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": build.robot_description}],
        output="both",
    )
    motion_loader = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *controller_order["inactive"],
            "--inactive",
            "--controller-manager-timeout",
            "90",
        ],
        output="both",
    )
    lifecycle_loader = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *controller_order["active"],
            "--controller-manager-timeout",
            "90",
        ],
        output="both",
    )

    control_adapter = (
        Node(
            package="control_api_adapter",
            executable="control_enable_adapter",
            output="both",
        )
        if jtc_only
        else None
    )

    actions = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=manager,
                on_exit=[EmitEvent(event=Shutdown(reason="V3 arm controller manager exited"))],
            )
        ),
        manager,
        motion_loader,
        RegisterEventHandler(
            OnProcessExit(
                target_action=motion_loader,
                on_exit=_start_if_succeeded(
                    lifecycle_loader, "V3 motion controller loading failed"
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=lifecycle_loader,
                on_exit=(
                    _start_if_succeeded(control_adapter, "V3 lifecycle controller loading failed")
                    if control_adapter is not None
                    else _shutdown_if_failed("V3 lifecycle controller loading failed")
                ),
            )
        ),
    ]
    if publish_robot_state:
        actions.insert(2, robot_state_publisher)
    if control_adapter is not None:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=control_adapter,
                    on_exit=[EmitEvent(event=Shutdown(reason="V3 control enable adapter exited"))],
                )
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("robot_variant", default_value="Alfa_V3"),
            DeclareLaunchArgument("machine_unit", default_value="Alfa_V3-01"),
            DeclareLaunchArgument("jtc_only", default_value="false"),
            DeclareLaunchArgument("publish_robot_state", default_value="true"),
            OpaqueFunction(function=setup),
        ]
    )
