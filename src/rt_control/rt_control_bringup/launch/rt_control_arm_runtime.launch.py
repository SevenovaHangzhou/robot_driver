"""V3 dual-seven-axis FJT/rolling runtime; PP grippers remain hold-only."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
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

from rt_control_bringup.arm_motion_runtime import build_arm_motion_runtime


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
    calibration_file = LaunchConfiguration("calibration_file").perform(context).strip()
    runtime = Path(tempfile.mkdtemp(prefix="alfa-v3-arm-runtime-"))
    bringup_share = Path(get_package_share_directory("rt_control_bringup"))
    build = build_arm_motion_runtime(
        hardware_share=get_package_share_directory("robot_hw_ethercat"),
        description_share=get_package_share_directory("robot_description"),
        runtime_dir=runtime,
        use_mock_hardware=use_mock_hardware,
        bringup_share=bringup_share,
        jtc_only=jtc_only,
        calibration_file=calibration_file or None,
    )
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
            "whole_body_jtc",
            *([] if jtc_only else ["rolling_trajectory_controller"]),
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
            "joint_state_broadcaster",
            "rt_internal_state_broadcaster",
            "enable_manager",
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
        if jtc_only else None
    )

    actions = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=manager,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="V3 arm controller manager exited")
                    )
                ],
            )
        ),
        manager,
        robot_state_publisher,
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
                    _start_if_succeeded(
                        control_adapter, "V3 lifecycle controller loading failed"
                    ) if control_adapter is not None else
                    _shutdown_if_failed("V3 lifecycle controller loading failed")
                ),
            )
        ),
    ]
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
            DeclareLaunchArgument("jtc_only", default_value="false"),
            DeclareLaunchArgument("calibration_file", default_value=""),
            OpaqueFunction(function=setup),
        ]
    )
