"""Draft interface preview. No actuator backend or hardware processes are admitted."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _setup(context):
    if _as_bool(context, "validation_only"):
        return [LogInfo(msg="V3 interface runtime validated; no nodes started")]
    if not _as_bool(context, "use_mock_hardware"):
        raise ValueError(
            "Draft interface runtime is preview-only; hardware execution requires "
            "completed lifecycle, stop/hold and readiness integration"
        )
    if _as_bool(context, "start_ultrasonic"):
        raise ValueError("Draft preview cannot start physical ultrasonic acquisition")

    bringup_share = Path(get_package_share_directory("rt_control_bringup"))
    api_config = str(bringup_share / "config/rt_io.yaml")
    return [
        Node(
            package="control_api_adapter",
            executable="vacuum_adapter",
            name="vacuum_adapter",
            parameters=[api_config],
            output="both",
        ),
        Node(
            package="control_api_adapter",
            executable="module_state_adapter",
            name="module_state_adapter",
            parameters=[api_config],
            output="both",
        ),
        Node(
            package="control_api_adapter",
            executable="position_action_adapter",
            name="position_action_adapter",
            parameters=[api_config],
            output="both",
        ),
        Node(
            package="control_api_adapter",
            executable="rt_status_adapter",
            name="rt_status_adapter",
            parameters=[api_config],
            output="both",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("validation_only", default_value="true"),
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("start_ultrasonic", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
