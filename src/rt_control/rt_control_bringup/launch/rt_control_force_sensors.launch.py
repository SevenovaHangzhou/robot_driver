"""Raw-only dual Blue Point bench; no motor plugins or enable controllers."""

from pathlib import Path
import uuid

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from rt_control_composer.artifact import load_artifact, runtime_arguments
from rt_control_composer.compose import preflight


def setup(context):
    raw_mock = LaunchConfiguration("use_mock_hardware").perform(context)
    if raw_mock not in ("true", "false"):
        raise ValueError("use_mock_hardware must be true or false")
    build = load_artifact(
        LaunchConfiguration("generated_dir").perform(context),
        robot_variant=LaunchConfiguration("robot_variant", default="Alfa_V3").perform(context),
        machine_unit=LaunchConfiguration("machine_unit", default="Alfa_V3-01").perform(context),
        physical_profile="force_bench",
        control_scope="force_bench",
        use_mock_hardware=raw_mock == "true",
        sync_mode=LaunchConfiguration("sync_mode").perform(context),
    )
    preflight(build)
    controller_names = build.controller_order["active"]
    config = Path(build.runtime_dir) / "controllers.yaml"
    startup_arguments = runtime_arguments(build, {"startup_id": f"force-bench-{uuid.uuid4()}"})
    manager = Node(
        package="rt_control_bringup",
        executable="ros2_control_node",
        parameters=[{"robot_description": build.robot_description}, str(config)],
        arguments=startup_arguments,
        output="screen",
    )
    loader = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *controller_names,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "90",
        ],
        output="screen",
    )

    def loader_exit(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason="Force broadcaster activation failed"))]
        return []

    return [
        LogInfo(msg=f"BluePoint raw bench: mock={raw_mock}; topology from artifact; wrench disabled"),
        RegisterEventHandler(
            OnProcessExit(
                target_action=manager,
                on_exit=[EmitEvent(event=Shutdown(reason="Force controller manager exited"))],
            )
        ),
        RegisterEventHandler(OnProcessExit(target_action=loader, on_exit=loader_exit)),
        manager,
        loader,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("robot_variant", default_value="Alfa_V3"),
            DeclareLaunchArgument("machine_unit", default_value="Alfa_V3-01"),
            DeclareLaunchArgument(
                "sync_mode", default_value="", description="Required for real hardware: sm or dc"
            ),
            DeclareLaunchArgument(
                "generated_dir",
                default_value=EnvironmentVariable("RT_CONTROL_GENERATED_DIR", default_value=""),
                description="Absolute artifact directory from compose_artifact",
            ),
            OpaqueFunction(function=setup),
        ]
    )
