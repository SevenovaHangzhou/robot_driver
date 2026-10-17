"""Generic scope runtime: artifact, bus locks, owner preflight, staged hardware, controllers."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from rt_control_bringup.scope_lifecycle import acquire_bus_locks
from rt_control_composer.artifact import load_artifact, runtime_arguments
from rt_control_composer.compose import preflight

# Lock files stay open for the lifetime of this launch process.
_HELD_LOCKS = []


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context)
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _then(next_actions, reason):
    def handler(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason=reason))]
        return list(next_actions)

    return handler


def setup(context):
    use_mock_hardware = _boolean(context, "use_mock_hardware")
    scope = LaunchConfiguration("control_scope").perform(context)
    profile = LaunchConfiguration("physical_profile").perform(context) or scope
    build = load_artifact(
        LaunchConfiguration("generated_dir").perform(context),
        robot_variant=LaunchConfiguration("robot_variant").perform(context),
        machine_unit=LaunchConfiguration("machine_unit").perform(context),
        physical_profile=profile,
        control_scope=scope,
        use_mock_hardware=use_mock_hardware,
    )
    if not build.staged_hardware_activation:
        raise ValueError(f"scope {scope} does not declare staged_hardware_activation")
    if not use_mock_hardware:
        # Bus ownership first, then read-only owner checks, before any Node exists.
        _HELD_LOCKS.extend(acquire_bus_locks(build.lifecycle["bus_locks"]))
    preflight(build)
    runtime = Path(build.runtime_dir)
    order = build.controller_order
    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": build.robot_description},
            str(runtime / "controllers.yaml"),
        ],
        arguments=runtime_arguments(build, {}),
        output="both",
    )
    hardware = Node(
        package="rt_control_bringup",
        executable="scope_hardware_start",
        arguments=[str(runtime)],
        output="both",
    )
    motion = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[*order["inactive"], "--inactive", "--controller-manager-timeout", "120"],
        output="both",
    )
    lifecycle = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[*order["active"], "--controller-manager-timeout", "120"],
        output="both",
    )
    adapter = Node(
        package="control_api_adapter", executable="control_enable_adapter", output="both"
    )
    # Published only once the stack is loaded: presence means the scope runtime is up.
    capabilities = Node(
        package="rt_control_bringup",
        executable="scope_capabilities",
        arguments=[str(runtime)],
        output="both",
    )
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=manager,
                on_exit=[EmitEvent(event=Shutdown(reason=f"{scope} controller manager exited"))],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=hardware, on_exit=_then([motion], "hardware activation failed")
            )
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=motion, on_exit=_then([lifecycle], "motion load failed"))
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=lifecycle,
                on_exit=_then([adapter, capabilities], "lifecycle load failed"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=adapter,
                on_exit=[EmitEvent(event=Shutdown(reason="control enable adapter exited"))],
            )
        ),
        manager,
        hardware,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("control_scope", default_value="chassis_only"),
            DeclareLaunchArgument("physical_profile", default_value=""),
            DeclareLaunchArgument("robot_variant", default_value="Alfa_V3"),
            DeclareLaunchArgument("machine_unit", default_value="Alfa_V3-01"),
            DeclareLaunchArgument(
                "generated_dir",
                default_value=EnvironmentVariable("RT_CONTROL_GENERATED_DIR", default_value=""),
                description="Absolute artifact directory from compose_artifact",
            ),
            OpaqueFunction(function=setup),
        ]
    )
