"""Generic scope runtime: artifact, bus locks, owner preflight, staged hardware, controllers."""

from pathlib import Path
import uuid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from integ_bringup.scope_lifecycle import acquire_bus_locks
from integ_composer.artifact import load_artifact, runtime_arguments
from integ_composer.compose import preflight
from integ_composer.self_check import SelfCheckFailed
from integ_diagnostics.report import write_self_check

# Lock files stay open for the lifetime of this launch process.
_HELD_LOCKS = []


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context)
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _optional_boolean(context, name, default):
    value = context.launch_configurations.get(name) or default
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
        jtc_only=_optional_boolean(context, "jtc_only", "false"),
        sync_mode=context.launch_configurations.get("sync_mode", ""),
    )
    if not build.staged_hardware_activation:
        raise ValueError(f"scope {scope} does not declare staged_hardware_activation")
    if not use_mock_hardware:
        # Bus ownership first, then read-only owner checks, before any Node exists.
        _HELD_LOCKS.extend(acquire_bus_locks(build.lifecycle["bus_locks"]))
    generated_dir = LaunchConfiguration("generated_dir").perform(context)
    try:
        write_self_check(scope, generated_dir, preflight(build))
    except SelfCheckFailed as failure:
        write_self_check(scope, generated_dir, failure.entries)
        raise
    publish_robot_state = _optional_boolean(
        context,
        "publish_robot_state",
        "true" if build.lifecycle.get("publish_robot_state", False) else "false",
    )
    runtime = Path(build.runtime_dir)
    order = build.controller_order
    manager = Node(
        package="integ_bringup",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": build.robot_description},
            str(runtime / "controllers.yaml"),
        ],
        # Per-start values the scope declares; unrequested keys are ignored.
        arguments=runtime_arguments(build, {"startup_id": f"{scope}-{uuid.uuid4()}"}),
        output="both",
    )
    hardware = Node(
        package="integ_bringup",
        executable="scope_hardware_start",
        arguments=[str(runtime)],
        output="both",
    )
    lifecycle = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[*order["active"], "--controller-manager-timeout", "120"],
        output="both",
    )
    adapter = Node(
        package="integ_api_adapter", executable="control_enable_adapter", output="both"
    )
    # Published only once the stack is loaded: presence means the scope runtime is up.
    capabilities = Node(
        package="integ_bringup",
        executable="scope_capabilities",
        arguments=[str(runtime)],
        output="both",
    )
    # Optional Runtime-LOG recorder: own spawner, started after the lifecycle controllers; its
    # exit (including a missing plugin) is not watched, so it can never stop the scope.
    recorder_names = build.lifecycle.get("recorder_controllers", [])
    after_lifecycle = [adapter, capabilities]
    if recorder_names:
        after_lifecycle.append(
            Node(
                package="controller_manager",
                executable="spawner",
                exec_name="runtime_log_recorder_spawner",
                arguments=[*recorder_names, "--controller-manager-timeout", "30"],
                output="both",
            )
        )
    # An enable-only scope has no motion controllers; the spawner needs at least one name.
    staged = []
    after_hardware = lifecycle
    if order["inactive"]:
        # Field tuning (operator console) is applied after the motion controllers load and
        # before the enable manager can enable anything.
        tuning = Node(
            package="integ_bringup",
            executable="scope_operator_tuning",
            arguments=[str(runtime)],
            output="both",
        )
        motion = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[*order["inactive"], "--inactive", "--controller-manager-timeout", "120"],
            output="both",
        )
        staged.append(
            RegisterEventHandler(
                OnProcessExit(target_action=motion, on_exit=_then([tuning], "motion load failed"))
            )
        )
        staged.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=tuning, on_exit=_then([lifecycle], "operator tuning failed")
                )
            )
        )
        after_hardware = motion
    robot_state = (
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": build.robot_description}],
                output="both",
            )
        ]
        if publish_robot_state
        else []
    )
    return [
        *robot_state,
        RegisterEventHandler(
            OnProcessExit(
                target_action=manager,
                on_exit=[EmitEvent(event=Shutdown(reason=f"{scope} controller manager exited"))],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=hardware,
                on_exit=_then([after_hardware], "hardware activation failed"),
            )
        ),
        *staged,
        RegisterEventHandler(
            OnProcessExit(
                target_action=lifecycle,
                on_exit=_then(after_lifecycle, "lifecycle load failed"),
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
            DeclareLaunchArgument("jtc_only", default_value="false"),
            DeclareLaunchArgument(
                "sync_mode", default_value="", description="EtherCAT force bench: sm or dc"
            ),
            DeclareLaunchArgument(
                "publish_robot_state",
                default_value="",
                description="true/false overrides the scope's publish_robot_state",
            ),
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
