import json
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def write_rolling_parameter_override(envelope_path, directory=None):
    """Write the controller-node override required by Humble's spawner."""
    resolved_envelope = Path(envelope_path).resolve(strict=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".yaml",
        dir=directory,
        delete=False,
    ) as parameter_file:
        json.dump(
            {
                "rolling_trajectory_controller": {
                    "ros__parameters": {"envelope_file": str(resolved_envelope)}
                }
            },
            parameter_file,
        )
        parameter_file.write("\n")
        return Path(parameter_file.name).resolve()


def remove_generated_parameter_file(_context, parameter_file):
    Path(parameter_file).unlink(missing_ok=True)
    return []


def make_mandatory_inactive_spawner(
    controller_name, next_action, parameter_files=()
):
    """Build an INACTIVE spawner and its fail-closed exit transition."""
    arguments = [
        controller_name,
        "--inactive",
        "--controller-manager",
        "/controller_manager",
    ]
    for parameter_file in parameter_files:
        arguments.extend(["--param-file", parameter_file])

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments,
        output="both",
    )

    def transition(event, _context):
        if event.returncode == 0:
            return [next_action]
        return [
            LogInfo(
                msg=(
                    f"{controller_name} did not reach configured INACTIVE state; "
                    "the command-writer chain will not continue and launch will stop"
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason=f"{controller_name} failed to configure INACTIVE"
                )
            ),
        ]

    return spawner, transition


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    start_plc = LaunchConfiguration("start_plc")
    start_bms = LaunchConfiguration("start_bms")
    description_file = PathJoinSubstitution(
        [FindPackageShare("rt_control_bringup"), "urdf", "rt_control.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("rt_control_bringup"), "config", "controllers.yaml"]
    )
    rolling_envelope_file = (
        Path(get_package_share_directory("rt_control_bringup"))
        / "config"
        / "rolling_envelope_provisional.yaml"
    )
    rolling_parameter_override = write_rolling_parameter_override(
        rolling_envelope_file
    )
    rt_io_file = PathJoinSubstitution(
        [FindPackageShare("rt_control_bringup"), "config", "rt_io.yaml"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    description_file,
                    " use_mock_hardware:=",
                    use_mock_hardware,
                ]
            ),
            value_type=str,
        )
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            controllers_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel_safe"),
            ("/diff_drive_controller/odom", "/wheel/odom"),
        ],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,
            {"publish_frequency": 50.0, "use_sim_time": use_sim_time},
        ],
    )
    diagnostics = Node(
        package="rt_diagnostics",
        executable="rt_diagnostics_node",
        output="both",
        parameters=[
            {
                "hardware_id": "robot-001",
                "dynamic_joint_states_topic": "/rt_internal_state_broadcaster/dynamic_joint_states",
                "use_sim_time": use_sim_time,
            }
        ],
    )
    control_adapter = Node(
        package="control_api_adapter",
        executable="control_enable_adapter",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    vacuum_adapter = Node(
        package="control_api_adapter",
        executable="vacuum_adapter",
        output="both",
        parameters=[rt_io_file, {"use_sim_time": use_sim_time}],
    )
    rt_status_adapter = Node(
        package="control_api_adapter",
        executable="rt_status_adapter",
        output="both",
        parameters=[rt_io_file, {"use_sim_time": use_sim_time}],
    )
    plc = Node(
        package="plc_node",
        executable="plc_node",
        output="both",
        parameters=[rt_io_file],
        condition=IfCondition(start_plc),
    )
    bms = Node(
        package="bms_node",
        executable="bms_node",
        output="both",
        parameters=[rt_io_file],
        condition=IfCondition(start_bms),
    )

    active_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="both",
        )
        for name in (
            "joint_state_broadcaster",
            "rt_internal_state_broadcaster",
            "diff_drive_controller",
        )
    ]
    enable_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["enable_manager", "--controller-manager", "/controller_manager"],
        output="both",
    )
    rolling_spawner, rolling_transition_handler = make_mandatory_inactive_spawner(
        "rolling_trajectory_controller",
        enable_spawner,
        parameter_files=[controllers_file, str(rolling_parameter_override)],
    )
    jtc_spawner, jtc_transition_handler = make_mandatory_inactive_spawner(
        "whole_body_jtc", rolling_spawner
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        OpaqueFunction(
                            function=remove_generated_parameter_file,
                            args=[str(rolling_parameter_override)],
                        )
                    ]
                )
            ),
            DeclareLaunchArgument(
                "start_plc",
                default_value=EnvironmentVariable(
                    "RT_CONTROL_START_PLC", default_value="false"
                ),
            ),
            DeclareLaunchArgument(
                "start_bms",
                default_value=EnvironmentVariable(
                    "RT_CONTROL_START_BMS", default_value="false"
                ),
            ),
            control_node,
            state_publisher,
            diagnostics,
            control_adapter,
            vacuum_adapter,
            rt_status_adapter,
            plc,
            bms,
            *active_spawners,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rolling_spawner,
                    on_exit=rolling_transition_handler,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=jtc_spawner,
                    on_exit=jtc_transition_handler,
                )
            ),
            jtc_spawner,
        ]
    )
