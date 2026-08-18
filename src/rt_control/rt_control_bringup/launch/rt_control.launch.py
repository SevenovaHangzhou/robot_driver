from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
    rolling_envelope_file = PathJoinSubstitution(
        [
            FindPackageShare("rt_control_bringup"),
            "config",
            "rolling_envelope_provisional.yaml",
        ]
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
            {
                "use_sim_time": use_sim_time,
                "rolling_trajectory_controller.envelope_file": ParameterValue(
                    rolling_envelope_file, value_type=str
                ),
            },
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
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "whole_body_jtc",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
        ],
        output="both",
    )
    enable_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["enable_manager", "--controller-manager", "/controller_manager"],
        output="both",
    )

    def start_enable_manager_after_jtc(event, _context):
        if event.returncode == 0:
            return [enable_spawner]
        return [
            LogInfo(
                msg=(
                    "whole_body_jtc did not reach configured INACTIVE state; "
                    "enable_manager will not be loaded and launch will stop"
                )
            ),
            EmitEvent(
                event=Shutdown(reason="whole_body_jtc failed to configure INACTIVE")
            ),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
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
            jtc_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=jtc_spawner,
                    on_exit=start_enable_manager_after_jtc,
                )
            ),
        ]
    )
