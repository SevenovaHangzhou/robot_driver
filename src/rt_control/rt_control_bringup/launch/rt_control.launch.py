from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    description_file = PathJoinSubstitution(
        [FindPackageShare("rt_control_bringup"), "urdf", "rt_control.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("rt_control_bringup"), "config", "controllers.yaml"]
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
        parameters=[robot_description, controllers_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    diagnostics = Node(
        package="rt_diagnostics",
        executable="rt_diagnostics_node",
        output="both",
        parameters=[{"hardware_id": "robot-001", "use_sim_time": use_sim_time}],
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
            "diff_drive_controller",
        )
    ]
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dual_arm_jtc",
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
                    "dual_arm_jtc did not reach configured INACTIVE state; "
                    "enable_manager will not be loaded and launch will stop"
                )
            ),
            EmitEvent(
                event=Shutdown(reason="dual_arm_jtc failed to configure INACTIVE")
            ),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            control_node,
            state_publisher,
            diagnostics,
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
