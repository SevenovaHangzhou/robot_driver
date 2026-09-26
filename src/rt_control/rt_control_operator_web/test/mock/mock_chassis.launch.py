"""ELECTRI-109 Mock-only chassis loop for the operator web console.

Starts a software-only ros2_control stack (mock_components, no device access),
the swerve controller with synthetic parameters, synthetic battery/ultrasonic
publishers, and the operator web server.
This file is intentionally NOT installed and NOT part of any production launch.

Usage (after sourcing the workspace):
  RT_OPERATOR_WEB_TOKEN_FILE=<chmod-600 file> ROS_DOMAIN_ID=<isolated> \
    ros2 launch <path>/mock_chassis.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = HERE.parents[1]


def generate_launch_description() -> LaunchDescription:
    robot_description = (HERE / "chassis_mock.urdf").read_text(encoding="utf-8")
    controllers = str(HERE / "mock_controllers.yaml")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers],
        output="screen",
    )
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_swerve = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    # Publishes the shared V3 Robot Model on /robot_description (latched) for the
    # operator top view, exactly as the production stack does.
    v3_urdf = (
        Path(get_package_share_directory("robot_description")) / "urdf" / "robot.urdf"
    ).read_text(encoding="utf-8")
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": v3_urdf}],
        output="screen",
    )
    operator = Node(
        package="rt_control_operator_web",
        executable="rt_control_operator_web",
        name="rt_control_operator_web",
        parameters=[
            LaunchConfiguration("operator_params"),
            {"static_dir": LaunchConfiguration("static_dir")},
        ],
        output="screen",
    )
    sensors = ExecuteProcess(
        cmd=["python3", str(HERE / "mock_sensors.py")],
        name="operator_web_mock_sensors",
        output="screen",
        condition=IfCondition(LaunchConfiguration("mock_sensors")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mock_sensors", default_value="true"),
            DeclareLaunchArgument(
                "operator_params", default_value=str(HERE / "mock_operator_web.yaml")
            ),
            DeclareLaunchArgument("static_dir", default_value=str(PACKAGE_ROOT / "web")),
            control_node,
            spawn_jsb,
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_swerve])
            ),
            state_publisher,
            operator,
            sensors,
        ]
    )
