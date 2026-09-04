from pathlib import Path

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
from rt_control_bringup.hardware_composition import (
    load_hardware_variants,
    validate_controller_compatibility,
    variant_descriptor_path,
)


def _raise_required_spawner_failure(
    _context, *, controller_name, expected_state
):
    raise RuntimeError(
        f"{controller_name} failed to reach {expected_state} state"
    )


def _start_next_spawner_or_stop(
    controller_name, expected_state, next_spawner
):
    def on_exit(event, _context):
        if event.returncode == 0:
            return [next_spawner] if next_spawner is not None else []
        reason = f"{controller_name} failed to reach {expected_state} state"
        return [
            LogInfo(
                msg=(
                    f"{controller_name} did not reach {expected_state} state; "
                    "launch will stop with failure"
                )
            ),
            EmitEvent(event=Shutdown(reason=reason)),
            OpaqueFunction(
                function=_raise_required_spawner_failure,
                kwargs={
                    "controller_name": controller_name,
                    "expected_state": expected_state,
                },
            ),
        ]

    return on_exit


def _launch_setup(context):
    use_mock_hardware_value = LaunchConfiguration("use_mock_hardware").perform(
        context
    )
    if use_mock_hardware_value not in {"true", "false"}:
        raise ValueError("use_mock_hardware must be true or false")

    bringup_share = Path(get_package_share_directory("rt_control_bringup"))
    ethercat_variant_name = LaunchConfiguration("ethercat_variant").perform(context)
    canopen_variant_name = LaunchConfiguration("canopen_variant").perform(context)
    hardware_composition = load_hardware_variants(
        variant_descriptor_path(
            Path(get_package_share_directory("robot_hw_ethercat")),
            ethercat_variant_name,
        ),
        variant_descriptor_path(
            Path(get_package_share_directory("robot_hw_canopen")),
            canopen_variant_name,
        ),
    )
    controllers_path = bringup_share / "config/controllers.yaml"
    validate_controller_compatibility(hardware_composition, controllers_path)
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = use_mock_hardware_value
    ethercat_variant = LaunchConfiguration("ethercat_variant")
    canopen_variant = LaunchConfiguration("canopen_variant")
    start_plc = LaunchConfiguration("start_plc")
    start_bms = LaunchConfiguration("start_bms")
    description_file = PathJoinSubstitution(
        [FindPackageShare("rt_control_bringup"), "urdf", "rt_control.urdf.xacro"]
    )
    controllers_file = str(controllers_path)
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
                    " ethercat_variant:=",
                    ethercat_variant,
                    " canopen_variant:=",
                    canopen_variant,
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
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel_safe"),
            ("/diff_drive_controller/odom", "/wheel/odom"),
        ],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
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
                "dynamic_joint_states_topic": (
                    "/rt_internal_state_broadcaster/dynamic_joint_states"
                ),
                "use_sim_time": use_sim_time,
                **hardware_composition.diagnostics_parameters(),
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

    active_controller_names = (
        "joint_state_broadcaster",
        "rt_internal_state_broadcaster",
        "diff_drive_controller",
    )
    active_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="both",
        )
        for name in active_controller_names
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

    spawner_sequence = [
        *[
            (controller_name, "ACTIVE", spawner)
            for controller_name, spawner in zip(
                active_controller_names, active_spawners
            )
        ],
        ("whole_body_jtc", "configured INACTIVE", jtc_spawner),
        ("enable_manager", "ACTIVE", enable_spawner),
    ]
    spawner_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawner,
                on_exit=_start_next_spawner_or_stop(
                    controller_name,
                    expected_state,
                    (
                        spawner_sequence[index + 1][2]
                        if index + 1 < len(spawner_sequence)
                        else None
                    ),
                ),
            )
        )
        for index, (controller_name, expected_state, spawner) in enumerate(
            spawner_sequence
        )
    ]

    return [
        control_node,
        state_publisher,
        diagnostics,
        control_adapter,
        vacuum_adapter,
        rt_status_adapter,
        plc,
        bms,
        *spawner_handlers,
        active_spawners[0],
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("ethercat_variant", default_value="alfa_v1"),
            DeclareLaunchArgument("canopen_variant", default_value="alfa_v1"),
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
