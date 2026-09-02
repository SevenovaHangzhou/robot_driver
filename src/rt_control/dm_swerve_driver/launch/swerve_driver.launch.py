from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    driver = LifecycleNode(
        package="dm_swerve_driver",
        executable="swerve_driver_node",
        name="swerve_driver",
        namespace="",
        output="screen",
        parameters=[params_file],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action == driver,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action == driver,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("dm_swerve_driver"), "config", "swerve_params.yaml"]
                ),
            ),
            RegisterEventHandler(OnProcessStart(target_action=driver, on_start=[configure])),
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=driver,
                    goal_state="inactive",
                    entities=[activate],
                )
            ),
            driver,
        ]
    )
