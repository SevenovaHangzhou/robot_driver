"""Static by default; runtime is an explicit standalone commissioning action."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _setup(context):
    config_file = LaunchConfiguration("config_file").perform(context)
    parameters = yaml.safe_load(Path(config_file).read_text())["/**"]["ros__parameters"]
    if LaunchConfiguration("validation_only").perform(context).lower() == "true":
        return [LogInfo(msg="LPMS-NAV3 draft: validation_only=true; no CAN node started")]
    interface = parameters.get("can_interface", "")
    frame = parameters.get("frame_id", "")
    node_id = parameters.get("node_id", 0)
    if (
        not isinstance(interface, str)
        or not interface
        or interface == "TBD"
        or len(interface) >= 16
        or interface in {"can0", "can1"}
        or type(node_id) is not int
        or not 1 <= node_id <= 127
        or not isinstance(frame, str)
        or not frame
        or frame == "TBD"
        or frame.startswith("/")
        or any(char.isspace() for char in frame)
    ):
        raise ValueError("LPMS runtime requires explicit dedicated interface, node_id and frame_id")
    return [
        Node(
            package="lpms_nav3_can",
            executable="lpms_nav3_can_node",
            namespace="imu",
            name="lpms_nav3_can_node",
            output="screen",
            parameters=[parameters],
        )
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("lpms_nav3_can"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("validation_only", default_value="true"),
            DeclareLaunchArgument(
                "config_file", default_value=str(share / "config/lpms_nav3_can.yaml")
            ),
            OpaqueFunction(function=_setup),
        ]
    )
