"""Hardware-free machine selection and explicit manifest readiness gate."""

import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from integ_composer.manifest import validate_selection


def _launch_setup(context):
    validation = LaunchConfiguration("validation_only").perform(context)
    if validation not in ("true", "false"):
        raise ValueError("validation_only must be true or false")
    selected = validate_selection(
        robot_variant=LaunchConfiguration("robot_variant").perform(context),
        machine_unit=LaunchConfiguration("machine_unit").perform(context),
        physical_profile=LaunchConfiguration("physical_profile").perform(context),
        control_scope=LaunchConfiguration("control_scope").perform(context),
        require_runtime_ready=validation == "false",
    )
    if validation == "false":
        raise RuntimeError("Module entry is validation-only; choose an explicit runtime entry")
    return [LogInfo(msg="RT-Control validation: " + json.dumps(selected, ensure_ascii=False))]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_variant", default_value="Alfa_V3"),
            DeclareLaunchArgument("machine_unit", default_value="Alfa_V3-01"),
            DeclareLaunchArgument("physical_profile", default_value="arms_only"),
            DeclareLaunchArgument("control_scope", default_value="arms_only"),
            DeclareLaunchArgument("validation_only", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
