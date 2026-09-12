"""Safe machine-profile validation entry point for modular bring-up.

The ELECTRI-118 manifest is intentionally draft-only until the pending drive
identities and mappings are supplied. This launch file therefore supports a
hardware-free validation mode and refuses to create ROS nodes for a draft
runtime selection.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # noqa: F401 - kept as the node seam for the next phase

from rt_control_bringup.machine_profile import (
    MachineProfileError,
    load_machine_manifest,
    select_hardware,
)


def _boolean_argument(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context)
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _mode_summary(mode_counts: dict[int, int]) -> str:
    labels = {1: "PP", 8: "CSP", 9: "CSV", 10: "CST"}
    return ", ".join(
        f"{labels.get(mode, f'mode_{mode}')}={count}"
        for mode, count in sorted(mode_counts.items())
    ) or "none"


def _layout_summary(values) -> str:
    if values is None:
        return "TBD"
    return ",".join(str(value) for value in values) or "none"


def _group_summary(group_counts: dict[str, int]) -> str:
    return ", ".join(
        f"{name}={count}" for name, count in group_counts.items()
    ) or "none"


def _manifest_path(robot_variant: str) -> Path:
    if not robot_variant or "/" in robot_variant or ".." in robot_variant:
        raise ValueError("robot_variant must be a package-owned identifier")
    return (
        Path(get_package_share_directory("rt_control_bringup"))
        / "config"
        / "machines"
        / f"{robot_variant}.yaml"
    )


def _launch_setup(context):
    validation_only = _boolean_argument(context, "validation_only")
    robot_variant = LaunchConfiguration("robot_variant").perform(context)
    physical_profile = LaunchConfiguration("physical_profile").perform(context)
    control_scope = LaunchConfiguration("control_scope").perform(context)

    try:
        manifest = load_machine_manifest(_manifest_path(robot_variant))
        selected = select_hardware(
            manifest,
            physical_profile=physical_profile,
            control_scope=control_scope,
            require_runtime_ready=not validation_only,
        )
    except (MachineProfileError, OSError, ValueError) as error:
        if validation_only:
            raise RuntimeError(f"invalid machine selection: {error}") from error
        raise RuntimeError(f"machine selection is not runtime-ready: {error}") from error

    if not validation_only:
        raise RuntimeError(
            "ELECTRI-118 modular runtime composition is static validation only; "
            "no hardware nodes were created"
        )

    inactive = ",".join(selected.inactive_modules) or "none"
    message = (
        f"ELECTRI-118 validation: robot_variant={selected.manifest_variant} "
        f"physical_profile={selected.physical_profile} "
        f"control_scope={selected.control_scope} "
        f"active={','.join(selected.active_modules)} "
        f"inactive={inactive} "
        f"actuators={selected.actuator_count} "
        f"modes={_mode_summary(dict(selected.mode_counts))} "
        f"groups={_group_summary(dict(selected.group_counts))} "
        f"jtc_modes={_mode_summary(dict(selected.jtc_mode_counts))} "
        f"state_sensors={selected.state_sensor_count} "
        f"ethercat_master={selected.ethercat_master_id} "
        f"ethercat_ring={_layout_summary(selected.ethercat_ring_positions)} "
        f"canopen_nodes={_layout_summary(selected.canopen_node_ids)} "
        f"damiao_can_nodes={_layout_summary(selected.damiao_can_node_ids)} "
        f"readiness={selected.global_readiness} "
        f"status={selected.validation_status}"
    )
    return [LogInfo(msg=message)]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_variant", default_value="alfa_v3"),
            DeclareLaunchArgument("physical_profile", default_value="arms_only"),
            DeclareLaunchArgument("control_scope", default_value="arms_only"),
            DeclareLaunchArgument("validation_only", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
