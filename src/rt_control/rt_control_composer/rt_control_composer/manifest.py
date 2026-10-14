"""Offline manifest selection for the validation entry; no rendering or devices."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .schema.cross_checks import manifest_links, runtime_admission
from .schema.manifest_load import load_machine


def validate_selection(
    *,
    robot_variant,
    machine_unit,
    physical_profile,
    control_scope,
    require_runtime_ready=False,
    config_share=None,
    package_share=get_package_share_directory,
):
    root = (
        Path(package_share("rt_control_machine_config"))
        if config_share is None
        else Path(config_share)
    )
    schemas = Path(package_share("rt_control_composer")) / "schema"
    data = load_machine(root, schemas, robot_variant, machine_unit, physical_profile, control_scope)
    manifest_links(data, root)
    if require_runtime_ready:
        runtime_admission(data, False, require_manifest_ready=True)
    scope = data["scope"]
    return {
        "robot_variant": robot_variant,
        "machine_unit": machine_unit,
        "physical_profile": physical_profile,
        "control_scope": control_scope,
        "active_modules": list(data["modules"]),
        "hardware_options": data["deployment"]["hardware_options"],
        "actuator_count": sum(
            data["manifest"]["modules"][name]["actuator_count"] for name in data["modules"]
        ),
        "state_sensor_count": sum(
            data["manifest"]["modules"][name]["sensor_count"] for name in data["modules"]
        ),
        "actuator_groups": scope["actuator_groups"],
        "jtc_groups": scope["jtc_groups"],
        "fault_dependencies": data["safety_policy"]["fault_dependencies"],
        "status": data["profile"]["status"],
    }
