"""Build the V3 head-only robot description and non-JTC controller configuration."""

from dataclasses import dataclass
from pathlib import Path

import xacro
import yaml


@dataclass(frozen=True)
class HeadRuntime:
    robot_description: str
    controllers: dict


def _mappings(config, description_share, hardware_share, use_mock_hardware):
    result = {
        "description_file": str(Path(description_share) / "urdf/robot_dual_gripper.urdf.xacro"),
        "hardware_file": str(Path(hardware_share) / "urdf/damiao.ros2_control.xacro"),
        "use_mock_hardware": "true" if use_mock_hardware else "false",
        "head_can_interface": config["can_interface"],
    }
    for field in (
        "configure_timeout_ms", "feedback_timeout_ms", "transition_timeout_ms",
        "command_rate_hz", "disabled_poll_interval_ms", "max_rx_frames_per_cycle",
    ):
        result[f"head_{field}"] = str(config[field])
    for index, joint in enumerate(config["joints"], 1):
        for field in (
            "name", "min", "max", "velocity_limit", "acceleration_krad_s2",
            "deceleration_krad_s2", "maximum_speed_rad_s",
        ):
            result[f"head_joint_{index}_{field}"] = str(joint[field])
        result[f"head_motor_{index}_can_id"] = str(joint["can_id"])
        result[f"head_motor_{index}_master_id"] = str(joint["master_id"])
    return result


def build_head_runtime(
    *, config, description_share: str | Path, hardware_share: str | Path,
    controller_share: str | Path, use_mock_hardware: bool,
    bringup_share: str | Path | None = None,
) -> HeadRuntime:
    root = Path(bringup_share) if bringup_share is not None else Path(__file__).resolve().parents[1]
    wrapper = root / "urdf/alfa_v3_head.ros2_control.xacro"
    document = xacro.process_file(
        str(wrapper),
        mappings=_mappings(config, description_share, hardware_share, use_mock_hardware),
    )
    controllers_path = Path(controller_share) / "config/controllers.yaml"
    controllers = yaml.safe_load(controllers_path.read_text(encoding="utf-8"))
    return HeadRuntime(document.toxml(), controllers)
