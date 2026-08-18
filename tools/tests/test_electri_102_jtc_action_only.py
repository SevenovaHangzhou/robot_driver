"""ELECTRI-102 JTC command-surface policy tests."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / "patches/ros2_controllers/0001-jtc-start-consistency.patch"
CONTROLLERS = ROOT / "src/rt_control/rt_control_bringup/config/controllers.yaml"


def test_consistency_check_disables_unadmitted_topic_surface() -> None:
    patch = PATCH.read_text(encoding="utf-8")

    assert "JTC_TOPIC_COMMAND_DISABLED" in patch
    assert "if (!params_.trajectory_start_consistency_check.enabled)" in patch
    assert '"~/joint_trajectory"' in patch


def test_whole_body_jtc_enables_action_only_admission() -> None:
    document = yaml.safe_load(CONTROLLERS.read_text(encoding="utf-8"))
    parameters = document["whole_body_jtc"]["ros__parameters"]

    assert parameters["trajectory_start_consistency_check"]["enabled"] is True
