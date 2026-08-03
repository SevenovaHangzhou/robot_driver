#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml

from tools.rt_control_axis_state_check import (
    AxisStateError,
    _load_first_document,
    check_axis_states,
)


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "tools/rt_control_ipc.sh"

AXES = (
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "turn",
    "updown",
)
TI5_AXES = {"right_joint2", "right_joint3", "left_joint2", "left_joint3"}


def dynamic_state(status_words):
    return {
        "joint_names": list(AXES),
        "interface_values": [
            {
                "interface_names": ["position", "status_word"],
                "values": [0.0, float(status_words[joint])],
            }
            for joint in AXES
        ],
    }


class AxisStateCheckTest(unittest.TestCase):
    def test_disabled_accepts_only_the_frozen_ti5_terminal_exception(self):
        words = {
            joint: (0x0021 if joint in TI5_AXES else 0x0040) for joint in AXES
        }

        observed = check_axis_states(dynamic_state(words), "disabled")

        self.assertEqual(observed, words)

    def test_disabled_rejects_ready_to_switch_on_for_a_non_ti5_axis(self):
        words = {joint: 0x0040 for joint in AXES}
        words["updown"] = 0x0021

        with self.assertRaisesRegex(
            AxisStateError,
            r"updown.*raw=0x0021.*expected=SwitchOnDisabled\(0x0040\)",
        ):
            check_axis_states(dynamic_state(words), "disabled")

    def test_enabled_requires_operation_enabled_on_all_fourteen_axes(self):
        words = {joint: 0x0027 for joint in AXES}
        self.assertEqual(check_axis_states(dynamic_state(words), "enabled"), words)

        words["right_joint3"] = 0x0023
        with self.assertRaisesRegex(
            AxisStateError,
            r"right_joint3.*raw=0x0023.*expected=OperationEnabled\(0x0027\)",
        ):
            check_axis_states(dynamic_state(words), "enabled")

    def test_rejects_missing_or_non_integral_status_words(self):
        message = dynamic_state({joint: 0x0040 for joint in AXES})
        message["interface_values"][4]["interface_names"] = ["position"]
        message["interface_values"][4]["values"] = [0.0]
        with self.assertRaisesRegex(AxisStateError, "right_joint5.*missing status_word"):
            check_axis_states(message, "disabled")

        message = dynamic_state({joint: 0x0027 for joint in AXES})
        message["interface_values"][0]["values"][1] = 39.5
        with self.assertRaisesRegex(AxisStateError, "right_joint1.*not an integer"):
            check_axis_states(message, "enabled")

    def test_loader_ignores_ros2_echo_loss_notice_before_yaml_document(self):
        message = dynamic_state({joint: 0x0040 for joint in AXES})
        stream = "\n".join(
            (
                "A message was lost!!!",
                "\ttotal count change:1",
                "\ttotal count: 1",
                "---",
                yaml.safe_dump(message),
            )
        )

        document = _load_first_document(stream)

        self.assertEqual(check_axis_states(document, "disabled"), {joint: 0x0040 for joint in AXES})


class RecoveryLauncherContractTest(unittest.TestCase):
    def test_recovery_is_explicit_and_follows_the_frozen_order(self):
        text = LAUNCHER.read_text(encoding="utf-8")
        begin = text.index("recover_power_loss()")
        end = text.index("print_help()", begin)
        recovery = text[begin:end]

        ordered = (
            "call_rt_service disable",
            "compose stop rt-control",
            "confirm_power_loss_recovery_authorization",
            "compose up -d --no-build rt-control",
            "call_rt_service reset_fault",
            "check_axis_states disabled",
            "call_rt_service enable",
            "check_axis_states enabled",
        )
        offsets = [recovery.index(token) for token in ordered]
        self.assertEqual(offsets, sorted(offsets))
        self.assertIn("recover-power-loss) recover_power_loss", text)

    def test_ordinary_start_still_never_resets_faults(self):
        text = LAUNCHER.read_text(encoding="utf-8")
        begin = text.index("start_rt_control()")
        end = text.index("recover_power_loss()", begin)

        self.assertNotIn("call_rt_service reset_fault", text[begin:end])

    def test_reset_ready_gate_rejects_restart_required_before_idle(self):
        text = LAUNCHER.read_text(encoding="utf-8")
        begin = text.index("wait_for_enable_manager_reset_ready()")
        end = text.index("strip_ansi()", begin)
        body = text[begin:end]

        self.assertIn("value: restart_required", body)
        self.assertLess(body.index("value: FAILED"), body.index("value: IDLE"))
        self.assertLess(body.index("value: restart_required"), body.index("value: IDLE"))


if __name__ == "__main__":
    unittest.main()
