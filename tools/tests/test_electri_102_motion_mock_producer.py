import json
import math
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import electri_102_motion_mock_producer as producer


class HoldSuffixTest(unittest.TestCase):
    def test_axis_order_and_digest_match_the_public_contract(self):
        self.assertEqual(
            producer.AXIS_NAMES,
            (
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
            ),
        )
        self.assertEqual(
            producer.AXIS_SET_HASH.hex(),
            "25c6e82bf505ca9eb99db1c645ab75d7ecde0153faaf6a7492c6210c4d362526",
        )

    def test_six_hold_knots_have_exact_100ms_spacing_and_500ms_horizon(self):
        positions = tuple(float(index) / 10.0 for index in range(14))
        knots = producer.build_hold_suffix(16_000_000, positions)

        self.assertEqual(len(knots), 6)
        self.assertEqual(knots[0].time_ns, 16_000_000)
        self.assertEqual(knots[-1].time_ns, 516_000_000)
        self.assertTrue(all(knot.positions == positions for knot in knots))
        self.assertTrue(all(knot.velocities == (0.0,) * 14 for knot in knots))

    def test_invalid_hold_input_fails_before_ros_publication(self):
        with self.assertRaisesRegex(ValueError, "14 finite"):
            producer.build_hold_suffix(0, (0.0,) * 13)
        invalid = [0.0] * 14
        invalid[4] = math.nan
        with self.assertRaisesRegex(ValueError, "14 finite"):
            producer.build_hold_suffix(0, invalid)
        with self.assertRaisesRegex(ValueError, "point_count"):
            producer.build_hold_suffix(0, (0.0,) * 14, point_count=1)


class PublicContractExampleTest(unittest.TestCase):
    def test_example_never_depends_on_driver_private_interfaces(self):
        source = (ROOT / "tools/electri_102_motion_mock_producer.py").read_text()
        self.assertNotIn("from rt_control_interfaces", source)
        self.assertNotIn("import rt_control_interfaces", source)
        self.assertIn("robot_motion_interfaces.msg", source)
        self.assertIn("robot_rt_control_interfaces.srv", source)
        self.assertIn("robot_interfaces_qos", source)

    def test_default_invocation_is_dry_run_and_never_initializes_ros(self):
        completed = subprocess.run(
            [sys.executable, str(ROOT / "tools/electri_102_motion_mock_producer.py")],
            capture_output=True,
            text=True,
            check=True,
        )
        payload = json.loads(completed.stdout)
        self.assertFalse(payload["command_publication_enabled"])
        self.assertEqual(payload["batch_rate_hz"], 30)
        self.assertEqual(payload["knot_interval_ms"], 100)
        self.assertEqual(payload["point_count"], 6)
        self.assertEqual(payload["initial_horizon_ms"], 500)


if __name__ == "__main__":
    unittest.main()
