import math
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
BUS_CONFIG = ROOT / "src/rt_control/robot_hw_canopen/config/bus.yml"
CONTROLLERS_CONFIG = (
    ROOT / "src/rt_control/rt_control_bringup/config/controllers.yaml"
)


class TrackMechanicsContractTest(unittest.TestCase):
    def test_track_scale_uses_1044_mm_sprocket_radius_once(self):
        bus = yaml.safe_load(BUS_CONFIG.read_text(encoding="utf-8"))
        controllers = yaml.safe_load(
            CONTROLLERS_CONFIG.read_text(encoding="utf-8")
        )

        sprocket_radius_m = 0.1044
        motor_revolutions_per_sprocket_revolution = 40.0
        drive_units_per_motor_revolution = 10000.0
        expected_to_device = -(
            motor_revolutions_per_sprocket_revolution
            * drive_units_per_motor_revolution
            / (2.0 * math.pi * sprocket_radius_m)
        )
        expected_from_device = 1.0 / expected_to_device

        for joint_name in ("left_track_joint", "right_track_joint"):
            node = bus["nodes"][joint_name]
            self.assertAlmostEqual(
                node["scale_pos_to_dev"], expected_to_device, places=9
            )
            self.assertAlmostEqual(
                node["scale_vel_to_dev"], expected_to_device, places=9
            )
            self.assertAlmostEqual(
                node["scale_pos_from_dev"], expected_from_device, places=18
            )
            self.assertAlmostEqual(
                node["scale_vel_from_dev"], expected_from_device, places=18
            )
            self.assertEqual(node["offset_pos_to_dev"], 0.0)
            self.assertEqual(node["offset_pos_from_dev"], 0.0)

        diff_drive = controllers["diff_drive_controller"]["ros__parameters"]
        self.assertEqual(diff_drive["wheel_radius"], 1.0)
        self.assertEqual(diff_drive["wheel_separation"], 1.9598)


if __name__ == "__main__":
    unittest.main()
