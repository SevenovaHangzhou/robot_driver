import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/rt_control/plc_node"))


class VacuumAndStatusContractTest(unittest.TestCase):
    def test_vacuum_idl_uses_fixed_channels_and_no_pressure_field(self):
        interface_root = ROOT / "src/interfaces/robot_rt_control_interfaces"
        texts = "\n".join(
            (
                interface_root / "msg/VacuumState.msg"
            ).read_text().splitlines()
            + (interface_root / "msg/VacuumChannelState.msg").read_text().splitlines()
            + (interface_root / "msg/VacuumChannelFeedback.msg").read_text().splitlines()
            + (interface_root / "msg/VacuumChannelResult.msg").read_text().splitlines()
            + (interface_root / "action/VacuumGrip.action").read_text().splitlines()
        )

        self.assertIn("string LEFT=left", texts)
        self.assertIn("string RIGHT=right", texts)
        self.assertIn("bool attached", texts)
        self.assertIn("bool data_fresh", texts)
        self.assertNotIn("pressure_pa", texts)

    def test_plc_vacuum_inputs_remain_confirmed_di_bit0_bit1(self):
        from plc_node.plc_logic import decode_io_snapshot

        left = decode_io_snapshot(di_status=0b01, do_status=0, io_alarm=0)
        right = decode_io_snapshot(di_status=0b10, do_status=0, io_alarm=0)

        self.assertTrue(left.left_vacuum_established)
        self.assertFalse(left.right_vacuum_established)
        self.assertFalse(right.left_vacuum_established)
        self.assertTrue(right.right_vacuum_established)



if __name__ == "__main__":
    unittest.main()
