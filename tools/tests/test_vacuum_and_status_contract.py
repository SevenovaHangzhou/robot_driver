import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/rt_control/plc_node"))


class VacuumAndStatusContractTest(unittest.TestCase):
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
