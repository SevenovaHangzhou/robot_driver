import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).resolve().parents[2] / "hostsetup/rt-control-chassis-can-bind.py"
if not SOURCE.exists():
    SOURCE = Path(__file__).with_name("rt-control-chassis-can-bind.py")
spec = importlib.util.spec_from_file_location("chassis_can_bind", SOURCE)
binding = importlib.util.module_from_spec(spec)
spec.loader.exec_module(binding)
C, B = binding.EXPECTED.values()


class BindingTests(unittest.TestCase):
    def test_correct_names_are_noop(self):
        self.assertEqual(binding.rename_plan({"can0": C, "can1": B}), [])
        binding.apply_plan([], lambda *a, **k: self.fail("must not touch a correct bus"))

    def test_swapped_names_move_aside_before_reusing_names(self):
        plan = binding.rename_plan({"can0": B, "can1": C})
        self.assertEqual(plan, [("can1", "can0"), ("can0", "can1")])
        calls = []

        def run(cmd, **kwargs):
            calls.append(cmd)
            return SimpleNamespace(returncode=1)

        binding.apply_plan(plan, run)
        self.assertEqual(
            calls[1:],
            [
                ["ip", "link", "set", "dev", "can1", "down"],
                ["ip", "link", "set", "dev", "can1", "name", "rtcan_tmp0"],
                ["ip", "link", "set", "dev", "can0", "down"],
                ["ip", "link", "set", "dev", "can0", "name", "rtcan_tmp1"],
                ["ip", "link", "set", "dev", "rtcan_tmp0", "name", "can0"],
                ["ip", "link", "set", "dev", "rtcan_tmp1", "name", "can1"],
            ],
        )

    def test_single_chassis_adapter_supported(self):
        self.assertEqual(binding.rename_plan({"can3": C}), [("can3", "can0")])
        self.assertEqual(binding.rename_plan({"can0": C}), [])

    def test_missing_unknown_duplicate_and_name_collision_rejected(self):
        for interfaces in [
            {"can0": B},
            {"can0": "unknown", "can2": C},
            {"can0": C, "can1": "unknown"},
            {"can0": C, "can2": C},
            {"can1": C, "rtcan_tmp0": "unknown"},
        ]:
            with self.subTest(interfaces=interfaces), self.assertRaises(RuntimeError):
                binding.rename_plan(interfaces)

    def test_running_runtime_or_failed_probe_never_renames(self):
        for code in [0, 2]:
            calls = []

            def run(cmd, **kwargs):
                calls.append(cmd)
                return SimpleNamespace(returncode=code)

            with self.assertRaises(RuntimeError):
                binding.apply_plan([("can1", "can0")], run)
            self.assertEqual(len(calls), 1)

    def test_ip_failure_stops_later_operations(self):
        calls = []

        def run(cmd, **kwargs):
            calls.append(cmd)
            if cmd[0] == "ip":
                raise binding.subprocess.CalledProcessError(1, cmd)
            return SimpleNamespace(returncode=1)

        with self.assertRaises(binding.subprocess.CalledProcessError):
            binding.apply_plan([("can1", "can0")], run)
        self.assertEqual(len(calls), 2)


if __name__ == "__main__":
    unittest.main()
