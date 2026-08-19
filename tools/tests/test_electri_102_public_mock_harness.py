import json
import os
import subprocess
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


class PublicMockHarnessTest(unittest.TestCase):
    def test_complete_motion_workflow_uses_public_dds_contract(self):
        environment = dict(os.environ)
        environment["ROS_DOMAIN_ID"] = str(50 + os.getpid() % 150)
        environment["ROS_LOCALHOST_ONLY"] = "1"
        completed = subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools/electri_102_public_mock_harness.py"),
                "--run-seconds",
                "0.2",
                "--timeout-seconds",
                "10",
            ],
            capture_output=True,
            text=True,
            env=environment,
            timeout=20,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, msg=completed.stderr)
        payload = json.loads(completed.stdout)
        self.assertEqual(payload["result"], "PASS")
        self.assertFalse(payload["hardware_started"])
        self.assertFalse(payload["controller_manager_started"])
        self.assertGreaterEqual(payload["accepted_batches"], 2)
        self.assertEqual(
            payload["mode_transitions"],
            ["FJT_READY->ROLLING_READY", "ROLLING_READY->FJT_READY"],
        )


if __name__ == "__main__":
    unittest.main()
