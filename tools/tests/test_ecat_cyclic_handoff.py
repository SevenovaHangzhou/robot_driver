import os
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


class CyclicHandoffContractTest(unittest.TestCase):
    def test_native_and_docker_enable_the_same_dc_timing_defaults(self):
        patch = "patches/ecat_icube/0011-dc-rate-diagnostics.patch"
        self.assertTrue((ROOT / patch).is_file())
        for path in ["tools/bootstrap_native_dev.sh", "docker/rt-control/Dockerfile"]:
            self.assertEqual((ROOT / path).read_text().count(patch), 2, path)
            self.assertEqual((ROOT / path).read_text().count(
                "patches/ecat_icube/0012-fix-timing-source-lint.patch"), 2, path)
        docker = (ROOT / "docker/rt-control/Dockerfile").read_text()
        for setting, value in [("RT_CONTROL_ECAT_CONTINUOUS_HANDOFF", "1"),
                               ("RT_CONTROL_ECAT_SYNC0_SHIFT_NS", "0"),
                               ("RT_CONTROL_ECAT_EXPLICIT_SEND_INTERVAL", "1")]:
            self.assertIn(f'ENV {setting}="{value}"', docker)
        self.assertNotIn("ENV RT_CONTROL_ECAT_ACTIVATE_AT_NS", docker)
        compose = yaml.safe_load((ROOT / "docker/compose.yaml").read_text())
        environment = compose["services"]["rt-control"]["environment"]
        self.assertTrue(environment["RT_CONTROL_ECAT_STARTUP_CPU"].startswith(
            "${RT_CONTROL_ECAT_STARTUP_CPU:?"))

    def test_native_and_docker_register_the_same_patch(self):
        patch = "patches/ecat_icube/0010-maintain-cyclic-handoff.patch"
        self.assertTrue((ROOT / patch).is_file())
        for path in ["tools/bootstrap_native_dev.sh", "docker/rt-control/Dockerfile"]:
            self.assertEqual((ROOT / path).read_text().count(patch), 2, path)

    def test_maintenance_does_not_publish_interfaces_or_advance_preload(self):
        source_root = os.environ.get("ECAT_STARTUP_RT_SOURCE")
        if not source_root:
            self.skipTest("set ECAT_STARTUP_RT_SOURCE to the frozen patched source")
        source = (Path(source_root) / "ethercat_interface/src/ec_master.cpp").read_text()
        body = source.split("bool EcMaster::maintainData(", 1)[1].split("\n}\n", 1)[0]
        self.assertNotIn("processData(", body)
        self.assertNotIn("onPdoCycleSent(", body)
        self.assertIn("CLOCK_MONOTONIC", body)
        self.assertIn("ecrt_master_sync_reference_clock", body)
        self.assertIn("ecrt_master_sync_slave_clocks", body)

    def test_normal_read_and_write_cannot_join_or_wait_for_handoff(self):
        patch = (ROOT / "patches/ecat_icube/0010-maintain-cyclic-handoff.patch").read_text()
        self.assertNotIn("ecrt_slave_config_dc", patch)
        source_root = os.environ.get("ECAT_STARTUP_RT_SOURCE")
        if not source_root:
            self.skipTest("set ECAT_STARTUP_RT_SOURCE to the frozen patched source")
        source = (Path(source_root) / "ethercat_driver/src/ethercat_driver.cpp").read_text()
        for method in ["read", "write"]:
            body = source.split(f"EthercatDriver::{method}(\n", 1)[1].split("\n}\n", 1)[0]
            self.assertIn("handoff_.", body)
            for operation in ["join(", "sleep", "wait(", "wait_for(", "stopHandoff(", "exportSendTrace("]:
                self.assertNotIn(operation, body)
