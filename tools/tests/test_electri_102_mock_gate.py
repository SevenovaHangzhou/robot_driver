import json
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import electri_102_mock_gate as gate


class ScenarioCatalogTest(unittest.TestCase):
    def test_catalog_covers_every_required_electri_102_mock_scenario(self):
        scenario_ids = {scenario.id for scenario in gate.scenario_catalog()}
        self.assertEqual(
            scenario_ids,
            {
                "continuous_update_soak",
                "frequency_jitter",
                "update_interruption_recovery",
                "out_of_order_sequence",
                "duplicate_sequence",
                "late_replacement",
                "capacity_exhaustion",
                "low_water",
                "prime_timeout",
                "clock_anomaly",
                "exit_realtime_mode",
                "session_during_disable",
                "mode_switch_safety",
                "rt_zero_allocation",
            },
        )

    def test_soak_command_uses_exact_fake_rate_and_requested_wall_duration(self):
        soak = next(
            scenario
            for scenario in gate.scenario_catalog()
            if scenario.id == "continuous_update_soak"
        )
        command, environment = gate.build_command(
            soak,
            Path("/tmp/build"),
            Path("/tmp/evidence"),
            soak_seconds=600,
            realtime=True,
        )
        self.assertEqual(environment["E102_SOAK_CYCLES"], "150000")
        self.assertEqual(environment["E102_SOAK_REALTIME"], "1")
        self.assertEqual(
            environment["E102_SOAK_REPORT"],
            "/tmp/evidence/continuous_update_soak.metrics.json",
        )
        self.assertIn("--gtest_filter=" + soak.gtest_filter, command)

    def test_unknown_selection_is_rejected_instead_of_silently_skipped(self):
        with self.assertRaisesRegex(ValueError, "unknown scenario"):
            gate.select_scenarios(["not_a_scenario"])


class ScenarioExecutionTest(unittest.TestCase):
    @staticmethod
    def _install_fake_gtest(build_root: Path, package: str, binary: str, exit_code: int):
        executable = build_root / package / binary
        executable.parent.mkdir(parents=True)
        executable.write_text(
            "#!/usr/bin/env python3\n"
            "import json, os, pathlib, sys\n"
            "xml_arg = next(a for a in sys.argv if a.startswith('--gtest_output=xml:'))\n"
            "xml_path = pathlib.Path(xml_arg.split(':', 1)[1])\n"
            "xml_path.write_text('<testsuites tests=\"1\" failures=\"0\"/>')\n"
            "metrics = os.environ.get('E102_SOAK_REPORT')\n"
            "if metrics:\n"
            "    payload = {'passed': True, 'cycles_requested': 250, "
            "'cycles_completed': 250, 'late_replace_count': 0, "
            "'rt_allocation_count': 0, 'invariant_failure_count': 0}\n"
            "    pathlib.Path(metrics).write_text(json.dumps(payload))\n"
            f"sys.exit({exit_code})\n"
        )
        executable.chmod(0o755)

    def test_success_preserves_log_xml_and_validated_soak_metrics(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            build_root = root / "build"
            evidence = root / "evidence"
            scenario = next(
                item
                for item in gate.scenario_catalog()
                if item.id == "continuous_update_soak"
            )
            self._install_fake_gtest(
                build_root, scenario.package, scenario.binary, exit_code=0
            )

            result = gate.run_scenario(
                scenario,
                build_root,
                evidence,
                soak_seconds=1,
                realtime=False,
                inherited_environment=os.environ,
            )

            self.assertEqual(result["status"], "PASS")
            self.assertTrue((evidence / "continuous_update_soak.log").is_file())
            self.assertTrue((evidence / "continuous_update_soak.junit.xml").is_file())
            self.assertEqual(result["metrics"]["cycles_completed"], 250)

    def test_nonzero_exit_is_a_machine_readable_failure(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            build_root = root / "build"
            evidence = root / "evidence"
            scenario = next(
                item
                for item in gate.scenario_catalog()
                if item.id == "frequency_jitter"
            )
            self._install_fake_gtest(
                build_root, scenario.package, scenario.binary, exit_code=7
            )

            result = gate.run_scenario(
                scenario,
                build_root,
                evidence,
                soak_seconds=1,
                realtime=False,
                inherited_environment=os.environ,
            )

            self.assertEqual(result["status"], "FAIL")
            self.assertEqual(result["exit_code"], 7)
            self.assertIn("frequency_jitter.log", result["artifacts"])

    def test_missing_binary_fails_closed(self):
        scenario = gate.scenario_catalog()[0]
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            result = gate.run_scenario(
                scenario,
                root / "build",
                root / "evidence",
                soak_seconds=1,
                realtime=False,
                inherited_environment=os.environ,
            )
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("missing executable", result["detail"])


class ReportTest(unittest.TestCase):
    def test_soak_metrics_fail_closed_on_any_safety_counter(self):
        valid = {
            "passed": True,
            "cycles_requested": 250,
            "cycles_completed": 250,
            "late_replace_count": 0,
            "rt_allocation_count": 0,
            "invariant_failure_count": 0,
        }
        self.assertEqual(gate.validate_soak_metrics(valid), [])
        invalid = dict(valid, rt_allocation_count=1)
        self.assertIn("rt_allocation_count=1", gate.validate_soak_metrics(invalid))

    def test_summary_and_aggregate_junit_preserve_all_failures(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory)
            results = [
                {"id": "a", "title": "A", "status": "PASS", "duration_ms": 4},
                {
                    "id": "b",
                    "title": "B",
                    "status": "FAIL",
                    "duration_ms": 5,
                    "detail": "boom",
                },
            ]
            summary = gate.build_summary(results, source_commit="f" * 40)
            junit_path = output / "gate.junit.xml"
            gate.write_aggregate_junit(results, junit_path)

            self.assertEqual(summary["verdict"], "FAIL")
            self.assertEqual(summary["counts"], {"pass": 1, "fail": 1})
            json.dumps(summary)
            suite = ET.parse(junit_path).getroot()
            self.assertEqual(suite.attrib["failures"], "1")
            self.assertEqual(len(suite.findall("testcase")), 2)


if __name__ == "__main__":
    unittest.main()
