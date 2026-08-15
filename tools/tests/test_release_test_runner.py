import json
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import release_test_runner as rtr

CASES_DIR = ROOT / "domains/rt_control/testing/cases"


def make_case(**overrides):
    case = {
        "id": "TC-XX-99",
        "title": "fixture",
        "purpose": "fixture",
        "area": "governance",
        "risk": "T0",
        "writes": {"reset": False, "enable": False, "motion": False, "plc": False},
        "env": "both",
        "covers": ["tools/**"],
        "traces": [],
        "preconditions": "none",
        "entry": "true",
        "pass": ["exit 0"],
        "evidence": "stdout",
        "cleanup": "none",
        "status": "manual",
        "v010_gate": "no",
    }
    case.update(overrides)
    return case


class CatalogLoadingTest(unittest.TestCase):
    def test_real_catalog_loads_all_cases_without_errors(self):
        cases, errors = rtr.load_cases(CASES_DIR)
        self.assertEqual(errors, [])
        self.assertEqual(len(cases), 33)
        self.assertEqual(len({c["id"] for c in cases}), 33)

    def test_real_catalog_v010_gate_counts_match_frozen_inventory(self):
        cases, _ = rtr.load_cases(CASES_DIR)
        gates = [c["v010_gate"] for c in cases]
        self.assertEqual(gates.count("run"), 16)
        self.assertEqual(gates.count("reference"), 8)

    def test_missing_required_field_is_reported(self):
        case = make_case()
        del case["risk"]
        errors = rtr.validate_case(case, source="fixture.yaml")
        self.assertTrue(any("risk" in e for e in errors))

    def test_invalid_risk_and_gate_values_are_reported(self):
        errors = rtr.validate_case(
            make_case(risk="T9", v010_gate="maybe"), source="fixture.yaml"
        )
        self.assertTrue(any("T9" in e for e in errors))
        self.assertTrue(any("maybe" in e for e in errors))


class SelectionTest(unittest.TestCase):
    def test_gate_selection_includes_run_and_reference_only(self):
        cases, _ = rtr.load_cases(CASES_DIR)
        selected = rtr.select_cases(cases, gate="v010")
        self.assertEqual(len(selected), 24)
        self.assertTrue(all(c["v010_gate"] in ("run", "reference") for c in selected))

    def test_tier_selection_filters_by_risk(self):
        cases, _ = rtr.load_cases(CASES_DIR)
        selected = rtr.select_cases(cases, tiers=["T0"])
        self.assertTrue(selected)
        self.assertTrue(all(c["risk"] == "T0" for c in selected))

    def test_retired_cases_are_always_excluded(self):
        cases = [make_case(), make_case(id="TC-XX-98", status="retired")]
        self.assertEqual(len(rtr.select_cases(cases)), 1)


class AutoExecutionSafetyTest(unittest.TestCase):
    def test_t3_case_can_never_auto_execute(self):
        case = make_case(risk="T3", auto="true")
        self.assertFalse(rtr.can_auto_execute(case))
        with self.assertRaises(rtr.AutoExecutionRefused):
            rtr.run_auto(case, Path("/tmp"), execute=True)

    def test_case_with_any_write_bit_can_never_auto_execute(self):
        case = make_case(
            writes={"reset": True, "enable": False, "motion": False, "plc": False},
            auto="true",
        )
        self.assertFalse(rtr.can_auto_execute(case))

    def test_auto_pass_and_fail_by_exit_code(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            ok = rtr.run_auto(make_case(auto="true"), Path(d), execute=True)
            bad = rtr.run_auto(
                make_case(id="TC-XX-98", auto="false"), Path(d), execute=True
            )
            self.assertEqual(ok["status"], "PASS")
            self.assertEqual(bad["status"], "FAIL")
            self.assertTrue((Path(d) / "TC-XX-99.log").exists())

    def test_no_auto_field_returns_none(self):
        self.assertIsNone(rtr.run_auto(make_case(), Path("/tmp"), execute=True))


class ReleaseIdentityTest(unittest.TestCase):
    def _fixture_repo(self, tmp, vendor_sha="a" * 40, contract_sha="c" * 40):
        run = lambda *a: subprocess.run(a, cwd=tmp, check=True, capture_output=True)
        run("git", "init", "-q")
        run("git", "config", "user.email", "t@t")
        run("git", "config", "user.name", "t")
        (tmp / "versions.env").write_text(
            "IGH_VERSION=stable-1.6\nIGH_COMMIT=" + "b" * 40 + "\n"
        )
        (tmp / "deps.repos").write_text(
            "repositories:\n  src/vendor/robot_interfaces:\n"
            "    type: git\n    url: u\n    version: "
            + vendor_sha
            + "\n"
        )
        lock = tmp / "src/interfaces"
        lock.mkdir(parents=True)
        (lock / "source-lock.yaml").write_text(
            "schema_version: 1\ncommit: " + contract_sha + "\ncontract_version: 0.6.1\n"
        )
        run("git", "add", "-A")
        run("git", "commit", "-q", "-m", "init")

    def test_identity_collects_six_tuple_fields(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            self._fixture_repo(tmp, vendor_sha="c" * 40)
            identity, findings = rtr.collect_release_identity(tmp)
            self.assertEqual(len(identity["source_commit"]), 40)
            self.assertFalse(identity["source_dirty"])
            self.assertEqual(identity["contract_version"], "0.6.1")
            self.assertEqual(identity["contract_commit"], "c" * 40)
            self.assertEqual(identity["igh_commit"], "b" * 40)
            self.assertEqual(
                identity["vendor_commits"],
                {"src/vendor/robot_interfaces": "c" * 40},
            )
            self.assertIsNone(identity["image_id"])
            self.assertEqual(findings, [])

    def test_short_vendor_sha_and_dirty_tree_produce_findings(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            self._fixture_repo(tmp, vendor_sha="main")
            (tmp / "junk.txt").write_text("x")
            _, findings = rtr.collect_release_identity(tmp)
            self.assertTrue(any("src/vendor/robot_interfaces" in f for f in findings))
            self.assertTrue(any("dirty" in f for f in findings))

    def test_contract_and_robot_interfaces_vendor_sha_must_match(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            self._fixture_repo(tmp, vendor_sha="a" * 40, contract_sha="c" * 40)
            _, findings = rtr.collect_release_identity(tmp)
            self.assertTrue(any("does not match" in finding for finding in findings))


class ReportTest(unittest.TestCase):
    def test_run_case_without_result_is_unverified_and_gate_incomplete(self):
        case = make_case(v010_gate="run")
        row = rtr.evaluate_case(case, result=None, auto_outcome=None)
        self.assertEqual(row["status"], "UNVERIFIED")
        self.assertEqual(rtr.gate_verdict([row]), "INCOMPLETE")

    def test_reference_case_requires_record_and_commit(self):
        case = make_case(v010_gate="reference")
        bad = rtr.evaluate_case(case, result={"status": "PASS"}, auto_outcome=None)
        good = rtr.evaluate_case(
            case,
            result={
                "status": "PASS",
                "reference": {"record": "docs/x.md", "commit": "d" * 40},
            },
            auto_outcome=None,
        )
        self.assertEqual(bad["status"], "UNVERIFIED")
        self.assertEqual(good["status"], "PASS")
        self.assertEqual(good["source"], "reference")

    def test_any_fail_makes_gate_fail(self):
        rows = [
            rtr.evaluate_case(
                make_case(v010_gate="run"),
                result={"status": "FAIL", "evidence": ["log"]},
                auto_outcome=None,
            ),
            rtr.evaluate_case(
                make_case(id="TC-XX-98", v010_gate="run"),
                result={"status": "PASS", "evidence": ["log"]},
                auto_outcome=None,
            ),
        ]
        self.assertEqual(rtr.gate_verdict(rows), "FAIL")

    def test_report_binds_identity_and_summary_counts(self):
        case = make_case(v010_gate="run")
        rows = [rtr.evaluate_case(case, result=None, auto_outcome=None)]
        report = rtr.build_report(
            rows, identity={"source_commit": "e" * 40}, findings=[], gate="v010"
        )
        self.assertEqual(report["release_identity"]["source_commit"], "e" * 40)
        self.assertEqual(report["summary"]["unverified"], 1)
        self.assertEqual(report["summary"]["gate_verdict"], "INCOMPLETE")
        json.dumps(report)

    def test_execution_cards_cover_manual_tiers_and_never_embed_phrases(self):
        cases = [make_case(id="TC-LC-97", risk="T3",
                           writes={"reset": True, "enable": False, "motion": False, "plc": False})]
        cards = rtr.render_execution_cards(cases)
        self.assertIn("TC-LC-97", cards)
        self.assertIn("现场人工授权", cards)
        for phrase in ("START_RT_CONTROL_NATIVE", "ENABLE_RT_CONTROL", "RECOVER_RT_CONTROL"):
            self.assertNotIn(phrase, cards)


class DeltaTest(unittest.TestCase):
    def test_delta_reports_review_and_missing_metrics(self):
        baseline = {"metrics": {"cpu": 4.0, "rss": 500.0}}
        candidate = {"metrics": {"cpu": 6.0}}
        rows = rtr.compute_delta(baseline, candidate)
        by_metric = {r["metric"]: r for r in rows}
        self.assertEqual(by_metric["cpu"]["delta"], 2.0)
        self.assertEqual(by_metric["cpu"]["verdict"], "REVIEW")
        self.assertEqual(by_metric["rss"]["verdict"], "MISSING")

    def test_candidate_only_metric_is_flagged_new(self):
        rows = rtr.compute_delta({"metrics": {}}, {"metrics": {"extra": 1.0}})
        self.assertEqual(rows[0]["verdict"], "NEW")

    def test_bq134_thresholds_fail_above_and_ok_below(self):
        thresholds = {"metrics": {"cpu14_busy_avg_percent": {"fail_above": 10.0}}}
        base = {"metrics": {"cpu14_busy_avg_percent": 4.945}}
        over = rtr.compute_delta(base, {"metrics": {"cpu14_busy_avg_percent": 12.0}}, thresholds)
        under = rtr.compute_delta(base, {"metrics": {"cpu14_busy_avg_percent": 6.0}}, thresholds)
        self.assertEqual(over[0]["verdict"], "FAIL")
        self.assertEqual(under[0]["verdict"], "OK")

    def test_bq134_zero_tolerance_items_review_not_fail(self):
        thresholds = {"review_if_nonzero": ["wc_error_delta"]}
        base = {"metrics": {"wc_error_delta": 0}}
        nonzero = rtr.compute_delta(base, {"metrics": {"wc_error_delta": 3}}, thresholds)
        zero = rtr.compute_delta(base, {"metrics": {"wc_error_delta": 0}}, thresholds)
        self.assertEqual(nonzero[0]["verdict"], "REVIEW")
        self.assertEqual(zero[0]["verdict"], "OK")


class CliTest(unittest.TestCase):
    def test_full_cli_flow_validate_plan_run_delta(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            self.assertEqual(rtr.main(["--repo-root", str(ROOT), "validate"]), 0)

            cards = tmp / "cards.md"
            self.assertEqual(
                rtr.main(["--repo-root", str(ROOT), "plan", "--gate", "v010",
                          "--cards", str(cards)]), 0)
            self.assertIn("TC-LC-02", cards.read_text())

            out, summary = tmp / "report.json", tmp / "summary.md"
            rc = rtr.main(["--repo-root", str(ROOT), "run", "--gate", "v010",
                           "--evidence-dir", str(tmp / "ev"),
                           "--out", str(out), "--summary", str(summary)])
            self.assertEqual(rc, 1)
            report = json.loads(out.read_text())
            self.assertEqual(report["summary"]["gate_verdict"], "INCOMPLETE")
            self.assertEqual(report["summary"]["total"], 24)
            self.assertIn("INCOMPLETE", summary.read_text())

            base, cand, dout = tmp / "b.yaml", tmp / "c.yaml", tmp / "d.json"
            base.write_text("metrics: {m: 1.0}\n")
            cand.write_text("metrics: {m: 2.0}\n")
            self.assertEqual(
                rtr.main(["--repo-root", str(ROOT), "delta", "--baseline", str(base),
                          "--candidate", str(cand), "--out", str(dout)]), 0)
            self.assertEqual(json.loads(dout.read_text())["delta"][0]["delta"], 1.0)

    def test_identity_cli_reports_findings_exit_code(self):
        rc = rtr.main(["--repo-root", str(ROOT), "identity"])
        self.assertIn(rc, (0, 1))


if __name__ == "__main__":
    unittest.main()
