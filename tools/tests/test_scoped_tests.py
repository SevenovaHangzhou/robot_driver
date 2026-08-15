import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import scoped_tests as st

CASES_DIR = ROOT / "domains/rt_control/testing/cases"


class ClassifyTest(unittest.TestCase):
    def test_docs_only_change_needs_repository_gate_only(self):
        plan = st.classify(["domains/rt_control/docs/areas/governance/README.md", "README.md"])
        self.assertFalse(plan["full"])
        self.assertEqual(plan["actions"], ["python3 tools/repository_gate.py"])

    def test_tools_change_runs_quality_gate(self):
        plan = st.classify(["tools/release_test_runner.py"])
        self.assertIn("tools/quality_gate.sh", plan["actions"])

    def test_single_package_change_selects_reverse_dependencies(self):
        plan = st.classify(["src/rt_control/plc_node/plc_node/plc_logic.py"])
        self.assertTrue(any("--packages-above plc_node" in a for a in plan["actions"]))
        self.assertFalse(plan["full"])

    def test_private_interface_change_selects_private_package_only(self):
        plan = st.classify(["src/interfaces/rt_control_interfaces/srv/RtEnable.srv"])
        joined = " ".join(plan["actions"])
        self.assertIn("rt_control_interfaces", joined)
        self.assertNotIn("robot_rt_control_interfaces", joined)
        self.assertNotIn("robot_system_interfaces", joined)

    def test_global_impact_paths_force_full_suite(self):
        for path in ("patches/ecat_icube/x.patch", "deps.repos", "versions.env",
                     "docker/rt-control/Dockerfile", ".github/workflows/rt-control-ci.yml"):
            plan = st.classify([path])
            self.assertTrue(plan["full"], path)

    def test_mixed_changes_deduplicate_actions(self):
        plan = st.classify([
            "src/rt_control/bms_node/bms_node/bms_node.py",
            "src/rt_control/bms_node/package.xml",
            "domains/rt_control/PROGRESS.md",
        ])
        colcon = [a for a in plan["actions"] if "colcon" in a]
        self.assertEqual(len(colcon), 1)
        self.assertIn("--packages-above bms_node", colcon[0])


class AffectedCasesTest(unittest.TestCase):
    def test_plc_change_maps_to_covering_cases(self):
        cases = st.load_case_covers(CASES_DIR)
        affected = st.affected_cases(["src/rt_control/plc_node/plc_node/plc_node.py"], cases)
        self.assertIn("TC-FM-03", affected)
        self.assertIn("TC-AR-04", affected)
        self.assertNotIn("TC-RT-01", affected)

    def test_glob_matcher_handles_double_star(self):
        self.assertTrue(st.glob_match("src/rt_control/plc_node/**", "src/rt_control/plc_node/a/b.py"))
        self.assertFalse(st.glob_match("src/rt_control/plc_node/**", "src/rt_control/bms_node/x.py"))
        self.assertTrue(st.glob_match("deps.repos", "deps.repos"))


class CliTest(unittest.TestCase):
    def test_changed_from_git_includes_staged_and_untracked(self):
        import subprocess
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            run = lambda *a: subprocess.run(a, cwd=tmp, check=True, capture_output=True)
            run("git", "init", "-q", "-b", "main")
            run("git", "config", "user.email", "t@t")
            run("git", "config", "user.name", "t")
            (tmp / "base.txt").write_text("x")
            run("git", "add", "-A")
            run("git", "commit", "-q", "-m", "base")
            (tmp / "staged.txt").write_text("s")
            run("git", "add", "staged.txt")
            (tmp / "untracked.txt").write_text("u")
            (tmp / "base.txt").write_text("modified")
            changed = st.changed_from_git(tmp, "main")
            self.assertIn("staged.txt", changed)
            self.assertIn("untracked.txt", changed)
            self.assertIn("base.txt", changed)

    def test_plan_mode_exits_zero_regardless_of_repo_state(self):
        rc = st.main(["--repo-root", str(ROOT), "--base", "HEAD"])
        self.assertEqual(rc, 0)


if __name__ == "__main__":
    unittest.main()
