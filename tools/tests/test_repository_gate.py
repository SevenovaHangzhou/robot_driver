#!/usr/bin/env python3

import contextlib
import io
import subprocess
import tempfile
import unittest
from pathlib import Path

from tools import repository_gate


class RepositoryGateTest(unittest.TestCase):
    def assert_has(self, findings, fragment):
        self.assertTrue(
            any(fragment in finding for finding in findings),
            f"expected {fragment!r} in {findings!r}",
        )

    def test_generated_artifacts_and_retired_watchdog_are_rejected(self):
        for path in (
            "build/pkg/output",
            "install/pkg/setup.bash",
            "log/latest/event.log",
            "src/pkg/__pycache__/module.pyc",
        ):
            with self.subTest(path=path):
                self.assert_has(repository_gate.check_path(path), "generated artifact")

        self.assert_has(
            repository_gate.check_path("src/rt_control/rt_watchdog/package.xml"),
            "retired rt_watchdog",
        )
        for path in ("PROGRESS.md", "BLOCKED-questions.md"):
            with self.subTest(path=path):
                self.assert_has(
                    repository_gate.check_path(path),
                    "must live under domains/<domain>/",
                )
        self.assertEqual(repository_gate.check_path("src/rt_control/enable_manager/package.xml"), [])

    def test_text_hygiene_detects_conflicts_secrets_and_missing_newline(self):
        fake_token = "ghp_" + "a" * 32
        findings = repository_gate.check_text(
            "config/example.yaml",
            f"token: {fake_token}\n<<<<<<< HEAD\nvalue: bad ",
        )
        self.assert_has(findings, "possible secret")
        self.assert_has(findings, "merge-conflict marker")
        self.assert_has(findings, "trailing whitespace")
        self.assert_has(findings, "newline at end of file")

        self.assertEqual(
            repository_gate.check_text("patches/vendor/change.patch", "+context with space \n"),
            [],
        )
        patch_with_secret = "+token: " + fake_token + "\n"
        self.assert_has(
            repository_gate.check_text("patches/vendor/change.patch", patch_with_secret),
            "possible secret",
        )

    def test_dependency_sources_and_igh_commit_must_use_full_sha(self):
        valid_deps = """repositories:
  src/vendor/example:
    type: git
    url: https://example.invalid/example.git
    version: 0123456789abcdef0123456789abcdef01234567
"""
        valid_versions = """IGH_VERSION=stable-1.6
IGH_COMMIT=89abcdef0123456789abcdef0123456789abcdef
"""
        self.assertEqual(repository_gate.check_dependency_pins(valid_deps, valid_versions), [])

        findings = repository_gate.check_dependency_pins(
            valid_deps.replace("0123456789abcdef0123456789abcdef01234567", "main"),
            valid_versions.replace("89abcdef0123456789abcdef0123456789abcdef", "deadbeef"),
        )
        self.assert_has(findings, "must be a full 40-character commit SHA")
        self.assert_has(findings, "IGH_COMMIT must be a full 40-character commit SHA")

    def test_shared_packages_cannot_depend_on_domain_implementation(self):
        manifest = """<?xml version="1.0"?>
<package format="3">
  <name>robot_description</name>
  <version>0.1.0</version>
  <exec_depend>rt_control_bringup</exec_depend>
</package>
"""
        self.assert_has(
            repository_gate.check_manifest_dependencies(
                "src/description/robot_description/package.xml", manifest
            ),
            "shared package must not depend on domain package",
        )

        feature_manifest = manifest.replace("robot_description", "enable_manager").replace(
            "src/description/robot_description", "src/rt_control/enable_manager"
        )
        self.assert_has(
            repository_gate.check_manifest_dependencies(
                "src/rt_control/enable_manager/package.xml", feature_manifest
            ),
            "feature package must not depend on rt_control_bringup",
        )

    def test_robot_description_ignores_comments_but_rejects_domain_data(self):
        comment_only = """<?xml version="1.0"?>
<robot name="robot">
  <!-- ros2_control belongs to rt_control_bringup. -->
  <link name="base_link"/>
</robot>
"""
        self.assertEqual(
            repository_gate.check_robot_description_xml("robot.urdf.xacro", comment_only), []
        )

        polluted = comment_only.replace(
            '<link name="base_link"/>', '<link name="pick_station"/>'
        )
        self.assert_has(
            repository_gate.check_robot_description_xml("robot.urdf.xacro", polluted),
            "domain-specific token",
        )

    def test_interface_package_rejects_business_implementation(self):
        self.assert_has(
            repository_gate.check_interface_path(
                "src/interfaces/robot_interfaces/src/task_planner.cpp"
            ),
            "interface package must not contain business implementation",
        )
        self.assertEqual(
            repository_gate.check_interface_path(
                "src/interfaces/robot_interfaces/srv/RtEnable.srv"
            ),
            [],
        )
        self.assert_has(
            repository_gate.check_interface_path(
                "src/interfaces/robot_interfaces/task_planner.py"
            ),
            "interface package must not contain business implementation",
        )

    def test_compose_policy_rejects_privilege_and_requires_cpu_gate(self):
        valid = """services:
  rt-control:
    cpuset: "${RT_CONTROL_CPUSET:?required}"
    devices: ["/dev/EtherCAT0:/dev/EtherCAT0"]
    cap_add: [SYS_NICE, IPC_LOCK, NET_RAW]
    volumes: ["./docker/cyclonedds.xml:/etc/cyclonedds.xml:ro"]
"""
        self.assertEqual(repository_gate.check_compose_policy(valid), [])

        unsafe = valid.replace(
            '    cpuset: "${RT_CONTROL_CPUSET:?required}"\n',
            "    cpuset: \"0\"\n    privileged: true\n",
        )
        findings = repository_gate.check_compose_policy(unsafe)
        self.assert_has(findings, "privileged mode is forbidden")
        self.assert_has(findings, "RT_CONTROL_CPUSET must remain a required input")

        incomplete = valid.replace(
            '    devices: ["/dev/EtherCAT0:/dev/EtherCAT0"]\n', "    devices: []\n"
        ).replace("    cap_add: [SYS_NICE, IPC_LOCK, NET_RAW]\n", "    cap_add: [SYS_NICE]\n")
        findings = repository_gate.check_compose_policy(incomplete)
        self.assert_has(findings, "device mapping must remain exact")
        self.assert_has(findings, "capabilities must remain exact")

    def test_dockerfile_must_keep_the_signal_gate_as_direct_command(self):
        valid = """FROM ros:humble-ros-base
ENTRYPOINT ["/rt-control-entrypoint.sh"]
CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]
"""
        self.assertEqual(repository_gate.check_dockerfile_policy(valid), [])

        bypassed = valid.replace(
            'CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]',
            'CMD ["ros2", "launch", "rt_control_bringup", "rt_control.launch.py"]',
        )
        self.assert_has(
            repository_gate.check_dockerfile_policy(bypassed),
            "Docker CMD must execute the installed rt_control_start directly",
        )

        commented_bypass = (
            valid.replace(
                'CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]',
                '# CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]',
            )
            + 'CMD ["ros2", "launch", "rt_control_bringup", "rt_control.launch.py"]\n'
        )
        self.assert_has(
            repository_gate.check_dockerfile_policy(commented_bypass),
            "Docker CMD must execute the installed rt_control_start directly",
        )

    def test_ci_workflow_must_preserve_governance_before_build(self):
        valid = """on:
  pull_request:
  push:
permissions:
  contents: read
jobs:
  governance:
    steps:
      - run: pre-commit run --all-files
      - run: python3 tools/pr_contract_gate.py --event-path event.json
  build:
    needs: governance
    steps:
      - run: colcon build
      - run: colcon test
      - run: check_urdf robot.urdf
      - run: python3 tools/diff_legacy.py
"""
        self.assertEqual(repository_gate.check_ci_workflow_policy(valid), [])

        weakened = valid.replace("    needs: governance\n", "").replace(
            "      - run: colcon test\n", ""
        )
        findings = repository_gate.check_ci_workflow_policy(weakened)
        self.assert_has(findings, "build job must depend on governance")
        self.assert_has(findings, "build job must run colcon test")

    def test_precommit_must_call_the_full_repository_gate(self):
        valid = """repos:
  - repo: local
    hooks:
      - id: robot-quality-gate
        entry: tools/quality_gate.sh
        language: system
        pass_filenames: false
        always_run: true
"""
        self.assertEqual(repository_gate.check_precommit_policy(valid), [])
        weakened = valid.replace("pass_filenames: false", "pass_filenames: true")
        self.assert_has(
            repository_gate.check_precommit_policy(weakened),
            "pass_filenames must remain false",
        )

    def make_valid_repository(self, root: Path):
        files = {
            "README.md": "# Robot monorepo\n",
            "AGENTS.md": "# Contract\n",
            "docs/README.md": "# System documentation\n",
            "domains/rt_control/README.md": "# rt-control\n",
            "domains/rt_control/AGENTS.md": "# rt-control contract\n",
            "domains/rt_control/PROGRESS.md": "# rt-control progress\n",
            "domains/rt_control/BLOCKED-questions.md": "# rt-control blockers\n",
            ".pre-commit-config.yaml": """repos:
  - repo: local
    hooks:
      - id: robot-quality-gate
        entry: tools/quality_gate.sh
        language: system
        pass_filenames: false
        always_run: true
""",
            ".github/pull_request_template.md": "# Review\n",
            ".github/workflows/rt-control-ci.yml": """on:
  pull_request:
  push:
permissions:
  contents: read
jobs:
  governance:
    steps:
      - run: pre-commit run --all-files
      - run: python3 tools/pr_contract_gate.py --event-path event.json
  build:
    needs: governance
    steps:
      - run: colcon build
      - run: colcon test
      - run: check_urdf robot.urdf
      - run: python3 tools/diff_legacy.py
""",
            "deps.repos": """repositories:
  src/vendor/example:
    type: git
    url: https://example.invalid/example.git
    version: 0123456789abcdef0123456789abcdef01234567
""",
            "versions.env": """IGH_VERSION=stable-1.6
IGH_COMMIT=89abcdef0123456789abcdef0123456789abcdef
""",
            "docker/compose.yaml": """services:
  rt-control:
    cpuset: "${RT_CONTROL_CPUSET:?required}"
    devices: ["/dev/EtherCAT0:/dev/EtherCAT0"]
    cap_add: [SYS_NICE, IPC_LOCK, NET_RAW]
    volumes: ["./docker/cyclonedds.xml:/etc/cyclonedds.xml:ro"]
""",
            "docker/rt-control/Dockerfile": """FROM ros:humble-ros-base
ENTRYPOINT ["/rt-control-entrypoint.sh"]
CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]
""",
            "src/description/robot_description/package.xml": """<package format="3">
  <name>robot_description</name>
  <version>0.1.0</version>
  <exec_depend>xacro</exec_depend>
</package>
""",
            "src/description/robot_description/urdf/robot.urdf.xacro": """<robot name="robot">
  <!-- ros2_control remains outside this package. -->
  <link name="base_link"/>
</robot>
""",
            "src/interfaces/robot_interfaces/msg/Example.msg": "string value\n",
            "tools/example.py": "print('ok')\n",
        }
        for relative_path, content in files.items():
            path = root / relative_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
        subprocess.run(["git", "init", "-q", str(root)], check=True)

    def test_collect_findings_accepts_a_minimal_compliant_repository(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.make_valid_repository(root)
            self.assertEqual(repository_gate.collect_findings(root), [])

    def test_collect_findings_reports_missing_governance_and_invalid_syntax(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            subprocess.run(["git", "init", "-q", str(root)], check=True)
            invalid_python = root / "tools/broken.py"
            invalid_python.parent.mkdir(parents=True)
            invalid_python.write_text("if True print('broken')\n", encoding="utf-8")
            findings = repository_gate.collect_findings(root)
            self.assert_has(findings, "required governance file is missing")
            self.assert_has(findings, "syntax error")

    def test_main_reports_success_policy_failure_and_execution_failure(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.make_valid_repository(root)
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                self.assertEqual(
                    repository_gate.main(["--repository-root", str(root)]), 0
                )
            self.assertIn("PASS", stdout.getvalue())

            (root / "build/generated.txt").parent.mkdir()
            (root / "build/generated.txt").write_text("generated\n", encoding="utf-8")
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                self.assertEqual(
                    repository_gate.main(["--repository-root", str(root)]), 1
                )
            self.assertIn("generated artifact", stderr.getvalue())

        with tempfile.TemporaryDirectory() as non_repository:
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                self.assertEqual(
                    repository_gate.main(["--repository-root", non_repository]), 2
                )
            self.assertIn("could not run", stderr.getvalue())

    def test_format_parsers_report_malformed_input(self):
        self.assert_has(
            repository_gate.check_dependency_pins("repositories: [", "IGH_COMMIT=bad\n"),
            "invalid YAML",
        )
        self.assert_has(
            repository_gate.check_manifest_dependencies("package.xml", "<package>"),
            "invalid package XML",
        )
        self.assert_has(
            repository_gate.check_robot_description_xml("robot.urdf.xacro", "<robot>"),
            "invalid robot-description XML",
        )
        self.assert_has(repository_gate.check_compose_policy("services: ["), "invalid YAML")


if __name__ == "__main__":
    unittest.main()
