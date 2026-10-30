#!/usr/bin/env python3

import json
import tempfile
import unittest
from pathlib import Path

from tools import pr_contract_gate


class PullRequestContractGateTest(unittest.TestCase):
    def valid_body(self):
        return """<!-- robot-pr-contract:v1 -->
## 变更范围
Implements the repository quality gate.

## 架构与安全审查
- [x] <!-- gate:scope --> 变更范围可追溯
- [X] <!-- gate:architecture --> 域边界与依赖方向符合契约
- [x] <!-- gate:interfaces --> 接口和模型兼容性已说明
- [x] <!-- gate:realtime-safety --> 实时与安全影响已审查

## 验证证据
`tools/quality_gate.sh`: PASS

## 未验证项与剩余风险
未执行实机验证；本变更只涉及仓库治理文件。

## 提交前确认
- [x] <!-- gate:tests --> 已如实记录实际执行的验证
- [x] <!-- gate:hygiene --> 无生成物、敏感信息和无关改动
"""

    def test_complete_body_passes(self):
        self.assertEqual(pr_contract_gate.validate_body(self.valid_body()), [])

    def test_missing_section_and_unchecked_gate_fail(self):
        body = self.valid_body().replace("## 未验证项与剩余风险", "## 风险")
        body = body.replace("[x] <!-- gate:tests -->", "[ ] <!-- gate:tests -->")
        findings = pr_contract_gate.validate_body(body)
        self.assertTrue(any("required section" in finding for finding in findings))
        self.assertTrue(any("gate:tests" in finding for finding in findings))

    def test_placeholder_only_evidence_fails(self):
        body = self.valid_body().replace(
            "`tools/quality_gate.sh`: PASS", "<!-- 请填写实际命令和结果 -->"
        )
        findings = pr_contract_gate.validate_body(body)
        self.assertTrue(any("verification evidence" in finding for finding in findings))

    def test_event_file_reads_pull_request_body(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            event_path = Path(temporary_directory) / "event.json"
            event_path.write_text(
                json.dumps({"pull_request": {"body": self.valid_body()}}),
                encoding="utf-8",
            )
            self.assertEqual(pr_contract_gate.validate_event(event_path), [])

            event_path.write_text(json.dumps({"push": {}}), encoding="utf-8")
            findings = pr_contract_gate.validate_event(event_path)
            self.assertTrue(any("pull_request.body" in finding for finding in findings))

    def test_repository_template_contains_every_machine_readable_marker(self):
        repository_root = Path(__file__).resolve().parents[2]
        template = (repository_root / ".github/pull_request_template.md").read_text(
            encoding="utf-8"
        )
        self.assertIn(pr_contract_gate.CONTRACT_MARKER, template)
        for heading in pr_contract_gate.REQUIRED_SECTIONS:
            with self.subTest(heading=heading):
                self.assertIn(f"## {heading}", template)
        for gate in pr_contract_gate.REQUIRED_GATES:
            with self.subTest(gate=gate):
                self.assertIn(f"<!-- gate:{gate} -->", template)


if __name__ == "__main__":
    unittest.main()


def test_safety_policy_text_is_a_reminder_not_human_approval():
    body = PullRequestContractGateTest().valid_body()
    paths = ["src/rt_control/hwcfg_machines/machines/demo/safety_policy.yaml"]
    assert pr_contract_gate.safety_policy_warnings(body, []) == []
    assert pr_contract_gate.safety_policy_warnings(body, paths)
    assert pr_contract_gate.validate_body(body) == []
    body += "\n## 安全策略变更\nConfirmed-by: reviewer\n"
    assert pr_contract_gate.safety_policy_warnings(body, paths) == []


def test_policy_diff_uses_merge_base_and_includes_add_modify_delete_and_rename(tmp_path):
    import subprocess

    def git(*args):
        return subprocess.run(["git", *args], cwd=tmp_path, check=True,
                              capture_output=True, text=True).stdout.strip()

    def commit(message):
        git("add", ".")
        git("-c", "user.name=Fixture", "-c", "user.email=fixture@example.invalid",
            "commit", "-qm", message)
        return git("rev-parse", "HEAD")

    git("init", "-q")
    for directory in ["modify", "delete", "rename"]:
        (tmp_path / directory).mkdir()
        (tmp_path / directory / "safety_policy.yaml").write_text("safe: true\n")
    ancestor = commit("initial")
    git("checkout", "-qb", "pr")
    (tmp_path / "added").mkdir()
    (tmp_path / "added/safety_policy.yaml").write_text("safe: true\n")
    (tmp_path / "modify/safety_policy.yaml").write_text("safe: false\n")
    git("rm", "delete/safety_policy.yaml")
    git("mv", "rename/safety_policy.yaml", "rename/moved.yaml")
    head = commit("policy changes")
    git("checkout", "--detach", ancestor)
    (tmp_path / "base-only").mkdir()
    (tmp_path / "base-only/safety_policy.yaml").write_text("safe: true\n")
    base = commit("base advanced")
    paths = pr_contract_gate.changed_policy_paths(tmp_path, base, head)
    assert set(paths) == {"added/safety_policy.yaml", "modify/safety_policy.yaml",
                          "delete/safety_policy.yaml", "rename/safety_policy.yaml", "rename/moved.yaml"}
    event = tmp_path / "event.json"
    event.write_text(json.dumps({"pull_request": {
        "body": PullRequestContractGateTest().valid_body(), "base": {"sha": base}, "head": {"sha": head},
    }}))
    assert pr_contract_gate.validate_event(event, tmp_path) == []
