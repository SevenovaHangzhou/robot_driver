#!/usr/bin/env python3
"""Validate that a pull request completed the repository contract template."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Sequence


CONTRACT_MARKER = "<!-- robot-pr-contract:v1 -->"
REQUIRED_SECTIONS = (
    "变更范围",
    "架构与安全审查",
    "验证证据",
    "未验证项与剩余风险",
    "提交前确认",
)
REQUIRED_GATES = (
    "scope",
    "architecture",
    "interfaces",
    "realtime-safety",
    "tests",
    "hygiene",
)


def _section_body(body: str, heading: str) -> str | None:
    pattern = re.compile(
        rf"^##\s+{re.escape(heading)}\s*$\n(.*?)(?=^##\s+|\Z)",
        re.MULTILINE | re.DOTALL,
    )
    match = pattern.search(body)
    return match.group(1) if match else None


def _visible_text(text: str) -> str:
    return re.sub(r"<!--.*?-->", "", text, flags=re.DOTALL).strip()


def validate_body(body: str) -> list[str]:
    findings: list[str] = []
    if CONTRACT_MARKER not in body:
        findings.append("pull_request.body: contract marker is missing")

    sections: dict[str, str] = {}
    for heading in REQUIRED_SECTIONS:
        section = _section_body(body, heading)
        if section is None:
            findings.append(f"pull_request.body: required section {heading!r} is missing")
        else:
            sections[heading] = section

    for gate in REQUIRED_GATES:
        checked = re.compile(
            rf"^-\s*\[[xX]\]\s*<!--\s*gate:{re.escape(gate)}\s*-->",
            re.MULTILINE,
        )
        if not checked.search(body):
            findings.append(f"pull_request.body: gate:{gate} must be checked")

    evidence = sections.get("验证证据")
    if evidence is not None and not _visible_text(evidence):
        findings.append("pull_request.body: verification evidence must be provided")
    risks = sections.get("未验证项与剩余风险")
    if risks is not None and not _visible_text(risks):
        findings.append("pull_request.body: unverified work and residual risk must be stated")
    return findings


def validate_event(event_path: Path) -> list[str]:
    try:
        event = json.loads(event_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return [f"GitHub event could not be read: {exc}"]
    pull_request = event.get("pull_request")
    body = pull_request.get("body") if isinstance(pull_request, dict) else None
    if not isinstance(body, str):
        return ["GitHub event does not contain pull_request.body"]
    return validate_body(body)


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--event-path", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv or sys.argv[1:])
    findings = validate_event(args.event_path)
    if findings:
        print("Pull request contract failed:", file=sys.stderr)
        for finding in findings:
            print(f"- {finding}", file=sys.stderr)
        return 1
    print("PASS: pull request contract")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
