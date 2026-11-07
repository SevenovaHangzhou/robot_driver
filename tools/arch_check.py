#!/usr/bin/env python3
"""Architecture check: the rules of ARCHITECTURE.md over the whole source tree.

Usage: tools/arch_check.py [layers|ownership|names|duplicates|size ...] [--json]
       tools/arch_check.py compose --against <ref>   (on demand; needs a sourced ROS environment)
Without a rule group every group runs. Output: `path:line: RULE-ID message`.
Exit codes: 0 no finding, 1 findings, 2 the tool itself failed.
Reads only the work tree (tracked and untracked, not ignored); no hardware, network or build.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Sequence

if __package__:
    from .archcheck import compose_against, duplicates, layers, names, repo, size
else:
    from archcheck import compose_against, duplicates, layers, names, repo, size

# Rule group -> (rule-ID family, check). Every group runs over the full tree.
GROUPS = {
    "layers": ("LAYER", lambda texts, rules: layers.check_layers(texts, rules)),
    "ownership": ("OWN", lambda texts, rules: layers.check_ownership(texts)),
    "names": ("NAME", lambda texts, rules: names.check_names(texts, rules.vendor_names)),
    "duplicates": (
        "DUP",
        lambda texts, rules: duplicates.check_duplicates(texts, rules.duplicates),
    ),
    "size": ("SIZE", lambda texts, rules: size.check_size(texts)),
}


def run(root: Path, groups: Sequence[str]) -> list[repo.Finding]:
    texts = repo.load_texts(root)
    rules, findings = repo.read_rules(texts)
    for group in groups:
        findings.extend(GROUPS[group][1](texts, rules))
    checked = {GROUPS[group][0] for group in groups}
    return repo.apply_exceptions(findings, rules, checked)


def report(findings: list[repo.Finding], as_json: bool, passed: str) -> int:
    for finding in findings:
        print(json.dumps(finding.__dict__, ensure_ascii=False) if as_json else finding.describe())
    if findings:
        print(f"arch_check: {len(findings)} finding(s)", file=sys.stderr)
        return 1
    print(passed)
    return 0


def compose_main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(
        prog="arch_check.py compose", description=compose_against.__doc__
    )
    parser.add_argument("--against", required=True, help="base revision, e.g. origin/main")
    parser.add_argument("--json", action="store_true", help="one JSON object per finding")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)
    try:
        findings, compared = compose_against.run(args.root, args.against)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"arch_check compose failed: {exc}", file=sys.stderr)
        return 2
    return report(
        findings, args.json, f"PASS: arch_check compose ({compared} selections unchanged)"
    )


def main(argv: Sequence[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv[:1] == ["compose"]:
        return compose_main(argv[1:])
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("groups", nargs="*", metavar="group", help=" | ".join(GROUPS))
    parser.add_argument("--json", action="store_true", help="one JSON object per finding")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(argv)
    unknown = sorted(set(args.groups) - set(GROUPS))
    if unknown:
        parser.error(f"unknown rule group: {', '.join(unknown)}")
    try:
        findings = run(args.root, args.groups or list(GROUPS))
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"arch_check failed: {exc}", file=sys.stderr)
        return 2
    return report(findings, args.json, "PASS: arch_check")


if __name__ == "__main__":
    raise SystemExit(main())
