#!/usr/bin/env python3
"""Architecture check: the rules of ARCHITECTURE.md over the whole source tree.

Usage: tools/arch_check.py [layers|ownership|names ...] [--json]
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
    from .archcheck import layers, repo
else:
    from archcheck import layers, repo

# Rule group -> (rule-ID family, check). Every group runs over the full tree.
GROUPS = {
    "layers": ("LAYER", lambda texts, rules: layers.check_layers(texts, rules)),
    "ownership": ("OWN", lambda texts, rules: layers.check_ownership(texts)),
    "names": ("NAME", lambda texts, rules: layers.check_names(texts)),
}


def run(root: Path, groups: Sequence[str]) -> list[repo.Finding]:
    texts = repo.load_texts(root)
    rules, findings = repo.read_rules(texts)
    for group in groups:
        findings.extend(GROUPS[group][1](texts, rules))
    checked = {GROUPS[group][0] for group in groups}
    return repo.apply_exceptions(findings, rules, checked)


def main(argv: Sequence[str] | None = None) -> int:
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
    for finding in findings:
        if args.json:
            print(json.dumps(finding.__dict__, ensure_ascii=False))
        else:
            print(finding.describe())
    if findings:
        print(f"arch_check: {len(findings)} finding(s)", file=sys.stderr)
        return 1
    print("PASS: arch_check")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
