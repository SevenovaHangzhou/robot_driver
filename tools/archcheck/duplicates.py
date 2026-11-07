"""DUP (ARCHITECTURE.md §3, reuse): an extracted implementation is not written a second time.

Each entry of `duplicates` in tools/arch_rules.yaml names a signature of code that has one
owner package; the signature in production source of any other package is reported.
Comments are skipped: a comment may name what it uses.
"""

from __future__ import annotations

from pathlib import PurePosixPath
import re

from .repo import PREFIX, Finding, package_inventory

SOURCE_SUFFIXES = {".py", ".cpp", ".hpp", ".h", ".cc", ".cxx"}
NOT_CHECKED_PARTS = {"test", "tests", "fixtures", "schema"}
COMMENT = re.compile(r"^\s*(?:#|//|/\*|\*)")


def check_duplicates(texts: dict[str, str], signatures: tuple[dict, ...]) -> list[Finding]:
    compiled = [(entry, re.compile(entry["pattern"])) for entry in signatures]
    inventory = package_inventory(texts)
    findings = []
    for path, text in sorted(texts.items()):
        file = PurePosixPath(path)
        if (
            not path.startswith(PREFIX)
            or file.suffix not in SOURCE_SUFFIXES
            or NOT_CHECKED_PARTS & set(file.parts)
        ):
            continue
        package = inventory.get(path.split("/")[2])
        candidates = [(entry, pattern) for entry, pattern in compiled if entry["owner"] != package]
        if package is None or not candidates:
            continue
        for number, line in enumerate(text.splitlines(), 1):
            if COMMENT.match(line):
                continue
            for entry, pattern in candidates:
                if pattern.search(line):
                    where = f"只在 {entry['owner']} 实现" if entry["owner"] else "已删除，不再实现"
                    findings.append(
                        Finding(
                            "DUP-1", path, number, f"{entry['name']}{where}：{line.strip()[:120]}"
                        )
                    )
    return findings
