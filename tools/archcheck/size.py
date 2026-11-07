"""SIZE (ARCHITECTURE.md §3): one production source or configuration file stays readable.

C/C++ <= 1000, Python <= 600, YAML <= 400 lines, for production files of the packages under
src/rt_control. Tests, golden outputs, fixtures, schemas and upstream sources imported
unchanged are not counted.
"""

from __future__ import annotations

from pathlib import PurePosixPath

from .repo import PREFIX, Finding

LIMITS = {
    ".cpp": 1000,
    ".hpp": 1000,
    ".h": 1000,
    ".cc": 1000,
    ".cxx": 1000,
    ".py": 600,
    ".yaml": 400,
    ".yml": 400,
}
NOT_CHECKED_PARTS = {"test", "tests", "fixtures", "schema", "golden", "upstream"}


def check_size(texts: dict[str, str]) -> list[Finding]:
    findings = []
    for path, text in sorted(texts.items()):
        file = PurePosixPath(path)
        limit = LIMITS.get(file.suffix)
        if limit is None or not path.startswith(PREFIX) or NOT_CHECKED_PARTS & set(file.parts):
            continue
        lines = len(text.splitlines())
        if lines > limit:
            findings.append(
                Finding(
                    "SIZE-1",
                    path,
                    lines,
                    f"{lines} 行，超过 {file.suffix} 上限 {limit} 行；按职责拆分",
                )
            )
    return findings
