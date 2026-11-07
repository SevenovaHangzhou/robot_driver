"""compose --against <ref>: a structural change leaves every composed artifact unchanged.

The base tree is exported with `git archive <ref>`; each tree is composed by
compose_worker.py in its own process (needs a sourced ROS environment for xacro). A
selection that composes at the base must compose identically now (ARCHITECTURE.md §7).
"""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile

from .repo import Finding

WORKER = Path(__file__).with_name("compose_worker.py")
EXPORTED = ("src/rt_control", "src/description")


def compare(base: dict, current: dict) -> list[Finding]:
    """COMPOSE-2 findings between two worker results (selection -> result)."""
    findings = []
    for key, before in sorted(base.items()):
        if not before["ok"]:
            continue  # not composable at the base: nothing to preserve
        after = current.get(key)
        if after is None or not after["ok"]:
            reason = "选择已不存在" if after is None else after["error"]
            findings.append(Finding("COMPOSE-2", key, 0, f"基线可组合，当前失败：{reason}"))
            continue
        for name in sorted(set(before["files"]) | set(after["files"])):
            old, new = before["files"].get(name), after["files"].get(name)
            if old == new:
                continue
            if old is None or new is None:
                change = "新增" if old is None else "删除"
                findings.append(Finding("COMPOSE-2", f"{key}/{name}", 0, f"产物文件{change}"))
                continue
            old_lines, new_lines = old.splitlines(), new.splitlines()
            line = next(
                (i for i, (a, b) in enumerate(zip(old_lines, new_lines), 1) if a != b),
                min(len(old_lines), len(new_lines)) + 1,
            )
            before_text = old_lines[line - 1].strip() if line <= len(old_lines) else "（文件结束）"
            after_text = new_lines[line - 1].strip() if line <= len(new_lines) else "（文件结束）"
            findings.append(
                Finding(
                    "COMPOSE-2",
                    f"{key}/{name}",
                    line,
                    f"产物不同：{before_text[:80]} → {after_text[:80]}",
                )
            )
    return findings


def _export(root: Path, ref: str, destination: Path) -> None:
    archive = destination / "base.tar"
    with archive.open("wb") as stream:
        subprocess.run(
            ["git", "-C", str(root), "archive", "--format=tar", ref, *EXPORTED],
            check=True,
            stdout=stream,
        )
    with tarfile.open(archive) as tar:
        tar.extractall(destination / "tree")  # noqa: S202 - our own repository archive


def run(root: Path, ref: str) -> tuple[list[Finding], int]:
    """(findings, number of selections composable at the base)."""
    with tempfile.TemporaryDirectory(prefix="arch-check-compose-") as directory:
        work = Path(directory)
        _export(root, ref, work)
        outputs = {"base": work / "base.json", "current": work / "current.json"}
        trees = {"base": work / "tree", "current": root}
        processes = {
            name: subprocess.Popen(
                [sys.executable, str(WORKER), str(trees[name]), str(outputs[name])],
                stderr=subprocess.PIPE,
                text=True,
            )
            for name in trees
        }
        for name, process in processes.items():
            _, error = process.communicate()
            if process.returncode != 0:
                raise RuntimeError(f"compose worker ({name}) failed:\n{error}")
        base, current = (
            json.loads(outputs[name].read_text("utf-8")) for name in ("base", "current")
        )
    return compare(base, current), sum(1 for result in base.values() if result["ok"])
