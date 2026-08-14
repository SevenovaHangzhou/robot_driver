#!/usr/bin/env python3
"""按改动范围裁剪本地测试（ELECTRI-81 scope resolver）。

从 git 改动集推导最小验证集：纯文档 → repository_gate；tools/** → quality_gate；
单包源码 → colcon test --packages-above（colcon 自算反向依赖）；接口包 → 三个
接口包及全部下游；全局影响路径（patches/deps.repos/versions.env/docker/CI）→
提示跑真·全量。同时按用例 covers 字段列出受影响的目录用例（信息性）。

这是内环工具：CI 全量兜底不变，发布门禁走 release_test_runner。
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

import yaml

FULL_SUITE_PATHS = ("patches/", "deps.repos", "versions.env", "docker/", ".github/")
INTERFACE_PREFIX = "src/interfaces/"
PACKAGE_PREFIXES = ("src/rt_control/", "src/description/")
INTERFACE_PACKAGES = (
    "robot_rt_control_interfaces", "robot_system_interfaces", "rt_control_interfaces",
)


def classify(changed_files: list[str]) -> dict:
    plan = {"full": False, "actions": [], "reasons": [], "packages": set()}
    needs_repo_gate = needs_quality_gate = False
    for path in changed_files:
        if any(path.startswith(p) or path == p.rstrip("/") for p in FULL_SUITE_PATHS):
            plan["full"] = True
            plan["reasons"].append(f"{path}: 全局影响路径，需真·全量（CI 或完整本地流程）")
        elif path.startswith(INTERFACE_PREFIX):
            plan["packages"].update(INTERFACE_PACKAGES)
            plan["reasons"].append(f"{path}: 接口层，选入三个接口包及全部下游")
        elif any(path.startswith(p) for p in PACKAGE_PREFIXES):
            package = path.split("/")[2]
            plan["packages"].add(package)
            plan["reasons"].append(f"{path}: 包 {package} 及其下游")
        elif path.startswith("tools/"):
            needs_quality_gate = True
            plan["reasons"].append(f"{path}: tools 层，跑 quality_gate")
        else:
            needs_repo_gate = True
            plan["reasons"].append(f"{path}: 文档/其他，跑 repository_gate")
    if plan["full"]:
        plan["actions"] = ["tools/quality_gate.sh",
                          "colcon build --symlink-install && colcon test && colcon test-result --verbose"]
    else:
        if needs_quality_gate:
            plan["actions"].append("tools/quality_gate.sh")
        elif needs_repo_gate:
            plan["actions"].append("python3 tools/repository_gate.py")
        if plan["packages"]:
            packages = " ".join(sorted(plan["packages"]))
            plan["actions"].append(
                f"colcon test --packages-above {packages} --event-handlers console_direct+"
            )
    plan["packages"] = sorted(plan["packages"])
    return plan


def glob_match(pattern: str, path: str) -> bool:
    regex = re.escape(pattern).replace(r"\*\*", ".*").replace(r"\*", "[^/]*")
    return re.fullmatch(regex, path) is not None


def load_case_covers(cases_dir: Path) -> dict[str, list[str]]:
    covers = {}
    for file in sorted(Path(cases_dir).glob("*.yaml")):
        for case in (yaml.safe_load(file.read_text()) or {}).get("cases", []):
            covers[case["id"]] = case.get("covers", [])
    return covers


def affected_cases(changed_files: list[str], covers: dict[str, list[str]]) -> list[str]:
    hits = []
    for case_id, patterns in covers.items():
        if any(glob_match(p, f) for p in patterns for f in changed_files):
            hits.append(case_id)
    return sorted(hits)


def changed_from_git(repo_root: Path, base: str) -> list[str]:
    files = set()
    for args in (["diff", "--name-only", f"{base}...HEAD"], ["diff", "--name-only"],
                 ["diff", "--cached", "--name-only"],
                 ["ls-files", "--others", "--exclude-standard"]):
        out = subprocess.run(["git", "-C", str(repo_root), *args],
                             capture_output=True, text=True)
        if out.returncode == 0:
            files.update(l for l in out.stdout.splitlines() if l)
    return sorted(files)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--base", default="origin/main")
    parser.add_argument("--run", action="store_true", help="执行动作（默认只打印计划）")
    args = parser.parse_args(argv)

    changed = changed_from_git(args.repo_root, args.base)
    if not changed:
        print("无改动，无需执行。")
        return 0
    plan = classify(changed)
    print(f"改动 {len(changed)} 个文件；判定依据：")
    for reason in plan["reasons"][:20]:
        print(f"  - {reason}")
    covers = load_case_covers(args.repo_root / "domains/rt_control/testing/cases")
    hits = affected_cases(changed, covers)
    if hits:
        print(f"受影响的目录用例（信息性）：{', '.join(hits)}")
    print("执行动作：")
    for action in plan["actions"]:
        print(f"  $ {action}")
    if plan["full"]:
        print("注意：命中全局影响路径，本地增量不适用，请跑全量（或交 CI）。")
    if not args.run:
        return 0
    for action in plan["actions"]:
        print(f"==> {action}")
        result = subprocess.run(["bash", "-c", action], cwd=args.repo_root)
        if result.returncode != 0:
            print(f"FAIL: {action}", file=sys.stderr)
            return result.returncode
    print("PASS: scoped tests")
    return 0


if __name__ == "__main__":
    sys.exit(main())
