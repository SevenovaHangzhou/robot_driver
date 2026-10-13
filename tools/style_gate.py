#!/usr/bin/env python3
"""Check changed lines/files against a merge base; never rewrite source files."""

import argparse
import os
import re
import shutil
import subprocess
from pathlib import Path

CPP_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
RUFF_VERSION = "0.11.13"


def run(args, root):
    return subprocess.run(args, cwd=root, text=True, capture_output=True, check=True).stdout


def changed_files(root, base):
    """Include committed, staged and unstaged changes; exclude deleted files."""
    merge_base = run(["git", "merge-base", "HEAD", base], root).strip()
    names = run(["git", "diff", "--name-only", "-z", "--diff-filter=ACMRT", merge_base], root)
    added = run(["git", "diff", "--name-only", "-z", "--diff-filter=A", merge_base], root)
    return merge_base, [p for p in names.split("\0") if p], set(added.split("\0"))


def is_python(root, name):
    path = root / name
    return path.suffix == ".py" or (
        not path.suffix and path.is_file() and b"python3" in path.read_bytes().split(b"\n", 1)[0]
    )


def find_tool(names, version, root, required):
    for name in names:
        executable = shutil.which(name)
        if executable:
            actual = run([executable, "--version"], root).strip()
            if re.search(version, actual):
                return executable
            raise ValueError(f"{name}: unexpected version: {actual}")
    message = f"missing {'/'.join(names)}"
    if required:
        raise ValueError(message)
    print(f"NOTICE: {message}; CI requires pinned style tools")
    return None


def check(root, base, required=False):
    merge_base, names, added = changed_files(root, base)
    print(f"Style merge base: {merge_base}")
    python = [p for p in names if is_python(root, p)]
    cpp = [p for p in names if Path(p).suffix in CPP_SUFFIXES]
    formatter = find_tool(["clang-format-14", "clang-format"], r"version 14\.", root, required)
    ruff = find_tool(["ruff"], r"^ruff 0\.11\.13$", root, required)
    if cpp and formatter:
        git_formatter = shutil.which("git-clang-format-14") or shutil.which("git-clang-format")
        if not git_formatter:
            if required:
                raise ValueError("missing git-clang-format-14/git-clang-format")
            print("NOTICE: missing git-clang-format; CI requires it")
        else:
            output = run(
                [git_formatter, "--binary", formatter, "--diff", merge_base, "--", *cpp], root
            )
            if "diff --git " in output:
                print(output)
                raise ValueError("C++ changed lines require clang-format 14")
    if python and ruff:
        print(run([ruff, "check", "--", *python], root))
        new_python = [p for p in python if p in added]
        if new_python:
            print(run([ruff, "format", "--check", "--", *new_python], root))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default=os.environ.get("STYLE_BASE_REF", "origin/main"))
    parser.add_argument("--required", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    try:
        check(root, args.base, args.required or os.environ.get("REQUIRE_STYLE_TOOLS") == "1")
    except (ValueError, subprocess.CalledProcessError) as error:
        print(f"FAIL: {error}")
        if isinstance(error, subprocess.CalledProcessError):
            print(error.stdout or "")
            print(error.stderr or "")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
