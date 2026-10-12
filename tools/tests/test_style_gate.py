"""Behavior tests for incremental style scope and tool policy."""

import subprocess
from unittest.mock import patch

import pytest

from tools import style_gate
from tools.repository_gate import check_cxx_warnings


def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()


@pytest.fixture
def repository(tmp_path):
    git(tmp_path, "init", "-b", "main")
    git(tmp_path, "config", "user.email", "test@example.invalid")
    git(tmp_path, "config", "user.name", "Test")
    (tmp_path / "old.py").write_text("x = 1\n")
    (tmp_path / "old.cpp").write_text("int x;\n")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "-m", "base")
    return tmp_path


def test_merge_base_collects_committed_staged_and_worktree_changes(repository):
    base = git(repository, "rev-parse", "HEAD")
    (repository / "old.py").write_text("x = 2\n")
    git(repository, "commit", "-am", "change")
    (repository / "new.py").write_text("x = 3\n")
    git(repository, "add", "new.py")
    (repository / "old.cpp").write_text("int y;\n")
    actual, names, added = style_gate.changed_files(repository, base)
    assert actual == base
    assert set(names) == {"old.py", "new.py", "old.cpp"}
    assert "new.py" in added and "old.py" not in added


def test_missing_base_fails_instead_of_silently_skipping(repository):
    with pytest.raises(subprocess.CalledProcessError):
        style_gate.check(repository, "missing-ref")


def test_missing_tools_notice_locally_and_fail_in_ci(repository, capsys):
    with patch.object(style_gate.shutil, "which", return_value=None):
        assert style_gate.find_tool(["ruff"], "0.11.13", repository, False) is None
        assert "NOTICE" in capsys.readouterr().out
        with pytest.raises(ValueError, match="missing"):
            style_gate.find_tool(["ruff"], "0.11.13", repository, True)


def test_wrong_tool_version_is_rejected(repository):
    with (
        patch.object(style_gate.shutil, "which", return_value="ruff"),
        patch.object(style_gate, "run", return_value="ruff 0.12.0"),
    ):
        with pytest.raises(ValueError, match="unexpected version"):
            style_gate.find_tool(["ruff"], r"^ruff 0\.11\.13$", repository, False)


def test_scope_only_formats_new_python_and_cpp_changed_lines(repository):
    calls = []

    def command(args, root):
        calls.append(args)
        return "no modified files to format"

    with (
        patch.object(
            style_gate,
            "changed_files",
            return_value=("BASE", ["old.py", "new.py", "old.cpp"], {"new.py"}),
        ),
        patch.object(style_gate, "find_tool", side_effect=["clang-format-14", "ruff"]),
        patch.object(style_gate.shutil, "which", return_value="git-clang-format-14"),
        patch.object(style_gate, "run", side_effect=command),
    ):
        style_gate.check(repository, "main", True)
    assert calls[0] == [
        "git-clang-format-14",
        "--binary",
        "clang-format-14",
        "--diff",
        "BASE",
        "--",
        "old.cpp",
    ]
    assert calls[1] == ["ruff", "check", "--", "old.py", "new.py"]
    assert calls[2] == ["ruff", "format", "--check", "--", "new.py"]


def test_cpp_diff_is_a_failure(repository):
    with (
        patch.object(style_gate, "changed_files", return_value=("BASE", ["old.cpp"], set())),
        patch.object(style_gate, "find_tool", side_effect=["clang-format-14", "ruff"]),
        patch.object(style_gate.shutil, "which", return_value="git-clang-format-14"),
        patch.object(style_gate, "run", return_value="diff --git a/old.cpp b/old.cpp"),
    ):
        with pytest.raises(ValueError, match="changed lines"):
            style_gate.check(repository, "main", True)


def test_extensionless_python_script_is_in_scope(repository):
    (repository / "launch_tool").write_text("#!/usr/bin/env python3\n")
    assert style_gate.is_python(repository, "launch_tool")
    assert not style_gate.is_python(repository, "old.cpp")


WARNING_BASE = "add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)\n"
OWNED = "src/rt_control/example/CMakeLists.txt"


@pytest.mark.parametrize(
    "target",
    [
        "add_library(core SHARED a.cpp)",
        "add_executable(tool a.cpp)",
        "ament_add_gtest(test_${name} test.cpp)",
    ],
)
def test_all_owned_target_kinds_inherit_warnings(target):
    assert check_cxx_warnings(OWNED, WARNING_BASE + target) == []


@pytest.mark.parametrize(
    "text",
    [
        "add_executable(core a.cpp)",
        "if(OFF)\n" + WARNING_BASE + "endif()\nadd_executable(core a.cpp)",
        "# " + WARNING_BASE + "add_executable(core a.cpp)",
        "add_executable(core a.cpp)\n" + WARNING_BASE,
        WARNING_BASE.replace("-Wshadow", "") + "add_executable(core a.cpp)",
        WARNING_BASE
        + "add_executable(core a.cpp)\ntarget_compile_options(core PRIVATE -Wno-conversion)",
    ],
)
def test_missing_late_commented_or_disabled_warnings_are_rejected(text):
    assert check_cxx_warnings(OWNED, text)


def test_vendor_and_data_only_packages_are_outside_warning_policy():
    assert check_cxx_warnings("src/vendor/example/CMakeLists.txt", "add_library(core a.cpp)") == []
    assert check_cxx_warnings(OWNED, "ament_package()") == []
