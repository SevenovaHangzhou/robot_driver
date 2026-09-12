"""Tests for the build-time machine-profile validator entry point."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys


BRINGUP_DIR = Path(__file__).resolve().parents[1]
SCRIPT = BRINGUP_DIR / "scripts/validate_machine_profile.py"
MANIFEST = BRINGUP_DIR / "config/machines/alfa_v3.yaml"
CMAKE = BRINGUP_DIR / "CMakeLists.txt"


def _run(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(SCRIPT), *arguments],
        cwd=BRINGUP_DIR.parents[2],
        text=True,
        capture_output=True,
        check=False,
    )


def test_validator_checks_every_profile_scope_pair():
    result = _run("--manifest", str(MANIFEST), "--all")

    assert result.returncode == 0, result.stderr
    assert "5 physical profiles" in result.stdout
    assert "10 profile/scope selections" in result.stdout


def test_validator_requires_a_complete_selection_without_all():
    result = _run("--manifest", str(MANIFEST))

    assert result.returncode == 1
    assert "required unless --all" in result.stderr


def test_validator_rejects_mixing_all_and_explicit_selection():
    result = _run(
        "--manifest",
        str(MANIFEST),
        "--all",
        "--physical-profile",
        "arms_only",
        "--control-scope",
        "arms_only",
    )

    assert result.returncode == 1
    assert "cannot be combined" in result.stderr


def test_validator_reports_selected_arm_counts():
    result = _run(
        "--manifest",
        str(MANIFEST),
        "--physical-profile",
        "arms_only",
        "--control-scope",
        "arms_only",
    )

    assert result.returncode == 0, result.stderr
    assert "actuators=16" in result.stdout


def test_validator_is_an_installed_build_target():
    cmake = CMAKE.read_text(encoding="utf-8")

    assert "validate_machine_profile ALL" in cmake
    assert "scripts/validate_machine_profile.py" in cmake
    assert "--all" in cmake


def test_validator_can_enforce_runtime_readiness():
    result = _run(
        "--manifest",
        str(MANIFEST),
        "--physical-profile",
        "arms_only",
        "--control-scope",
        "arms_only",
        "--require-runtime-ready",
    )

    assert result.returncode == 1
    assert "not runtime-ready" in result.stderr
