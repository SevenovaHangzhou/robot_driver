"""Field tuning file checks (ELECTRI-127); no ROS graph is needed for these cases."""

from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_bringup import operator_tuning  # noqa: E402


def write(path: Path, document) -> Path:
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def test_valid_file_returns_only_whitelisted_numbers(tmp_path):
    path = write(
        tmp_path / "t.yaml",
        {
            "schema_version": 1,
            "controller": "swerve_controller",
            "parameters": {"max_steering_slew": 0.5, "alignment_threshold": 1},
        },
    )
    assert operator_tuning.load_tuning(path, "swerve_controller") == {
        "max_steering_slew": 0.5,
        "alignment_threshold": 1.0,
    }


@pytest.mark.parametrize(
    "document,message",
    [
        ({"schema_version": 2, "controller": "swerve_controller", "parameters": {}}, "header"),
        ({"schema_version": 1, "controller": "other", "parameters": {}}, "header"),
        (
            {
                "schema_version": 1,
                "controller": "swerve_controller",
                "parameters": {"max_linear_speed": 1.0},
            },
            "not tunable",
        ),
        (
            {
                "schema_version": 1,
                "controller": "swerve_controller",
                "parameters": {"max_steering_slew": True},
            },
            "number",
        ),
        (
            {
                "schema_version": 1,
                "controller": "swerve_controller",
                "parameters": {"max_steering_slew": -1.0},
            },
            "positive",
        ),
    ],
)
def test_invalid_files_are_rejected(tmp_path, document, message):
    path = write(tmp_path / "t.yaml", document)
    with pytest.raises(ValueError, match=message):
        operator_tuning.load_tuning(path, "swerve_controller")


def test_without_a_tuning_file_the_start_continues_with_defaults(tmp_path, monkeypatch, capsys):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    write(runtime / "artifact.yaml", {"selection": {"machine_unit": "Unit-1"}})
    monkeypatch.setattr(operator_tuning, "TUNING_ROOT", tmp_path / "tuning")
    assert operator_tuning.main([str(runtime)]) == 0
    assert "RT_SCOPE_TUNING none unit=Unit-1" in capsys.readouterr().out


def test_an_invalid_file_is_reported_and_never_blocks_the_start(tmp_path, monkeypatch, capsys):
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    write(runtime / "artifact.yaml", {"selection": {"machine_unit": "Unit-1"}})
    (tmp_path / "tuning/Unit-1").mkdir(parents=True)
    write(tmp_path / "tuning/Unit-1/swerve_controller.yaml", {"schema_version": 9})
    monkeypatch.setattr(operator_tuning, "TUNING_ROOT", tmp_path / "tuning")
    assert operator_tuning.main([str(runtime)]) == 0
    output = capsys.readouterr().out
    assert "rejected_file=" in output and "RT_SCOPE_TUNING none" in output
