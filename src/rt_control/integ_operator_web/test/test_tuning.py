"""Field tuning file handling (ELECTRI-127); no ROS needed."""

import json
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integ_operator_web.tuning import FieldTuning, TuningError  # noqa: E402

DEFAULTS = {
    "max_steering_slew": 0.61,
    "max_steering_acceleration": 0.61,
    "max_steering_jerk": 1.75,
    "alignment_threshold": 1.57,
    "align_first_threshold": 0.087,
    "align_first_release": 0.052,
}


@pytest.fixture
def tuning(tmp_path):
    artifact = tmp_path / "artifact"
    artifact.mkdir()
    (artifact / "artifact.yaml").write_text(
        yaml.safe_dump({"selection": {"machine_unit": "Unit-1"}})
    )
    (artifact / "controllers.yaml").write_text(
        yaml.safe_dump({"swerve_controller": {"ros__parameters": {**DEFAULTS, "other": 1}}})
    )
    return FieldTuning(artifact, "swerve_controller", root=tmp_path / "tuning")


def test_paths_follow_the_unit_of_the_artifact(tuning, tmp_path):
    assert tuning.path == tmp_path / "tuning/Unit-1/swerve_controller.yaml"
    assert tuning.source() == {"source": "default", "updated_at": ""}
    assert tuning.defaults() == DEFAULTS


def test_store_writes_the_file_read_by_scope_operator_tuning_and_logs_history(tuning):
    tuning.store({"max_steering_slew": 0.5}, "set")
    document = yaml.safe_load(tuning.path.read_text())
    assert document["schema_version"] == 1
    assert document["controller"] == "swerve_controller"
    assert document["parameters"] == {"max_steering_slew": 0.5}
    assert tuning.source()["source"] == "field"
    tuning.store(None, "restore_defaults")
    assert not tuning.path.exists()
    history = [json.loads(line) for line in tuning.history.read_text().splitlines()]
    assert [entry["action"] for entry in history] == ["set", "restore_defaults"]
    assert history[1]["before"] == {"max_steering_slew": 0.5}
    assert history[1]["after"] is None


def test_only_steering_parameters_can_be_stored(tuning):
    with pytest.raises(TuningError):
        tuning.store({"max_linear_speed": 1.0}, "set")
    assert not tuning.path.exists()


def test_missing_defaults_are_reported(tuning):
    (tuning.generated_dir / "controllers.yaml").write_text(yaml.safe_dump({}))
    with pytest.raises(TuningError, match="默认值"):
        tuning.defaults()
