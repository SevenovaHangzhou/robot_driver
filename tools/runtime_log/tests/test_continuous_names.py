import json
import math

import pytest

from runtime_log.continuous_names import MovementTotals, display_name
from runtime_log.chassis_waveform import ChassisFeedback
from test_chassis_waveform import parameters, feedback


@pytest.mark.parametrize("direction,axis", [((0.1, 0, 0), "x_m"), ((0, 0.1, 0), "y_m")])
def test_round_trip_accumulates_instead_of_cancelling(direction, axis):
    p = parameters()
    total = MovementTotals(ChassisFeedback(p))
    for seq, position in enumerate((0, 10, 0), 1):
        row = feedback(p, *direction)
        row["seq"] = seq
        for joint in p["drive_joints"]:
            row[joint + "/position"] = position
        total.add(row)
    result = total.result()
    assert result[axis] == pytest.approx(2)
    assert result["yaw_deg"] == pytest.approx(0)


def test_rotation_round_trip_uses_degrees():
    p = parameters()
    total = MovementTotals(ChassisFeedback(p))
    for seq, scale in enumerate((0, 1, 0), 1):
        row = feedback(p, 0, 0, 0.5)
        row["seq"] = seq
        for joint in p["drive_joints"]:
            row[joint + "/position"] = row[joint + "/velocity"] * scale
        total.add(row)
    result = total.result()
    assert result["yaw_deg"] == pytest.approx(math.degrees(1))
    assert result["x_m"] == pytest.approx(0, abs=1e-10)
    assert result["y_m"] == pytest.approx(0, abs=1e-10)


@pytest.mark.parametrize("gap", ["sequence", "invalid"])
def test_missing_feedback_does_not_count_a_jump(gap):
    p = parameters()
    total = MovementTotals(ChassisFeedback(p))
    for seq, position in enumerate((0, 0, 100, 101), 1):
        row = feedback(p, 0.1, 0, 0)
        row["seq"] = seq + (10 if gap == "sequence" and seq >= 3 else 0)
        for joint in p["drive_joints"]:
            row[joint + "/position"] = position
        if gap == "invalid" and seq == 2:
            row["ethercat_domain/process_data_age_ms"] = math.inf
        total.add(row)
    assert total.result()["x_m"] == pytest.approx(0.1)


def test_no_valid_samples_is_unavailable_not_zero():
    total = MovementTotals(ChassisFeedback(parameters()))
    assert total.result()["x_m"] is None
    assert "—" in display_name({"source": "auto:continuous"}, total.result())


def test_continuous_display_name_and_manual_name_preservation():
    totals = {"x_m": 2.3456, "y_m": 0.125, "yaw_deg": 90, "valid_intervals": 10}
    name = display_name(
        {"name": "continuous-runtime_log_recorder-1", "source": "auto:continuous"}, totals
    )
    assert "2.346 m" in name and "0.125 m" in name and "90.00°" in name
    assert "continuous-runtime_log_recorder" not in name
    assert display_name({"name": "manual trial", "source": "manual"}, totals) == "manual trial"


def test_archived_name_is_computed_once_cached_and_used_by_pages(tmp_path, monkeypatch):
    from mcap.writer import Writer
    from runtime_log.continuous_names import ContinuousNames
    from runtime_log import experiment_report

    p = parameters()
    root = tmp_path / "run"
    directory = root / "experiments" / "original-url"
    directory.mkdir(parents=True)
    (root / "run.json").write_text(
        json.dumps(
            {
                "artifact": {
                    "controllers": {"chassis": "swerve_driver/SwerveController"},
                    "controller_parameters": {"chassis": p},
                }
            }
        )
    )
    meta = {"name": "continuous-runtime_log_recorder-1", "source": "auto:continuous", "summary": {}}
    (directory / "meta.json").write_text(json.dumps(meta))
    rows = []
    for seq, position in enumerate((0, 10, 0), 1):
        row = feedback(p, 0.1, 0, 0)
        row["seq"] = seq
        for joint in p["drive_joints"]:
            row[joint + "/position"] = position
        rows.append(row)
    source = directory / "data.mcap"
    with source.open("wb") as f:
        writer = Writer(f)
        writer.start()
        schema = writer.register_schema(
            "cycle",
            "jsonschema",
            json.dumps({"properties": {name: {"type": "number"} for name in rows[0]}}).encode(),
        )
        channel = writer.register_channel("cycles", "json", schema)
        for row in rows:
            writer.add_message(
                channel, row["seq"] * 1000000, json.dumps(row).encode(), row["seq"] * 1000000
            )
        writer.finish()
    names = ContinuousNames()
    try:
        names.name(meta, source)
        for future in list(names.jobs.values()):
            future.result(timeout=5)
        name = names.name(meta, source)
        assert "2.000 m" in name
        monkeypatch.setattr(experiment_report, "NAMES", names)
        html = experiment_report.experiments_section({"id": "run"}, root)
        assert name in html and "experiments/original-url/" in html
        detail = experiment_report.render_experiment("run", {**meta, "display_name": name}, {})
        assert name in detail
        # A new service instance uses the disk cache without scheduling another scan.
        fresh = ContinuousNames()
        try:
            assert fresh.name(meta, source) == name
            assert not fresh.jobs
        finally:
            fresh.pool.shutdown(wait=True)
    finally:
        names.pool.shutdown(wait=True)
