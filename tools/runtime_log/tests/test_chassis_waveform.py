import json
import math

import pytest
from mcap.writer import Writer

from runtime_log.chassis_waveform import (
    CHANNEL,
    DISPLACEMENT_CHANNEL,
    ENCODER_CHANNEL,
    ENCODER_DISPLACEMENT_CHANNEL,
    ChassisDisplacement,
    ChassisFeedback,
    for_source,
)
from runtime_log.experiment_report import render_experiment
from runtime_log.waveform import waveform
from runtime_log.waveform_queries import WaveformQueries


def parameters():
    return dict(
        drive_joints=[f"wheel{i}_joint" for i in range(1, 5)],
        steering_joints=[f"steer{i}_joint" for i in range(1, 5)],
        steering_encoders=[f"steering_encoder_{i}" for i in range(1, 5)],
        module_x=[-0.305, 0.305, -0.305, 0.305],
        module_y=[0.305, 0.305, -0.305, -0.305],
        wheel_radius=[0.1] * 4,
        steering_to_yaw_sign=[-1.0] * 4,
        steering_min=[-2.356194490192345] * 4,
        steering_max=[2.356194490192345] * 4,
        steering_limit_margin=[0.2617993877991494] * 4,
        feedback_timeout=0.2,
        slip_residual_threshold=0.1,
    )


def feedback(p, vx, vy, w):
    row = {"seq": 1, "period_us": 1000, "ethercat_domain/process_data_age_ms": 1}
    for i, (drive, steer) in enumerate(zip(p["drive_joints"], p["steering_joints"])):
        u, v = vx - w * p["module_y"][i], vy + w * p["module_x"][i]
        angle, speed = math.atan2(v, u), math.hypot(u, v)
        if abs(angle) > math.pi / 2:
            angle += -math.pi if angle > 0 else math.pi
            speed = -speed
        row.update(
            {
                drive + "/velocity": speed / p["wheel_radius"][i],
                drive + "/position": 0,
                drive + "/status_word": 0x27,
                drive + "/mode_of_operation_display": 9,
                drive + "/sent_velocity": 9999,  # must never be used as actual motion
                steer + "/position": angle / p["steering_to_yaw_sign"][i],
                steer + "/status_word": 0x27,
                steer + "/mode_of_operation_display": 8,
                f"steering_encoder_{i + 1}/position": angle,
                f"steering_encoder_{i + 1}/feedback_age_ms": 1,
                f"steering_encoder_{i + 1}/ready": 1,
            }
        )
    return row


@pytest.mark.parametrize(
    "twist",
    [(0, 0, 0), (0.2, 0, 0), (0, 0.2, 0), (0, 0, 0.5), (-0.2, -0.1, -0.3), (0.2, -0.1, 0.4)],
)
def test_actual_twist_matches_forward_kinematics_and_signs(twist):
    p = parameters()
    assert ChassisFeedback(p).sample(feedback(p, *twist)) == pytest.approx(twist)


def test_noncentred_geometry_and_unequal_radii():
    p = parameters()
    p["module_x"] = [x + 0.7 for x in p["module_x"]]
    p["module_y"] = [y - 0.4 for y in p["module_y"]]
    p["wheel_radius"] = [0.08, 0.09, 0.1, 0.11]
    assert ChassisFeedback(p).sample(feedback(p, 0.3, -0.1, 0.2)) == pytest.approx((0.3, -0.1, 0.2))


def test_external_encoder_twist_uses_encoder_angle_and_validity():
    p = parameters()
    fit = ChassisFeedback(p)
    row = feedback(p, 0.2, 0.1, 0.3)
    for steer in p["steering_joints"]:
        row[steer + "/position"] = 9999
    assert fit.sample(row, ENCODER_CHANNEL) == pytest.approx((0.2, 0.1, 0.3))

    for encoder in p["steering_encoders"][:3]:
        row[encoder + "/ready"] = 0
    assert fit.sample(row, ENCODER_CHANNEL) is None


@pytest.mark.parametrize("channel", [DISPLACEMENT_CHANNEL, ENCODER_DISPLACEMENT_CHANNEL])
def test_interval_displacement_uses_wheel_positions_and_selected_angles(channel):
    p = parameters()
    odometry = ChassisDisplacement(ChassisFeedback(p), channel)
    initial = feedback(p, 0.2, 0, 0)
    assert odometry.sample(initial) == pytest.approx((0, 0, 0, 0))

    moved = feedback(p, 0.2, 0, 0)
    for drive in p["drive_joints"]:
        moved[drive + "/position"] = 1.0
    assert odometry.sample(moved) == pytest.approx((0.1, 0, 0, 0.1))


def test_interval_displacement_integrates_rotation():
    p = parameters()
    odometry = ChassisDisplacement(ChassisFeedback(p), DISPLACEMENT_CHANNEL)
    initial = feedback(p, 0, 0, 0.5)
    assert odometry.sample(initial) == pytest.approx((0, 0, 0, 0))
    moved = feedback(p, 0, 0, 0.5)
    for drive in p["drive_joints"]:
        moved[drive + "/position"] = moved[drive + "/velocity"] * 0.1
    x, y, yaw, distance = odometry.sample(moved)
    assert (x, y, distance) == pytest.approx((0, 0, 0), abs=1e-12)
    assert yaw == pytest.approx(0.05)


def test_invalid_feedback_is_missing_and_slipping_wheel_is_rejected():
    p = parameters()
    fit = ChassisFeedback(p)
    row = feedback(p, 0.2, 0.1, 0.3)
    row["wheel1_joint/velocity"] += 100
    assert fit.sample(row) == pytest.approx((0.2, 0.1, 0.3))
    row = feedback(p, 0.2, 0.1, 0.3)
    row["wheel1_joint/velocity"] = math.nan
    row["wheel2_joint/status_word"] = 0x40
    assert fit.sample(row) == pytest.approx((0.2, 0.1, 0.3))
    row["steer3_joint/position"] = 3
    assert fit.sample(row) is None
    row = feedback(p, 0.2, 0.1, 0.3)
    row["ethercat_domain/process_data_age_ms"] = 201
    assert fit.sample(row) is None
    row["ethercat_domain/process_data_age_ms"] = -1
    assert fit.sample(row) is None


def test_csp_wheel_feedback_remains_measured_velocity():
    p = parameters()
    row = feedback(p, 0.2, 0.1, 0.3)
    for drive in p["drive_joints"]:
        row[drive + "/mode_of_operation_display"] = 8
    assert ChassisFeedback(p).sample(row) == pytest.approx((0.2, 0.1, 0.3))


def archive(tmp_path):
    p = parameters()
    run = tmp_path / "run"
    experiment = run / "experiments" / "motion"
    experiment.mkdir(parents=True)
    document = {
        "artifact": {
            "controllers": {"swerve_controller": "swerve_driver/SwerveController"},
            "controller_parameters": {"swerve_controller": p},
        }
    }
    (run / "run.json").write_text(json.dumps(document))
    path = experiment / "data.mcap"
    with path.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        channel = writer.register_channel("/cycles", "json", 0)
        for i in range(20):
            row = feedback(p, 0.2, 0.1, 0.3)
            row["seq"] = i + 1
            for drive in p["drive_joints"]:
                row[drive + "/position"] = row[drive + "/velocity"] * i * 0.001
            if 5 <= i < 10:
                row["ethercat_domain/process_data_age_ms"] = 201
            writer.add_message(
                channel,
                log_time=1800000000000000000 + i * 1000000,
                publish_time=1800000000000000000 + i * 1000000,
                data=json.dumps(row).encode(),
            )
        writer.finish()
    return path, document


def test_waveform_uses_archived_configuration_and_gaps(tmp_path):
    path, document = archive(tmp_path)
    result = waveform(path, CHANNEL, width=1000)
    assert result["source"] == "measured_wheel_feedback"
    assert result["valid_feedback_samples"] == 15
    assert "不是 /cmd_vel" in result["description"]
    signals = {s["key"]: s for s in result["series"]}
    assert set(signals) == {"vx", "vy", "w"}
    for key, expected, unit in [("vx", 0.2, "m/s"), ("vy", 0.1, "m/s"), ("w", 0.3, "rad/s")]:
        assert signals[key]["unit"] == unit
        points = dict(signals[key]["points"])
        assert points[0] == pytest.approx(expected)
        assert points[0.005] is None
        assert points[0.010] == pytest.approx(expected)
    encoder_result = waveform(path, ENCODER_CHANNEL, width=1000)
    assert encoder_result["source"] == "measured_wheel_external_steering_encoder_feedback"
    for signal in encoder_result["series"]:
        assert dict(signal["points"])[0] == pytest.approx(
            {"vx": 0.2, "vy": 0.1, "w": 0.3}[signal["key"]]
        )
    for channel, source in [
        (DISPLACEMENT_CHANNEL, "measured_wheel_displacement_motor_steering"),
        (ENCODER_DISPLACEMENT_CHANNEL, "measured_wheel_displacement_external_steering_encoder"),
    ]:
        displacement = waveform(path, channel, width=1000)
        assert displacement["source"] == source
        signals = {signal["key"]: signal for signal in displacement["series"]}
        assert set(signals) == {"x", "y", "yaw", "distance"}
        assert all(signal["points"][0][1] == 0 for signal in signals.values())
        assert dict(signals["distance"]["points"])[0.019] > 0
    json.dumps(result, allow_nan=False)
    (path.parents[2] / "run.json").write_text("{}")
    assert for_source(path) is None
    with pytest.raises(ValueError, match="缺少底盘运动学"):
        waveform(path, CHANNEL)


def test_chassis_cache_invalidated_when_archived_geometry_changes(tmp_path):
    path, document = archive(tmp_path)
    queries = WaveformQueries()
    args = (path, CHANNEL, 0, 0.019, 1000)
    try:
        assert queries.request(*args)[0] == 202
        queries.pool.shutdown(wait=True)
        assert queries.request(*args)[0] == 200
        document["artifact"]["controller_parameters"]["swerve_controller"]["wheel_radius"] = [
            0.2
        ] * 4
        (path.parents[2] / "run.json").write_text(json.dumps(document))
        other = WaveformQueries()
        try:
            assert other.request(*args)[0] == 202
            other.pool.shutdown(wait=True)
            status, data = other.request(*args)
            assert status == 200
            assert data["series"][0]["points"][0][1] == pytest.approx(0.4)
        finally:
            other.pool.shutdown(wait=True)
    finally:
        queries.pool.shutdown(wait=True)


def test_viewer_has_actual_chassis_channel():
    page = render_experiment(
        "run", {"waveform_channels": ["wheel1_joint"]}, {}, chassis_available=True
    )
    assert f'value="{CHANNEL}"' in page
    assert f'value="{ENCODER_CHANNEL}"' in page
    assert f'value="{DISPLACEMENT_CHANNEL}"' in page
    assert f'value="{ENCODER_DISPLACEMENT_CHANNEL}"' in page
    assert "实际 vx / m/s" in page and "实际 w / rad/s" in page
    assert f'value="{CHANNEL}"' not in render_experiment("run", {}, {})
    assert f'value="{ENCODER_CHANNEL}"' not in render_experiment("run", {}, {})
    assert f'value="{DISPLACEMENT_CHANNEL}"' not in render_experiment("run", {}, {})
    assert f'value="{ENCODER_DISPLACEMENT_CHANNEL}"' not in render_experiment("run", {}, {})
