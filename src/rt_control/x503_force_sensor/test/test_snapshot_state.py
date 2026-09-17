"""Runtime conversion can only consume its own startup snapshot and PDO state."""
import copy
import json

import pytest

from x503_force_sensor.snapshot import SnapshotState, SnapshotValidationError


def payload():
    values = {
        "snapshot_valid": "true", "slave_position": "14",
        "engineering_unit_contract": "force_N_torque_Nm",
        "validity_policy": "sample_codes_in_range",
        "sample_code_min": "-999999", "sample_code_max": "999999",
        **{f"decimal_{i}": str(1 if i <= 3 else 3) for i in range(1, 7)},
        **{f"unit_{i}": str(5 if i <= 3 else 7) for i in range(1, 7)},
    }
    return {"schema_version": 1, "startup_id": "this-start", "source": "preop_sdo",
            "sensors": [{"sensor_name": "right_force_sensor", "slave_position": 14,
                         "values": values, "error": ""}]}


def state(document=None, startup_id="this-start"):
    return SnapshotState(json.dumps(document or payload()), startup_id, {"right_force_sensor": 14})


def test_old_startup_snapshot_is_rejected():
    with pytest.raises(SnapshotValidationError):
        state(startup_id="different-start")


@pytest.mark.parametrize("mutation", ["unit", "decimal", "position", "duplicate", "source"])
def test_invalid_snapshot_cannot_create_valid_force(mutation):
    doc = copy.deepcopy(payload())
    if mutation == "unit": doc["sensors"][0]["values"]["unit_1"] = "6"
    elif mutation == "decimal": doc["sensors"][0]["values"]["decimal_1"] = "11"
    elif mutation == "position": doc["sensors"][0]["slave_position"] = 15
    elif mutation == "duplicate": doc["sensors"].append(copy.deepcopy(doc["sensors"][0]))
    elif mutation == "source": doc["source"] = "historical_cache"
    with pytest.raises(SnapshotValidationError):
        state(doc)


def test_snapshot_renders_cpp_controller_parameters_without_runtime_device_access():
    parameters = state().controller_parameters(
        "right_force_sensor",
        frame_id="right_ft_sensor_link",
        wrench_topic="/rt_control/right_x503b/wrench",
        raw_topic="/rt_control/right_x503b/raw",
        calibration_topic="/rt_control/x503b/calibration",
    )

    assert parameters == {
        "sensor_name": "right_force_sensor",
        "frame_id": "right_ft_sensor_link",
        "value_interfaces": [
            f"right_force_sensor/channel_{index}_raw" for index in range(1, 7)
        ],
        "auxiliary_interfaces": [
            f"right_force_sensor/sample_code_{index}_raw" for index in range(1, 7)
        ],
        "scale_factors": [0.1, 0.1, 0.1, 0.001, 0.001, 0.001],
        "validity_policy": "all_exact_in_range",
        "minimum_auxiliary_value": -999999,
        "maximum_auxiliary_value": 999999,
        "calibration_valid": True,
        "startup_id": "this-start",
        "snapshot_source": "preop_sdo",
        "decimals": [1, 1, 1, 3, 3, 3],
        "unit_codes": [5, 5, 5, 7, 7, 7],
        "wrench_topic": "/rt_control/right_x503b/wrench",
        "raw_topic": "/rt_control/right_x503b/raw",
        "calibration_topic": "/rt_control/x503b/calibration",
        "diagnostic_name": (
            "/robot/rt_control/x503b/right_force_sensor/calibration"
        ),
        "link_interface": "ethercat_master/link_up",
        "al_state_interface": "ethercat_slave_14/al_state",
    }


def test_invalid_mock_snapshot_uses_neutral_scales_and_cannot_publish_wrench():
    document = payload()
    document["source"] = "mock"
    values = document["sensors"][0]["values"]
    values["snapshot_valid"] = "false"
    for key in [
        *(f"decimal_{index}" for index in range(1, 7)),
        *(f"unit_{index}" for index in range(1, 7)),
    ]:
        values.pop(key)

    parameters = state(document).controller_parameters(
        "right_force_sensor",
        frame_id="right_ft_sensor_link",
        wrench_topic="/rt_control/right_x503b/wrench",
        raw_topic="/rt_control/right_x503b/raw",
        calibration_topic="/rt_control/x503b/calibration",
    )

    assert parameters["calibration_valid"] is False
    assert parameters["scale_factors"] == [1.0] * 6
    assert "decimals" not in parameters
    assert "unit_codes" not in parameters
