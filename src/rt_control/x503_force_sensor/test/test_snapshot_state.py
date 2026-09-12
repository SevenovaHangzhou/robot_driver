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


def test_initial_non_op_does_not_activate_force_or_destroy_metadata():
    cache = state()
    cache.observe("right_force_sensor", al_state=2, link_up=True)
    assert cache.calibration("right_force_sensor").valid is False
    cache.observe("right_force_sensor", al_state=8, link_up=True)
    assert cache.calibration("right_force_sensor").valid is True


@pytest.mark.parametrize("al_state,link_up", [(2, True), (0, True), (8, False), (None, True)])
def test_loss_after_op_permanently_invalidates_this_startup(al_state, link_up):
    cache = state()
    cache.observe("right_force_sensor", al_state=8, link_up=True)
    assert cache.calibration("right_force_sensor").valid
    assert cache.observe("right_force_sensor", al_state=al_state, link_up=link_up)
    cache.observe("right_force_sensor", al_state=8, link_up=True)
    assert cache.calibration("right_force_sensor").valid is False
    assert cache.statuses()[0]["values"]["snapshot_valid"] == "false"


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
