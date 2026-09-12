"""Immutable startup calibration and runtime invalidation; no device access."""
from __future__ import annotations

import copy
import json

from .bridge import CalibrationSnapshot, calibration_from_values


class SnapshotValidationError(ValueError):
    """A calibration snapshot does not belong to this runtime or is malformed."""


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise SnapshotValidationError("Duplicate snapshot key")
        result[key] = value
    return result


class SnapshotState:
    def __init__(self, payload: str, startup_id: str, expected_positions: dict[str, int]):
        try:
            if not startup_id or not isinstance(payload, str) or len(payload) > 65536:
                raise SnapshotValidationError("Missing startup snapshot")
            document = json.loads(payload, object_pairs_hook=_unique_object)
            if (document["schema_version"] != 1 or document["startup_id"] != startup_id
                    or document["source"] not in {"preop_sdo", "mock"}):
                raise SnapshotValidationError("Snapshot belongs to another startup or source")
            self._records, self._calibration, self._seen_op, self._invalid = {}, {}, set(), set()
            for raw in document["sensors"]:
                name, position, values = raw["sensor_name"], raw["slave_position"], raw["values"]
                if (name not in expected_positions or name in self._records
                        or type(position) is not int or position != expected_positions[name]
                        or values["slave_position"] != str(position)
                        or values["snapshot_valid"] not in {"true", "false"}):
                    raise SnapshotValidationError("Snapshot sensor identity mismatch")
                calibration = calibration_from_values(values)
                if values["snapshot_valid"] == "true":
                    if (document["source"] != "preop_sdo" or not calibration.engineering_units_valid
                            or not calibration.sample_validity_confirmed
                            or calibration.units != (5, 5, 5, 7, 7, 7)):
                        raise SnapshotValidationError("Invalid X503 unit/decimal snapshot")
                else:
                    self._invalid.add(name)
                self._records[name] = copy.deepcopy(raw)
                self._calibration[name] = calibration
            if set(self._records) != set(expected_positions):
                raise SnapshotValidationError("Incomplete sensor set in startup snapshot")
            self.startup_id = startup_id
            self.source = document["source"]
        except SnapshotValidationError:
            raise
        except (KeyError, TypeError, ValueError, AttributeError) as error:
            raise SnapshotValidationError("Malformed startup snapshot") from error

    def observe(self, name: str, *, al_state, link_up: bool) -> bool:
        """Once an observed OP sensor goes offline, only a new startup may restore it."""
        if name in self._invalid:
            return False
        if al_state == 8 and link_up is True:
            self._seen_op.add(name)
        elif name in self._seen_op:
            self._invalid.add(name)
            self._records[name]["values"]["snapshot_valid"] = "false"
            self._records[name]["error"] = "Sensor left OP or link was lost; fresh PREOP initialization required"
            return True
        return False

    def calibration(self, name: str) -> CalibrationSnapshot:
        if name not in self._seen_op or name in self._invalid:
            return CalibrationSnapshot(False, (), (), "unresolved")
        return self._calibration[name]

    def statuses(self) -> list[dict]:
        return copy.deepcopy(list(self._records.values()))
