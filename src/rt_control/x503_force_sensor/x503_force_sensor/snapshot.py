"""Immutable startup calibration and runtime invalidation; no device access."""
from __future__ import annotations

import copy
import json

from .calibration import calibration_from_values


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
            self._records, self._calibration, self._invalid = {}, {}, set()
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

    def controller_parameters(
        self,
        name: str,
        *,
        frame_id: str,
        wrench_topic: str,
        raw_topic: str,
        calibration_topic: str,
    ) -> dict:
        """Render immutable parameters for one C++ broadcaster instance."""
        try:
            record = self._records[name]
            calibration = self._calibration[name]
            position = record["slave_position"]
            values = record["values"]
        except (KeyError, TypeError) as error:
            raise SnapshotValidationError("Unknown snapshot sensor") from error
        for value in (name, frame_id, wrench_topic, raw_topic, calibration_topic):
            if (
                not isinstance(value, str)
                or not value.strip()
                or value != value.strip()
                or "TBD" in value
            ):
                raise SnapshotValidationError(
                    "Controller names, topics and frame must be explicit"
                )
        if values.get("validity_policy") != "sample_codes_in_range":
            raise SnapshotValidationError(
                "C++ broadcaster requires the approved X503 range policy"
            )
        try:
            sample_code_min = int(values["sample_code_min"])
            sample_code_max = int(values["sample_code_max"])
        except (KeyError, TypeError, ValueError) as error:
            raise SnapshotValidationError("Invalid X503 sample-code range") from error
        if not -2147483648 <= sample_code_min <= sample_code_max <= 2147483647:
            raise SnapshotValidationError("Invalid X503 sample-code range")
        valid = (
            calibration.engineering_units_valid
            and calibration.sample_validity_confirmed
            and calibration.units == (5, 5, 5, 7, 7, 7)
            and name not in self._invalid
        )
        decimals = list(calibration.decimals) if valid else []
        units = list(calibration.units) if valid else []
        scales = [10.0 ** (-decimal) for decimal in decimals] if valid else [1.0] * 6
        parameters = {
            "sensor_name": name,
            "frame_id": frame_id,
            "value_interfaces": [
                f"{name}/channel_{index}_raw" for index in range(1, 7)
            ],
            "auxiliary_interfaces": [
                f"{name}/sample_code_{index}_raw" for index in range(1, 7)
            ],
            "scale_factors": scales,
            "validity_policy": "all_exact_in_range",
            "minimum_auxiliary_value": sample_code_min,
            "maximum_auxiliary_value": sample_code_max,
            "calibration_valid": valid,
            "startup_id": self.startup_id,
            "snapshot_source": self.source,
            "wrench_topic": wrench_topic,
            "raw_topic": raw_topic,
            "calibration_topic": calibration_topic,
            "diagnostic_name": (
                f"/robot/rt_control/x503b/{name}/calibration"
            ),
            "link_interface": "ethercat_master/link_up",
            "al_state_interface": f"ethercat_slave_{position}/al_state",
        }
        if valid:
            parameters["decimals"] = decimals
            parameters["unit_codes"] = units
        return parameters
