"""Validated X503 engineering-unit metadata used before runtime activation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


CONFIRMED_ENGINEERING_UNIT_CONTRACT = "force_N_torque_Nm"


@dataclass(frozen=True)
class CalibrationSnapshot:
    valid: bool
    decimals: tuple[int, ...]
    units: tuple[int, ...]
    validity_policy: str
    valid_sample_codes: tuple[int, ...] = ()
    engineering_unit_contract: str = "unresolved"
    sample_code_min: int = -2147483648
    sample_code_max: int = 2147483647

    @property
    def engineering_units_valid(self) -> bool:
        return (
            self.valid
            and len(self.decimals) == 6
            and len(self.units) == 6
            and all(0 <= decimal <= 10 for decimal in self.decimals)
            and all(0 <= unit <= 0xFFFFFFFF for unit in self.units)
            and self.engineering_unit_contract
            == CONFIRMED_ENGINEERING_UNIT_CONTRACT
        )

    @property
    def sample_validity_confirmed(self) -> bool:
        if self.validity_policy == "sample_codes_equal":
            return (
                len(self.valid_sample_codes) == 6
                and all(
                    -2147483648 <= code <= 2147483647
                    for code in self.valid_sample_codes
                )
            )
        if self.validity_policy == "sample_codes_in_range":
            return (
                -2147483648 <= self.sample_code_min <= self.sample_code_max
                <= 2147483647
                and len(self.valid_sample_codes) == 0
            )
        return False


def calibration_from_values(values: Mapping[str, str]) -> CalibrationSnapshot:
    """Parse one immutable PREOP record without contacting hardware."""
    try:
        decimals = tuple(
            int(values[f"decimal_{index}"]) for index in range(1, 7)
        )
        units = tuple(int(values[f"unit_{index}"]) for index in range(1, 7))
    except (KeyError, TypeError, ValueError):
        return CalibrationSnapshot(False, (), (), "unresolved")
    snapshot_valid = values.get("snapshot_valid", "false")
    valid = (
        isinstance(snapshot_valid, str)
        and snapshot_valid.lower() == "true"
    )
    validity_policy = values.get("validity_policy", "unresolved")
    try:
        sample_codes = (
            tuple(
                int(values[f"valid_sample_code_{index}"])
                for index in range(1, 7)
            )
            if all(
                f"valid_sample_code_{index}" in values
                for index in range(1, 7)
            )
            else ()
        )
    except (TypeError, ValueError):
        sample_codes = ()
    if validity_policy == "sample_codes_in_range":
        try:
            sample_code_min = int(values["sample_code_min"])
            sample_code_max = int(values["sample_code_max"])
        except (KeyError, TypeError, ValueError):
            return CalibrationSnapshot(False, (), (), "unresolved")
    else:
        sample_code_min = -2147483648
        sample_code_max = 2147483647
    return CalibrationSnapshot(
        valid=valid,
        decimals=decimals,
        units=units,
        validity_policy=validity_policy,
        valid_sample_codes=sample_codes,
        engineering_unit_contract=values.get(
            "engineering_unit_contract", "unresolved"
        ),
        sample_code_min=sample_code_min,
        sample_code_max=sample_code_max,
    )
