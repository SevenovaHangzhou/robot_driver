"""Pure conversion and frame validation for the X503B shadow bridge.

The module deliberately has no EtherCAT access.  It consumes one complete
DynamicJointState frame produced by rt-control and only emits a WrenchStamped
when an independently read-only calibration snapshot has been validated.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


RAW_CHANNELS = tuple(f"channel_{index}_raw" for index in range(1, 7))
SAMPLE_CHANNELS = tuple(f"sample_code_{index}_raw" for index in range(1, 7))
CONFIRMED_ENGINEERING_UNIT_CONTRACT = "force_N_torque_Nm"


@dataclass(frozen=True)
class SensorConfig:
    sensor_name: str
    wrench_topic: str
    raw_topic: str
    frame_id: str


@dataclass(frozen=True)
class CalibrationSnapshot:
    valid: bool
    decimals: tuple[int, ...]
    units: tuple[int, ...]
    validity_policy: str
    valid_sample_codes: tuple[int, ...] = ()
    engineering_unit_contract: str = "unresolved"

    @property
    def engineering_units_valid(self) -> bool:
        return (
            self.valid
            and len(self.decimals) == 6
            and len(self.units) == 6
            and all(0 <= decimal <= 9 for decimal in self.decimals)
            and all(0 <= unit <= 0xFFFFFFFF for unit in self.units)
            and self.engineering_unit_contract
            == CONFIRMED_ENGINEERING_UNIT_CONTRACT
        )

    @property
    def sample_validity_confirmed(self) -> bool:
        return (
            self.validity_policy == "sample_codes_equal"
            and len(self.valid_sample_codes) == 6
            and all(
                -2147483648 <= code <= 2147483647
                for code in self.valid_sample_codes
            )
        )


@dataclass(frozen=True)
class SensorFrame:
    stamp: Any
    raw_values: tuple[int, ...]
    sample_codes: tuple[int, ...]


def _exact_int(value: Any) -> int | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        numeric = float(value)
    except OverflowError:
        return None
    if not math.isfinite(numeric) or int(numeric) != value:
        return None
    integer = int(value)
    if integer < -2147483648 or integer > 2147483647:
        return None
    return integer


def extract_sensor_frame(message: Any, sensor_name: str) -> SensorFrame | None:
    """Extract all twelve channels from one DynamicJointState frame.

    Returning ``None`` is intentional fail-closed behavior for malformed,
    partial, duplicated, or non-integral input.
    """
    if len(message.joint_names) != len(message.interface_values):
        return None
    matches = [
        (name, values)
        for name, values in zip(message.joint_names, message.interface_values)
        if name == sensor_name
    ]
    if len(matches) != 1:
        return None
    _, values = matches[0]
    if len(values.interface_names) != len(values.values):
        return None
    indexed: dict[str, Any] = {}
    for name, value in zip(values.interface_names, values.values):
        if not name or name in indexed:
            return None
        indexed[name] = value
    raw = tuple(_exact_int(indexed.get(name)) for name in RAW_CHANNELS)
    samples = tuple(_exact_int(indexed.get(name)) for name in SAMPLE_CHANNELS)
    if any(value is None for value in raw + samples):
        return None
    stamp = message.header.stamp
    if stamp.sec == 0 and stamp.nanosec == 0:
        return None
    return SensorFrame(stamp, tuple(raw), tuple(samples))


def convert_to_wrench(
    frame: SensorFrame, calibration: CalibrationSnapshot
) -> tuple[float, float, float, float, float, float] | None:
    """Convert raw calibrated DINT values only after strict metadata checks."""
    if (
        not calibration.engineering_units_valid
        or not calibration.sample_validity_confirmed
        or frame.sample_codes != calibration.valid_sample_codes
    ):
        return None
    scale = tuple(10.0 ** (-decimal) for decimal in calibration.decimals)
    values = tuple(
        raw * factor for raw, factor in zip(frame.raw_values, scale)
    )
    if not all(math.isfinite(value) for value in values):
        return None
    return values  # Fx,Fy,Fz,Mx,My,Mz


def calibration_from_values(
    values: Mapping[str, str],
) -> CalibrationSnapshot:
    """Parse the private diagnostic snapshot from the read-only reader."""
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
    try:
        sample_codes = (
            tuple(
                int(values[f"valid_sample_code_{index}"])
                for index in range(1, 7)
            )
            if all(
                f"valid_sample_code_{index}" in values for index in range(1, 7)
            )
            else ()
        )
    except (TypeError, ValueError):
        sample_codes = ()
    return CalibrationSnapshot(
        valid=valid,
        decimals=decimals,
        units=units,
        validity_policy=values.get("validity_policy", "unresolved"),
        valid_sample_codes=sample_codes,
        engineering_unit_contract=values.get(
            "engineering_unit_contract", "unresolved"
        ),
    )
