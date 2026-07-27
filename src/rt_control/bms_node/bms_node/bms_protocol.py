from __future__ import annotations

import math
import struct
from dataclasses import dataclass


BASIC_STATUS_FRAME_ID = 0x3FC


@dataclass
class BmsSample:
    voltage_v: float | None = None
    soc_fraction: float | None = None
    last_frame_s: float | None = None

    def is_fresh(self, now_s: float, timeout_s: float) -> bool:
        if self.last_frame_s is None:
            return False
        values = (float(self.last_frame_s), float(now_s), float(timeout_s))
        if not all(math.isfinite(value) for value in values):
            return False
        age_s = now_s - self.last_frame_s
        return timeout_s > 0.0 and 0.0 <= age_s <= timeout_s


def ingest_basic_frame(sample: BmsSample, can_id: int, data: bytes, now_s: float) -> bool:
    if can_id != BASIC_STATUS_FRAME_ID or len(data) < 5 or not math.isfinite(now_s):
        return False
    soc_percent = int(data[4])
    if soc_percent > 100:
        return False

    sample.voltage_v = struct.unpack(">H", data[0:2])[0] * 0.1
    sample.soc_fraction = soc_percent / 100.0
    sample.last_frame_s = now_s
    return True


def project_battery_values(
    sample: BmsSample,
    now_s: float,
    timeout_s: float,
) -> tuple[float, float]:
    if not sample.is_fresh(now_s, timeout_s):
        return math.nan, math.nan
    if sample.voltage_v is None or sample.soc_fraction is None:
        return math.nan, math.nan
    if not math.isfinite(sample.voltage_v) or not math.isfinite(sample.soc_fraction):
        return math.nan, math.nan
    if not 0.0 <= sample.soc_fraction <= 1.0:
        return math.nan, math.nan
    return float(sample.voltage_v), float(sample.soc_fraction)
