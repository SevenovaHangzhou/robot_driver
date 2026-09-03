from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class BatteryDisplay:
    state: str
    percent_text: str
    voltage_text: str
    fraction: float | None
    tooltip: str


def project_battery(
    present: bool,
    percentage: float,
    voltage: float,
    age_s: float,
    *,
    stale_after_s: float = 6.0,
) -> BatteryDisplay:
    voltage_text = f"{voltage:.1f} V" if math.isfinite(voltage) else "-- V"
    if not present:
        return BatteryDisplay("offline", "离线", voltage_text, None, "BMS 离线")
    if not math.isfinite(age_s) or age_s < 0.0 or age_s > stale_after_s:
        return BatteryDisplay(
            "stale", "数据过期", voltage_text, None, f"BMS 数据超过 {stale_after_s:g} 秒未更新"
        )
    if not math.isfinite(percentage) or not 0.0 <= percentage <= 1.0:
        return BatteryDisplay(
            "invalid", "--%", voltage_text, None, f"电量数据无效 · {voltage_text}"
        )
    percent = int(round(percentage * 100.0))
    return BatteryDisplay(
        "ok",
        f"{percent}%",
        voltage_text,
        percentage,
        f"电量 {percent}% · 电压 {voltage_text} · 数据年龄 {age_s:.1f} 秒",
    )
