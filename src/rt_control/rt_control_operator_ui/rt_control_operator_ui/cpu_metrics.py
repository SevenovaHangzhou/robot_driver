from __future__ import annotations

from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class CpuCounters:
    total: int
    idle: int


@dataclass(frozen=True)
class CpuUsageDisplay:
    current_percent: float
    average_10s_percent: float
    peak_60s_percent: float
    level: str


def parse_proc_stat(text: str, *, cpu: int = 14) -> CpuCounters:
    prefix = f"cpu{cpu} "
    for raw_line in text.splitlines():
        if not raw_line.startswith(prefix):
            continue
        fields = raw_line.split()
        if len(fields) < 6:
            raise ValueError(f"{prefix.strip()} line has too few counters")
        counters = [int(field) for field in fields[1:]]
        return CpuCounters(total=sum(counters), idle=counters[3] + counters[4])
    raise ValueError(f"{prefix.strip()} is missing from /proc/stat")


def _level(usage_percent: float) -> str:
    if usage_percent > 70.0:
        return "alarm"
    if usage_percent >= 50.0:
        return "attention"
    return "normal"


class CpuUsageHistory:
    def __init__(self, max_samples: int = 60) -> None:
        if max_samples < 1:
            raise ValueError("max_samples must be positive")
        self._previous: CpuCounters | None = None
        self._samples: deque[float] = deque(maxlen=max_samples)

    def ingest(self, counters: CpuCounters) -> CpuUsageDisplay | None:
        previous = self._previous
        self._previous = counters
        if previous is None:
            return None
        total_delta = counters.total - previous.total
        idle_delta = counters.idle - previous.idle
        if total_delta <= 0 or idle_delta < 0 or idle_delta > total_delta:
            self._samples.clear()
            return None
        usage = 100.0 * (total_delta - idle_delta) / total_delta
        self._samples.append(usage)
        last_10 = list(self._samples)[-10:]
        return CpuUsageDisplay(
            current_percent=usage,
            average_10s_percent=sum(last_10) / len(last_10),
            peak_60s_percent=max(self._samples),
            level=_level(usage),
        )
