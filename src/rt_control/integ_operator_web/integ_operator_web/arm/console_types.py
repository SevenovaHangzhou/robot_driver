"""Arm console configuration, errors and state records shared by the console and the page."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable, Mapping, Optional, Protocol, Sequence

from .trajectory import Limits, Plan, TrajectoryPoint

LEASE_TIMEOUT_S = 3.0
STATE_STALE_S = 2.5
MAX_SEQUENCE = 50


class ConsoleError(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class Gear:
    name: str
    label: str
    limits: Limits  # rad units


@dataclass(frozen=True)
class ConsoleConfig:
    gears: tuple[Gear, ...]
    limit_margin: float  # rad kept inside each Robot Model position limit
    max_feedback_age: float  # s
    start_tolerance: float  # rad; must stay below the JTC start-consistency tolerance
    sample_period: float  # s between trajectory points
    stop_lead: float  # s; the stop trajectory joins the running one this far ahead
    start_delay: float  # s; trajectory t = 0 is this far after the send time
    hidden_view_links: tuple[str, ...] = ()  # 3D view only

    def gear(self, name: str) -> Gear:
        for gear in self.gears:
            if gear.name == name:
                return gear
        raise ConsoleError("bad_gear", f"未知档位：{name}")


def config_from_mapping(document: Mapping) -> ConsoleConfig:
    radians = math.radians
    gears = tuple(
        Gear(
            name,
            str(item["label"]),
            Limits(
                radians(float(item["velocity_deg_s"])),
                radians(float(item["acceleration_deg_s2"])),
                radians(float(item["jerk_deg_s3"])),
            ),
        )
        for name, item in document["gears"].items()
    )
    if not gears:
        raise ValueError("at least one gear is required")
    config = ConsoleConfig(
        gears=gears,
        limit_margin=radians(float(document["limit_margin_deg"])),
        max_feedback_age=float(document["max_feedback_age_s"]),
        start_tolerance=radians(float(document["start_tolerance_deg"])),
        sample_period=float(document["sample_period_s"]),
        stop_lead=float(document["stop_lead_s"]),
        start_delay=float(document["start_delay_s"]),
        hidden_view_links=tuple(document.get("hidden_view_links", [])),
    )
    for name in ("sample_period", "stop_lead", "start_delay"):
        if not 0.0 < getattr(config, name) <= 0.1:
            raise ValueError(f"{name}_s must be in (0, 0.1]")
    if config.limit_margin < 0.0 or config.max_feedback_age <= 0.0:
        raise ValueError("limit_margin_deg must be >= 0 and max_feedback_age_s > 0")
    return config


class Driver(Protocol):
    """Implemented by the ROS bridge.

    ``start`` is the monotonic time of trajectory t = 0. The driver stamps the
    trajectory with it, so a stop can continue the running motion on the same
    timeline instead of the controller's receive time.
    """

    def send(
        self,
        joints: Sequence[str],
        points: Sequence[TrajectoryPoint],
        done: Callable[[bool, str], None],
        start: float,
    ) -> None: ...

    def cancel(self) -> None: ...


@dataclass
class _Execution:
    kind: str  # "move" | "stop"
    plan: Plan
    gear: Gear
    started: float  # monotonic time of trajectory t = 0 (the stamp)
    time_offset: float = 0.0  # trajectory time of ``plan`` t = 0

    def plan_time(self, now: float) -> float:
        return max(now - self.started - self.time_offset, 0.0)

    label: str = ""


@dataclass
class Robot:
    """Latest robot-side facts, written by the ROS bridge."""

    positions: dict[str, float] = field(default_factory=dict)
    feedback_at: float = -math.inf
    enable_state: str = "DISCONNECTED"
    enable_stage: str = ""
    enable_at: float = -math.inf
    controller_active: bool = False
    controller_at: float = -math.inf
    mock: Optional[bool] = None
    capability: Optional[dict] = None
