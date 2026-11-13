"""Read-only inputs of the status page: joints, batteries, chassis, CPU and diagnostic alarms.

The page shows what the stack publishes; it never commands anything. Every value carries the
time it was received so the page can tell "missing" from "stale" instead of guessing health.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import threading
import time
from typing import Callable, Optional

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import robot_interfaces_qos
from robot_system_interfaces.msg import DomainReadiness
from sensor_msgs.msg import BatteryState, JointState, Range

READINESS_TOPIC = "/rt/readiness"
DIAGNOSTICS_TOPIC = "/diagnostics"
ODOM_TOPIC = "/wheel/odom"
JOINT_STATE_TOPIC = "/joint_states"
BATTERY_TOPICS = ("/rt/battery/left/state", "/rt/battery/right/state")
SONAR_TOPICS = tuple(f"/rt/ultrasonic/channel{index}/range" for index in range(1, 9))

STALE_S = 3.0  # a topic silent for longer than this is shown as missing, not as its last value
# Joint names come from the deployment (node parameters), never from this module: the machine
# configuration owns them.
ROLE_KEYS = ("updown", "head_yaw", "head_pitch", "steer1", "steer2", "steer3", "steer4")
LEVEL = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


@dataclass
class Sample:
    value: object
    at: float


@dataclass
class Feed:
    """Latest message per topic with its receive time; one lock, no ROS in the readers."""

    clock: Callable[[], float] = time.monotonic
    lock: threading.Lock = field(default_factory=threading.Lock)
    joints: Optional[Sample] = None
    batteries: list = field(default_factory=lambda: [None, None])
    odometry: Optional[Sample] = None
    sonar: list = field(default_factory=lambda: [None] * len(SONAR_TOPICS))
    diagnostics: Optional[Sample] = None
    readiness: Optional[Sample] = None

    def set(self, name: str, value: object, index: Optional[int] = None) -> None:
        sample = Sample(value, self.clock())
        with self.lock:
            if index is None:
                setattr(self, name, sample)
            else:
                getattr(self, name)[index] = sample

    def read(self) -> dict:
        with self.lock:
            return {
                "joints": self.joints,
                "batteries": list(self.batteries),
                "odometry": self.odometry,
                "sonar": list(self.sonar),
                "diagnostics": self.diagnostics,
                "readiness": self.readiness,
            }


def _age(sample: Optional[Sample], now: float) -> Optional[float]:
    return None if sample is None else round(now - sample.at, 2)


def _fresh(sample: Optional[Sample], now: float) -> bool:
    return sample is not None and now - sample.at <= STALE_S


def _levels(message: DiagnosticArray) -> list[dict]:
    rows = []
    for status in message.status:
        level = status.level if isinstance(status.level, int) else status.level[0]
        rows.append(
            {
                "name": status.name,
                "level": LEVEL.get(level, str(level)),
                "message": status.message,
                "values": {item.key: item.value for item in status.values},
            }
        )
    return rows


def alarms(rows: list[dict], readiness: Optional[dict]) -> list[dict]:
    """Diagnostic rows and readiness blockers as one list; wording stays as the producer wrote it."""
    entries = []
    for row in rows:
        if row["level"] not in ("WARN", "ERROR", "STALE"):
            continue
        entries.append(
            {
                "level": {"ERROR": "fault", "WARN": "warn", "STALE": "stale"}[row["level"]],
                "code": row["name"],
                "title": row["message"] or row["name"],
                "part": row["values"].get("hardware_id", ""),
                "values": row["values"],
            }
        )
    for error in (readiness or {}).get("errors", []):
        entries.append(
            {
                "level": "fault" if error.get("severity") == "FAULT" else "warn",
                "code": error.get("code", ""),
                "title": error.get("name", ""),
                "part": error.get("source", ""),
                "values": {"detail": error.get("detail", ""), "retryable": str(error.get("retryable"))},
            }
        )
    # Blockers are tokens, not codes; when readiness already carries errors they describe the same
    # failure twice, so they are only shown when no error explains them.
    if not (readiness or {}).get("errors"):
        for blocker in (readiness or {}).get("blockers", []):
            entries.append(
                {"level": "fault", "code": blocker, "title": blocker, "part": "readiness", "values": {}}
            )
    order = {"fault": 0, "warn": 1, "stale": 2}
    return sorted(entries, key=lambda entry: order[entry["level"]])


def cpu() -> dict:
    """Total and per-core utilisation from /proc/stat, plus isolated cores as the real-time set."""
    cores, total = _proc_stat()
    isolated = _isolated_cores()
    return {
        "usage_pct": total,
        "cores": cores,
        "isolated": isolated,
        "temperature_c": _cpu_temperature(),
        "count": len(cores),
    }


def _proc_stat() -> tuple[list[dict], Optional[float]]:
    rows = []
    total_used = total_all = 0
    try:
        for line in Path("/proc/stat").read_text(encoding="utf-8").splitlines():
            if not line.startswith("cpu"):
                continue
            fields = line.split()
            values = [int(value) for value in fields[1:9]]
            idle = values[3] + values[4]
            busy = sum(values) - idle
            if fields[0] == "cpu":
                total_used, total_all = busy, sum(values)
            else:
                rows.append({"id": int(fields[0][3:]), "busy": busy, "total": sum(values)})
    except (OSError, ValueError):
        return [], None
    return rows, (None if not total_all else round(100 * total_used / total_all, 1))


def _isolated_cores() -> list[int]:
    try:
        text = Path("/sys/devices/system/cpu/isolated").read_text(encoding="utf-8").strip()
    except OSError:
        return []
    cores: list[int] = []
    for part in filter(None, text.split(",")):
        if "-" in part:
            first, last = part.split("-", 1)
            cores.extend(range(int(first), int(last) + 1))
        else:
            cores.append(int(part))
    return sorted(cores)


def _cpu_temperature() -> Optional[float]:
    """Hottest readable thermal zone, °C. Absent on hosts without readable zones."""
    hottest = None
    for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        try:
            value = int((zone / "temp").read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            continue
        celsius = value / 1000 if value > 1000 else float(value)
        hottest = celsius if hottest is None else max(hottest, celsius)
    return None if hottest is None else round(hottest, 1)


def _number(value: float) -> Optional[float]:
    # BatteryState and Range use NaN for "no value"; the page shows "—" instead.
    return None if value != value else round(float(value), 3)


class FeedNode(Node):
    """Subscriptions of the status page. Public topics only, no service calls, no writes."""

    def __init__(self, feed: Feed):
        super().__init__("integ_operator_web_status")
        self._feed = feed
        self.create_subscription(JointState, JOINT_STATE_TOPIC, self._joints, robot_interfaces_qos.state())
        for index, topic in enumerate(BATTERY_TOPICS):
            self.create_subscription(
                BatteryState, topic, lambda message, position=index: self._battery(position, message),
                robot_interfaces_qos.state(),
            )
        self.create_subscription(Odometry, ODOM_TOPIC, self._odometry, robot_interfaces_qos.state())
        for index, topic in enumerate(SONAR_TOPICS):
            self.create_subscription(
                Range, topic, lambda message, position=index: self._sonar(position, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            DiagnosticArray, DIAGNOSTICS_TOPIC, self._diagnostics, robot_interfaces_qos.diagnostic()
        )
        self.create_subscription(
            DomainReadiness, READINESS_TOPIC, self._readiness, robot_interfaces_qos.state()
        )

    def _joints(self, message: JointState) -> None:
        self._feed.set("joints", {name: position for name, position in zip(message.name, message.position)})

    def _battery(self, index: int, message: BatteryState) -> None:
        self._feed.set(
            "batteries",
            {
                "present": bool(message.present),
                "soc_pct": None if not message.present else round(message.percentage * 100, 1),
                "current_a": _number(message.current),
                "voltage_v": _number(message.voltage),
                "temperature_c": _number(message.temperature),
                "health": message.power_supply_health,
            },
            index,
        )

    def _odometry(self, message: Odometry) -> None:
        twist = message.twist.twist
        self._feed.set(
            "odometry", {"vx": twist.linear.x, "vy": twist.linear.y, "wz": twist.angular.z}
        )

    def _sonar(self, index: int, message: Range) -> None:
        self._feed.set("sonar", _number(message.range), index)

    def _diagnostics(self, message: DiagnosticArray) -> None:
        self._feed.set("diagnostics", _levels(message))

    def _readiness(self, message: DomainReadiness) -> None:
        self._feed.set(
            "readiness",
            {
                "ready": bool(message.ready),
                "status": message.status,
                "operational_state": message.operational_state,
                "blockers": list(message.blockers),
                "errors": [
                    {
                        "code": error.code,
                        "name": error.message,
                        "severity": {0: "OK", 1: "WARN", 2: "FAULT"}.get(error.severity, str(error.severity)),
                        "source": error.source,
                        "detail": error.detail,
                        "retryable": error.retryable,
                    }
                    for error in message.errors
                ],
            },
        )


def document(
    feed: Feed,
    *,
    state_period_s: float,
    roles: Optional[dict] = None,
    now: Optional[float] = None,
) -> dict:
    """The snapshot the page renders. `None` means "no value"; nothing is inferred from it."""
    snapshot = feed.read()
    now = time.monotonic() if now is None else now
    joints_sample = snapshot["joints"]
    joints_fresh = _fresh(joints_sample, now)
    joints = (joints_sample.value if joints_fresh else {}) or {}

    batteries = []
    for sample in snapshot["batteries"]:
        fresh = _fresh(sample, now)
        value = (sample.value if fresh else None) or {}
        batteries.append(
            {
                "present": bool(value.get("present")),
                "soc_pct": value.get("soc_pct"),
                "current_a": value.get("current_a"),
                "voltage_v": value.get("voltage_v"),
                "temperature_c": value.get("temperature_c"),
                "health": value.get("health"),
                "age_s": _age(sample, now),
            }
        )

    odometry = snapshot["odometry"] if _fresh(snapshot["odometry"], now) else None
    speed = odometry.value if odometry else {}
    sonar = [
        sample.value if _fresh(sample, now) else None for sample in snapshot["sonar"]
    ]

    readiness_sample = snapshot["readiness"]
    readiness = readiness_sample.value if _fresh(readiness_sample, now) else None
    rows = snapshot["diagnostics"].value if _fresh(snapshot["diagnostics"], now) else []
    entries = alarms(rows or [], readiness)

    if not joints_fresh:
        state = "offline"
    elif any(entry["level"] == "fault" for entry in entries) or (readiness and not readiness["ready"]):
        state = "fault"
    elif entries:
        state = "warn"
    else:
        state = "ok"

    roles = {key: name for key, name in (roles or {}).items() if name}
    return {
        "state": state,
        "joints": {name: joints[name] for name in joints if _finite(joints[name])},
        "pose": {
            "updown_m": _role_value(joints, roles.get("updown")),
            "head_pitch_deg": _degrees(joints, roles.get("head_pitch")),
            "head_yaw_deg": _degrees(joints, roles.get("head_yaw")),
        },
        "batteries": batteries,
        "chassis": {
            "steer_deg": [_degrees(joints, roles.get(f"steer{index}")) for index in range(1, 5)],
            "vx": speed.get("vx"),
            "vy": speed.get("vy"),
            "wz": speed.get("wz"),
            "sonar": sonar,
            "odometry_age_s": _age(snapshot["odometry"], now),
        },
        "cpu": cpu(),
        "readiness": readiness,
        "alerts": entries,
        "ages": {
            "joints_s": _age(joints_sample, now),
            "state_period_s": state_period_s,
        },
    }


def _finite(value: Optional[float]) -> bool:
    return value is not None and value == value


def _role_value(joints: dict, name: Optional[str]) -> Optional[float]:
    value = joints.get(name) if name else None
    return round(value, 4) if _finite(value) else None


def _degrees(joints: dict, name: Optional[str]) -> Optional[float]:
    value = joints.get(name) if name else None
    return round(value * 57.29577951308232, 1) if _finite(value) else None


__all__ = ["Feed", "FeedNode", "ROLE_KEYS", "SONAR_TOPICS", "alarms", "cpu", "document"]
