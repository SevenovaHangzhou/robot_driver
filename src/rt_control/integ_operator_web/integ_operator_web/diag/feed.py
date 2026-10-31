"""Read-only ROS inputs of the diagnostics page: R-OUT-09 readiness and raw /diagnostics rows."""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
import robot_interfaces_qos
from robot_system_interfaces.msg import DomainReadiness, ErrorInfo

READINESS_TOPIC = "/rt_control/readiness"
SEVERITY = {ErrorInfo.OK: "OK", ErrorInfo.WARN: "WARN", ErrorInfo.FAULT: "FAULT"}
LEVEL = {b"\x00": "OK", b"\x01": "WARN", b"\x02": "ERROR", b"\x03": "STALE"}


def readiness_document(message: DomainReadiness) -> dict:
    return {
        "readiness_name": message.readiness_name,
        "ready": message.ready,
        "status": message.status,
        "operational_state": message.operational_state,
        "blockers": list(message.blockers),
        "producer_instance_id": message.producer_instance_id,
        "errors": [
            {
                "code": error.code,
                "name": error.message,
                "severity": SEVERITY.get(error.severity, str(error.severity)),
                "retryable": error.retryable,
                "source": error.source,
                "detail": error.detail,
            }
            for error in message.errors
        ],
    }


def diagnostic_rows(message: DiagnosticArray) -> list[dict]:
    return [
        {
            "name": status.name,
            "level": LEVEL.get(
                status.level if isinstance(status.level, bytes) else bytes([status.level]),
                str(status.level),
            ),
            "message": status.message,
            "values": {item.key: item.value for item in status.values},
        }
        for status in message.status
    ]


class DiagnosticsFeed:
    """Latest readiness and the latest row per /diagnostics name, with receive times."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._readiness: Optional[dict] = None
        self._readiness_at = 0.0
        self._rows: dict[str, dict] = {}

    def on_readiness(self, message: DomainReadiness) -> None:
        with self._lock:
            self._readiness, self._readiness_at = readiness_document(message), self._clock()

    def on_diagnostics(self, message: DiagnosticArray) -> None:
        now = self._clock()
        with self._lock:
            for row in diagnostic_rows(message):
                self._rows[row["name"]] = {**row, "received_at": now}

    def snapshot(self) -> dict:
        now = self._clock()
        with self._lock:
            readiness = None
            if self._readiness is not None:
                readiness = {**self._readiness, "age_s": round(now - self._readiness_at, 1)}
            rows = [
                {
                    **{k: v for k, v in row.items() if k != "received_at"},
                    "age_s": round(now - row["received_at"], 1),
                }
                for _, row in sorted(self._rows.items())
            ]
        return {"readiness": readiness, "diagnostics": rows}


class FeedNode(Node):
    def __init__(self, feed: DiagnosticsFeed) -> None:
        super().__init__("integ_operator_web_diag")
        self.create_subscription(
            DomainReadiness, READINESS_TOPIC, feed.on_readiness, robot_interfaces_qos.latched()
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", feed.on_diagnostics, robot_interfaces_qos.diagnostic()
        )
