"""Read-only, sequence-matched evidence from the Rolling non-RT callback."""

import math
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class RollingRejections:
    def __init__(self, node, controller, group):
        self.name = "/" + controller.strip("/") + "/rejection"
        self.lock = threading.Lock()
        self.detail, self.received_at = None, -math.inf
        self.subscription = node.create_subscription(
            DiagnosticArray,
            self.name,
            self.receive,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
            callback_group=group,
        )

    def receive(self, message):
        for report in message.status:
            if report.name != self.name:
                continue
            values = {item.key: item.value for item in report.values}
            try:
                detail = {key: int(values[key]) for key in ("epoch", "sequence", "result")}
                if any(v < 0 for v in detail.values()) or detail["result"] == 0:
                    continue
                if "reason" in values:
                    reason = values["reason"]
                    if reason not in (
                        "snapshot_stale",
                        "state_not_running",
                        "epoch_changed",
                        "invalid_desired",
                        "time_overflow",
                        "snapshot_publish_failed",
                        "feedback_stale",
                        "takeover_mismatch",
                        "sequence_overflow",
                        "none",
                    ):
                        continue
                    detail["reason"] = reason
                if "quantity" in values:
                    detail.update(
                        {key: int(values[key]) for key in ("axis", "segment", "start_ns", "end_ns")}
                    )
                    detail.update({key: float(values[key]) for key in ("value", "limit")})
                    detail.update(joint=values["joint"], quantity=values["quantity"])
                    if (
                        not 0 <= detail["axis"] < 14
                        or detail["segment"] < 0
                        or not 0 <= detail["start_ns"] < detail["end_ns"]
                        or not math.isfinite(detail["value"])
                        or not math.isfinite(detail["limit"])
                        or len(detail["joint"]) > 128
                        or detail["quantity"]
                        not in (
                            "position_lower",
                            "position_upper",
                            "velocity_positive",
                            "velocity_negative",
                            "acceleration_positive",
                            "acceleration_negative",
                            "stopping_lower",
                            "stopping_upper",
                        )
                    ):
                        continue
            except (KeyError, ValueError, OverflowError):
                continue
            with self.lock:
                self.detail, self.received_at = detail, time.monotonic()

    def matching(self, controller, names):
        if controller is None:
            return None
        with self.lock:
            detail = self.detail
            if detail is None or time.monotonic() - self.received_at > 2.0:
                return None
            if (detail["epoch"], detail["sequence"], detail["result"]) != (
                controller["epoch"],
                controller["last_sequence"],
                controller["last_result"],
            ):
                return None
            if "axis" in detail and (
                detail["axis"] >= len(names) or names[detail["axis"]] != detail["joint"]
            ):
                return None
            return {"source": "controller_admission", **detail}
