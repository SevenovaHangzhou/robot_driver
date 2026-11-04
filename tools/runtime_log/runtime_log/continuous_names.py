"""Cached absolute odometry totals for completed continuous-recording names."""

from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import logging
import math
import threading

from mcap.reader import make_reader

from .archive import write_json
from .chassis_waveform import ChassisDisplacement, DISPLACEMENT_CHANNEL, for_source

LOG = logging.getLogger(__name__)


class MovementTotals:
    def __init__(self, feedback):
        self.odometry = ChassisDisplacement(feedback, DISPLACEMENT_CHANNEL)
        self.last_seq = None
        self.previous = None
        self.totals = [0.0, 0.0, 0.0]
        self.valid_intervals = 0
        self.excluded_samples = 0

    def reset_baseline(self):
        self.odometry.previous = [None] * 4
        self.previous = None

    def add(self, row):
        seq = row.get("seq")
        if self.last_seq is not None and seq != self.last_seq + 1:
            self.reset_baseline()
            self.excluded_samples += 1
        self.last_seq = seq
        # Invalid PDO feedback must not be integrated as a stale held position.
        for drive, steer, *_ in self.odometry.feedback.modules:
            for joint in (drive, steer):
                age = row.get(joint + "/feedback_age_ms")
                if age is not None and (
                    not math.isfinite(age) or age < 0 or age > self.odometry.feedback.timeout * 1000
                ):
                    row[joint + "/status_word"] = 0
        measured = self.odometry.sample(row)
        if measured is None:
            self.reset_baseline()
            self.excluded_samples += 1
            return
        current = measured[:3]
        if self.previous is not None:
            for i in range(3):
                self.totals[i] += abs(current[i] - self.previous[i])
            self.valid_intervals += 1
        self.previous = current

    def result(self):
        return {
            "x_m": self.totals[0] if self.valid_intervals else None,
            "y_m": self.totals[1] if self.valid_intervals else None,
            "yaw_deg": math.degrees(self.totals[2]) if self.valid_intervals else None,
            "valid_intervals": self.valid_intervals,
            "excluded_samples": self.excluded_samples,
            "basis": "motor-steering wheel odometry; segment-start coordinates; sum of absolute increments",
        }


def display_name(meta, totals=None):
    if meta.get("source") != "auto:continuous":
        return meta.get("name") or ""
    if totals is None:
        return "持续记录｜累计位移统计中"
    if not totals.get("valid_intervals"):
        return "持续记录｜累计 |Δx| — m · |Δy| — m · |Δyaw| —°（无有效位移统计）"
    return (
        f"持续记录｜累计 |Δx| {totals['x_m']:.3f} m · "
        f"|Δy| {totals['y_m']:.3f} m · |Δyaw| {totals['yaw_deg']:.2f}°"
    )


class ContinuousNames:
    def __init__(self):
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="movement-summary")
        self.lock = threading.Lock()
        self.jobs = OrderedDict()

    def name(self, meta, source):
        if meta.get("source") != "auto:continuous":
            return display_name(meta)
        try:
            stat = source.stat()
            feedback = for_source(source)
            if feedback is None:
                return display_name(meta, {})
            identity = [stat.st_size, stat.st_mtime_ns, feedback.p, 1]
            key = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
            cache = source.parent / ".movement-summary" / (key + ".json")
            try:
                return display_name(meta, json.loads(cache.read_text()))
            except (OSError, ValueError):
                pass
            with self.lock:
                job_key = (str(source.resolve()), key)
                future = self.jobs.get(job_key)
                if future is not None and future.done():
                    return display_name(meta, future.result())
                if future is None and sum(not job.done() for job in self.jobs.values()) < 8:
                    self.jobs[job_key] = self.pool.submit(self._build, source, feedback, cache)
                    for old in list(self.jobs):
                        if len(self.jobs) <= 32:
                            break
                        if self.jobs[old].done():
                            del self.jobs[old]
            return display_name(meta)
        except OSError:
            return display_name(meta, {})

    @staticmethod
    def _build(source, feedback, cache):
        try:
            totals = MovementTotals(feedback)
            with source.open("rb") as stream:
                reader = make_reader(stream)
                summary = reader.get_summary()
                fields = set()
                if summary:
                    for schema in summary.schemas.values():
                        fields.update(json.loads(schema.data).get("properties", {}))
                if not feedback.fields_available(fields, DISPLACEMENT_CHANNEL):
                    return {}
                for _, _, message in reader.iter_messages():
                    totals.add(json.loads(message.data))
            result = totals.result()
        except Exception:
            LOG.exception("continuous movement summary failed: %s", source)
            return {}  # Failed statistics must not break the report or produce fabricated zeroes.
        try:
            write_json(cache, result)
        except OSError:
            LOG.warning("movement summary cache could not be saved: %s", cache)
        return result


NAMES = ContinuousNames()
