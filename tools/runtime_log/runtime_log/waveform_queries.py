"""Bounded background queries and disposable, source-versioned waveform caches."""

from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import threading

from mcap.reader import make_reader

from .archive import write_json
from .waveform import waveform
from .chassis_waveform import CHANNELS, for_source


class WaveformQueries:
    def __init__(self):
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="waveform-query")
        self.lock = threading.Lock()
        self.jobs = OrderedDict()
        self.durations = OrderedDict()

    def _range(self, source, stat, start, end):
        start = float(start)
        end = float(end) if end is not None else None
        if (
            not math.isfinite(start)
            or start < 0
            or (end is not None and (not math.isfinite(end) or end <= start))
        ):
            raise ValueError("时间范围无效")
        identity = (str(source.resolve()), stat.st_size, stat.st_mtime_ns)
        with self.lock:
            duration = self.durations.get(identity)
        if duration is None:
            with source.open("rb") as stream:
                summary = make_reader(stream).get_summary()
                stats = summary.statistics if summary else None
                if stats is None:
                    raise ValueError("归档缺少索引，请下载 CSV 查看")
                duration = (stats.message_end_time - stats.message_start_time) / 1e9
            with self.lock:
                self.durations[identity] = duration
                while len(self.durations) > 64:
                    self.durations.popitem(last=False)
        if start > duration:
            raise ValueError("时间范围超出实验长度")
        # Identical full ranges share one task/cache regardless of numeric spelling
        # or whether the client omitted the endpoint or supplied the full duration.
        return start, None if end is None or end >= duration else end

    def request(self, source, joint, start, end, width):
        stat = source.stat()
        start, end = self._range(source, stat, start, end)
        identity = [
            str(source.resolve()),
            stat.st_size,
            stat.st_mtime_ns,
            joint,
            start,
            end,
            max(100, min(2000, width)),
            3,
        ]
        if joint in CHANNELS:
            chassis = for_source(source)
            identity.append(chassis.p if chassis else None)
        key = hashlib.sha256(json.dumps(identity).encode()).hexdigest()
        cache = source.parent / ".waveform-cache" / (key + ".json")
        try:
            return 200, json.loads(cache.read_text())
        except (OSError, ValueError):
            pass
        with self.lock:
            future = self.jobs.get(key)
            if future is not None:
                self.jobs.move_to_end(key)
                if future.done():
                    try:
                        return 200, future.result()
                    except Exception:
                        del self.jobs[key]
                        raise
            else:
                if sum(not job.done() for job in self.jobs.values()) >= 8:
                    return 429, {"status": "busy", "message": "波形任务较多，正在等待空闲"}
                future = self.pool.submit(self._build, cache, source, joint, start, end, width)
                self.jobs[key] = future
                for old in list(self.jobs):
                    if len(self.jobs) <= 32:
                        break
                    if self.jobs[old].done():
                        del self.jobs[old]
            return 202, {"status": "processing", "message": "正在后台读取原始采样，完成后自动显示"}

    @staticmethod
    def _build(cache, source, joint, start, end, width):
        result = waveform(source, joint, start, end, width)
        try:
            write_json(cache, result)
        except OSError:
            pass  # A read-only archive can still serve the in-memory result.
        return result
