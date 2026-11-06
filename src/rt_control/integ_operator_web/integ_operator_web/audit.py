"""Append-only operator audit log (JSON lines) for run timelines (Runtime-LOG reads it).

One line per command request: wall time, ROS domain, action, sanitised arguments, outcome and a
short random alias per browser session. The token and the session value are never written. Failures to
write never affect the command.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time
import uuid
from typing import Optional

DEFAULT_AUDIT = Path.home() / ".ros/log/integ_operator_web/audit.jsonl"
# Arguments worth keeping per action; anything else in the request body is dropped.
KEPT = {
    "target",
    "gear",
    "name",
    "names",
    "enabled",
    "overwrite",
    "op",
    "modules",
    "slew_deg_s",
    "accel_deg_s2",
    "jerk_deg_s3",
    "alignment_deg",
    "release_deg",
}


class AuditLog:
    def __init__(self, path: Optional[Path]):
        self.path = path
        self.lock = threading.Lock()
        self.domain = os.environ.get("ROS_DOMAIN_ID", "0")
        self._aliases: dict[str, str] = {}
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)

    def record(
        self,
        action: str,
        body: dict,
        *,
        ok: bool,
        error: str = "",
        message: str = "",
        session: Optional[str] = None,
        client: str = "",
    ) -> None:
        if self.path is None:
            return
        entry = {
            "ts": round(time.time(), 6),
            "domain": self.domain,
            "pid": os.getpid(),
            "action": action,
            "args": {key: body[key] for key in KEPT if key in body},
            "ok": ok,
        }
        if error:
            entry["error"] = error
        if message:
            entry["message"] = message[:200]
        if session:
            entry["session"] = self._aliases.setdefault(session, uuid.uuid4().hex[:8])
        if client:
            entry["client"] = client
        line = json.dumps(entry, ensure_ascii=False, separators=(",", ":")) + "\n"
        try:
            with self.lock, self.path.open("a", encoding="utf-8") as stream:
                stream.write(line)
        except OSError:
            pass
