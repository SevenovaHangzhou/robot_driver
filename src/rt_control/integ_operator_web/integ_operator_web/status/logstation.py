"""Log-station client of the status page.

The station publishes ``/anomalies.json`` with the cycle statistics of the running stack and the
saved black-box windows. The statistics are cumulative since the run started and are refreshed with
``run.json`` (about every 5 s), so the page shows them with their age instead of treating them as
live values.
"""

from __future__ import annotations

import asyncio
import time
from typing import Optional

import aiohttp

POLL_PERIOD_S = 5.0
REQUEST_TIMEOUT_S = 3.0
STALE_AFTER_S = 15.0  # two missed refreshes: the numbers are no longer current


def normalise(payload: dict, base: str, now: Optional[float] = None) -> dict:
    """The first run of the station's answer, with absolute links and an age for the statistics."""
    now = time.time() if now is None else now
    runs = payload.get("runs") or []
    if not runs:
        return {"available": True, "run": None, "cycle": None, "records": [], "fetched_at": now}
    run = runs[0]
    cycle = run.get("cycle") or None
    if cycle is not None:
        age = run.get("update_age_s")
        cycle = dict(cycle, age_s=age, stale=age is None or age > STALE_AFTER_S)
    records = [
        {
            "id": window.get("id"),
            "time": window.get("time"),
            "reasons": window.get("reasons") or [],
            "subjects": window.get("subjects") or [],
            "url": base + (window.get("url") or ""),
        }
        for window in run.get("blackbox") or []
    ]
    return {
        "available": True,
        "run": {
            "id": run.get("id"),
            "running": bool(run.get("running")),
            "update_age_s": run.get("update_age_s"),
            "report_url": base + (run.get("report_url") or ""),
        },
        "cycle": cycle,
        "records": records,
        "fetched_at": now,
    }


def evidence(alerts: list[dict], log: dict) -> list[dict]:
    """Attach a log-station target to each alarm and append the recordings nothing explains any more.

    A black-box window belongs to an alarm when one of the subjects it was raised on (a joint, a
    slave, a counter) appears in the alarm's code or values; otherwise the run report is offered.
    """
    run_url = (log.get("run") or {}).get("report_url")
    records = log.get("records") or []
    used = set()
    merged = []
    for alert in alerts:
        text = " ".join(
            [str(alert.get("code", "")), str(alert.get("title", ""))]
            + [str(value) for value in (alert.get("values") or {}).values()]
        )
        match = next(
            (
                record
                for record in records
                if record["id"] not in used
                and any(subject in text for subject in record["subjects"])
            ),
            None,
        )
        entry = dict(alert)
        if match is not None:
            used.add(match["id"])
            entry["evidence"] = {"kind": "waveform", "url": match["url"], "at": match["time"]}
        elif run_url:
            entry["evidence"] = {"kind": "run", "url": run_url, "at": None}
        else:
            entry["evidence"] = {"kind": "none", "url": None, "at": None}
        merged.append(entry)
    for record in records:
        if record["id"] in used:
            continue
        merged.append(
            {
                "level": "recovered",
                "code": ", ".join(record["reasons"]) or record["id"],
                "title": ", ".join(record["reasons"]) or record["id"],
                "part": ", ".join(record["subjects"]),
                "values": {},
                "evidence": {"kind": "waveform", "url": record["url"], "at": record["time"]},
            }
        )
    return merged


class LogStation:
    """Polls one station; a failed poll keeps the previous answer and marks it stale."""

    def __init__(
        self, base: str, period: float = POLL_PERIOD_S, timeout: float = REQUEST_TIMEOUT_S
    ):
        self.base = base.rstrip("/")
        self.period = period
        self.timeout = timeout
        self._last: Optional[dict] = None
        self._error: Optional[str] = None
        self._task: Optional[asyncio.Task] = None

    async def poll_once(self, session: aiohttp.ClientSession) -> dict:
        if not self.base:
            return {
                "available": False,
                "run": None,
                "cycle": None,
                "records": [],
                "error": "not_configured",
            }
        try:
            async with session.get(f"{self.base}/anomalies.json", timeout=self.timeout) as response:
                response.raise_for_status()
                payload = await response.json()
        except (aiohttp.ClientError, asyncio.TimeoutError, ValueError) as error:
            self._error = f"{type(error).__name__}: {error}"
            return dict(
                self._last or {"available": False, "run": None, "cycle": None, "records": []},
                error=self._error,
            )
        self._error = None
        self._last = normalise(payload, self.base)
        return self._last

    def snapshot(self) -> dict:
        if self._last is None:
            return {
                "available": False,
                "run": None,
                "cycle": None,
                "records": [],
                "error": self._error or "pending",
            }
        return dict(self._last, error=self._error)

    async def _loop(self) -> None:
        async with aiohttp.ClientSession() as session:
            while True:
                await self.poll_once(session)
                await asyncio.sleep(self.period)

    def start(self) -> None:
        """Start polling once a loop is running; calling it again is harmless."""
        if not self.base or self._task is not None:
            return
        self._task = asyncio.ensure_future(self._loop())

    def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None


__all__ = ["LogStation", "evidence", "normalise"]
