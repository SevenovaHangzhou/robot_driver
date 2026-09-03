from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FaultEvent:
    timestamp_s: float
    source: str
    joint: str
    address: int
    vendor: str
    status_word: int
    error_code: int
    message: str
    severity: str
    reset_policy: str
    active: bool = True
    possible_causes: tuple[str, ...] = ()
    operator_actions: tuple[str, ...] = ()
    manual_title: str = ""
    manual_version: str = ""
    manual_pages: str = ""
    source_reference: str = ""

    @property
    def fingerprint(self) -> tuple[object, ...]:
        return (
            self.source,
            self.joint,
            self.address,
            self.vendor.casefold(),
            self.status_word,
            self.error_code,
            self.message,
        )


@dataclass
class FaultRow:
    event: FaultEvent
    first_timestamp_s: float
    last_timestamp_s: float
    repeat_count: int
    active: bool


class FaultIncidentModel:
    """Bounded chronological model that preserves the incident's first fault."""

    def __init__(self, max_rows: int = 500) -> None:
        if max_rows < 1:
            raise ValueError("max_rows must be positive")
        self._max_rows = int(max_rows)
        self._rows: list[FaultRow] = []
        self._by_fingerprint: dict[tuple[object, ...], FaultRow] = {}
        self._first_fault: FaultEvent | None = None

    @property
    def rows(self) -> tuple[FaultRow, ...]:
        return tuple(self._rows)

    @property
    def first_fault(self) -> FaultEvent | None:
        return self._first_fault

    def ingest(self, event: FaultEvent) -> FaultRow:
        existing = self._by_fingerprint.get(event.fingerprint)
        if existing is not None:
            existing.last_timestamp_s = event.timestamp_s
            existing.active = event.active
            if event.active:
                existing.repeat_count += 1
            return existing

        if self._first_fault is None and event.active:
            self._first_fault = event

        row = FaultRow(
            event=event,
            first_timestamp_s=event.timestamp_s,
            last_timestamp_s=event.timestamp_s,
            repeat_count=1,
            active=event.active,
        )
        if len(self._rows) >= self._max_rows:
            if len(self._rows) == 1:
                return self._rows[0]
            evicted = self._rows.pop(1)
            self._by_fingerprint.pop(evicted.event.fingerprint, None)
        self._rows.append(row)
        self._by_fingerprint[event.fingerprint] = row
        return row
