from __future__ import annotations

from dataclasses import dataclass


VALID_COMMANDS = frozenset({"reset_fault", "enable", "stop"})


@dataclass(frozen=True)
class PendingCommand:
    operation_id: int
    command: str


class SingleFlightCommands:
    def __init__(self) -> None:
        self._next_id = 1
        self._pending: PendingCommand | None = None

    @property
    def pending(self) -> PendingCommand | None:
        return self._pending

    def begin(self, command: str) -> PendingCommand | None:
        if command not in VALID_COMMANDS:
            raise ValueError(f"unsupported operator command: {command}")
        if self._pending is not None:
            return None
        pending = PendingCommand(self._next_id, command)
        self._next_id += 1
        self._pending = pending
        return pending

    def complete(self, operation_id: int) -> bool:
        pending = self._pending
        if pending is None or pending.operation_id != operation_id:
            return False
        self._pending = None
        return True
