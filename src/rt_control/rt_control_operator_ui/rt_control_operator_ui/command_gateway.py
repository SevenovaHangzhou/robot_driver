from __future__ import annotations

from collections.abc import Callable

from .commands import SingleFlightCommands


class CommandGateway:
    def __init__(
        self,
        *,
        dispatch: Callable[[str, int], None],
        on_result: Callable[[bool, str], None],
    ) -> None:
        self._dispatch = dispatch
        self._on_result = on_result
        self._commands = SingleFlightCommands()

    def request(self, command: str) -> bool:
        pending = self._commands.begin(command)
        if pending is None:
            return False
        self._dispatch(command, pending.operation_id)
        return True

    def complete(
        self, operation_id: int, success: bool, result_text: str
    ) -> bool:
        if not self._commands.complete(operation_id):
            return False
        self._on_result(bool(success), str(result_text))
        return True
