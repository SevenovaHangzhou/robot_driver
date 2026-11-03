"""One operator at a time across pages.

Each page keeps its own lease (chassis arbiter, arm console), keyed by WebSocket connection.
A connection may take a page lease only while every lease held on the other pages belongs to
connections of the same login session.
"""

from __future__ import annotations

import threading
from typing import Callable, Optional


class Operators:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._login_of: dict[str, str] = {}
        self._holders: dict[str, Callable[[], Optional[str]]] = {}

    def register_page(self, page: str, holder: Callable[[], Optional[str]]) -> None:
        self._holders[page] = holder

    def connect(self, connection: str, login: str) -> None:
        with self._lock:
            self._login_of[connection] = login

    def disconnect(self, connection: str) -> None:
        with self._lock:
            self._login_of.pop(connection, None)

    def may_acquire(self, page: str, connection: str) -> bool:
        with self._lock:
            login = self._login_of.get(connection)
            for other, holder in self._holders.items():
                held_by = holder() if other != page else None
                if held_by is not None and self._login_of.get(held_by) != login:
                    return False
            return login is not None
