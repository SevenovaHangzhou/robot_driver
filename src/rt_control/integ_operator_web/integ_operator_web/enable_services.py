"""Blocking calls to the public enable and fault-reset services, shared by the arm and head pages.

Callers run them from a worker thread (never the ROS executor or the aiohttp loop).
"""

from __future__ import annotations

import threading

from robot_rt_control_interfaces.srv import RtEnable

ENABLE_TIMEOUT_S = 95.0
# enable_manager waits up to 30 s for the group reset to settle.
RESET_TIMEOUT_S = 35.0


class EnableServices:
    def __init__(self, node, callback_group, error) -> None:
        self._error = error
        self._enable = {
            enabled: node.create_client(RtEnable, name, callback_group=callback_group)
            for enabled, name in ((True, "/rt/enable"), (False, "/rt/disable"))
        }
        # Explicit operator fault reset (runbook: "explicit group reset"); enable_manager decides.
        self._reset = node.create_client(RtEnable, "/rt/reset_fault", callback_group=callback_group)

    def _call(self, client, request, timeout, unavailable, late):
        if not client.service_is_ready():
            raise self._error(*unavailable)
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            client.remove_pending_request(future)
            raise self._error(*late)
        return future.result()

    def set_enabled(self, enabled: bool) -> dict:
        result = self._call(
            self._enable[enabled],
            RtEnable.Request(),
            ENABLE_TIMEOUT_S,
            ("enable_unavailable", "使能服务不可用"),
            ("enable_timeout", "使能服务响应超时，请查看实际状态"),
        )
        return {
            "accepted": bool(result.ok),
            "enabled": enabled if result.ok else False,
            "message": result.error.message,
        }

    def reset_fault(self) -> dict:
        """Group fault reset through enable_manager: Fault -> Switch On Disabled, never enables."""
        result = self._call(
            self._reset,
            RtEnable.Request(),
            RESET_TIMEOUT_S,
            ("reset_unavailable", "故障复位服务不可用"),
            ("reset_timeout", "故障复位响应超时，请查看实际状态"),
        )
        return {
            "ok": bool(result.ok),
            "stage": result.stage,
            "failed_joint": result.failed_joint,
            "status_word": int(result.status_word),
        }
