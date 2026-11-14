"""Read the stationary command without substituting measured motor positions."""

import math
import threading

from control_msgs.srv import QueryTrajectoryState

from .console_types import ConsoleError


def holding_identity(snapshot, protocol):
    state, age = snapshot.get("controller"), snapshot.get("state_age_s")
    if (
        state is None
        or state["state"] != protocol.HOLDING
        or type(age) not in (int, float)
        or not 0 <= age <= protocol.STATE_MAX_AGE_S
    ):
        raise ConsoleError("jog_hold_changed", "Rolling 保持状态已变化或过期，请等待停稳后重新按键")
    return tuple(state[k] for k in ("epoch", "last_sequence", "last_result", "stop_reason"))


class RollingHold:
    def __init__(self, node, controller, group):
        self.client = node.create_client(
            QueryTrajectoryState, f"/{controller}/query_state", callback_group=group
        )

    def query(self, names):
        if not self.client.service_is_ready():
            raise ConsoleError("jog_hold_unavailable", "Rolling 指令保持值查询不可用，未启动运动")
        future = self.client.call_async(QueryTrajectoryState.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(0.2):
            self.client.remove_pending_request(future)
            raise ConsoleError("jog_hold_timeout", "Rolling 指令保持值查询超时，未启动运动")
        try:
            response = future.result()
        except Exception as exc:
            raise ConsoleError(
                "jog_hold_unavailable", "Rolling 指令保持值查询失败，未启动运动"
            ) from exc
        if not response.success:
            raise ConsoleError("jog_hold_unavailable", "Rolling 尚未处于可读取的静止保持状态")
        if (
            list(response.name) != list(names)
            or len(response.position) != len(names)
            or not all(math.isfinite(q) for q in response.position)
            or list(response.velocity) != [0.0] * len(names)
            or list(response.acceleration) != [0.0] * len(names)
        ):
            raise ConsoleError("jog_hold_invalid", "Rolling 指令保持值的轴序或静止状态无效")
        return dict(zip(names, response.position))
