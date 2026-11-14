"""Worker-thread calls to the public mode service, followed by fresh readback."""

import threading
import time

from controller_manager_msgs.srv import ListControllers
from robot_rt_control_interfaces.msg import JointControlMode
from robot_rt_control_interfaces.srv import SetJointControlMode

from .modes import ModeError

# Transport/observation deadlines, not hardware motion or standstill thresholds.
MODE_RESPONSE_TIMEOUT_S = 5.0
MODE_READBACK_TIMEOUT_S = 2.0


class ModeService:
    def __init__(self, bridge):
        self.bridge = bridge
        self.client = bridge.create_client(
            SetJointControlMode, "/rt/arms/set_control_mode", callback_group=bridge._group
        )

    def available(self):
        return self.client.service_is_ready()

    def matches(self, target):
        return self.bridge._readback.evidence.mode_matches(target)

    def _call(self, client, request):
        if not client.service_is_ready():
            raise ModeError("mode_unavailable", "控制模式服务不可用")
        event = threading.Event()
        try:
            future = client.call_async(request)
            future.add_done_callback(lambda _: event.set())
            if not event.wait(MODE_RESPONSE_TIMEOUT_S):
                client.remove_pending_request(future)
                raise ModeError(
                    "mode_timeout", "模式切换响应超时，保留控制权等待核查", uncertain=True
                )
            return future.result()
        except ModeError:
            raise
        except Exception as exc:
            raise ModeError("mode_unknown", f"模式切换结果未确认：{exc}", uncertain=True) from exc

    def switch(self, target):
        value = {"jtc": JointControlMode.FJT_READY, "rolling": JointControlMode.ROLLING_READY}[
            target
        ]
        response = self._call(
            self.client, SetJointControlMode.Request(target_mode=JointControlMode(value=value))
        )
        if not response.accepted:
            raise ModeError(
                "mode_rejected",
                response.error.message or "模式切换被拒绝",
                uncertain=response.mode.value == JointControlMode.RESTART_REQUIRED,
            )
        if response.mode.value != value:
            raise ModeError("mode_unconfirmed", "服务响应的实际模式与请求不一致", uncertain=True)
        since = self.bridge.console._clock()
        controllers = self._call(self.bridge._controllers, ListControllers.Request())
        self.bridge._received_controllers(controllers)
        deadline = time.monotonic() + MODE_READBACK_TIMEOUT_S
        while time.monotonic() < deadline:
            if self.matches(target) and self.bridge.console.idle_probe(since):
                return
            time.sleep(0.01)
        raise ModeError("mode_readback_timeout", "切换后尚未收到完整停稳反馈，未发送轨迹")
