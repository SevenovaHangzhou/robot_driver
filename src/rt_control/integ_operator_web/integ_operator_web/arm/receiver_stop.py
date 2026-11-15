"""Domain-private stop client; the controller owns the actual takeover time."""

from rt_control_interfaces.srv import StopJointTrajectory


class ReceiverStopClient:
    def __init__(self, node, group):
        self._node = node
        self._group = group
        self._endpoint = ""
        self._client = None

    def bind(self, response) -> None:
        endpoint = response.values[0].string_value if response.values else ""
        if self._endpoint and endpoint != self._endpoint:
            self._client = None  # A changed endpoint requires a fresh console instance.
            return
        if endpoint and self._client is None and not self._endpoint:
            self._endpoint = endpoint
            self._client = self._node.create_client(
                StopJointTrajectory, endpoint, callback_group=self._group
            )

    def ready(self) -> bool:
        return self._client is not None and self._client.service_is_ready()

    def request(self, accepted_future, limits, done) -> None:
        if not self.ready() or accepted_future is None:
            raise RuntimeError("停止服务或当前轨迹标识不可用")

        def accepted(future):
            try:
                handle = future.result()
                if not handle.accepted:
                    done(False, "原轨迹未被控制器接受")
                    return
                request = StopJointTrajectory.Request(
                    goal_id=handle.goal_id,
                    velocity=limits.velocity,
                    acceleration=limits.acceleration,
                    jerk=limits.jerk,
                )
                self._client.call_async(request).add_done_callback(finished)
            except Exception as exc:
                done(False, str(exc))

        def finished(future):
            try:
                response = future.result()
                done(
                    response.status == StopJointTrajectory.Response.REFERENCE_STOPPED,
                    response.message,
                )
            except Exception as exc:
                done(False, str(exc))

        accepted_future.add_done_callback(accepted)
