"""Optional ROS transport for the authoritative Rolling interfaces.

The caller owns the node, executor and public-contract QoS profiles. Call from
a non-executor thread while that executor spins. No controller mode switching,
enable, reset, or autonomous motion is performed by construction.
"""
from copy import deepcopy
import threading
import time

from .errors import JogError
from .rolling import PREFIX


class RosRollingTransport:
    def __init__(self, node, *, command_qos, state_qos, service_timeout=1., state_timeout=.15,
                 clock=time.monotonic):
        # Generated types must come from the pinned robot_interfaces workspace.
        from robot_motion_interfaces.msg import RollingJointTargetBatch
        from robot_rt_control_interfaces.msg import RollingJointControlState
        from robot_rt_control_interfaces.srv import OpenRollingJointSession, CloseRollingJointSession
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.set_message import set_message_fields

        if service_timeout <= 0 or state_timeout <= 0:
            raise ValueError('ROS transport timeouts must be positive')
        self.node, self.clock = node, clock
        self.service_timeout, self.state_timeout = service_timeout, state_timeout
        self.batch_type, self.open_type, self.close_type = RollingJointTargetBatch, OpenRollingJointSession, CloseRollingJointSession
        self.decode, self.encode = message_to_ordereddict, set_message_fields
        self.latest, self.received_at, self.rolling_mode = None, 0., None
        self.lock = threading.Lock()
        self.state_changed = threading.Event()
        self.pending_services = {}
        self.service_events, self.service_owners, self.service_clients = {}, {}, {}
        self.publisher = node.create_publisher(RollingJointTargetBatch, PREFIX + '/update', command_qos)
        self.subscription = node.create_subscription(RollingJointControlState, PREFIX + '/state', self._on_state, state_qos)
        self.open_client = node.create_client(OpenRollingJointSession, PREFIX + '/open')
        self.close_client = node.create_client(CloseRollingJointSession, PREFIX + '/close')

    def _on_state(self, message):
        decoded = self.decode(message)
        with self.lock:
            if self.latest is not None and decoded['controller_boot_id'] == self.latest['controller_boot_id'] and \
                    decoded['state_sequence'] <= self.latest['state_sequence']:
                return
            self.latest, self.received_at = decoded, self.clock()
            self.rolling_mode = message.control_mode.ROLLING_READY
            self.state_changed.set()

    def state(self):
        with self.lock:
            if self.latest is None or self.clock() - self.received_at > self.state_timeout:
                raise JogError('rolling_state_stale', 'Rolling 状态未到达或已超时')
            if self.latest['control_mode']['value'] != self.rolling_mode:
                raise JogError('rolling_not_active', 'Rolling 控制器尚未处于 ROLLING_READY 模式')
            return deepcopy(self.latest)

    def _service(self, client, kind, body):
        key = (kind.__name__, bytes(body['request_id']['uuid']))
        with self.lock:
            if key not in self.pending_services:
                if not client.service_is_ready():
                    raise JogError('rolling_service_unavailable', 'Rolling 服务未就绪')
                if len(self.pending_services) >= 4:
                    raise JogError('rolling_service_pending', 'Rolling 存在未确认的服务请求')
                message = kind.Request()
                self.encode(message, body)
                self.pending_services[key] = client.call_async(message)
            future = self.pending_services[key]
            # One event/callback per request, including arbitrarily many retries.
            if not hasattr(self, 'service_events'):
                self.service_events, self.service_owners, self.service_clients = {}, {}, {}
            if key not in self.service_events:
                self.service_events[key] = threading.Event()
                self.service_owners[key] = {k: body[k] for k in ('controller_boot_id', 'client_instance_id', 'session_id') if k in body}
                self.service_clients[key] = client
                finished = self.service_events[key]
                future.add_done_callback(lambda _, event=finished: event.set())
            finished = self.service_events[key]
        if not finished.wait(self.service_timeout):
            # Keep the same request/future for an idempotent retry. A timeout
            # does not prove that the server failed to open or close a session.
            raise TimeoutError('Rolling service response timed out; outcome unknown')
        response = future.result()
        with self.lock:
            self.pending_services.pop(key, None)
            self.service_events.pop(key, None)
            self.service_owners.pop(key, None)
            self.service_clients.pop(key, None)
        decoded = self.decode(response)
        if decoded['request_id'] != body['request_id']:
            raise JogError('rolling_response_identity', 'Rolling 服务响应身份不匹配')
        return decoded

    def session_finalized(self, identity):
        """A confirmed FINALIZE makes older service outcomes irrelevant."""
        with self.lock:
            owners = getattr(self, 'service_owners', {})
            keys = [key for key, owner in owners.items() if owner and
                    all(identity.get(name) == ident for name, ident in owner.items())]
            for key in keys:
                future = self.pending_services.pop(key)
                client = self.service_clients.pop(key)
                remove = getattr(client, 'remove_pending_request', None)
                if remove:
                    remove(future)
                future.cancel()
                self.service_events.pop(key, None)
                owners.pop(key, None)

    def open(self, request):
        response = self._service(self.open_client, self.open_type, request)
        if not response['accepted']:
            return response
        # Service responses and state messages have independent DDS ordering.
        # Confirm this session before the caller can publish or compare state.
        # A missing confirmation leaves the open outcome uncertain; the owner
        # must reconcile the same request and close it without resuming motion.
        deadline = self.clock() + self.state_timeout
        while True:
            with self.lock:
                current = self.latest
                if current is not None and current['has_session'] and \
                        self.clock() - self.received_at <= self.state_timeout and \
                        all(current[key] == response[key] for key in
                            ('controller_boot_id', 'session_id', 'client_instance_id')):
                    return response
                self.state_changed.clear()
            remaining = deadline - self.clock()
            if remaining <= 0 or not self.state_changed.wait(remaining):
                raise TimeoutError('Rolling open accepted but session state is unconfirmed')

    def publish(self, batch):
        self.state()  # Do not publish using stale controller state.
        message = self.batch_type()
        self.encode(message, batch)
        self.publisher.publish(message)

    def close(self, request):
        body = deepcopy(request)
        body['operation'] = getattr(self.close_type.Request, body['operation'])
        return self._service(self.close_client, self.close_type, body)

    def destroy(self):
        """Release ROS handles after the owning application has closed its session."""
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_publisher(self.publisher)
        self.node.destroy_client(self.open_client)
        self.node.destroy_client(self.close_client)
