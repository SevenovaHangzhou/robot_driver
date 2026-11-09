"""Use the frozen, generated ROS types; no ROS daemon or sockets required."""
from concurrent.futures import Future
import threading
import time

import pytest

pytest.importorskip('robot_rt_control_interfaces.msg')
from robot_rt_control_interfaces.msg import RollingJointControlState
from robot_rt_control_interfaces.srv import OpenRollingJointSession
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from gripper_jog.errors import JogError
from gripper_jog.rolling import identifier
from gripper_jog.rolling_ros import RosRollingTransport


def bare_transport():
    transport = RosRollingTransport.__new__(RosRollingTransport)
    transport.lock = threading.Lock()
    transport.state_changed = threading.Event()
    transport.clock = time.monotonic
    transport.latest, transport.received_at, transport.rolling_mode = None, 0., None
    transport.state_timeout, transport.service_timeout = .15, .001
    transport.pending_services = {}
    transport.decode, transport.encode = message_to_ordereddict, set_message_fields
    return transport


def test_actual_generated_control_mode_and_stale_state_filter():
    transport = bare_transport()
    message = RollingJointControlState()
    message.control_mode.value = message.control_mode.ROLLING_READY
    message.state_sequence = 2
    transport._on_state(message)
    assert transport.state()['control_mode']['value'] == 2
    message.state_sequence = 1
    message.control_mode.value = message.control_mode.FJT_READY
    transport._on_state(message)
    assert transport.state()['state_sequence'] == 2
    message.state_sequence = 3
    transport._on_state(message)
    with pytest.raises(JogError, match='ROLLING_READY'):
        transport.state()


def test_timed_out_service_reuses_future_even_after_service_disappears():
    transport = bare_transport()
    future = Future()

    class Client:
        ready, calls = True, 0

        def service_is_ready(self):
            return self.ready

        def call_async(self, request):
            self.calls += 1
            return future

    client = Client()
    body = {'request_id': identifier()}
    with pytest.raises(TimeoutError):
        transport._service(client, OpenRollingJointSession, body)
    response = OpenRollingJointSession.Response()
    set_message_fields(response, body)
    future.set_result(response)
    client.ready = False
    resolved = transport._service(client, OpenRollingJointSession, body)
    assert resolved['request_id']['uuid'] == body['request_id']['uuid']
    assert client.calls == 1 and not transport.pending_services


def opening_transport():
    transport = bare_transport()
    transport.open_client, transport.open_type = object(), OpenRollingJointSession
    response = OpenRollingJointSession.Response()
    response.accepted = True
    for name in ('controller_boot_id', 'session_id', 'client_instance_id'):
        set_message_fields(getattr(response, name), identifier())
    decoded = message_to_ordereddict(response)
    transport._service = lambda *args: decoded
    initial = RollingJointControlState()
    initial.controller_boot_id = response.controller_boot_id
    initial.control_mode.value = initial.control_mode.ROLLING_READY
    initial.state_sequence = 1
    transport._on_state(initial)
    current = RollingJointControlState()
    current.controller_boot_id = response.controller_boot_id
    current.session_id = response.session_id
    current.client_instance_id = response.client_instance_id
    current.control_mode.value = current.control_mode.ROLLING_READY
    current.has_session, current.state_sequence = True, 2
    return transport, decoded, current


def test_open_waits_for_its_state_after_service_response():
    transport, response, current = opening_transport()
    arrival = threading.Timer(.02, transport._on_state, args=(current,))
    arrival.start()
    try:
        assert transport.open({}) == response
        assert transport.state()['session_id'] == response['session_id']
    finally:
        arrival.join()


def test_open_without_session_state_remains_uncertain():
    transport, _, _ = opening_transport()
    transport.state_timeout = .01
    with pytest.raises(TimeoutError, match='session state is unconfirmed'):
        transport.open({})


def test_open_never_accepts_another_clients_state():
    transport, _, current = opening_transport()
    current.client_instance_id.uuid = identifier()['uuid']
    transport._on_state(current)
    transport.state_timeout = .01
    with pytest.raises(TimeoutError, match='session state is unconfirmed'):
        transport.open({})
