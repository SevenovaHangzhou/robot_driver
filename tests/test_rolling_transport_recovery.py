"""Service bookkeeping with in-memory Futures; no generated-type claim."""
from concurrent.futures import Future
from copy import deepcopy
import threading
from types import SimpleNamespace

import pytest

from gripper_jog.rolling import RollingMotion, identifier
from gripper_jog.rolling_mock import MockRollingTransport
from gripper_jog.rolling_ros import RosRollingTransport
from test_rolling import rolling


class Service:
    class Request:
        pass


class CountedFuture(Future):
    callback_count = 0

    def add_done_callback(self, fn):
        self.callback_count += 1
        super().add_done_callback(fn)


def service_transport():
    transport = RosRollingTransport.__new__(RosRollingTransport)
    transport.lock = threading.Lock()
    transport.service_timeout = .000001
    transport.pending_services = {}
    transport.encode = lambda message, body: vars(message).update(deepcopy(body))
    transport.decode = deepcopy
    return transport


def test_repeated_unknown_request_keeps_one_future_and_one_callback():
    transport = service_transport()
    future = CountedFuture()
    calls = []
    client = SimpleNamespace(service_is_ready=lambda: True,
                             call_async=lambda body: (calls.append(body), future)[1])
    body = {'request_id': identifier()}
    for _ in range(1000):
        with pytest.raises(TimeoutError):
            transport._service(client, Service, body)
    assert len(calls) == 1 and future.callback_count == 1
    future.set_result(deepcopy(body))
    assert transport._service(client, Service, body) == body
    assert not transport.pending_services and not transport.service_events


def test_permanently_lost_stop_responses_are_released_after_confirmed_finalize(rolling):
    jog, _, clock, _ = rolling
    plan = jog.planner.step(jog.q, 'x+', .0005, require_collision=False)
    services = service_transport()
    futures, removed = [], []

    class Transport(MockRollingTransport):
        def close(self, request):
            return services._service(client, Service, request)

        def session_finalized(self, identity):
            services.session_finalized(identity)

    import numpy as np
    transport = Transport(np.r_[jog.q + .1, jog.q], .07, clock)

    def call_async(message):
        body = vars(message)
        response = MockRollingTransport.close(transport, body)
        future = CountedFuture()
        if body['operation'] == 'REQUEST_STOP':
            futures.append(future)  # Server executes; response never arrives.
        else:
            future.set_result({**response, 'request_id': body['request_id']})
        return future

    client = SimpleNamespace(service_is_ready=lambda: True, call_async=call_async,
                             remove_pending_request=lambda future: removed.append(future))
    for _ in range(6):
        motion = RollingMotion(plan, transport, clock)
        assert not motion.fault
        motion.brake(0)
        motion.state(0)
        assert motion.complete and transport.status == 0
        assert not services.pending_services and not services.service_events
    assert len(futures) == len(removed) == 6 and all(f.cancelled() for f in futures)


def test_finalization_does_not_cancel_another_sessions_pending_request():
    transport = service_transport()
    identity = {name: identifier() for name in ('controller_boot_id', 'client_instance_id', 'session_id')}
    future = CountedFuture()
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda _: future)
    with pytest.raises(TimeoutError):
        transport._service(client, Service, {**identity, 'request_id': identifier()})
    transport.session_finalized({**identity, 'session_id': identifier()})
    assert len(transport.pending_services) == 1 and not future.cancelled()
