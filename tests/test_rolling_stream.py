"""Continuous Rolling contracts, exercised without ROS or a live controller."""
from copy import deepcopy
from types import SimpleNamespace
import weakref

import numpy as np
from numpy.polynomial import Polynomial
import pytest

from gripper_jog.errors import JogError
from gripper_jog.rolling import JOINT_NAMES, RollingMotion, identifier
from gripper_jog.rolling_mock import MockRollingTransport
from gripper_jog.rolling_stream import Knot, RollingStream, validate_window
from test_rolling import rolling
from test_rolling_recovery import LostOpen


class MonotonicClock:
    """Integer ticks with nonzero uptime, independent of session timestamps."""
    def __init__(self):
        self.now_ns = 30_000_000_000

    def __call__(self):
        return self.now_ns / 1e9

    def advance(self, nanoseconds=20_000_000):
        assert nanoseconds >= 0
        self.now_ns += nanoseconds


class AnalyticSource:
    """Seven selected axes in a full dual-arm constant-jerk prediction.

    A damped velocity response supplies cubics by integrating a constant jerk.
    This fixture has no session, window, revision, or acknowledgement logic.
    """
    duration = float('inf')
    acceleration = .07
    jerk = .28

    def __init__(self, chain, initial):
        self.chain, self.initial = chain, initial.copy()
        self.selected = np.array([JOINT_NAMES.index(n) for n in chain.names])
        self.lower, self.upper = np.tile(chain.lower, 2), np.tile(chain.upper, 2)
        self.speed = np.minimum(np.tile(chain.velocity, 2), .035)
        self.rollouts = self.validations = self.stops = 0
        self.stop_until_ns = None
        self.stop_origin = None

    def state(self, at):
        assert at == 0
        return self.initial[self.selected].copy(), np.zeros(7), np.zeros(7), np.zeros(7)

    @staticmethod
    def integrate(anchor, time_ns, jerk):
        dt = (time_ns - anchor.time_ns) / 1e9
        return Knot(time_ns, anchor.q + anchor.v * dt + anchor.a * dt**2 / 2 + jerk * dt**3 / 6,
                    anchor.v + anchor.a * dt + jerk * dt**2 / 2, anchor.a + jerk * dt)

    def rollout(self, anchor, times, velocity):
        self.rollouts += 1
        target = np.zeros(14)
        target[self.selected[:3]] = velocity
        knots = []
        for at in times:
            # Critically damped velocity servo; its exact cubic is the test input.
            jerk = 8**2 * (target - anchor.v) - 2 * 8 * anchor.a
            anchor = self.integrate(anchor, at, jerk)
            knots.append(anchor)
        return knots

    def validate(self, knots):
        self.validations += 1
        return validate_window(knots, self.lower, self.upper, self.speed,
                               self.acceleration, self.jerk)

    def stop_rollout(self, anchor, times):
        self.stops += 1
        if self.stop_origin is None:
            self.stop_origin = deepcopy(anchor)
            self.stop_half_ns = 6 * (times[0] - anchor.time_ns)
            half = self.stop_half_ns / 1e9
            # Two constant-jerk arcs solve v(T)=a(T)=0 analytically.
            self.stop_jerk1 = -anchor.v / half**2 - 1.5 * anchor.a / half
            self.stop_jerk2 = anchor.v / half**2 + .5 * anchor.a / half
            self.stop_middle = self.integrate(anchor, anchor.time_ns + self.stop_half_ns, self.stop_jerk1)
            self.stop_until_ns = anchor.time_ns + 2 * self.stop_half_ns
            self.stop_end = self.integrate(self.stop_middle, self.stop_until_ns, self.stop_jerk2)
            np.testing.assert_allclose(self.stop_end.v, 0, atol=1e-15)
            np.testing.assert_allclose(self.stop_end.a, 0, atol=1e-15)
        result = []
        for at in times:
            if at <= self.stop_middle.time_ns:
                knot = self.integrate(self.stop_origin, at, self.stop_jerk1)
            elif at < self.stop_until_ns:
                knot = self.integrate(self.stop_middle, at, self.stop_jerk2)
            else:
                knot = Knot(at, self.stop_end.q.copy(), np.zeros(14), np.zeros(14))
            result.append(knot)
        return result


@pytest.fixture
def make_stream(rolling, monkeypatch):
    jog, _, _, _ = rolling

    def make(transport_type=MockRollingTransport):
        clock = MonotonicClock()
        monkeypatch.setattr('gripper_jog.rolling_stream.time', SimpleNamespace(perf_counter=clock))
        source = AnalyticSource(jog.planner.chain, np.r_[jog.q + .1, jog.q])
        transport = transport_type(source.initial, source.acceleration, clock)
        motion = RollingStream(source, transport, [.002, 0, 0], clock)
        return SimpleNamespace(source=source, transport=transport, clock=clock, motion=motion)

    return make


@pytest.fixture
def rig(make_stream):
    return make_stream()


def tick(rig, count=1, nanoseconds=20_000_000):
    for _ in range(count):
        rig.clock.advance(nanoseconds)
        rig.motion.state(0)


def acknowledge_start(rig):
    rig.motion.state(0)  # ACK hold and immediately publish command revision 1.
    rig.motion.state(0)  # ACK that replacement without advancing session time.
    assert not rig.motion.pending and not rig.motion.fault


def assert_knots_equal(actual, expected):
    assert [k.time_ns for k in actual] == [k.time_ns for k in expected]
    for new, old in zip(actual, expected):
        for name in ('q', 'v', 'a'):
            np.testing.assert_array_equal(getattr(new, name), getattr(old, name))


def test_first_batch_holds_then_first_ack_immediately_replaces_revision_one(rig):
    motion, transport, source = rig.motion, rig.transport, rig.source
    assert isinstance(motion, RollingMotion)
    first = deepcopy(transport.batches[0])
    assert first['sequence'] == 1 and first['replace_from_ns'] == 0
    assert first['points'][-1]['time_from_session_start_ns'] >= transport.negotiated['required_initial_horizon_ns']
    for point in first['points']:
        np.testing.assert_array_equal(point['positions'], source.initial)
        np.testing.assert_array_equal(point['velocities'], np.zeros(14))
    assert not motion.committed and motion.proposed and motion.pending == 1
    assert source.rollouts == 0
    hold_proposal = motion.proposed
    before = rig.clock()

    motion.state(0)

    assert rig.clock() == before
    assert motion.committed is hold_proposal
    assert len(transport.batches) == 2 and motion.pending == 2
    second = transport.batches[-1]
    assert 0 < second['replace_from_ns'] < first['points'][-1]['time_from_session_start_ns']
    assert second['replace_from_ns'] in [p['time_from_session_start_ns'] for p in first['points']]
    assert any(np.any(p['velocities']) for p in second['points'][1:])
    assert source.rollouts == 1
    assert motion.publication()['command_revision'] == 1
    assert motion.publication()['accepted_revision'] is None
    motion.state(0)
    assert motion.publication()['accepted_revision'] == 1 and not motion.pending


@pytest.mark.parametrize('phase', ['initial_hold', 'replacement'])
def test_unacknowledged_proposal_is_neither_committed_nor_used_for_another_send(rig, monkeypatch, phase):
    motion, transport = rig.motion, rig.transport
    if phase == 'replacement':
        acknowledge_start(rig)
        motion.set_velocity([-.002, 0, 0])
        motion.state(0)
    sequence = motion.sequence
    committed, proposal = motion.committed, motion.proposed
    old = deepcopy(committed)
    rollouts = rig.source.rollouts
    state = transport.state

    def without_ack():
        snapshot = state()
        snapshot['last_accepted_sequence'] = sequence - 1
        return snapshot

    monkeypatch.setattr(transport, 'state', without_ack)
    for velocity in ([0, .001, 0], [0, .002, 0], [0, .003, 0], [0, .004, 0]):
        motion.set_velocity(velocity)
        tick(rig)
        assert motion.sequence == sequence and transport.sequence == sequence
        assert motion.pending == sequence and motion.proposed is proposal
        assert motion.committed is committed
        assert_knots_equal(motion.committed, old)
        assert rig.source.rollouts == rollouts
        assert motion.publication()['accepted_revision'] is None
        assert not motion.fault

    monkeypatch.setattr(transport, 'state', state)
    motion.state(0)
    assert motion.committed is proposal
    assert motion.pending == sequence + 1
    assert motion.proposed is not proposal
    motion.state(0)
    assert motion.publication()['accepted_revision'] == motion.publication()['command_revision']


def test_long_running_stream_uses_one_session_and_discards_old_knots(rig, monkeypatch):
    motion, transport = rig.motion, rig.transport
    identity = deepcopy(motion.identity)
    open_again = []
    original_open = transport.open

    def open_session(request):
        open_again.append(deepcopy(request))
        return original_open(request)

    monkeypatch.setattr(transport, 'open', open_session)
    acknowledge_start(rig)
    tick(rig, 50)
    old_knots = [weakref.ref(k) for k in motion.committed]
    capacities = transport.negotiated
    maximum = 0
    # Two minutes is hundreds of negotiated horizons, with irregular monotonic ticks.
    for i in range(6_000):
        tick(rig, nanoseconds=(19_000_001, 20_999_999, 20_000_000)[i % 3])
        assert not motion.fault and not motion.complete
        assert motion.identity == identity
        publication = motion.publication()
        retained = len(motion.committed) + len(motion.proposed or ())
        maximum = max(maximum, retained)
        assert retained <= 2 * capacities['buffer_capacity']
        assert publication['buffered_points'] <= capacities['buffer_capacity']
        assert len(transport.points) <= capacities['buffer_capacity']
        assert len(transport.batches) <= 32
        batch = transport.batches[-1]
        assert {key: batch[key] for key in identity} == identity
        assert 2 <= len(batch['points']) <= min(capacities['transport_max_points'], capacities['buffer_capacity'])
        times = [p['time_from_session_start_ns'] for p in batch['points']]
        assert times[0] == batch['replace_from_ns'] and np.all(np.diff(times) > 0)
        assert times[-1] - transport.execution <= capacities['max_horizon_ns']

    assert motion.sequence > 1_000 and not open_again and not transport.close_operations
    assert maximum > 0 and all(ref() is None for ref in old_knots)
    # A nonzero clock origin must not leak into session-relative knot timestamps.
    assert motion.execution_ns == pytest.approx(121_000_000_000, abs=25_000_000)
    assert transport.q[rig.source.selected[0]] - rig.source.initial[rig.source.selected[0]] > .2
    other = np.setdiff1d(np.arange(14), rig.source.selected)
    np.testing.assert_array_equal(transport.q[other], rig.source.initial[other])
    np.testing.assert_array_equal(transport.v[other], np.zeros(7))
    for batch in transport.batches:
        for point in batch['points']:
            np.testing.assert_array_equal(np.asarray(point['positions'])[other], rig.source.initial[other])
            np.testing.assert_array_equal(np.asarray(point['velocities'])[other], np.zeros(7))


def test_direction_change_replaces_an_accepted_future_knot_with_c2_and_preserves_prefix(rig):
    acknowledge_start(rig)
    tick(rig, 12)
    motion, transport = rig.motion, rig.transport
    motion.state(0)
    assert not motion.pending and np.linalg.norm(transport.v) > 0
    previous_curve, previous_until = transport.curve, transport.until
    previous_points, committed = deepcopy(transport.points), deepcopy(motion.committed)
    snapshot = transport.state()
    motion.set_velocity([0, -.002, 0])

    motion.state(0)

    assert not motion.fault and motion.pending
    batch = transport.batches[-1]
    seam = batch['replace_from_ns']
    assert snapshot['execution_time_ns'] < seam < previous_until
    assert seam >= snapshot['replaceable_from_ns']
    assert seam in [p['time_from_session_start_ns'] for p in previous_points]
    assert seam in [k.time_ns for k in committed]
    for derivative in range(3):
        np.testing.assert_allclose(transport.curve(seam / 1e9, derivative),
                                   previous_curve(seam / 1e9, derivative), atol=2e-9, rtol=0)
        before_seam = np.linspace(snapshot['execution_time_ns'], seam, 17) / 1e9
        np.testing.assert_allclose(transport.curve(before_seam, derivative),
                                   previous_curve(before_seam, derivative), atol=2e-9, rtol=0)
    retained = [k for k in motion.proposed if k.time_ns < seam]
    old_by_time = {k.time_ns: k for k in committed}
    assert retained
    assert_knots_equal(retained, [old_by_time[k.time_ns] for k in retained])
    assert_knots_equal(motion.committed, committed)  # The replacement still needs its ACK.
    after_seam = (seam + batch['points'][-1]['time_from_session_start_ns']) / 2e9
    assert not np.allclose(transport.curve(after_seam), previous_curve(after_seam), atol=1e-8, rtol=0)
    tick(rig, 100)
    assert not motion.fault
    np.testing.assert_allclose(transport.v[rig.source.selected[:3]], [0, -.002, 0], atol=1e-6)


@pytest.mark.parametrize('fault', ['rejected', 'lost_ack', 'late', 'boot_changed'])
def test_fault_abandons_stream_and_healthy_feedback_never_resumes_it(rig, monkeypatch, fault):
    acknowledge_start(rig)
    tick(rig, 12)
    motion, transport = rig.motion, rig.transport
    motion.state(0)
    motion.set_velocity([-.002, 0, 0])
    motion.state(0)
    sequence, committed = motion.sequence, motion.committed
    assert motion.pending == sequence
    state, changed_boot = transport.state, identifier()
    if fault == 'lost_ack':
        # Isolate the client ACK deadline from the simulator's own watchdog.
        monkeypatch.setitem(transport.negotiated, 'update_timeout_ns', 10 * motion.timeout)

    def faulty_state():
        snapshot = state()
        if fault == 'rejected':
            snapshot.update(last_rejected_sequence=sequence, last_reject=18)
        elif fault == 'lost_ack':
            snapshot['last_accepted_sequence'] = sequence - 1
        elif fault == 'late':
            snapshot['replaceable_from_ns'] = motion.until + 1
        else:
            snapshot['controller_boot_id'] = changed_boot
        return snapshot

    monkeypatch.setattr(transport, 'state', faulty_state)
    for _ in range(15):
        tick(rig)
        assert motion.sequence == sequence
        if motion.fault:
            break
    assert motion.fault and motion.stopping
    if fault == 'lost_ack':
        assert '确认超时' in motion.fault
    if fault != 'late':
        assert motion.committed is committed
    rollouts = rig.source.rollouts
    monkeypatch.setattr(transport, 'state', state)
    with pytest.raises(JogError) as error:
        motion.set_velocity([.003, 0, 0])
    assert error.value.code == 'rolling_stopping'
    tick(rig, 50)
    assert motion.sequence == sequence and transport.sequence == sequence
    assert rig.source.rollouts == rollouts
    assert motion.complete and motion.fault and transport.status == 0
    assert transport.close_operations == ['REQUEST_STOP', 'FINALIZE']


def test_generation_that_outlives_its_future_seam_stops_without_publishing(rig, monkeypatch):
    acknowledge_start(rig)
    motion, transport = rig.motion, rig.transport
    sequence, committed = motion.sequence, motion.committed
    rollout = rig.source.rollout

    def slow_rollout(anchor, times, velocity):
        result = rollout(anchor, times, velocity)
        rig.clock.advance(160_000_000)
        return result

    monkeypatch.setattr(rig.source, 'rollout', slow_rollout)
    motion.set_velocity([0, .002, 0])
    motion.state(0)
    assert motion.fault and '接缝已过期' in motion.fault
    assert motion.sequence == sequence and transport.sequence == sequence
    assert motion.committed is committed and motion.proposed is None
    tick(rig, 30)
    assert motion.complete and motion.sequence == sequence


def test_halt_plans_c2_stop_before_request_stop_and_finalize(rig):
    acknowledge_start(rig)
    tick(rig, 50)
    motion, transport, source = rig.motion, rig.transport, rig.source
    motion.state(0)
    old_curve, revision = transport.curve, motion.revision
    assert np.linalg.norm(transport.v) > 0
    motion.request_halt()
    motion.request_halt()
    assert motion.revision == revision + 1 and motion.publication()['stopping']
    motion.state(0)
    assert source.stops == 1 and source.stop_until_ns is not None
    assert not motion.fault and not transport.close_operations
    seam = transport.batches[-1]['replace_from_ns'] / 1e9
    for derivative in range(3):
        np.testing.assert_allclose(transport.curve(seam, derivative), old_curve(seam, derivative), atol=2e-9, rtol=0)
    assert transport.until >= source.stop_until_ns
    for _ in range(60):
        tick(rig)
        assert not motion.fault
        if transport.close_operations:
            assert motion.execution_ns >= source.stop_until_ns
        if motion.complete:
            break
    assert motion.complete and transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    np.testing.assert_allclose(transport.q, source.stop_end.q, atol=1e-12, rtol=0)
    np.testing.assert_array_equal(transport.v, np.zeros(14))
    np.testing.assert_array_equal(motion.last_v, np.zeros(7))


def test_lost_open_keeps_original_request_and_only_cleans_up(make_stream):
    rig = make_stream(LostOpen)
    motion, transport = rig.motion, rig.transport
    assert motion.open_pending and motion.stopping and motion.fault
    assert motion.publication()['session_id'] is None
    tick(rig, 20)
    assert not motion.complete and transport.status == 4
    assert not transport.batches and motion.proposed is None
    transport.available = True
    motion.state(0)
    assert motion.complete and motion.fault and not motion.open_pending
    assert transport.status == 0 and transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
    assert all(request == transport.requests[0] for request in transport.requests)
    assert not transport.batches and rig.source.rollouts == rig.source.validations == 0


@pytest.mark.parametrize('operation', ['REQUEST_STOP', 'FINALIZE'])
def test_lost_cleanup_response_reuses_request_and_never_generates_again(rig, monkeypatch, operation):
    acknowledge_start(rig)
    tick(rig, 12)
    motion, transport = rig.motion, rig.transport
    assert np.linalg.norm(transport.v) > 0
    close, requests, blocked = transport.close, [], True

    def lose_response(request):
        response = close(request)
        if request['operation'] == operation:
            requests.append(deepcopy(request))
            if blocked:
                raise TimeoutError('cleanup response lost after execution')
        return response

    monkeypatch.setattr(transport, 'close', lose_response)
    sequence, rollouts = motion.sequence, rig.source.rollouts
    motion.brake(0)
    motion.state(0)
    for _ in range(20):
        if requests:
            break
        tick(rig)
    assert requests and motion.stopping and not motion.complete
    blocked = False
    tick(rig, 10)
    assert motion.complete and transport.status == 0
    assert motion.sequence == sequence and rig.source.rollouts == rollouts
    assert len(requests) >= 2 and all(request == requests[0] for request in requests)
    assert transport.close_operations == ['REQUEST_STOP', 'FINALIZE']


def scalar_knots(polynomial, axis=7, times=(0., 1.)):
    """Known polynomials give independent q/v/a oracles for interpolation tests."""
    knots = []
    for at in times:
        q, v, a = np.zeros(14), np.zeros(14), np.zeros(14)
        q[axis], v[axis], a[axis] = (polynomial.deriv(d)(at) for d in range(3))
        knots.append(Knot(round(at * 1e9), q, v, a))
    return knots


def check_window(knots, *, lower=-10., upper=10., speed=1., acceleration=1., jerk=1.):
    return validate_window(knots, np.broadcast_to(lower, (14,)), np.broadcast_to(upper, (14,)),
                           np.broadcast_to(speed, (14,)), acceleration, jerk)


def test_validate_window_accepts_c2_cubic_and_returns_its_exact_derivatives():
    polynomial = Polynomial([.2, .006, -.015, .01])
    knots = scalar_knots(polynomial, times=(0., .23, .61, 1.))
    curve = check_window(knots, speed=.035, acceleration=.07, jerk=.28)
    times = np.array([0., .13, .37, .72, 1.])
    for derivative in range(4):
        np.testing.assert_allclose(curve(times, derivative)[:, 7], polynomial.deriv(derivative)(times), atol=1e-12)


@pytest.mark.parametrize('axis', [0, 7, 13])
@pytest.mark.parametrize('sign', [-1, 1])
def test_validate_window_rejects_interior_speed_peak_missed_by_five_samples(axis, sign):
    # v(t) = 1.0002 - .2*(t-.37)^2: all five usual samples are legal.
    polynomial = sign * Polynomial([0., 1.0002 - .2 * .37**2, .2 * .37, -.2 / 3])
    assert np.max(np.abs(polynomial.deriv()(np.linspace(0, 1, 5)))) < 1
    assert abs(polynomial.deriv()(.37)) > 1
    with pytest.raises(JogError) as error:
        check_window(scalar_knots(polynomial, axis))
    assert error.value.code == 'rolling_dynamics'


@pytest.mark.parametrize('sign', [-1, 1])
def test_validate_window_rejects_position_extremum_between_legal_knots(sign):
    polynomial = sign * Polynomial([1.0002 - .2 * .37**2, .4 * .37, -.2])
    assert np.max(np.abs(polynomial(np.linspace(0, 1, 5)))) < 1
    assert abs(polynomial(.37)) > 1
    with pytest.raises(JogError) as error:
        check_window(scalar_knots(polynomial), lower=-1., upper=1.)
    assert error.value.code == 'rolling_dynamics'


@pytest.mark.parametrize('acceleration,jerk', [(.05, 1.), (1., .05)])
def test_validate_window_rejects_acceleration_or_jerk_limit(acceleration, jerk):
    with pytest.raises(JogError) as error:
        check_window(scalar_knots(Polynomial([0., 0., 0., .01])), acceleration=acceleration, jerk=jerk)
    assert error.value.code == 'rolling_dynamics'


def test_validate_window_rejects_wrong_acceleration_metadata():
    knots = scalar_knots(Polynomial([0., .1]))
    knots[0].a[7] = .001
    with pytest.raises(JogError) as error:
        check_window(knots)
    assert error.value.code == 'rolling_seam'


def test_validate_window_rejects_c1_join_with_acceleration_jump():
    # Both segments match q/v at t=1; their accelerations there are -.2 and +.2.
    first = scalar_knots(Polynomial([0., 0., .2, -.1]))
    end = scalar_knots(Polynomial([.1, .1, .1, -.1]))[-1]
    end.time_ns = 2_000_000_000
    with pytest.raises(JogError) as error:
        check_window([*first, end])
    assert error.value.code == 'rolling_seam'


@pytest.mark.parametrize('sign', [-1, 1])
def test_validate_window_requires_braking_room_even_when_curve_is_within_limits(sign):
    polynomial = sign * Polynomial([.79, .2])
    assert np.max(np.abs(polynomial([0., 1.]))) < 1
    with pytest.raises(JogError) as error:
        check_window(scalar_knots(polynomial), lower=-1., upper=1., speed=.3)
    assert error.value.code == 'rolling_stopping_envelope'
    # The same legal velocity does fit when its final position leaves .05 room.
    check_window(scalar_knots(sign * Polynomial([.75, .2])), lower=-1., upper=1., speed=.3)


def test_validate_window_uses_synchronized_braking_time_for_all_axes():
    knots = scalar_knots(Polynomial([.86, .1]))
    knots[-1].q[13] = 1.
    for knot in knots:
        knot.v[13] = 1.
    upper = np.full(14, 2.)
    upper[7] = 1.
    # Axis 7 alone needs .005 room, but the shared one-second stop needs .05.
    assert knots[-1].q[7] + .1**2 / 2 < upper[7]
    with pytest.raises(JogError) as error:
        check_window(knots, upper=upper)
    assert error.value.code == 'rolling_stopping_envelope'
