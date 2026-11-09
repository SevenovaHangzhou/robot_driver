"""Real controller timestamps must not create near-zero Hermite intervals."""
from copy import deepcopy

import numpy as np

from gripper_jog.rolling import RollingMotion
from gripper_jog.rolling_mock import MockRollingTransport
from test_rolling import rolling


def test_window_with_one_ns_remainder_uses_controller_period(rolling):
    jog, _, clock, _ = rolling
    plan = jog.planner.step(jog.q, 'x+', .0005, require_collision=False)
    transport = MockRollingTransport(np.r_[jog.q + .1, jog.q], .07, clock)
    motion = RollingMotion(plan, transport, clock)
    captured = []
    transport.publish = lambda batch: captured.append(deepcopy(batch))
    start = motion.until
    motion._publish(start, start + 3 * motion.spacing + 1)
    times = np.array([p['time_from_session_start_ns'] for p in captured[0]['points']])
    assert times[-1] == start + 3 * motion.spacing
    assert np.all(np.diff(times) >= motion.period)
    assert not motion.fault


def test_irregular_ticks_finish_without_relaxing_dynamic_limits(rolling):
    jog, execution, clock, body = rolling
    jog.dispatch('step', 'session', body)
    for tick in range(100):
        clock.now += [.021999999, .018000002, .020000001][tick % 3]
        jog.tick()
        assert not execution.motion.fault
        if jog.armed:
            jog.dispatch('heartbeat', 'session', {**body, 'seq': tick + 100})
        if jog.active is None:
            break
    assert jog.active is None and execution.motion.complete
    assert execution.transport.close_operations == ['REQUEST_STOP', 'FINALIZE']
