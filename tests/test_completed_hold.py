from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.interpolate import CubicHermiteSpline

from gripper_jog.completed_hold import CompletedHold
from gripper_jog.real_source import RealRollingSource
from gripper_jog.rolling_stream import Knot
from test_real_collision import collision_model, artifact_model, observer  # noqa: F401


def completed(model, positions):
    target = np.array([positions[n] for n in model.names])
    stream = NS(epoch=3, sequence=17, finished=True, final_sent=True, pending=None,
                committed=[Knot(400_000_000, target, np.zeros(14), np.zeros(14))])
    snapshot = {'phase': 3, 'state_age_s': .01,
                'controller': {'epoch': 3, 'last_sequence': 17, 'state': 0,
                               'last_result': 0, 'stop_reason': 6}}
    return stream, snapshot


@pytest.mark.parametrize('side', ['left', 'right'])
def test_repeated_start_uses_the_completed_command_despite_feedback_offset(collision_model, side):
    model, target = collision_model
    entry = object()
    cache = CompletedHold(model.names)
    stream, snapshot = completed(model, target)
    cache.capture(entry, stream, snapshot)
    feedback = dict(target)
    # Even the frozen opposite arm has a tracking offset. It must hold its command.
    feedback['right_joint2' if side == 'left' else 'left_joint2'] += .0002
    held = cache.take(entry, {**snapshot, 'phase': 0})  # sender.reset() before begin reply
    source = RealRollingSource(model, feedback, side, held_positions=held)
    knots = [source.anchor, *source.rollout(source.anchor, [20_000_000, 40_000_000], [.02, 0., 0.])]
    source.validate(knots)
    times = [k.time_ns / 1e9 for k in knots]
    curve = CubicHermiteSpline(times, [k.q for k in knots], [k.v for k in knots])
    assert source.anchor.q == pytest.approx(stream.committed[-1].q, abs=0)
    assert np.max(np.abs(curve(np.linspace(0, .04, 41), 2)) - model.bounds[3]) <= 1e-8
    other = [i for i, n in enumerate(model.names) if not n.startswith(side + '_')]
    assert np.array_equal(knots[-1].q[other], stream.committed[-1].q[other])
    # The old measured anchor becomes a high-acceleration cubic when RT substitutes its hold.
    old = RealRollingSource(model, feedback, side)
    old_knots = [old.anchor, *old.rollout(old.anchor, [20_000_000, 40_000_000], [.02, 0., 0.])]
    old_knots[0] = stream.committed[-1]
    bad = CubicHermiteSpline(times, [k.q for k in old_knots], [k.v for k in old_knots])
    assert np.max(np.abs(bad(0, 2)) - model.bounds[3]) > 0
    assert cache.take(entry, snapshot) is None  # consume once; failures never reuse blindly


@pytest.mark.parametrize('change', ['epoch', 'sequence', 'rejection', 'moving', 'stop_reason',
                                   'stale', 'missing_age', 'missing_state', 'connection'])
def test_a_changed_or_unconfirmed_controller_cannot_reuse_the_target(collision_model, change):
    model, positions = collision_model
    stream, snapshot = completed(model, positions)
    cache, entry = CompletedHold(model.names), object()
    cache.capture(entry, stream, snapshot)
    changed = deepcopy(snapshot)
    if change in ('epoch', 'sequence', 'rejection', 'moving', 'stop_reason'):
        key = {'sequence': 'last_sequence', 'rejection': 'last_result', 'moving': 'state'}.get(change, change)
        changed['controller'][key] += 1
    elif change == 'stale':
        changed['state_age_s'] = .3
    elif change == 'missing_age':
        changed['state_age_s'] = None
    elif change == 'missing_state':
        changed['controller'] = None
    else:
        entry = object()
    assert cache.take(entry, changed) is None
    assert cache.take(entry, snapshot) is None


@pytest.mark.parametrize('change', ['pending', 'unfinished', 'not_final', 'rejected', 'nonzero_velocity'])
def test_only_an_acknowledged_completed_final_can_be_saved(collision_model, change):
    model, positions = collision_model
    stream, snapshot = completed(model, positions)
    if change == 'pending':
        stream.pending = 17
    elif change == 'unfinished':
        stream.finished = False
    elif change == 'not_final':
        stream.final_sent = False
    elif change == 'rejected':
        snapshot['controller']['last_result'] = 6
    else:
        stream.committed[-1].v[0] = .001
    cache = CompletedHold(model.names)
    cache.capture(object(), stream, snapshot)
    assert cache.saved is None


def test_actual_collision_still_blocks_restarting_from_a_clear_command(collision_model):
    model, target = collision_model
    measured = {**target, 'left_joint2': .1}
    with pytest.raises(Exception, match='碰撞余量'):
        RealRollingSource(model, measured, 'left', held_positions=target)
