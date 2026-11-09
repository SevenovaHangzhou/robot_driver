from copy import deepcopy

import numpy as np
import pytest

from gripper_jog.current_stream import CurrentStream
from gripper_jog.errors import JogError
from gripper_jog.real_source import RealRollingSource
from test_real_collision import collision_model, artifact_model, observer  # noqa: F401


PARAMETERS = {'replace_lead_ms': 4, 'max_horizon_ms': 600, 'buffer_capacity': 64}


def snapshot(sequence=0, phase=2, result=0):
    return {'controller': {'epoch': 7, 'state': 1, 'last_sequence': sequence,
                            'last_result': result, 'stop_reason': 0},
            'state_age_s': .01, 'phase': phase, 'last_sent_sequence': sequence}


@pytest.mark.parametrize('lead_ns', [100_000_000, 80_000_000])
@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direction', ([.0005, 0, 0], [0, .0005, 0], [0, 0, .0005]))
def test_current_continuous_windows_preserve_seams_and_stop(collision_model, side, direction, lead_ns):
    model, positions = collision_model
    source = RealRollingSource(model, positions, side)
    stream = CurrentStream(source, PARAMETERS, 1_000_000_000, lead_ns=lead_ns)
    stream.acknowledge(snapshot())
    seq = 0
    now = 880_000_000
    for _ in range(12):
        old = list(stream.committed)
        window = stream.propose(now, np.array(direction))
        assert len(window['points']) == 21 and not window['final']
        assert stream.propose(now, np.array(direction)) is None
        seq += 1
        stream.sent({'send_result': 0, 'rolling': snapshot(seq)})
        assert stream.pending == seq
        stream.acknowledge(snapshot(seq))
        if old:
            start = window['replace_from']['sec'] * 10**9 + window['replace_from']['nanosec'] - stream.origin
            before = next(k for k in old if k.time_ns == start)
            after = next(k for k in stream.committed if k.time_ns == start)
            for key in ('q', 'v', 'a'):
                assert np.array_equal(getattr(before, key), getattr(after, key))
        assert len(stream.committed) < 64
        now += 40_000_000
    while not stream.final_sent:
        window = stream.propose(now, np.zeros(3), finishing=True)
        seq += 1
        stream.sent({'send_result': 0, 'rolling': snapshot(seq)})
        stream.acknowledge(snapshot(seq))
        now += 40_000_000
    assert window['points'][-1]['velocities'] == [0.] * 14
    stream.acknowledge(snapshot(seq, phase=3))
    assert stream.finished and stream.replacements >= 12
    other = [i for i, n in enumerate(model.names) if not n.startswith(side + '_')]
    assert np.array_equal(stream.committed[-1].q[other], source.initial[other])


@pytest.mark.parametrize('fault', ['stale', 'epoch', 'rejected', 'foreign', 'interrupted'])
def test_uncertain_or_rejected_window_never_becomes_an_anchor(collision_model, fault):
    model, positions = collision_model
    stream = CurrentStream(RealRollingSource(model, positions, 'left'), PARAMETERS, 10**9)
    stream.acknowledge(snapshot())
    stream.propose(880_000_000, np.array([.0005, 0, 0]))
    stream.sent({'send_result': 0, 'rolling': snapshot(1)})
    bad = snapshot(1)
    if fault == 'stale':
        bad['state_age_s'] = .3
    elif fault == 'epoch':
        bad['controller']['epoch'] += 1
    elif fault == 'rejected':
        bad['controller']['last_result'] = 4
    elif fault == 'foreign':
        bad['controller']['last_sequence'] = 2
    else:
        bad['phase'] = 7
    with pytest.raises(JogError):
        stream.acknowledge(bad)
    assert stream.committed == [] and stream.pending == 1
