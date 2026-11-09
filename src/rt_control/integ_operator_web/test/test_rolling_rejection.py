from pathlib import Path
import sys
from types import SimpleNamespace as NS

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.arm.rolling_rejection import RollingRejections


def report(**changes):
    values = dict(epoch=7, sequence=2, result=6, joint='left_joint2', axis=1, segment=3,
                  start_ns=60_000_000, end_ns=80_000_000, quantity='acceleration_positive', value=1.1, limit=1.)
    values.update(changes)
    return DiagnosticArray(status=[DiagnosticStatus(name='/rolling/rejection',
        values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])])


@pytest.mark.parametrize('bad', [{}, {'axis': 14}, {'value': 'nan'}, {'quantity': 'guess'}, {'start_ns': 80_000_000}])
def test_rejection_evidence_is_validated_and_matched_to_exact_public_state(bad):
    rejection = RollingRejections(NS(create_subscription=lambda *a, **kw: None), 'rolling', None)
    rejection.receive(report(**bad))
    state = dict(epoch=7, last_sequence=2, last_result=6)
    names = ['left_joint1', 'left_joint2']
    evidence = rejection.matching(state, names)
    if bad:
        assert evidence is None
        return
    assert evidence['source'] == 'controller_admission' and evidence['value'] == 1.1
    for key in ('epoch', 'last_sequence', 'last_result'):
        assert rejection.matching({**state, key: state[key] + 1}, names) is None
    assert rejection.matching(state, list(reversed(names))) is None
    rejection.received_at -= 3.
    assert rejection.matching(state, names) is None


@pytest.mark.parametrize('reason', ['snapshot_busy', 'state_not_running', 'epoch_changed', 'snapshot_publish_failed'])
def test_admission_reason_survives_only_the_matching_sequence(reason):
    rejection = RollingRejections(NS(create_subscription=lambda *a, **kw: None), 'rolling', None)
    message = DiagnosticArray(status=[DiagnosticStatus(name='/rolling/rejection', values=[
        KeyValue(key=k, value=str(v)) for k, v in dict(epoch=7, sequence=5, result=9, reason=reason).items()])])
    rejection.receive(message)
    state = dict(epoch=7, last_sequence=5, last_result=9)
    assert rejection.matching(state, [])['reason'] == reason
    assert rejection.matching({**state, 'last_sequence': 6}, []) is None
