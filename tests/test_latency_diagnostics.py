"""Timing advice and exact-cubic evidence without a motion transport."""
import json
import math

import numpy as np
import pytest

from gripper_jog.errors import JogError
from gripper_jog.failure import controller_error
from gripper_jog.latency import Latency
from gripper_jog.rolling_stream import Knot, validate_window


def test_statistics_are_bounded_and_a_single_stall_prevents_shorter_lead_advice():
    timing = Latency()
    for _ in range(300):
        for name in ('window_plan', 'window_rpc', 'ack', 'loop_interval'):
            timing.add(name, 5.)
    for invalid in (math.inf, math.nan, -1.):
        timing.add('ack', invalid)
    stats = timing.snapshot()['ack']
    assert stats == {'count': 300, 'retained': 256, 'last': 5., 'max': 5., 'p50': 5., 'p95': 5., 'p99': 5.}
    advice = timing.assessment({'replace_lead_ms': 4}, 'baseline')
    assert advice['trial_fits_observed_budget'] and not advice['automatic_change']
    assert (advice['initial_lead_ms'], advice['replacement_lead_ms']) == (120, 100)
    timing.add('ack', 100.)
    assert timing.snapshot()['ack']['p99'] == 5.
    assert not timing.assessment({'replace_lead_ms': 4}, 'baseline')['trial_fits_observed_budget']
    json.dumps(timing.snapshot(), allow_nan=False)


def test_incomplete_samples_never_recommend_a_timing_trial():
    timing = Latency()
    for _ in range(29):
        for name in ('window_plan', 'window_rpc', 'ack', 'loop_interval'):
            timing.add(name, 1.)
    assert not timing.assessment({'replace_lead_ms': 4}, 'baseline')['sample_ready']
    assert not timing.assessment({}, 'baseline')['sample_ready']


@pytest.mark.parametrize('quantity', ['acceleration', 'speed', 'position_upper', 'stopping_upper', 'seam_acceleration', 'jerk'])
def test_exact_interior_limits_report_joint_interval_value_and_limit(quantity):
    q0, q1, v0, v1 = (np.zeros(14) for _ in range(4))
    # q(t)=3t²-2t³ on joint 5, t in [0,1]. Interior peak speed = 1.5.
    q1[5] = 1.
    a0, a1 = np.zeros(14), np.zeros(14)
    a0[5], a1[5] = 6., -6.
    knots = [Knot(200_000_000, q0, v0, a0), Knot(1_200_000_000, q1, v1, a1)]
    lower, upper, speed, accel, jerk = -100., 100., 100., 100., 100.
    if quantity == 'acceleration': accel = 5.
    if quantity == 'speed': speed = 1.4
    if quantity == 'position_upper': upper = .9
    if quantity == 'stopping_upper': upper, accel = 1.05, 10.
    if quantity == 'seam_acceleration': knots[1].a[5] = 0.
    if quantity == 'jerk': jerk = 11.
    with pytest.raises(JogError) as caught:
        validate_window(knots, lower, upper, speed, accel, jerk,
                        joint_names=[f'joint_{i}' for i in range(14)])
    d = caught.value.details
    assert d['joint'] == 'joint_5' and d['axis'] == 5 and d['segment'] == 0
    assert d['quantity'] == quantity and d['start_ns'] == 200_000_000 and d['end_ns'] == 1_200_000_000
    assert math.isfinite(d['value']) and math.isfinite(d['limit'])
    assert 'joint_5' in str(caught.value) and 'ms' in str(caught.value)
    json.dumps(d, allow_nan=False)


def test_controller_evidence_requires_exact_epoch_sequence_and_result():
    detail = {'source': 'controller_admission', 'epoch': 3, 'sequence': 8, 'result': 6,
              'joint': 'left_joint2', 'axis': 1, 'segment': 2, 'start_ns': 40_000_000,
              'end_ns': 60_000_000, 'quantity': 'acceleration_positive', 'value': 1.1, 'limit': 1.}
    snapshot = {'controller': {'epoch': 3, 'last_sequence': 8, 'last_result': 6}, 'rejection_detail': detail}
    error = controller_error('rolling_rejected', '拒绝码 6', snapshot)
    assert error.details == detail and 'left_joint2' in str(error) and '1.1' in str(error)
    for field in ('epoch', 'sequence', 'result'):
        wrong = {**snapshot, 'rejection_detail': {**detail, field: detail[field] + 1}}
        error = controller_error('rolling_rejected', '拒绝码 6', wrong)
        assert error.details is None and 'left_joint2' not in str(error)


def test_nonfinite_acceleration_is_rejected_before_detailed_extrema_checks():
    knots = [Knot(0, np.zeros(14), np.zeros(14), np.zeros(14)),
             Knot(20_000_000, np.zeros(14), np.zeros(14), np.zeros(14))]
    knots[1].a[0] = np.nan
    with pytest.raises(JogError) as caught:
        validate_window(knots, -1., 1., 1., 1., np.inf)
    assert caught.value.code == 'rolling_seam'


def test_running_admission_failure_is_explained_without_inventing_joint_values():
    detail = {'source': 'controller_admission', 'epoch': 7, 'sequence': 5, 'result': 9,
              'reason': 'snapshot_busy'}
    snapshot = {'controller': {'epoch': 7, 'last_sequence': 5, 'last_result': 9}, 'rejection_detail': detail}
    error = controller_error('rolling_rejected', 'Rolling 拒绝未来窗口：9', snapshot)
    assert error.details == detail and '快照' in str(error)
    assert 'joint' not in error.details
