"""Default startup must not depend on or call the optional static library."""
import os
from pathlib import Path

import numpy as np
import pytest

from gripper_jog.mink_sim import build_mink


@pytest.mark.parametrize('rolling', [False, True])
def test_default_does_not_load_or_call_static_gravity(rolling, monkeypatch, tmp_path):
    root = Path(os.environ.get('JOG_TEST_REAL2SIM', Path(__file__).resolve().parents[3] / 'sim/real2sim_v3'))
    if not (root / 'real2sim/mink_demo.py').is_file():
        pytest.skip('Set JOG_TEST_REAL2SIM to the real2sim_v3 reference')

    def forbidden(*args, **kwargs):
        pytest.fail('Default startup called the optional gravity library')

    monkeypatch.setattr('gripper_jog.gravity.load_static_gravity', forbidden)
    jog, _ = build_mink(root, autostart=False, rolling=rolling,
                        gravity_source=tmp_path / 'missing', gravity_cache=tmp_path / 'cache')
    try:
        for _ in range(5):
            jog._cycle()
        state = jog.state('test', 'test')['gravity_compensation']
        assert state['mode'] == 'off' and not state['enabled']
        assert jog.backend.calculator is None and state['parameter_signature'] is None
        np.testing.assert_array_equal(state['torques_nm'], np.zeros(14))
        assert not (tmp_path / 'cache').exists()
    finally:
        jog.close()
