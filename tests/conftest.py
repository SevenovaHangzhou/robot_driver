from pathlib import Path
import os

import numpy as np
import pytest

from gripper_jog.model import Chain
from gripper_jog.planner import Planner


@pytest.fixture
def xml():
    # Analytic Cartesian wrist with one redundant rotation, not a copy of a robot model.
    axes = ['1 0 0', '0 1 0', '0 0 1', '1 0 0', '0 1 0', '0 0 1', '0 0 1']
    parts = ['<robot name="analytic_fixture"><link name="base"/>']
    for i, axis in enumerate(axes):
        parent = 'base' if i == 0 else f'l{i}'
        parts.append(f'''<link name="l{i+1}"/><joint name="j{i+1}" type="{'prismatic' if i < 3 else 'revolute'}">
          <parent link="{parent}"/><child link="l{i+1}"/><axis xyz="{axis}"/>
          <limit lower="-2" upper="2" velocity="1"/></joint>''')
    parts.append('</robot>')
    return ''.join(parts)


@pytest.fixture
def chain(xml):
    return Chain(xml, 'base', 'l7')


@pytest.fixture
def q():
    return np.array([.2, .3, .4, .1, .2, -.1, .05])


@pytest.fixture
def planner(chain):
    return Planner(chain)


@pytest.fixture
def actual_chain():
    source = os.environ.get('JOG_TEST_URDF')
    if not source:
        pytest.skip('Set JOG_TEST_URDF to test against the external authoritative model')
    return Chain(Path(source).read_text(), 'arm_carriage', 'left_tool0')
