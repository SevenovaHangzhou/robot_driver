"""The full model vocabulary must not become an implicit execution group."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_control_scope_and_home_cover_only_declared_axes():
    scope = yaml.safe_load((ROOT/'config/motion_scope.yaml').read_text())
    home = yaml.safe_load((ROOT/scope['home_resource']).read_text())
    model = ET.parse(ROOT/scope['model_entry']).getroot()
    joints = {j.get('name'): j for j in model.findall('joint') if j.get('type') != 'fixed'}
    controlled, other = scope['controlled_joints'], scope['state_only_joints']
    assert len(controlled) == len(set(controlled)) == 15
    assert len(other) == len(set(other)) == 10
    assert not set(controlled) & set(other)
    assert set(controlled) | set(other) == set(joints)
    assert set(home['positions']) == set(controlled)
    assert home['model_revision'] == scope['model_revision']
    assert scope['missing_state_policy'] == 'require_explicit_state'
    for name, value in home['positions'].items():
        assert math.isfinite(value)
        limit = joints[name].find('limit')
        assert float(limit.get('lower')) <= value <= float(limit.get('upper'))
    assert home['positions']['updown'] == -.3
    for side, degrees in [('left',[65,-75,20,-90,0,-40,90]), ('right',[-65,75,-20,90,0,40,-90])]:
        for index,value in enumerate(degrees,1):
            assert math.isclose(home['positions'][f'{side}_joint{index}'],math.radians(value),abs_tol=1e-12)
