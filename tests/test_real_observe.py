from copy import deepcopy
import json
import threading

import numpy as np
import pytest

from gripper_jog.errors import JogError
from gripper_jog.real_observe import ReadOnlyJog
from gripper_jog.server import MockServer
from test_http import request


@pytest.fixture
def observer(xml):
    # Two independent analytic chains; feedback order intentionally differs from
    # chain order, so a positional zip cannot accidentally pass this test.
    arms = []
    for side in ('left', 'right'):
        body = xml[xml.index('<link name="l1"'):xml.rindex('</robot>')]
        body = body.replace('name="l', f'name="{side}_link').replace('link="l', f'link="{side}_link')
        body = body.replace('name="j', f'name="{side}_joint')
        body = body.replace('link="base"', 'link="arm_carriage"')
        body += f'<link name="{side}_tool0"/><joint name="{side}_tip" type="fixed"><parent link="{side}_link7"/><child link="{side}_tool0"/></joint>'
        arms.append(body)
    model = ('<robot name="feedback_fixture"><link name="base_link"/><link name="arm_carriage"/>'
             '<joint name="mount" type="fixed"><parent link="base_link"/><child link="arm_carriage"/></joint>'
             '<link name="head"/><joint name="head_joint" type="revolute"><parent link="arm_carriage"/>'
             '<child link="head"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1"/></joint>'
             + ''.join(arms) + '</robot>')
    clock = [100.]
    jog = ReadOnlyJog(model, 78, clock=lambda: clock[0])
    positions = {f'{side}_joint{i}': i / 10 + (0 if side == 'left' else .1)
                 for side in ('right', 'left') for i in range(7, 0, -1)}
    sample = {'readonly': True, 'domain_id': 78, 'source_valid': True,
              'model_sha256': jog.planner.chain.model_hash,
              'observed': {'positions': positions, 'velocities': {name: .001 for name in positions}},
              'receive_age_s': {'positions': .02}, 'controllers': []}
    return jog, sample, clock


def test_actual_feedback_by_name_and_both_arms(observer):
    jog, sample, _ = observer
    assert jog.ingest(sample)
    state = jog.state('any', 'page')
    assert state['mode'] == 'real-observe' and state['hardware_connected']
    assert state['q_rad'] == pytest.approx(np.arange(1, 8) / 10)
    assert state['tcp_mm'] == pytest.approx([100, 200, 300])
    assert state['tcp_actual_speed_mm_s'] == pytest.approx(np.sqrt(3))
    assert state['unobserved_model_joints'] == ['head_joint']
    assert not state['armed'] and not state['can_arm']
    assert state['control_authority']['required'] is True
    assert state['control_authority']['verified'] is False
    jog.dispatch('select_arm', '', {'arm': 'right'})
    assert jog.state('', '')['tcp_mm'] == pytest.approx([200, 300, 400])
    assert jog.planner.chain.tip == 'right_tool0'


@pytest.mark.parametrize('action', ['acquire', 'release', 'arm', 'heartbeat', 'step', 'velocity', 'halt', 'reset'])
def test_every_control_action_rejected_even_if_client_claims_an_epoch(observer, action):
    jog, sample, _ = observer
    jog.ingest(sample)
    with pytest.raises(JogError) as error:
        jog.dispatch(action, 'session', {'client': 'page-000001', 'epoch': 'forged', 'seq': 1})
    assert error.value.code == 'real_readonly'
    assert jog.dispatch('stop', '', {}) == {'readonly': True, 'stopping': False}


def test_collected_model_preview_never_claims_live_feedback(observer, tmp_path):
    from gripper_jog.real_observe import prepare_local
    import yaml
    jog, _, _ = observer
    source = tmp_path / 'source'
    source.mkdir()
    (source / 'robot_description.xml').write_text(jog.chains['left'].source_xml)
    (source / 'controllers.yaml').write_text('controller_manager: {}')
    (source / 'artifact.yaml').write_text(yaml.safe_dump({
        'selection': {'use_mock_hardware': False},
        'files': ['robot_description.xml', 'controllers.yaml']}))
    preview, _, report, probe = prepare_local(source, tmp_path / 'preview', 78, source)
    state = preview.state('', '')
    assert state['model_only'] and state['readonly']
    assert not state['feedback_fresh'] and not state['hardware_connected'] and not state['can_arm']
    assert state['tcp_mm'] is None and 'q_rad' not in state
    assert report['runtime_running'] is False and probe is None
    with pytest.raises(JogError, match='只读'):
        preview.dispatch('velocity', '', {'speed_mm_s': .5})


@pytest.mark.parametrize('invalid', ['missing_axis', 'nan', 'domain', 'model', 'runtime', 'format'])
def test_invalid_feedback_freezes_last_valid_pose_and_marks_unavailable(observer, invalid):
    jog, sample, _ = observer
    assert jog.ingest(sample)
    pose = jog.state('', '')['tcp_mm']
    bad = deepcopy(sample)
    if invalid == 'missing_axis':
        del bad['observed']['positions']['left_joint2']
    elif invalid == 'nan':
        bad['observed']['positions']['left_joint2'] = float('nan')
    elif invalid == 'domain':
        bad['domain_id'] = 77
    elif invalid == 'model':
        bad['model_sha256'] = 'different-model'
    elif invalid == 'runtime':
        bad['source_valid'] = False
    else:
        bad = []
    assert not jog.ingest(bad)
    state = jog.state('', '')
    assert state['tcp_mm'] == pose
    assert not state['feedback_fresh'] and not state['hardware_connected'] and not state['can_arm']


def test_feedback_age_and_disconnect_never_present_frozen_pose_as_live(observer):
    jog, sample, clock = observer
    jog.ingest(sample)
    clock[0] += 1
    state = jog.state('', '')
    assert not state['feedback_fresh']
    assert 'tcp_actual_speed_mm_s' not in state
    assert jog.ingest(sample)
    jog.disconnected()
    assert not jog.state('', '')['feedback_fresh']


def test_http_observer_refuses_motion_and_can_switch_without_motion_lease(observer):
    jog, sample, _ = observer
    jog.ingest(sample)
    server = MockServer(('127.0.0.1', 0), jog)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        _, _, cookie = request(server, 'POST', '/api/login', {'token': 'mock'})
        for action in ('acquire', 'arm', 'step', 'velocity', 'halt'):
            status, raw, _ = request(server, 'POST', '/api/jog/' + action, {'speed_mm_s': .5}, cookie)
            assert status == 409 and json.loads(raw)['error'] == 'real_readonly'
        assert request(server, 'POST', '/api/jog/select_arm', {'arm': 'right'}, cookie)[0] == 200
        status, raw, _ = request(server, 'GET', '/api/jog/state', cookie=cookie)
        assert status == 200
        assert json.loads(raw)['selected_arm'] == 'right'
        assert json.loads(raw)['can_arm'] is False
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)
