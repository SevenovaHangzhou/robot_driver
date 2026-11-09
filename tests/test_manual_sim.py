from argparse import Namespace
from pathlib import Path
from types import SimpleNamespace

import pytest

from gripper_jog.errors import JogError
from gripper_jog.manual_sim import (
    JOINT_NAMES, MOCK_PLUGIN, ManualSimulation, RosExecution,
    child_environment, instance_lock, verify_mock_artifact,
)


def artifact(tmp_path, plugins, names=JOINT_NAMES):
    controls = ''.join(f'<ros2_control><hardware><plugin>{p}</plugin></hardware>'
                       + ''.join(f'<joint name="{n}"/>' for n in names)
                       + '</ros2_control>' for p in plugins)
    (tmp_path / 'robot_description.xml').write_text(f'<robot>{controls}</robot>')
    return tmp_path


def test_generated_mock_accepts_only_complete_dual_arm(tmp_path):
    verify_mock_artifact(artifact(tmp_path, [MOCK_PLUGIN]))
    with pytest.raises(RuntimeError, match='拒绝使能'):
        verify_mock_artifact(artifact(tmp_path, [MOCK_PLUGIN], JOINT_NAMES[:-1]))


@pytest.mark.parametrize('plugins', [[], ['robot_hw_ethercat/EthercatSystem'],
                                    [MOCK_PLUGIN, 'robot_hw_ethercat/EthercatSystem']])
def test_generated_real_or_mixed_hardware_is_rejected(tmp_path, plugins):
    with pytest.raises(RuntimeError, match='拒绝使能'):
        verify_mock_artifact(artifact(tmp_path, plugins))


def test_duplicate_manual_instance_fails_without_replacing_lock(tmp_path):
    with instance_lock(tmp_path):
        with pytest.raises(RuntimeError, match='已有手动联调'):
            with instance_lock(tmp_path):
                pytest.fail('second instance acquired the lock')
    with instance_lock(tmp_path):
        pass


def test_child_keeps_ros_but_removes_client_cffi_path():
    client = Path('/web/.venv/lib/python3.10/site-packages')
    env = {'PYTHONPATH': f'{client}:/ros:/usr/lib/python3/dist-packages', 'DISPLAY': ':0'}
    result = child_environment(env, client)
    assert result == {'PYTHONPATH': '/ros', 'DISPLAY': ':0', 'ROS_LOCALHOST_ONLY': '1'}
    assert str(client) in env['PYTHONPATH']


@pytest.mark.parametrize('desired', [[0.] * 13, [1.] + [0.] * 13])
def test_changed_or_incomplete_anchor_never_opens_motion(desired):
    transport = SimpleNamespace(state=lambda: {'desired_positions': desired})
    execution = RosExecution(transport)
    with pytest.raises(JogError, match='起点'):
        execution.start(None, dict.fromkeys(JOINT_NAMES, 0.))
    assert execution.motion is None


def test_unfinished_session_is_not_replaced():
    execution = RosExecution(None)
    motion = execution.motion = SimpleNamespace(complete=False)
    with pytest.raises(JogError, match='上一会话'):
        execution.start(None, dict.fromkeys(JOINT_NAMES, 0.))
    assert execution.motion is motion


def test_finalize_response_before_idle_state_does_not_revoke_keyboard():
    execution = RosExecution(None)
    identity = {'controller_boot_id': {'uuid': [1] * 16}, 'client_instance_id': {'uuid': [2] * 16}}
    execution.motion = SimpleNamespace(complete=True, identity=identity)
    delayed_state = {'has_session': True, **identity}
    assert not execution.foreign_session(delayed_state)
    assert execution.foreign_session({**delayed_state, 'client_instance_id': {'uuid': [3] * 16}})
    assert not execution.foreign_session({'has_session': False})


def test_cleanup_continues_after_web_stop_failure(tmp_path):
    runtime = ManualSimulation(Namespace(port=18132), tmp_path)
    called = []
    def fail():
        raise RuntimeError('stop failed')
    runtime.server = SimpleNamespace(server_close=fail)
    runtime.contexts = [SimpleNamespace(shutdown=lambda: called.append('context closed'))]
    runtime.close()
    assert called == ['context closed']
    assert runtime.stage == 'failed'
    assert 'stop failed' in runtime.errors[0]
    assert (tmp_path / 'status.json').is_file()


def test_interrupted_startup_wait_does_not_call_next_action(tmp_path):
    runtime = ManualSimulation(Namespace(port=18132), tmp_path)
    runtime.stop.set()
    with pytest.raises(InterruptedError):
        runtime.wait(lambda: pytest.fail('startup continued after stop'))
