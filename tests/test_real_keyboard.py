import asyncio
import json
import time
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
import numpy as np
from scipy.interpolate import CubicHermiteSpline

from gripper_jog.errors import JogError
from gripper_jog.real_keyboard import RealKeyboard
from test_real_collision import collision_model, artifact_model, observer  # noqa: F401


@pytest.mark.parametrize('command', [dict(robot_direction='x+',speed_mm_s=.5),
    dict(robot_rotation='yaw+',rotation_frame='j7',speed_deg_s=1)])
def test_release_while_begin_is_pending_never_starts_a_trajectory(collision_model, observer, command):
    model, _ = collision_model
    jog, _, _ = observer
    keyboard = RealKeyboard(model, jog)
    entry = NS(compatible=True, received_at=time.monotonic(), status={'lease': 'you',
               'runtime_identity': {'joint_names': list(model.names)}})
    keyboard.entry, keyboard.epoch, keyboard.input_at = entry, 'input', time.monotonic()

    async def scenario():
        begin = asyncio.get_running_loop().create_future()
        calls = []

        async def rpc(kind, body):
            calls.append(kind)
            return await begin if kind == 'begin_motion' else {}

        moving = asyncio.create_task(keyboard.command('velocity', entry,
                    {'epoch': 'input', 'seq': 1, **command}, rpc))
        await asyncio.sleep(0)
        assert keyboard.starting
        await keyboard.command('halt', entry, {'epoch': 'input', 'seq': 2}, rpc)
        begin.set_result({})
        with pytest.raises(JogError, match='松键'):
            await moving
        assert keyboard.task is None and not keyboard.starting
        assert calls == ['begin_motion', 'stop']

    asyncio.run(scenario())


def test_release_and_stop_keep_the_browser_event_source(collision_model, observer, caplog):
    model, _ = collision_model
    keyboard = RealKeyboard(model, observer[0])
    entry = NS(compatible=True, received_at=time.monotonic(), status={'lease': 'you'})
    keyboard.entry, keyboard.epoch, keyboard.input_at = entry, 'input', time.monotonic()
    events = [{'type': 'keydown', 'code': 'ArrowUp', 'repeat': True, 'at_ms': 500},
              {'type': 'keyup', 'code': 'ArrowUp', 'repeat': False, 'at_ms': 501}]

    async def scenario():
        calls = []

        async def rpc(kind, body):
            calls.append(kind)
            return {}

        result = await keyboard.command('halt', entry,
            {'epoch': 'input', 'seq': 1, 'reason': '已松开方向键', 'input_events': events}, rpc)
        assert keyboard.last_input['action'] == 'halt'
        assert keyboard.last_input['events'] == events
        assert calls == []
        await keyboard.command('stop', entry,
            {'epoch': result['epoch'], 'reason': '页面失焦', 'input_events': events}, rpc)
        assert calls == ['stop'] and not keyboard.epoch
        assert keyboard.result == '页面失焦'
        assert keyboard.last_exit['source'] == 'browser_or_disconnect'
        assert keyboard.last_exit['input']['events'] == events
        json.dumps(keyboard.last_exit, allow_nan=False)
        assert 'jog_input' in caplog.text and 'jog_exit' in caplog.text

    asyncio.run(scenario())


def test_solver_failure_is_retained_after_the_browser_stop(collision_model, observer):
    model, positions = collision_model
    keyboard = RealKeyboard(model, observer[0])
    entry = NS(compatible=True, received_at=time.monotonic(), status={'lease': 'you'})
    keyboard.entry, keyboard.epoch, keyboard.input_at = entry, 'input', time.monotonic()

    async def scenario():
        calls = []

        async def fail(*args, **kwargs):
            raise JogError('mink_infeasible', '连续 Mink 求解没有可行解')

        async def rpc(kind, body):
            calls.append(kind)
            return {}

        keyboard.plan = fail
        begin = {'positions': positions, 'rolling': {'phase': 0}}
        await keyboard.run(entry, begin, rpc, time.monotonic())
        original = keyboard.last_exit
        assert original['source'] == 'planner' and original['stage'] == 'source'
        assert original['code'] == 'mink_infeasible'
        await keyboard.command('stop', entry, {'reason': '服务端已退出接管'}, rpc)
        assert keyboard.last_exit is original
        assert calls == ['stop', 'stop']
        json.dumps(keyboard.last_exit, allow_nan=False)

    asyncio.run(scenario())


def test_input_diagnostics_are_bounded_and_drop_unrelated_fields(collision_model, observer):
    keyboard = RealKeyboard(collision_model[0], observer[0])
    keyboard.record_input('halt', {'reason': 'x' * 500,
        'input_events': [{'type': 'keyup', 'code': 'ArrowUp', 'at_ms': float('inf'),
                          'unrelated': 'must not be recorded'}] * 100})
    assert len(keyboard.last_input['reason']) == 200
    assert len(keyboard.last_input['events']) == 24
    assert keyboard.last_input['events'][0] == {'type': 'keyup', 'code': 'ArrowUp'}
    json.dumps(keyboard.last_input, allow_nan=False)


def test_late_controller_diagnostic_enriches_only_the_stopped_run(collision_model, observer):
    keyboard = RealKeyboard(collision_model[0], observer[0])
    detail = {'epoch': 7, 'sequence': 8, 'result': 6, 'source': 'controller_admission',
              'axis': 0, 'joint': 'left_joint1', 'segment': 0, 'start_ns': 0, 'end_ns': 20_000_000,
              'quantity': 'acceleration_positive', 'value': 1.1, 'limit': 1.}
    rolling = {'controller': {'epoch': 7, 'last_sequence': 8, 'last_result': 6}, 'rejection_detail': detail}
    entry = NS(compatible=True, received_at=time.monotonic(), status={'lease': 'you', 'rolling': rolling})
    keyboard.entry = entry
    original = {'source': 'planner', 'message': '拒绝码 6', 'rolling': rolling, 'details': None}
    keyboard.last_exit = deepcopy(original)
    keyboard.epoch, keyboard.fault = 'next-run', ''
    assert keyboard.status(entry)['fault'] == ''
    assert keyboard.last_exit['details'] is None
    keyboard.epoch = ''
    state = keyboard.status(entry)
    assert 'left_joint1' in state['fault'] and keyboard.last_exit['details'] == detail
    assert keyboard.status(entry)['fault'].count('left_joint1') == 1
    keyboard.last_exit = deepcopy(original)
    rolling['rejection_detail'] = {**detail, 'sequence': 9}
    keyboard.status(entry)
    assert keyboard.last_exit['details'] is None


def test_expired_planning_window_is_stopped_without_publication(collision_model, observer):
    model, positions = collision_model
    keyboard = RealKeyboard(model, observer[0])
    now = time.monotonic()
    snapshot = {'phase': 0, 'state_age_s': .01, 'ros_time_ns': 1_000_000_000,
                'controller': {'epoch': 1, 'last_sequence': 0, 'last_result': 0}}
    begin = {'positions': positions, 'parameters': {**model.parameters,
             'replace_lead_ms': 4, 'max_horizon_ms': 600, 'buffer_capacity': 64}, 'rolling': snapshot}
    entry = NS(compatible=True, received_at=now, status={'lease': 'you'})
    keyboard.entry, keyboard.epoch, keyboard.input_at = entry, 'input', now

    async def scenario():
        calls = []
        original = keyboard.plan

        async def slow(function, *args, **kwargs):
            result = await original(function, *args, **kwargs)
            if function.__name__ == 'propose':
                await asyncio.sleep(.14)
            return result

        async def rpc(kind, body):
            calls.append(kind)
            return {}

        keyboard.plan = slow
        await keyboard.run(entry, begin, rpc, now)
        assert calls == ['stop']
        assert keyboard.last_exit['code'] == 'rolling_late'
        assert keyboard.last_exit['details']['source'] == 'client_timing'
        assert not keyboard.epoch and keyboard.task is None

    asyncio.run(scenario())


def test_an_expired_input_cannot_be_renewed_by_a_delayed_heartbeat(collision_model, observer):
    model, _ = collision_model
    keyboard = RealKeyboard(model, observer[0])
    entry = NS(compatible=True, received_at=time.monotonic(), status={'lease': 'you'})
    keyboard.entry, keyboard.epoch, keyboard.input_at = entry, 'input', time.monotonic() - 1
    with pytest.raises(JogError, match='失效'):
        keyboard.require(entry, {'epoch': 'input', 'seq': 1, '_action': 'heartbeat'})
    assert not keyboard.epoch


@pytest.mark.parametrize('old_phase', [4, 5, 6, 7])
@pytest.mark.parametrize('controller_reference', [False, True])
def test_new_begin_does_not_reuse_previous_trajectory_status(collision_model, observer, old_phase, controller_reference):
    model, positions = collision_model
    keyboard = RealKeyboard(model, observer[0])
    now = time.monotonic()
    old = {'phase': old_phase, 'state_age_s': .01, 'ros_time_ns': 1_000_000_000,
           'controller': {'epoch': 1, 'state': 0, 'last_sequence': 13, 'last_result': 0, 'stop_reason': 0}}
    begin = {'positions': positions, 'parameters': {**model.parameters,
             'replace_lead_ms': 4, 'max_horizon_ms': 600, 'buffer_capacity': 64},
             'rolling': {**old, 'phase': 0, 'controller': {**old['controller'], 'epoch': 2, 'last_sequence': 0}}}
    if controller_reference:
        begin['hold_reference'] = {'positions': {n: positions[n] for n in model.names}}
        begin['positions'] = {**positions, 'right_joint2': positions['right_joint2'] + .0002}
    entry = NS(compatible=True, received_at=now, status={'lease': 'you', 'rolling': old,
               'runtime_identity': {'joint_names': list(model.names)}})
    keyboard.entry, keyboard.epoch, keyboard.input_at = entry, 'input', now

    async def scenario():
        published = []

        async def rpc(kind, body):
            if kind == 'begin_motion':
                return begin
            if kind == 'window':
                published.append(body['window'])
                raise asyncio.CancelledError  # No motor transport; stop after inspecting the first window.
            return {}

        await keyboard.command('velocity', entry,
            {'epoch': 'input', 'seq': 1, 'robot_direction': 'x+', 'speed_mm_s': .5}, rpc)
        task = keyboard.task
        try:
            await task
        except asyncio.CancelledError:
            pass
        assert len(published) == 1, keyboard.fault
        assert keyboard.fault == ''
        assert len(published[0]['points']) == 21
        if controller_reference:
            assert keyboard.anchor_source == 'controller_hold'
            assert published[0]['points'][0]['positions'] == [positions[n] for n in model.names]

    asyncio.run(scenario())


def test_three_same_direction_restarts_keep_the_completed_hold(collision_model, observer):
    model, positions = collision_model
    keyboard = RealKeyboard(model, observer[0])
    entry = NS(compatible=True, received_at=time.monotonic(), status={'lease': 'you',
               'runtime_identity': {'joint_names': list(model.names)}})
    held = np.array([positions[n] for n in model.names])
    parameters = {**model.parameters, 'replace_lead_ms': 4,
                  'max_horizon_ms': 600, 'buffer_capacity': 64}
    current = {'phase': 0, 'state_age_s': 0., 'last_sent_sequence': 0, 'controller':
               {'epoch': 3, 'state': 0, 'last_sequence': 0, 'last_result': 0, 'stop_reason': 0}}
    sequence, final_until = 0, None
    starts = []

    async def scenario():
        nonlocal sequence, final_until, held
        finished = False

        async def feedback():
            while not finished:
                now = time.monotonic()
                if final_until is not None and round(now * 1e9) >= final_until:
                    current['phase'] = 3
                    current['controller'].update(state=0, stop_reason=6)
                current['ros_time_ns'] = round(now * 1e9)
                entry.status['rolling'] = deepcopy(current)
                entry.received_at = keyboard.input_at = now
                await asyncio.sleep(.01)

        async def rpc(kind, body):
            nonlocal sequence, final_until, held
            if kind == 'begin_motion':
                current['phase'] = 0
                measured = dict(zip(model.names, held))
                if starts:
                    measured['right_joint2'] += .0002
                starts.append(held.copy())
                return {'positions': measured, 'parameters': parameters,
                        'rolling': {**deepcopy(current), 'ros_time_ns': round(time.monotonic() * 1e9)}}
            if kind == 'window':
                window = body['window']
                points = window['points']
                if keyboard.stream.sequence == 0:
                    assert np.array_equal(points[0]['positions'], held)
                    assert points[0]['velocities'] == [0.] * 14
                times = [p['time_from_start']['sec'] + p['time_from_start']['nanosec'] / 1e9
                         for p in points]
                curve = CubicHermiteSpline(times, [p['positions'] for p in points],
                                          [p['velocities'] for p in points])
                assert np.max(np.abs(curve(np.linspace(0, times[-1], 81), 2)) - model.bounds[3]) < 1e-8
                sequence += 1
                current['phase'] = 2
                current['last_sent_sequence'] = sequence
                current['controller'].update(last_sequence=sequence, state=1, stop_reason=0)
                if window['final']:
                    final_until = (window['replace_from']['sec'] * 10**9
                                   + window['replace_from']['nanosec'] + round(times[-1] * 1e9))
                    held = np.array(points[-1]['positions'])
                else:
                    keyboard.finishing = True  # operator release; the real stop rollout is exercised
                return {'send_result': 0, 'rolling':
                        {**deepcopy(current), 'ros_time_ns': round(time.monotonic() * 1e9)}}
            raise AssertionError('unexpected stop or transport error: ' + kind)

        watcher = asyncio.create_task(feedback())
        try:
            for cycle in range(3):
                final_until = None
                keyboard.entry, keyboard.epoch = entry, 'input'
                keyboard.input_at = entry.received_at = time.monotonic()
                await keyboard.command('velocity', entry,
                    {'epoch': 'input', 'seq': cycle + 1, 'robot_direction': 'x+', 'speed_mm_s': 20}, rpc)
                await asyncio.wait_for(keyboard.task, timeout=2)
                assert not keyboard.fault, keyboard.fault
                assert keyboard.completed_hold.saved is not None
                assert keyboard.anchor_source == ('measured_feedback' if cycle == 0 else 'completed_final')
        finally:
            finished = True
            await watcher

    asyncio.run(scenario())
