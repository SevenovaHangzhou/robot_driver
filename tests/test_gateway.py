"""Two HTTP ports, actual upstream arm/jog handlers, and a real read-only observer.

Set JOG_DRIVER_WORKTREE to the reviewed robot_driver worktree. No ROS is used.
"""
import asyncio
import os
from pathlib import Path
import sys

from aiohttp import ClientSession, CookieJar, WSMsgType, web
from aiohttp.test_utils import TestClient, TestServer
import pytest

from gripper_jog.gateway import GatewayServer, COOKIE
from gripper_jog.model_assets import ModelAssets
from test_real_observe import observer

DRIVER = os.environ.get('JOG_DRIVER_WORKTREE')
if not DRIVER:
    pytest.skip('Set JOG_DRIVER_WORKTREE for the two-console integration tests', allow_module_level=True)
PACKAGE = Path(DRIVER) / 'src/rt_control/integ_operator_web'
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / 'test'))
from integ_operator_web.arm.jog import JogPage
from integ_operator_web.arm.model import load_arm_model
from test_arm_console import setup
from integ_operator_web.arm.console import ArmConsole
from test_arm_page import build, next_of, command


@pytest.fixture
def rig(observer, setup, tmp_path, monkeypatch):
    jog, sample, _ = observer
    jog.ingest(sample)
    original, driver, clock = setup
    console = ArmConsole(original.config, original.controller, driver, tmp_path / 'gateway-poses.json', clock)
    console.idle_probe = lambda _since: True
    console.set_model(load_arm_model(jog.planner.chain.source_xml,
                      [n for c in jog.chains.values() for n in c.names]))
    model = tmp_path / 'model.urdf'
    model.write_text(jog.planner.chain.source_xml)
    assets = ModelAssets(model, tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '78')
    app, arm = build(console)
    JogPage(arm).add_routes(app)
    app.router.add_post('/api/logout', arm._sessions.logout)
    return app, jog, assets, console, driver


def exercise(rig, scenario):
    app, jog, assets, console, driver = rig

    async def run():
        async with TestClient(TestServer(app)) as primary:
            gateway = GatewayServer(str(primary.make_url('/')).rstrip('/'), jog, assets, 78)
            async with TestClient(TestServer(gateway.build_app())) as client:
                await scenario(client, primary, gateway, console, driver)
    asyncio.run(run())


async def logged(client, page='0123456789abcdef'):
    assert (await client.post('/api/login', json={'token': 'secret'})).status == 200
    ws = await client.ws_connect('/ws/jog?client=' + page)
    await next_of(ws, 'auth_ok')
    await next_of(ws, 'status')
    return ws


def test_actual_proxy_login_mutual_exclusion_and_motion_closed(rig):
    async def scenario(client, primary, gateway, console, driver):
        assert (await client.get('/api/jog/state')).status == 401
        assert (await client.get('/api/jog/model')).status == 401
        assert (await client.get('/static/jog/gateway.mjs')).status == 200
        ws = await logged(client)
        state = await (await client.get('/api/jog/state?client=0123456789abcdef')).json()
        assert state['authority_available'] and state['control_authority']['verified']
        assert not state['can_arm'] and state['readonly']
        assert (await client.get('/api/jog/model')).status == 200
        cookie = client.session.cookie_jar.filter_cookies(client.make_url('/'))[COOKIE].value
        arm = await primary.ws_connect('/ws/arm', headers={'Cookie': COOKIE + '=' + cookie})
        await next_of(arm, 'status')
        assert (await command(ws, 'acquire'))['ok']
        assert (await command(arm, 'acquire'))['error'] == 'lease_busy'
        assert (await command(ws, 'velocity'))['error'] == 'jog_motion_disabled'
        assert (await client.post('/api/jog/arm', json={})).status == 409
        await command(ws, 'release')
        assert (await command(arm, 'acquire'))['ok']
        assert (await command(ws, 'acquire'))['error'] == 'lease_busy'
        assert not driver.sent
        await arm.close()
        await ws.close()
    exercise(rig, scenario)


@pytest.mark.parametrize('mismatch', ['domain', 'model', 'joints'])
def test_mismatched_runtime_never_acquires(rig, mismatch):
    async def scenario(client, primary, gateway, console, driver):
        if mismatch == 'domain':
            gateway.domain_id = 77
        elif mismatch == 'model':
            from dataclasses import replace
            console.model = replace(console.model, source_sha256='different')
        else:
            from dataclasses import replace
            console.model = replace(console.model, joints=console.model.joints[:-1])
        ws = await logged(client)
        result = await command(ws, 'acquire')
        assert result['error'] == 'gateway_runtime_mismatch'
        assert console.authority.owner() is None and not driver.sent
        state = await (await client.get('/api/jog/state?client=0123456789abcdef')).json()
        assert not state['authority_available'] and not state['can_arm']
    exercise(rig, scenario)


def test_no_cookie_bleed_and_logout_closes_primary_connection(rig):
    async def scenario(client, primary, gateway, console, driver):
        ws = await logged(client)
        await command(ws, 'acquire')
        async with ClientSession(cookie_jar=CookieJar(unsafe=True)) as other:
            url = str(client.make_url('/api/jog/model'))
            async with other.get(url) as reply:
                assert reply.status == 401
            async with other.post(str(client.make_url('/api/login')), json={'token': 'secret'}) as reply:
                assert reply.status == 200
            end = await other.ws_connect(str(client.make_url('/ws/jog?client=second-tab-012345')))
            await next_of(end, 'status')
            assert (await command(end, 'acquire'))['error'] == 'lease_busy'
            await client.post('/api/logout')
            for _ in range(100):
                message = await ws.receive(timeout=2)
                if message.type in (WSMsgType.CLOSE, WSMsgType.CLOSED):
                    break
            assert ws.close_code == 4401
            for _ in range(100):
                if console.authority.owner() is None:
                    break
                await asyncio.sleep(.01)
            for _ in range(100):
                if (await next_of(end, 'status'))['lease'] == 'none':
                    break
            acquired = await command(end, 'acquire')
            assert acquired['ok'], acquired
            await end.close()
        assert not driver.sent
    exercise(rig, scenario)


def test_cross_origin_bad_json_and_old_route_are_rejected(rig):
    async def scenario(client, primary, gateway, console, driver):
        assert (await client.post('/api/login', json={'token': 'secret'},
                                  headers={'Origin': 'http://evil.example'})).status == 403
        assert (await client.post('/api/login', data='{')).status == 400
        gateway.upstream = str(primary.make_url('/obsolete')).rstrip('/')
        assert (await client.get('/ws/jog?client=0123456789abcdef')).status == 502
        assert not gateway.connections and console.authority.owner() is None
        assert not driver.sent
    exercise(rig, scenario)


def test_connected_but_unreconciled_runtime_cannot_grant(rig):
    async def scenario(client, primary, gateway, console, driver):
        console.idle_probe = lambda _since: False
        ws = await logged(client)
        state = await (await client.get('/api/jog/state?client=0123456789abcdef')).json()
        assert not state['authority_available'] and not state['control_authority']['verified']
        assert (await command(ws, 'acquire'))['error'] == 'authority_unverified'
        assert console.authority.owner() is None and not driver.sent
    exercise(rig, scenario)
