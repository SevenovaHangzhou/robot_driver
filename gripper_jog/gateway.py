"""Jog webpage with authentication and arm authority from one primary console.

An optional keyboard planner sends windows through that console's /ws/jog gate.
The observer entry has no motion transport.
"""
import asyncio
from dataclasses import dataclass, field
import json
from pathlib import Path
import re
import time
from urllib.parse import urlsplit

from aiohttp import ClientError, ClientSession, ClientTimeout, DummyCookieJar, WSMsgType, web

from .errors import JogError
from .server import HEADERS, ROOT

COOKIE = 'integ_operator_session'
PROTOCOL = 'shared_arm_authority_v1'


@dataclass
class Connection:
    authenticated: bool = False
    status: dict = field(default_factory=dict)
    received_at: float = -float('inf')
    compatible: bool = False
    upstream: object = None
    pending: dict = field(default_factory=dict)


class GatewayServer:
    def __init__(self, upstream, jog, assets, domain_id, *, web_dir=ROOT / 'web', keyboard=None):
        url = urlsplit(upstream)
        if url.scheme not in ('http', 'https') or not url.hostname or url.username or url.password or \
                url.path not in ('', '/') or url.query or url.fragment:
            raise ValueError('Gateway must be an HTTP(S) origin without credentials or a path')
        self.upstream, self.jog, self.assets = upstream.rstrip('/'), jog, assets
        self.domain_id, self.web_dir = domain_id, Path(web_dir)
        self.connections = {}
        self.sockets = set()
        self.keyboard = keyboard
        self.rpc_sequence = 0

    def build_app(self):
        app = web.Application(middlewares=[self.headers], client_max_size=4096)
        app.router.add_get('/', self.index)
        app.router.add_get('/jog', self.index)
        app.router.add_get('/api/jog/config', self.config)
        app.router.add_post('/api/login', self.login)
        app.router.add_post('/api/logout', self.logout)
        app.router.add_get('/ws/jog', self.websocket)
        app.router.add_get('/api/jog/state', self.state)
        app.router.add_get('/api/jog/model', self.model)
        app.router.add_get('/mesh/{path:.+}', self.mesh)
        app.router.add_post('/api/jog/{action}', self.command)
        app.router.add_static('/static/jog/', self.web_dir, show_index=False)
        app.router.add_static('/static/vendor/', self.web_dir / 'vendor', show_index=False)
        app.cleanup_ctx.append(self.client)
        app.on_shutdown.append(self.shutdown)
        return app

    async def shutdown(self, _app):
        await asyncio.gather(*(ws.close(code=1001) for ws in list(self.sockets)))

    @web.middleware
    async def headers(self, request, handler):
        origin = request.headers.get('Origin')
        if origin and origin != f'{request.scheme}://{request.host}':
            raise web.HTTPForbidden(text='cross-origin request rejected')
        try:
            response = await handler(request)
        except (ClientError, asyncio.TimeoutError):
            response = web.json_response({'error': 'primary_unavailable',
                'message': '主操作台连接失败，控制权未确认'}, status=502)
        except (json.JSONDecodeError, UnicodeDecodeError):
            response = web.json_response({'error': 'bad_json'}, status=400)
        for key, value in HEADERS.items():
            if key != 'Content-Type':
                response.headers.setdefault(key, value)
        return response

    async def client(self, _app):
        self.http = ClientSession(timeout=ClientTimeout(total=5), cookie_jar=DummyCookieJar(), trust_env=False)
        yield
        await self.http.close()

    async def rpc(self, entry, kind, body):
        self.rpc_sequence += 1
        identity = f'planner-{self.rpc_sequence}'
        future = asyncio.get_running_loop().create_future()
        entry.pending[identity] = future
        try:
            await entry.upstream.send_json({'type': kind, 'id': identity, **body})
            result = await asyncio.wait_for(future, timeout=15 if kind == 'begin_motion' else 2)
            if not result.get('ok'):
                raise JogError(result.get('error', 'primary_rejected'), result.get('message', '主操作台拒绝请求'),
                               details=result.get('details'))
            return result
        finally:
            entry.pending.pop(identity, None)

    def ingest_feedback(self, entry, payload):
        if not self.keyboard or not entry.compatible:
            return
        positions, age = payload.get('positions'), payload.get('feedback_age_s')
        if not isinstance(positions, dict) or age is None:
            return
        self.jog.ingest({'readonly': True, 'domain_id': self.domain_id, 'source_valid': True,
            'model_sha256': payload['runtime_identity']['model_sha256'],
            'observed': {'positions': positions, 'rolling': (payload.get('rolling') or {}).get('controller')},
            'receive_age_s': {'positions': age}})

    def cookie(self, request):
        value = request.cookies.get(COOKIE, '')
        return {COOKIE: value} if re.fullmatch(r'[A-Za-z0-9_-]{16,128}', value) else {}

    def connection(self, request, *, tab=True):
        cookie = request.cookies.get(COOKIE, '')
        client = request.query.get('client', '')
        entries = [entry for (session, page), entry in self.connections.items()
                   if session == cookie and (not tab or page == client)]
        return next((entry for entry in entries if entry.authenticated), None)

    def require_login(self, request, *, tab=False):
        entry = self.connection(request, tab=tab)
        if entry is None:
            raise web.HTTPUnauthorized(text=json.dumps({'error': 'login_required'}),
                                      content_type='application/json')
        return entry

    async def login(self, request):
        body = await request.json()
        async with self.http.post(self.upstream + '/api/login', json=body,
                                  allow_redirects=False) as response:
            result = web.Response(status=response.status, body=await response.read(),
                                  content_type='application/json')
            for cookie in response.headers.getall('Set-Cookie', []):
                result.headers.add('Set-Cookie', cookie)
            return result

    async def logout(self, request):
        async with self.http.post(self.upstream + '/api/logout', cookies=self.cookie(request),
                                  allow_redirects=False) as response:
            result = web.Response(status=response.status, body=await response.read(),
                                  content_type='application/json')
            for cookie in response.headers.getall('Set-Cookie', []):
                result.headers.add('Set-Cookie', cookie)
            return result

    def compatible(self, status):
        identity = status.get('runtime_identity', {})
        authority = status.get('control_authority', {})
        names = {n for c in self.jog.chains.values() for n in c.names}
        return (status.get('protocol') == PROTOCOL and
                authority.get('integrated') is True and
                authority.get('resource') == 'dual_arms' and
                authority.get('phase') in ('unverified', 'idle', 'leased', 'moving', 'stopping') and
                status.get('lease') in ('none', 'you', 'other') and
                identity.get('domain_id') == self.domain_id and
                identity.get('model_sha256') == self.jog.planner.chain.model_hash and
                set(identity.get('joint_names', [])) == names)

    async def websocket(self, request):
        client = request.query.get('client', '')
        if not re.fullmatch(r'[A-Za-z0-9_-]{16,128}', client):
            raise web.HTTPBadRequest(text='invalid page identity')
        key = (request.cookies.get(COOKIE, ''), client)
        if key in self.connections:
            raise web.HTTPConflict(text='page connection already exists')
        ws = web.WebSocketResponse(heartbeat=3, max_msg_size=16384)
        # Establish upstream first: a missing route must never fall back to local ownership.
        try:
            upstream = await self.http.ws_connect(self.upstream + '/ws/jog',
                headers={'Cookie': '; '.join(f'{key}={value}' for key, value in self.cookie(request).items())},
                heartbeat=3, max_msg_size=256000)
        except Exception:
            raise web.HTTPBadGateway(text='shared authority gateway unavailable') from None
        await ws.prepare(request)
        entry = self.connections[key] = Connection(upstream=upstream)

        self.sockets.add(ws)

        async def receive():
            try:
                async for message in upstream:
                    if message.type != WSMsgType.TEXT:
                        break
                    payload = json.loads(message.data)
                    if not isinstance(payload, dict):
                        break
                    if payload.get('type') == 'auth_ok':
                        if payload.get('protocol') != PROTOCOL or payload.get('entry') != 'jog':
                            break
                        entry.authenticated = True
                    if payload.get('type') == 'status':
                        entry.status, entry.received_at = payload, time.monotonic()
                        entry.compatible = self.compatible(payload)
                        self.ingest_feedback(entry, payload)
                    if payload.get('type') == 'result' and payload.get('id') in entry.pending:
                        future = entry.pending[payload['id']]
                        if not future.done():
                            future.set_result(payload)
                        continue
                    await ws.send_str(message.data)
            except (ClientError, ConnectionError, ValueError):
                pass
            finally:
                entry.authenticated = False
                for future in entry.pending.values():
                    if not future.done():
                        future.set_exception(ConnectionError('primary disconnected'))
                await ws.close(code=upstream.close_code or 1011)

        receiver = asyncio.create_task(receive())
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    break
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    await ws.close(code=1003)
                    break
                kind = payload.get('type')
                fresh = time.monotonic() - entry.received_at <= .75
                if kind == 'acquire' and not (entry.authenticated and entry.compatible and fresh):
                    await ws.send_json({'type': 'result', 'id': payload.get('id'), 'ok': False,
                        'error': 'gateway_runtime_mismatch',
                        'message': '主操作台的 Domain、模型或双臂关节与观测来源不一致'})
                else:
                    await upstream.send_str(message.data)
        except (ValueError, ClientError, ConnectionError):
            await ws.close(code=1003)
        finally:
            self.sockets.discard(ws)
            entry.authenticated = False
            if self.keyboard and self.keyboard.entry is entry:
                try:
                    await self.keyboard.stop(lambda kind, body: self.rpc(entry, kind, body), '页面连接已关闭')
                except Exception:
                    pass
            self.connections.pop(key, None)
            await upstream.close()
            receiver.cancel()
            try:
                await receiver
            except (asyncio.CancelledError, ConnectionResetError):
                pass
        return ws

    async def config(self, _request):
        return web.json_response({'authority_gateway': True, 'primary_console': self.upstream,
                                  'keyboard_configured': self.keyboard is not None})

    async def index(self, _request):
        return web.FileResponse(self.web_dir / 'jog.html')

    async def state(self, request):
        entry = self.require_login(request, tab=True)
        state = self.keyboard.status(entry) if self.keyboard else self.jog.state('', '')
        fresh = time.monotonic() - entry.received_at <= .75
        ready = bool(entry.compatible and fresh)
        authority = entry.status.get('control_authority', {})
        available = ready and authority.get('phase') != 'unverified'
        state.update(lease=entry.status.get('lease', 'none') if ready else 'none',
                     authority_gateway=True, authority_available=available,
                     control_authority={**authority, 'verified': available,
                                        'primary_console': self.upstream})
        if ready:
            state['blockers'] = [b for b in state['blockers'] if '尚未接入同一个双臂控制权仲裁' not in b]
            if not self.keyboard:
                state['result'] = '统一双臂控制权已连接；末端运动仍关闭'
        if available:
            state['blockers'] = [b for b in state['blockers']
                                 if '共享控制权尚未现场验证' not in b]
        if ready and not available:
            state['blockers'].insert(0, '双臂执行状态尚未完成启动对账')
        if not ready:
            state['blockers'].insert(0, '统一控制权状态过期，或主操作台与当前观测栈不一致')
        return web.json_response(state)

    async def model(self, request):
        self.require_login(request)
        return web.json_response(self.assets.info(self.jog.planner.chain))

    async def mesh(self, request):
        self.require_login(request)
        path = self.assets.meshes.get(request.match_info['path'])
        if path is None:
            raise web.HTTPNotFound()
        return web.FileResponse(path)

    async def command(self, request):
        self.require_login(request)
        try:
            body = await request.json()
            if not isinstance(body, dict):
                return web.json_response({'error': 'bad_request'}, status=400)
            if self.keyboard:
                key = (request.cookies.get(COOKIE, ''), body.get('client', ''))
                entry = self.connections.get(key)
                if entry is None or not entry.authenticated:
                    raise web.HTTPUnauthorized()
                result = await self.keyboard.command(request.match_info['action'], entry, body,
                    lambda kind, payload: self.rpc(entry, kind, payload))
            else:
                result = self.jog.dispatch(request.match_info['action'], '', body)
            return web.json_response({'ok': True, **result})
        except JogError as exc:
            return web.json_response({'error': exc.code, 'message': str(exc), 'details': exc.details}, status=409)
