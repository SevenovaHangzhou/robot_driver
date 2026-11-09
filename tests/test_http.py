import http.client
import json
import threading

import pytest

from gripper_jog.control import MockJog
from gripper_jog.server import MockServer


@pytest.fixture
def server(planner, q):
    instance = MockServer(('127.0.0.1', 0), MockJog(planner, q))
    worker = threading.Thread(target=instance.serve_forever, daemon=True)
    worker.start()
    yield instance
    instance.shutdown(); instance.server_close(); worker.join(timeout=2)


def request(server, method, path, body=None, cookie=None, origin=None):
    connection = http.client.HTTPConnection(*server.server_address, timeout=5)
    headers = {'Content-Type': 'application/json'}
    if cookie: headers['Cookie'] = cookie
    if origin: headers['Origin'] = origin
    connection.request(method, path, None if body is None else json.dumps(body), headers)
    response = connection.getresponse()
    status, raw, cookie = response.status, response.read(), response.getheader('Set-Cookie')
    connection.close()
    return status, raw, cookie


def test_login_origin_static_and_no_hardware_route(server):
    status, _, _ = request(server, 'GET', '/jog'); assert status == 200
    status, _, _ = request(server, 'GET', '/static/jog/../../config/mock.json'); assert status == 404
    status, _, _ = request(server, 'POST', '/api/jog/step', {}); assert status == 401
    status, _, cookie = request(server, 'POST', '/api/login', {'token': 'mock'}); assert status == 200
    status, body, _ = request(server, 'GET', '/api/jog/state?client=page-00000001', cookie=cookie)
    assert status == 200 and json.loads(body)['hardware_connected'] is False
    status, _, _ = request(server, 'POST', '/api/jog/acquire', {'client': 'page-00000001'}, cookie, 'http://evil.example')
    assert status == 403
    status, _, _ = request(server, 'POST', '/api/execute', {}, cookie); assert status == 404


def test_mock_http_command_lifecycle(server):
    _, _, cookie = request(server, 'POST', '/api/login', {'token': 'mock'})
    body = {'client': 'page-00000001'}
    status, _, _ = request(server, 'POST', '/api/jog/acquire', body, cookie); assert status == 200
    status, raw, _ = request(server, 'POST', '/api/jog/arm', body, cookie); assert status == 200
    body.update(epoch=json.loads(raw)['epoch'], seq=1)
    status, _, _ = request(server, 'POST', '/api/jog/heartbeat', body, cookie); assert status == 200
    status, _, _ = request(server, 'POST', '/api/jog/heartbeat', body, cookie); assert status == 409
    status, _, _ = request(server, 'POST', '/api/jog/stop', body, cookie); assert status == 200
    body.update(seq=2, step_mm=.5, direction='x+')
    status, raw, _ = request(server, 'POST', '/api/jog/step', body, cookie)
    assert status == 409 and json.loads(raw)['error'] == 'epoch_expired'


def test_model_endpoint_requires_login_and_matches_planning_source(server, tmp_path, xml):
    from gripper_jog.model_assets import ModelAssets
    source = tmp_path/'urdf/model.urdf'
    source.parent.mkdir()
    source.write_text(xml)
    server.model_assets = ModelAssets(source)
    for path in ('/api/jog/model', '/mesh/robot_description/missing.STL'):
        status, _, _ = request(server, 'GET', path)
        assert status == 401
    _, _, cookie = request(server, 'POST', '/api/login', {'token':'mock'})
    status, body, _ = request(server, 'GET', '/api/jog/model', cookie=cookie)
    assert status == 200
    info = json.loads(body)
    assert info['model_hash'] == server.jog.planner.chain.model_hash
    assert info['frame'] == server.jog.planner.chain.base
    status, _, _ = request(server, 'GET', '/mesh/../config/mock.json', cookie=cookie)
    assert status == 404
    for path in ('/static/jog/model-view.mjs', '/static/jog/diagnostics.mjs',
                 '/static/jog/gateway.mjs', '/static/vendor/three.module.min.js'):
        status, _, _ = request(server, 'GET', path)
        assert status == 200
