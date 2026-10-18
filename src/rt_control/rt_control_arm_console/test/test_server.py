"""HTTP front: login, same-origin, status stream and command routing. No ROS."""

import http.client
import json
from pathlib import Path
import sys
import threading

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from rt_control_arm_console.server import ConsoleServer  # noqa: E402
from test_console import JOINTS, setup  # noqa: E402, F401

WEB = Path(__file__).resolve().parents[1] / "web"


@pytest.fixture
def server(setup):  # noqa: F811
    console, driver, _ = setup
    instance = ConsoleServer(
        ("127.0.0.1", 0),
        console,
        "secret",
        WEB,
        lambda _: "/nonexistent",
        lambda enabled: {"accepted": True, "enabled": enabled, "message": ""},
    )
    thread = threading.Thread(target=instance.serve_forever, daemon=True)
    thread.start()
    yield instance, console, driver
    instance.stopping.set()
    instance.shutdown()
    instance.server_close()


def request(server, method, path, body=None, cookie=None, origin=None):
    connection = http.client.HTTPConnection(*server.server_address, timeout=5)
    headers = {"Content-Type": "application/json"}
    if cookie:
        headers["Cookie"] = cookie
    if origin:
        headers["Origin"] = origin
    connection.request(method, path, json.dumps(body) if body is not None else None, headers)
    response = connection.getresponse()
    return response, response.read()


def login(server):
    response, _ = request(server, "POST", "/api/login", {"token": "secret"})
    assert response.status == 200
    return response.getheader("Set-Cookie").split(";")[0]


def test_commands_require_login_and_same_origin(server):
    instance, _, driver = server
    response, _ = request(instance, "POST", "/api/stop", {})
    assert response.status == 401
    response, _ = request(instance, "GET", "/api/info")
    assert response.status == 401
    cookie = login(instance)
    response, _ = request(instance, "POST", "/api/stop", {}, cookie, origin="http://evil.example")
    assert response.status == 403
    response, body = request(instance, "GET", "/static/../server.py", cookie=cookie)
    assert response.status == 404


def test_info_events_and_execute(server):
    instance, console, driver = server
    cookie = login(instance)
    response, body = request(instance, "GET", "/api/info", cookie=cookie)
    info = json.loads(body)
    assert [joint["name"] for joint in info["joints"]] == JOINTS
    assert info["urdf"].startswith("<robot")
    connection = http.client.HTTPConnection(*instance.server_address, timeout=5)
    connection.request("GET", "/api/events", headers={"Cookie": cookie})
    stream = connection.getresponse()
    line = stream.fp.readline().decode()
    status = json.loads(line[len("data: ") :])
    assert status["positions"]["left_joint2"] == pytest.approx(0.2)
    connection.close()
    # The fixture lease belongs to another session: this browser must acquire first.
    response, body = request(
        instance,
        "POST",
        "/api/execute",
        {"target": {"left_joint1": 0.4, "left_joint2": 0.2}, "gear": "leisure"},
        cookie,
    )
    assert response.status == 409 and json.loads(body)["error"] == "lease_required"
    console.release("s1")
    response, _ = request(instance, "POST", "/api/acquire", {}, cookie)
    assert response.status == 200
    response, body = request(
        instance,
        "POST",
        "/api/execute",
        {"target": {"left_joint1": 0.4, "left_joint2": 0.2}, "gear": "leisure"},
        cookie,
    )
    assert response.status == 200, body
    assert len(driver.sent) == 1
    response, body = request(instance, "POST", "/api/enable", {"enabled": True}, cookie)
    assert response.status == 409  # moving


def test_static_assets_are_served_from_a_symlink_install(setup, tmp_path):  # noqa: F811
    console, _, _ = setup
    web = tmp_path / "share/web"
    web.mkdir(parents=True)
    for name in ("index.html", "style.css"):
        (web / name).symlink_to(WEB / name)  # what colcon --symlink-install produces
    (tmp_path / "secret.txt").write_text("outside")
    instance = ConsoleServer(
        ("127.0.0.1", 0),
        console,
        "secret",
        web,
        lambda _: "/nonexistent",
        lambda enabled: {"accepted": True, "enabled": enabled, "message": ""},
    )
    thread = threading.Thread(target=instance.serve_forever, daemon=True)
    thread.start()
    try:
        response, body = request(instance, "GET", "/static/style.css")
        assert response.status == 200 and body == (WEB / "style.css").read_bytes()
        response, _ = request(instance, "GET", "/static/../../secret.txt")
        assert response.status == 404
    finally:
        instance.stopping.set()
        instance.shutdown()
        instance.server_close()


def test_run_sequence_route_reports_console_checks(server):
    instance, console, driver = server
    console.release("s1")  # the shared fixture holds the lease for session "s1"
    cookie = login(instance)
    response, _ = request(instance, "POST", "/api/acquire", {}, cookie)
    assert response.status == 200
    response, body = request(
        instance, "POST", "/api/run_sequence", {"names": ["没有"], "gear": "leisure"}, cookie
    )
    assert response.status == 409 and "第 1 个" in json.loads(body)["message"]
    assert not driver.sent


def test_audit_log_records_commands_without_secrets(setup, tmp_path):  # noqa: F811
    from rt_control_arm_console.audit import AuditLog

    console, _, _ = setup
    path = tmp_path / "audit" / "audit.jsonl"
    instance = ConsoleServer(
        ("127.0.0.1", 0),
        console,
        "secret",
        WEB,
        lambda _: "/nonexistent",
        lambda enabled: {"accepted": False, "enabled": False, "message": "drives faulted"},
        AuditLog(path),
    )
    thread = threading.Thread(target=instance.serve_forever, daemon=True)
    thread.start()
    try:
        response, _ = request(instance, "POST", "/api/login", {"token": "wrong"})
        assert response.status == 401
        cookie = login(instance)
        request(instance, "POST", "/api/stop", {}, cookie)
        console.release("s1")
        request(instance, "POST", "/api/acquire", {}, cookie)
        response, _ = request(
            instance, "POST", "/api/enable", {"enabled": True, "extra": "x"}, cookie
        )
        assert response.status == 409
    finally:
        instance.stopping.set()
        instance.shutdown()
        instance.server_close()
    entries = [json.loads(line) for line in path.read_text().splitlines()]
    assert [(e["action"], e["ok"]) for e in entries] == [
        ("login", False),
        ("login", True),
        ("stop", True),
        ("acquire", True),
        ("enable", False),
    ]
    assert entries[-1]["args"] == {"enabled": True} and entries[-1]["error"] == "enable_rejected"
    assert entries[-1]["message"] == "drives faulted"
    text = path.read_text()
    assert "secret" not in text and cookie.split("=", 1)[1] not in text
    assert len(entries[1]["session"]) == 8 and entries[1]["session"] == entries[2]["session"]
