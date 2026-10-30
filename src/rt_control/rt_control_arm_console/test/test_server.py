"""HTTP front: login, same-origin, status stream and command routing. No ROS."""

import http.client
import json
from pathlib import Path
import sys
import threading
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from rt_control_arm_console.server import ConsoleServer  # noqa: E402
from test_console import JOINTS, setup  # noqa: E402, F401
from test_console import URDF  # noqa: E402

WEB = Path(__file__).resolve().parents[1] / "web"


def test_selected_view_model_serves_its_meshes_and_preserves_live_limits(setup, tmp_path):
    console, driver, _ = setup
    live_model = console.model
    share = tmp_path / "description"
    (share / "urdf").mkdir(parents=True)
    (share / "meshes").mkdir()
    robot = ET.fromstring(URDF.replace("package://pkg/", "package://robot_description/"))
    robot.find("joint[@name='left_joint1']/limit").set("upper", "2.0")
    ET.SubElement(robot.find("joint[@name='left_joint1']"), "axis", xyz="0 0 -1")
    (share / "urdf/robot_alfav3_mobile.urdf").write_text(ET.tostring(robot, encoding="unicode"))
    (share / "meshes/mount.stl").write_bytes(b"latest mesh")
    instance = ConsoleServer(
        ("127.0.0.1", 0), console, "secret", WEB, lambda _: "/nonexistent",
        lambda _: {}, view_model_share=share,
    )
    thread = threading.Thread(target=instance.serve_forever, daemon=True)
    thread.start()
    try:
        path = instance.view_mesh_prefix + "robot_description/meshes/mount.stl"
        response, _ = request(instance, "GET", path)
        assert response.status == 401
        cookie = login(instance)
        response, body = request(instance, "GET", "/api/info", cookie=cookie)
        assert response.status == 200
        info = json.loads(body)
        view = ET.fromstring(info["urdf"])
        assert view.find("joint[@name='left_joint1']/axis").get("xyz") == "0 0 -1"
        assert info["joints"] == console.static_info()["joints"]
        assert console.model is live_model
        assert info["view_model"]["urdf_sha256"]
        mesh = view.find("link[@name='mount']/visual/geometry/mesh").get("filename")
        assert mesh == path and mesh.endswith(".stl")
        response, body = request(instance, "GET", mesh, cookie=cookie)
        assert response.status == 200 and body == b"latest mesh"
        response, body = request(
            instance, "POST", "/api/preview",
            {"target": {"left_joint1": 1.5, "left_joint2": 0.2}, "gear": "leisure"}, cookie,
        )
        assert response.status == 409 and json.loads(body)["error"] == "target_out_of_range"
        assert not driver.sent
        # The selected view remains available when the runtime has not supplied a model.
        console.model = None
        response, body = request(instance, "GET", "/api/info", cookie=cookie)
        assert response.status == 200
        offline = json.loads(body)
        assert offline["joints"] == [] and offline["arm_links"] == []
        assert ET.fromstring(offline["urdf"]).find("link[@name='mount']") is not None
        assert offline["view_model"] == info["view_model"]
        assert not driver.sent
    finally:
        instance.stopping.set()
        instance.shutdown()
        instance.server_close()


def test_selected_view_model_does_not_fall_back_when_joint_is_missing(setup, tmp_path):
    console, _, _ = setup
    (tmp_path / "urdf").mkdir()
    (tmp_path / "urdf/robot_alfav3_mobile.urdf").write_text("<robot name='missing'/>")
    instance = ConsoleServer(
        ("127.0.0.1", 0), console, "secret", WEB, lambda _: "/nonexistent",
        lambda _: {}, view_model_share=tmp_path,
    )
    thread = threading.Thread(target=instance.serve_forever, daemon=True)
    thread.start()
    try:
        cookie = login(instance)
        response, body = request(instance, "GET", "/api/info", cookie=cookie)
        assert response.status == 503
        assert json.loads(body)["error"] == "view_model_unavailable"
    finally:
        instance.stopping.set()
        instance.shutdown()
        instance.server_close()


def test_mesh_manifest_change_gets_a_new_browser_cache_namespace(setup, tmp_path):
    console, _, _ = setup
    (tmp_path / "urdf").mkdir()
    (tmp_path / "config").mkdir()
    (tmp_path / "urdf/robot_alfav3_mobile.urdf").write_text(URDF)
    namespaces = []
    for version in ("old mesh hash", "new mesh hash"):
        (tmp_path / "config/mesh_assets.json").write_text(json.dumps({"mesh": version}))
        instance = ConsoleServer(
            ("127.0.0.1", 0), console, "secret", WEB, lambda _: "/nonexistent",
            lambda _: {}, view_model_share=tmp_path,
        )
        namespaces.append(instance.view_mesh_prefix)
        instance.server_close()
    assert namespaces[0] != namespaces[1]


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


def test_fault_reset_requires_the_lease_and_reports_enable_manager_stage(setup, tmp_path):  # noqa: F811
    from rt_control_arm_console.audit import AuditLog

    console, _, _ = setup
    replies = [
        {"ok": True, "stage": "success", "failed_joint": "", "status_word": 0},
        {"ok": False, "stage": "restart_required", "failed_joint": "", "status_word": 0},
    ]
    calls = []

    def reset_fault():
        calls.append(1)
        return replies[len(calls) - 1]

    path = tmp_path / "audit.jsonl"
    instance = ConsoleServer(
        ("127.0.0.1", 0),
        console,
        "secret",
        WEB,
        lambda _: "/nonexistent",
        lambda enabled: {"accepted": True, "enabled": enabled, "message": ""},
        AuditLog(path),
        reset_fault,
    )
    thread = threading.Thread(target=instance.serve_forever, daemon=True)
    thread.start()
    try:
        cookie = login(instance)
        response, body = request(instance, "POST", "/api/reset_fault", {}, cookie)
        assert response.status == 409 and not calls  # the fixture lease belongs to session "s1"
        console.release("s1")
        request(instance, "POST", "/api/acquire", {}, cookie)
        response, body = request(instance, "POST", "/api/reset_fault", {}, cookie)
        assert response.status == 200 and json.loads(body)["stage"] == "success"
        response, body = request(instance, "POST", "/api/reset_fault", {}, cookie)
        reply = json.loads(body)
        assert response.status == 409 and reply["error"] == "reset_rejected"
        assert "restart_required" in reply["message"] and "arm restart" in reply["message"]
    finally:
        instance.stopping.set()
        instance.shutdown()
        instance.server_close()
    actions = [
        (e["action"], e["ok"])
        for e in map(json.loads, path.read_text().splitlines())
        if e["action"] == "reset_fault"
    ]
    assert actions == [("reset_fault", False), ("reset_fault", True), ("reset_fault", False)]


def test_fault_reset_is_unavailable_without_a_backend(server):
    instance, console, _ = server
    cookie = login(instance)
    console.release("s1")
    request(instance, "POST", "/api/acquire", {}, cookie)
    response, body = request(instance, "POST", "/api/reset_fault", {}, cookie)
    assert response.status == 409 and json.loads(body)["error"] == "reset_unavailable"


@pytest.mark.parametrize('dwell', [2, 8])
def test_sequence_dwell_http_route_and_selector(server, dwell):
    instance, console, driver = server
    console.save_pose('s1', 'here', overwrite=False)
    console.release('s1')
    jobs = []
    console._spawn = jobs.append
    cookie = login(instance)
    assert request(instance, 'POST', '/api/acquire', {}, cookie)[0].status == 200
    response, body = request(instance, 'POST', '/api/run_sequence',
                             {'names': ['here'], 'gear': 'leisure', 'dwell_s': dwell}, cookie)
    assert response.status == 200
    assert console._sequence['dwell_s'] == dwell
    assert console._sequence['wait_until'] is not None
    assert not driver.sent  # already there; it must hold without submitting a move
    response, body = request(instance, 'GET', '/')
    assert response.status == 200 and b'id="sequence-dwell"' in body
    assert b'type="number" min="0" step="0.1"' in body
    assert request(instance, 'POST', '/api/stop', {}, cookie)[0].status == 200
    jobs.pop()()
    response, body = request(instance, 'POST', '/api/run_sequence',
                             {'names': ['here'], 'gear': 'leisure', 'dwell_s': '2'}, cookie)
    assert response.status == 409 and json.loads(body)['error'] == 'bad_dwell'
    assert not driver.sent


def test_per_step_sequence_http_route(server):
    instance, console, driver = server
    console.save_pose("s1", "here", overwrite=False)
    console.release("s1")
    jobs = []
    console._spawn = jobs.append
    cookie = login(instance)
    assert request(instance, "POST", "/api/acquire", {}, cookie)[0].status == 200
    steps = [{"name": "here", "gear": "leisure", "dwell_s": 0.4},
             {"name": "here", "gear": "sport", "dwell_s": 1.2}]
    response, body = request(instance, "POST", "/api/run_sequence", {"steps": steps}, cookie)
    assert response.status == 200
    assert json.loads(body)["total"] == 2
    assert console.status(next(iter(instance.sessions)))["sequence"]["steps"] == steps
    assert not driver.sent
    assert request(instance, "POST", "/api/stop", {}, cookie)[0].status == 200
    jobs.pop()()
