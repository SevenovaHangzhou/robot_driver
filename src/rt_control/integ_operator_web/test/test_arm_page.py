"""Arm page: login, same-origin, model/mesh GETs, WebSocket status and command routing. No ROS.

Ported from rt_control_arm_console's HTTP/SSE server tests; the console underneath is unchanged.
"""

import asyncio
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from aiohttp import WSMsgType, web
from aiohttp.test_utils import TestClient, TestServer
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from integ_operator_web.arm.console import ConsoleError  # noqa: E402
from integ_operator_web.arm.page import ArmPage  # noqa: E402
from integ_operator_web.audit import AuditLog  # noqa: E402
from integ_operator_web.operators import Operators  # noqa: E402
from integ_operator_web.session import Sessions  # noqa: E402
from test_arm_console import JOINTS, URDF, setup  # noqa: E402, F401

WEB = Path(__file__).resolve().parents[1] / "web/arm"
TARGET = {"target": {"left_joint1": 0.4, "left_joint2": 0.2}, "gear": "leisure"}


def build(console, **options):
    sessions = Sessions("secret", failure_delay=0.0)
    operators = options.pop("operators", Operators())
    page = ArmPage(
        console,
        sessions,
        operators,
        options.pop("web", WEB),
        lambda _: "/nonexistent",
        options.pop(
            "set_enabled", lambda enabled: {"accepted": True, "enabled": enabled, "message": ""}
        ),
        status_period=0.01,
        **options,
    )
    app = web.Application()
    app.router.add_post("/api/login", sessions.login)
    page.add_routes(app)
    return app, page


def run(app, scenario):
    async def main():
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            await scenario(client)
        finally:
            await client.close()

    asyncio.run(main())


async def login(client, token="secret"):
    response = await client.post("/api/login", json={"token": token})
    return response.status


async def connect(client):
    ws = await client.ws_connect("/ws/arm")
    status = await next_of(ws, "status")
    return ws, status


async def next_of(ws, kind, timeout=2.0):
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while True:
        message = await ws.receive_json(timeout=max(0.01, deadline - loop.time()))
        if message["type"] == kind:
            return message


async def command(ws, kind, **body):
    command.counter = getattr(command, "counter", 0) + 1
    await ws.send_json({"type": kind, "id": command.counter, **body})
    while True:
        message = await next_of(ws, "result")
        if message["id"] == command.counter:
            return message


def test_view_model_meshes_and_live_limits(setup, tmp_path):
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
    app, page = build(console, view_model_share=share)
    path = page._view_prefix + "robot_description/meshes/mount.stl"

    async def scenario(client):
        assert (await client.get(path)).status == 401
        assert await login(client) == 200
        response = await client.get("/api/arm/info")
        info = await response.json()
        view = ET.fromstring(info["urdf"])
        assert view.find("joint[@name='left_joint1']/axis").get("xyz") == "0 0 -1"
        assert info["joints"] == console.static_info()["joints"]
        assert console.model is live_model and info["view_model"]["urdf_sha256"]
        mesh = view.find("link[@name='mount']/visual/geometry/mesh").get("filename")
        assert mesh == path
        response = await client.get(mesh)
        assert response.status == 200 and await response.read() == b"latest mesh"
        assert response.headers["Cache-Control"] == "private, max-age=3600"
        ws, _ = await connect(client)
        target = {"target": {"left_joint1": 1.5, "left_joint2": 0.2}, "gear": "leisure"}
        assert (await command(ws, "preview", **target))["error"] == "target_out_of_range"
        assert not driver.sent
        # The selected view remains available when the runtime has not supplied a model.
        console.model = None
        offline = await (await client.get("/api/arm/info")).json()
        assert offline["joints"] == [] and offline["arm_links"] == []
        assert offline["view_model"] == info["view_model"]

    run(app, scenario)


def test_view_model_without_the_joints_does_not_fall_back(setup, tmp_path):
    console, _, _ = setup
    (tmp_path / "urdf").mkdir()
    (tmp_path / "urdf/robot_alfav3_mobile.urdf").write_text("<robot name='missing'/>")
    app, _ = build(console, view_model_share=tmp_path)

    async def scenario(client):
        await login(client)
        response = await client.get("/api/arm/info")
        assert response.status == 503
        assert (await response.json())["error"] == "view_model_unavailable"

    run(app, scenario)


def test_mesh_manifest_change_gets_a_new_browser_cache_namespace(setup, tmp_path):
    console, _, _ = setup
    (tmp_path / "urdf").mkdir()
    (tmp_path / "config").mkdir()
    (tmp_path / "urdf/robot_alfav3_mobile.urdf").write_text(URDF)
    namespaces = []
    for version in ("old mesh hash", "new mesh hash"):
        (tmp_path / "config/mesh_assets.json").write_text(json.dumps({"mesh": version}))
        namespaces.append(build(console, view_model_share=tmp_path)[1]._view_prefix)
    assert namespaces[0] != namespaces[1]


def test_everything_requires_login_and_same_origin(setup):
    console, driver, _ = setup
    app, _ = build(console)

    async def scenario(client):
        assert (await client.get("/api/arm/info")).status == 401
        assert (await client.get("/api/arm/vacuum")).status == 401
        ws = await client.ws_connect("/ws/arm")
        await ws.send_json({"type": "stop", "id": 1})
        closing = await ws.receive(timeout=2)
        assert closing.type in (WSMsgType.CLOSE, WSMsgType.CLOSED) and ws.close_code == 4401
        assert await login(client, "wrong") == 401
        assert await login(client) == 200
        response = await client.get(
            "/ws/arm",
            headers={
                "Origin": "http://evil.example",
                "Connection": "Upgrade",
                "Upgrade": "websocket",
                "Sec-WebSocket-Version": "13",
                "Sec-WebSocket-Key": "dGhlIHNhbXBsZSBub25jZQ==",
            },
        )
        assert response.status == 403
        assert (await client.get("/mesh/pkg/../secret.stl")).status == 404
        assert not driver.sent

    run(app, scenario)


def test_status_stream_lease_and_execute(setup):
    console, driver, _ = setup
    app, _ = build(console)

    async def scenario(client):
        await login(client)
        info = await (await client.get("/api/arm/info")).json()
        assert [joint["name"] for joint in info["joints"]] == JOINTS
        ws, status = await connect(client)
        assert status["positions"]["left_joint2"] == pytest.approx(0.2)
        # The fixture lease belongs to another holder: this connection must acquire first.
        assert (await command(ws, "execute", **TARGET))["error"] == "lease_required"
        console.release("s1")
        assert (await command(ws, "acquire"))["ok"]
        assert (await next_of(ws, "status"))["lease"] == "you"
        assert (await command(ws, "execute", **TARGET))["ok"]
        assert len(driver.sent) == 1
        assert not (await command(ws, "enable", enabled=True))["ok"]  # moving

    run(app, scenario)


def test_closing_the_holding_socket_stops_and_releases(setup):
    console, driver, _ = setup
    app, _ = build(console)

    async def scenario(client):
        await login(client)
        ws, _ = await connect(client)
        console.release("s1")
        await command(ws, "acquire")
        await command(ws, "execute", **TARGET)
        await ws.close()
        for _ in range(100):
            if console.lease_holder is None:
                break
            await asyncio.sleep(0.01)
        assert console.lease_holder is None
        assert len(driver.sent) == 2  # the move, then its smooth stop

    run(app, scenario)


def test_another_login_holding_a_page_blocks_the_arm_lease(setup):
    console, _, _ = setup
    console.release("s1")
    operators = Operators()
    chassis = {"holder": None}
    operators.register_page("chassis", lambda: chassis["holder"])
    operators.connect("chassis-connection", "other-login")
    app, _ = build(console, operators=operators)

    async def scenario(client):
        await login(client)
        ws, _ = await connect(client)
        chassis["holder"] = "chassis-connection"
        reply = await command(ws, "acquire")
        assert reply["error"] == "lease_busy" and console.lease_holder is None
        chassis["holder"] = None
        assert (await command(ws, "acquire"))["ok"]

    run(app, scenario)


def test_run_sequence_reports_console_checks(setup):
    console, driver, _ = setup
    console.release("s1")
    app, _ = build(console)

    async def scenario(client):
        await login(client)
        ws, _ = await connect(client)
        await command(ws, "acquire")
        reply = await command(ws, "run_sequence", names=["没有"], gear="leisure")
        assert not reply["ok"] and "第 1 个" in reply["message"]
        assert not driver.sent

    run(app, scenario)


def test_audit_log_records_commands_without_secrets(setup, tmp_path):
    console, _, _ = setup
    path = tmp_path / "audit" / "audit.jsonl"
    app, _ = build(
        console,
        set_enabled=lambda enabled: {
            "accepted": False,
            "enabled": False,
            "message": "drives faulted",
        },
        audit=AuditLog(path),
    )

    async def scenario(client):
        await login(client)
        ws, _ = await connect(client)
        await command(ws, "stop")
        console.release("s1")
        await command(ws, "acquire")
        assert not (await command(ws, "enable", enabled=True, extra="x"))["ok"]

    run(app, scenario)
    entries = [json.loads(line) for line in path.read_text().splitlines()]
    assert [(e["action"], e["ok"]) for e in entries] == [
        ("stop", True),
        ("acquire", True),
        ("enable", False),
    ]
    assert entries[-1]["args"] == {"enabled": True} and entries[-1]["error"] == "enable_rejected"
    assert entries[-1]["message"] == "drives faulted"
    assert "secret" not in path.read_text()
    assert len(entries[0]["session"]) == 8 and entries[0]["session"] == entries[2]["session"]


def test_fault_reset_requires_the_lease_and_reports_the_stage(setup, tmp_path):
    console, _, _ = setup
    replies = [
        {"ok": True, "stage": "success", "failed_joint": "", "status_word": 0},
        {"ok": False, "stage": "restart_required", "failed_joint": "", "status_word": 0},
    ]
    calls = []

    def reset_fault():
        calls.append(1)
        return replies[len(calls) - 1]

    app, _ = build(console, reset_fault=reset_fault)

    async def scenario(client):
        await login(client)
        ws, _ = await connect(client)
        assert not (await command(ws, "reset_fault"))["ok"] and not calls
        console.release("s1")
        await command(ws, "acquire")
        assert (await command(ws, "reset_fault"))["stage"] == "success"
        reply = await command(ws, "reset_fault")
        assert reply["error"] == "reset_rejected"
        assert "restart_required" in reply["message"] and "arm restart" in reply["message"]

    run(app, scenario)


def test_fault_reset_is_unavailable_without_a_backend(setup):
    console, _, _ = setup
    console.release("s1")
    app, _ = build(console)

    async def scenario(client):
        await login(client)
        ws, _ = await connect(client)
        await command(ws, "acquire")
        assert (await command(ws, "reset_fault"))["error"] == "reset_unavailable"

    run(app, scenario)


@pytest.mark.parametrize("dwell", [2, 8])
def test_sequence_dwell_and_selector(setup, dwell):
    console, driver, _ = setup
    console.save_pose("s1", "here", overwrite=False)
    console.release("s1")
    jobs = []
    console._spawn = jobs.append
    app, _ = build(console)

    async def scenario(client):
        await login(client)
        ws, _ = await connect(client)
        await command(ws, "acquire")
        assert (await command(ws, "run_sequence", names=["here"], gear="leisure", dwell_s=dwell))[
            "ok"
        ]
        assert console._sequence["dwell_s"] == dwell and console._sequence["wait_until"] is not None
        assert not driver.sent  # already there; it must hold without submitting a move
        page = await (await client.get("/arm")).text()
        assert 'id="sequence-dwell"' in page and 'type="number" min="0" step="0.1"' in page
        assert (await command(ws, "stop"))["ok"]
        jobs.pop()()
        reply = await command(ws, "run_sequence", names=["here"], gear="leisure", dwell_s="2")
        assert reply["error"] == "bad_dwell" and not driver.sent

    run(app, scenario)


def test_per_step_sequence(setup):
    console, driver, _ = setup
    console.save_pose("s1", "here", overwrite=False)
    console.release("s1")
    jobs = []
    console._spawn = jobs.append
    app, _ = build(console)
    steps = [
        {"name": "here", "gear": "leisure", "dwell_s": 0.4},
        {"name": "here", "gear": "sport", "dwell_s": 1.2},
    ]

    async def scenario(client):
        await login(client)
        ws, _ = await connect(client)
        await command(ws, "acquire")
        assert (await command(ws, "run_sequence", steps=steps))["total"] == 2
        status = await next_of(ws, "status")
        assert status["sequence"]["steps"] == steps and not driver.sent
        assert (await command(ws, "stop"))["ok"]
        jobs.pop()()

    run(app, scenario)


class FakeVacuum:
    def __init__(self, failure=False):
        self.reads, self.writes, self.failure = 0, [], failure

    def status(self):
        self.reads += 1
        return {
            "configured": True,
            "valid": True,
            "busy": False,
            "outputs": {"pump": False, "left": False, "right": False},
        }

    def set_output(self, output, enabled, expected):
        if (
            output not in ("pump", "left", "right")
            or type(enabled) is not bool
            or type(expected) is not bool
        ):
            raise ValueError("invalid output request")
        self.writes.append((output, enabled, expected))
        if self.failure:
            raise ConsoleError("vacuum_unconfirmed", "设备响应丢失")
        snapshot = self.status()
        snapshot["outputs"][output] = enabled
        return {"vacuum": snapshot, "message": "confirmed"}


def test_vacuum_reads_need_login_and_writes_need_the_lease(setup):
    console, driver, _ = setup
    vacuum = FakeVacuum()
    app, _ = build(console, vacuum=vacuum)
    pump = {"output": "pump", "enabled": True, "expected": False}

    async def scenario(client):
        assert (await client.get("/api/arm/vacuum")).status == 401 and vacuum.reads == 0
        await login(client)
        assert (await (await client.get("/api/arm/vacuum")).json())["valid"]
        ws, _ = await connect(client)
        assert (await command(ws, "vacuum", **pump))["error"] == "lease_required"
        console.release("s1")
        await command(ws, "acquire")
        assert (await command(ws, "vacuum", **pump))["vacuum"]["outputs"]["pump"]
        assert vacuum.writes == [("pump", True, False)] and not driver.sent
        await command(ws, "execute", **TARGET)
        reply = await command(ws, "vacuum", output="left", enabled=True, expected=False)
        assert reply["error"] == "busy" and len(vacuum.writes) == 1

    run(app, scenario)


def test_absent_or_failing_vacuum_tool_is_reported(setup):
    console, _, _ = setup
    console.release("s1")
    app, _ = build(console)

    async def absent(client):
        await login(client)
        assert not (await (await client.get("/api/arm/vacuum")).json())["configured"]
        ws, _ = await connect(client)
        assert (await command(ws, "vacuum"))["error"] == "vacuum_unavailable"

    run(app, absent)
    vacuum = FakeVacuum(failure=True)
    app, _ = build(console, vacuum=vacuum)

    async def failing(client):
        await login(client)
        ws, _ = await connect(client)
        await command(ws, "acquire")
        reply = await command(ws, "vacuum", output="right", enabled=True, expected=False)
        assert reply["error"] == "vacuum_unconfirmed" and len(vacuum.writes) == 1
        for body in ({}, {"output": "pump", "enabled": "yes", "expected": False}):
            assert (await command(ws, "vacuum", **body))["error"] == "bad_request"
        assert len(vacuum.writes) == 1

    run(app, failing)
