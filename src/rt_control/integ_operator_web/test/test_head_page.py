"""Head page: login, WebSocket commands, lease rules across pages, enable and reset routing."""

import asyncio
import json
import math
from pathlib import Path
import sys

from aiohttp import WSMsgType, web
from aiohttp.test_utils import TestClient, TestServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from integ_operator_web.audit import AuditLog  # noqa: E402
from integ_operator_web.head.page import HeadPage  # noqa: E402
from integ_operator_web.operators import Operators  # noqa: E402
from integ_operator_web.session import Sessions  # noqa: E402
from test_head_console import ready_console  # noqa: E402

WEB = Path(__file__).resolve().parents[1] / "web/head"


def build(console, operators=None, audit=None, reset=None):
    sessions = Sessions("secret", failure_delay=0.0)
    page = HeadPage(
        console,
        sessions,
        operators or Operators(),
        WEB,
        lambda enabled: {"accepted": True, "enabled": enabled, "message": ""},
        reset or (lambda: {"ok": True, "stage": "success", "failed_joint": "", "status_word": 0}),
        audit,
        status_period=0.01,
    )
    app = web.Application()
    app.router.add_post("/api/login", sessions.login)
    page.add_routes(app)
    return app


def run(app, scenario):
    async def main():
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            await scenario(client)
        finally:
            await client.close()

    asyncio.run(main())


async def next_of(ws, kind):
    while True:
        message = await ws.receive_json(timeout=2)
        if message["type"] == kind:
            return message


async def command(ws, kind, **body):
    command.counter = getattr(command, "counter", 0) + 1
    await ws.send_json({"type": kind, "id": command.counter, **body})
    while True:
        message = await next_of(ws, "result")
        if message["id"] == command.counter:
            return message


def test_page_requires_login_then_moves_holds_and_stops_on_disconnect(tmp_path):
    console, sent, _ = ready_console()
    console.release("c1")
    path = tmp_path / "audit.jsonl"
    app = build(console, audit=AuditLog(path))

    async def scenario(client):
        assert "ALFA 头部" in await (await client.get("/head")).text()
        ws = await client.ws_connect("/ws/head")
        closing = await ws.receive(timeout=2)
        assert closing.type in (WSMsgType.CLOSE, WSMsgType.CLOSED) and ws.close_code == 4401
        assert (await client.post("/api/login", json={"token": "secret"})).status == 200
        ws = await client.ws_connect("/ws/head")
        status = await next_of(ws, "status")
        assert status["lease"] == "none" and [j["name"] for j in status["joints"]] == [
            "H1_joint",
            "H2_joint",
        ]
        target = {"H1_joint": 0.2, "H2_joint": 0.1}
        assert (await command(ws, "move", target=target))["error"] == "lease_required"
        assert (await command(ws, "acquire"))["ok"]
        assert (await command(ws, "move", target=target))["target"] == target
        assert (await command(ws, "hold"))["ok"]
        reply = await command(ws, "move", target={"H1_joint": 0.0, "H2_joint": math.pi})
        assert reply["error"] == "target_out_of_range"
        await ws.close()
        for _ in range(100):
            if console.lease_holder is None:
                break
            await asyncio.sleep(0.01)

    run(app, scenario)
    assert sent == [[0.2, 0.1], [0.1, -0.2], [0.1, -0.2]]  # move, hold, hold on disconnect
    actions = [(e["action"], e["ok"]) for e in map(json.loads, path.read_text().splitlines())]
    assert actions == [
        ("head_move", False),
        ("head_acquire", True),
        ("head_move", True),
        ("head_hold", True),
        ("head_move", False),
    ]


def test_enable_and_reset_need_the_lease_and_report_rejections():
    console, _, _ = ready_console()
    console.release("c1")
    app = build(
        console,
        reset=lambda: {
            "ok": False,
            "stage": "restart_required",
            "failed_joint": "",
            "status_word": 0,
        },
    )

    async def scenario(client):
        await client.post("/api/login", json={"token": "secret"})
        ws = await client.ws_connect("/ws/head")
        assert (await command(ws, "enable", enabled=True))["error"] == "lease_required"
        await command(ws, "acquire")
        assert (await command(ws, "enable", enabled=True))["enabled"] is True
        assert (await command(ws, "enable", enabled="yes"))["error"] == "bad_request"
        reply = await command(ws, "reset_fault")
        assert reply["error"] == "reset_rejected" and "restart_required" in reply["message"]

    run(app, scenario)


def test_another_login_holding_a_page_blocks_the_head_lease():
    console, _, _ = ready_console()
    console.release("c1")
    operators = Operators()
    arm = {"holder": "arm-connection"}
    operators.register_page("arm", lambda: arm["holder"])
    operators.connect("arm-connection", "other-login")
    app = build(console, operators=operators)

    async def scenario(client):
        await client.post("/api/login", json={"token": "secret"})
        ws = await client.ws_connect("/ws/head")
        assert (await command(ws, "acquire"))["error"] == "lease_busy"
        arm["holder"] = None
        assert (await command(ws, "acquire"))["ok"]

    run(app, scenario)
