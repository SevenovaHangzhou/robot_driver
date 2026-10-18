import asyncio
import sys
import time
from pathlib import Path

import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rt_control_operator_web.server import (  # noqa: E402
    CLOSE_AUTH_FAILED,
    CLOSE_AUTH_TIMEOUT,
    OperatorServer,
)
from rt_control_operator_web.teleop import (  # noqa: E402
    ZERO,
    AxisLimits,
    DriveArbiter,
    Gear,
    MotionLimits,
)

TOKEN = "0123456789abcdef-token"
WEB_DIR = Path(__file__).resolve().parents[1] / "web"
LIMITS = MotionLimits(AxisLimits(0.3, 0.5, 1.0, 2.0), AxisLimits(0.6, 1.0, 2.0, 4.0))
GEARS = {"leisure": Gear(0.05, 0.1), "sport": Gear(0.15, 0.3)}
STATIC = {"ultrasonic": {"channels": [], "layout_confirmed": False, "bands_m": [0.3]}}


def make_server() -> tuple[OperatorServer, DriveArbiter]:
    arbiter = DriveArbiter(GEARS, LIMITS, time.monotonic)
    server = OperatorServer(
        arbiter,
        TOKEN,
        lambda: {"controller_connected": True, "battery": None},
        WEB_DIR,
        STATIC,
        auth_timeout=0.3,
        auth_failure_delay=0.0,
        status_period=0.05,
    )
    return server, arbiter


def run(scenario):
    async def main():
        server, arbiter = make_server()
        client = TestClient(TestServer(server.build_app()))
        await client.start_server()
        try:
            await scenario(client, arbiter)
        finally:
            await client.close()

    asyncio.run(main())


async def login(client: TestClient):
    ws = await client.ws_connect("/ws")
    await ws.send_json({"type": "auth", "token": TOKEN})
    reply = await ws.receive_json(timeout=2)
    assert reply["type"] == "auth_ok"
    return ws, reply


async def next_of(ws, kind: str, timeout: float = 2.0):
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while True:
        message = await ws.receive_json(timeout=max(0.01, deadline - loop.time()))
        if message["type"] == kind:
            return message


def test_index_served_with_security_headers() -> None:
    async def scenario(client, _arbiter):
        response = await client.get("/")
        assert response.status == 200
        assert "ALFA 底盘操作台" in await response.text()
        assert "frame-ancestors 'none'" in response.headers["Content-Security-Policy"]
        assert response.headers["X-Frame-Options"] == "DENY"
        static = await client.get("/static/app.js")
        assert static.status == 200
        listing = await client.get("/static/")
        assert listing.status in (403, 404)

    run(scenario)


def test_wrong_token_closes_socket() -> None:
    async def scenario(client, arbiter):
        ws = await client.ws_connect("/ws")
        await ws.send_json({"type": "auth", "token": "wrong-token-value"})
        error = await ws.receive_json(timeout=2)
        assert error["code"] == "auth_failed"
        closing = await ws.receive(timeout=2)
        assert closing.type in (WSMsgType.CLOSE, WSMsgType.CLOSED)
        assert ws.close_code == CLOSE_AUTH_FAILED
        assert arbiter.snapshot().lease_holder is None

    run(scenario)


def test_commands_before_auth_are_rejected() -> None:
    async def scenario(client, arbiter):
        ws = await client.ws_connect("/ws")
        await ws.send_json({"type": "acquire"})
        await ws.receive(timeout=2)
        await ws.receive(timeout=2)
        assert ws.closed and ws.close_code == CLOSE_AUTH_FAILED
        assert arbiter.snapshot().lease_holder is None

    run(scenario)


def test_auth_timeout() -> None:
    async def scenario(client, _arbiter):
        ws = await client.ws_connect("/ws")
        await ws.receive(timeout=2)
        assert ws.close_code == CLOSE_AUTH_TIMEOUT

    run(scenario)


def test_cross_origin_websocket_rejected() -> None:
    async def scenario(client, _arbiter):
        response = await client.get(
            "/ws",
            headers={
                "Origin": "http://evil.example",
                "Connection": "Upgrade",
                "Upgrade": "websocket",
                "Sec-WebSocket-Version": "13",
                "Sec-WebSocket-Key": "dGhlIHNhbXBsZSBub25jZQ==",
            },
        )
        assert response.status == 403

    run(scenario)


def test_auth_ok_exposes_gears_timing_and_static_info() -> None:
    async def scenario(client, _arbiter):
        _ws, reply = await login(client)
        assert reply["gears"]["sport"] == {"linear": 0.15, "angular": 0.3}
        assert reply["min_send_period_ms"] == 33
        assert reply["heartbeat_period_ms"] == 50
        assert reply["command_timeout_ms"] == 200
        assert reply["stick"] == {"deadzone": 0.1, "expo": 2.0, "snap_deg": 8.0}
        assert reply["ultrasonic"] == STATIC["ultrasonic"]

    run(scenario)


def test_drive_flow_gear_and_disconnect_clears_lease() -> None:
    async def scenario(client, arbiter):
        ws, _ = await login(client)
        await ws.send_json({"type": "acquire"})
        await ws.send_json({"type": "gear", "gear": "sport"})
        await ws.send_json({"type": "drive", "x": 0, "y": 0, "yaw": 1})
        status = await next_of(ws, "status")
        while status["target"]["wz"] == 0.0:
            status = await next_of(ws, "status")
        assert status["lease"] == "you"
        assert status["gear"] == "sport"
        assert status["moving"] is True
        assert status["target"]["wz"] == pytest.approx(0.3)
        assert status["robot"]["controller_connected"] is True
        await ws.close()
        for _ in range(50):
            if arbiter.snapshot().lease_holder is None:
                break
            await asyncio.sleep(0.02)
        assert arbiter.snapshot().lease_holder is None
        assert arbiter.snapshot().last_stop_reason == "client_disconnected"
        assert arbiter.tick() == ZERO

    run(scenario)


def test_second_client_sees_other_lease_and_can_soft_stop() -> None:
    async def scenario(client, arbiter):
        first, _ = await login(client)
        await first.send_json({"type": "acquire"})
        second, _ = await login(client)
        status = await next_of(second, "status")
        while status["lease"] != "other":
            status = await next_of(second, "status")
        await second.send_json({"type": "acquire"})
        error = await next_of(second, "error")
        assert error["code"] == "lease_busy"
        await second.send_json({"type": "drive", "x": 1, "y": 0})
        error = await next_of(second, "error")
        assert error["code"] == "lease_required"
        await second.send_json({"type": "soft_stop"})
        for _ in range(50):
            if arbiter.snapshot().lease_holder is None:
                break
            await asyncio.sleep(0.02)
        assert arbiter.snapshot().last_stop_reason == "soft_stop"

    run(scenario)


def test_mixed_and_unknown_messages_report_errors_and_stop() -> None:
    async def scenario(client, arbiter):
        ws, _ = await login(client)
        await ws.send_json({"type": "acquire"})
        await ws.send_json({"type": "drive", "x": 1, "y": 0, "yaw": 0})
        await ws.send_json({"type": "drive", "x": 1, "y": 0, "yaw": 1})
        error = await next_of(ws, "error")
        assert error["code"] == "mixed_input_rejected"
        assert arbiter.snapshot().target == ZERO
        await ws.send_json({"type": "drive", "x": 1, "y": 0, "yaw": 0})
        await ws.send_str('{"type": "enable_all"}')
        error = await next_of(ws, "error")
        assert error["code"] == "unknown_type"
        assert arbiter.snapshot().target == ZERO
        await ws.send_json({"type": "drive", "x": 1, "y": 0, "yaw": 0})
        await ws.send_json({"type": "gear", "gear": "sport"})
        error = await next_of(ws, "error")
        assert error["code"] == "gear_change_while_moving"
        assert arbiter.snapshot().target == ZERO
        await ws.send_json({"type": "auth", "token": TOKEN})
        error = await next_of(ws, "error")
        assert error["code"] == "bad_request"
        assert arbiter.snapshot().lease_holder is not None

    run(scenario)


def test_shutdown_soft_stops() -> None:
    async def main():
        server, arbiter = make_server()
        client = TestClient(TestServer(server.build_app()))
        await client.start_server()
        ws, _ = await login(client)
        await ws.send_json({"type": "acquire"})
        await next_of(ws, "status")
        await client.close()
        assert arbiter.snapshot().lease_holder is None
        assert arbiter.tick() == ZERO

    asyncio.run(main())


def test_missing_assets_or_token_rejected(tmp_path: Path) -> None:
    arbiter = DriveArbiter(GEARS, LIMITS, time.monotonic)
    with pytest.raises(ValueError):
        OperatorServer(arbiter, TOKEN, dict, tmp_path)
    with pytest.raises(ValueError):
        OperatorServer(arbiter, "", dict, WEB_DIR)


def test_get_model_is_read_only_and_needs_no_lease() -> None:
    async def main():
        arbiter = DriveArbiter(GEARS, LIMITS, time.monotonic)
        server = OperatorServer(
            arbiter,
            TOKEN,
            lambda: {},
            WEB_DIR,
            STATIC,
            model_provider=lambda: {"type": "model", "version": "abc", "modules": []},
        )
        client = TestClient(TestServer(server.build_app()))
        await client.start_server()
        try:
            ws, _ = await login(client)
            await ws.send_json({"type": "get_model"})
            reply = await next_of(ws, "model")
            assert reply["version"] == "abc"
            assert arbiter.snapshot().lease_holder is None
        finally:
            await client.close()

    asyncio.run(main())


def test_get_model_without_model_returns_empty_version() -> None:
    async def scenario(client, _arbiter):
        ws, _ = await login(client)
        await ws.send_json({"type": "get_model"})
        reply = await next_of(ws, "model")
        assert reply["version"] is None

    run(scenario)


def make_module_server(operation):
    arbiter = DriveArbiter(GEARS, LIMITS, time.monotonic)
    server = OperatorServer(
        arbiter,
        TOKEN,
        lambda: {},
        WEB_DIR,
        STATIC,
        module_operation=operation,
        history_provider=lambda: [{"key": "op/1", "kind": "operation"}],
    )
    return server, arbiter


def run_with(server, scenario):
    async def main():
        client = TestClient(TestServer(server.build_app()))
        await client.start_server()
        try:
            await scenario(client)
        finally:
            await client.close()

    asyncio.run(main())


def test_module_op_requires_lease_and_runs_in_background():
    calls = []
    release = asyncio.Event()

    async def operation(op, modules):
        calls.append((op, modules))
        await release.wait()
        return {"ok": True, "stage": "success", "modules": [{"module": "arms", "ok": True}]}

    server, arbiter = make_module_server(operation)

    async def scenario(client):
        ws, _ = await login(client)
        await ws.send_json({"type": "module_op", "op": "enable", "modules": ["arms"]})
        assert (await next_of(ws, "error"))["code"] == "lease_required"
        await ws.send_json({"type": "acquire"})
        await ws.send_json({"type": "module_op", "op": "enable", "modules": ["arms"]})
        await ws.send_json({"type": "module_op", "op": "disable", "modules": ["arms"]})
        assert (await next_of(ws, "error"))["code"] == "module_op_busy"
        # The long-running operation does not block other requests (drive heartbeats).
        await ws.send_json({"type": "drive", "x": 0, "y": 0, "yaw": 0})
        await ws.send_json({"type": "get_fault_history"})
        history = await next_of(ws, "fault_history")
        assert history["items"][0]["kind"] == "operation"
        release.set()
        result = await next_of(ws, "module_op_result")
        assert result["ok"] is True and result["op"] == "enable"
        assert result["requested"] == ["arms"]
        assert calls == [("enable", ("arms",))]
        assert arbiter.snapshot().lease_holder is not None

    run_with(server, scenario)


def test_module_op_exceptions_are_reported():
    async def operation(op, modules):
        raise RuntimeError("boom")

    server, _ = make_module_server(operation)

    async def scenario(client):
        ws, _ = await login(client)
        await ws.send_json({"type": "acquire"})
        await ws.send_json({"type": "module_op", "op": "reset", "modules": ["head_gimbal"]})
        result = await next_of(ws, "module_op_result")
        assert result["ok"] is False and "boom" in result["stage"]

    run_with(server, scenario)


def test_module_op_unavailable_without_backend():
    async def scenario(client, _arbiter):
        ws, _ = await login(client)
        await ws.send_json({"type": "acquire"})
        await ws.send_json({"type": "module_op", "op": "enable", "modules": ["arms"]})
        assert (await next_of(ws, "error"))["code"] == "module_op_unavailable"

    run(scenario)


def make_steering_server(calls):
    async def steering(slew, accel, jerk, alignment):
        calls.append(("set", slew, accel, jerk, alignment))
        return {"ok": True, "message": "saved"}

    async def defaults():
        calls.append(("defaults",))
        return {"ok": True, "message": "restored"}

    arbiter = DriveArbiter(GEARS, LIMITS, time.monotonic)
    server = OperatorServer(
        arbiter,
        TOKEN,
        lambda: {},
        WEB_DIR,
        STATIC,
        auth_timeout=0.3,
        auth_failure_delay=0.0,
        status_period=0.05,
        steering_operation=steering,
        steering_defaults=defaults,
    )
    return server


def run_steering(scenario):
    calls = []

    async def main():
        client = TestClient(TestServer(make_steering_server(calls).build_app()))
        await client.start_server()
        try:
            await scenario(client)
        finally:
            await client.close()

    asyncio.run(main())
    return calls


STEERING = {
    "type": "steering_params",
    "slew_deg_s": 30,
    "accel_deg_s2": 20,
    "jerk_deg_s3": 100,
    "alignment_deg": 90,
}


def test_steering_parameters_need_the_lease_and_the_token_again() -> None:
    async def scenario(client):
        ws, _ = await login(client)
        await ws.send_json({**STEERING, "token": TOKEN})
        assert (await next_of(ws, "error"))["code"] == "lease_required"
        await ws.send_json({"type": "acquire"})
        await ws.send_json({**STEERING, "token": "wrong-token-value"})
        assert (await next_of(ws, "error"))["code"] == "token_mismatch"
        await ws.send_json({**STEERING, "token": TOKEN})
        assert (await next_of(ws, "steering_params_result"))["message"] == "saved"
        await ws.send_json({"type": "steering_defaults", "token": TOKEN})
        assert (await next_of(ws, "steering_params_result"))["message"] == "restored"

    calls = run_steering(scenario)
    assert calls == [("set", 30.0, 20.0, 100.0, 90.0), ("defaults",)]


def test_steering_parameters_unavailable_without_backend() -> None:
    async def scenario(client, _arbiter):
        ws, _ = await login(client)
        await ws.send_json({"type": "acquire"})
        await ws.send_json({**STEERING, "token": TOKEN})
        assert (await next_of(ws, "error"))["code"] == "unsupported_operation"

    run(scenario)
