"""Diagnostics page: read-only feed conversion, self-check reports, login and WebSocket push."""

import asyncio
import json
import os
from pathlib import Path
import sys
import time

from aiohttp import WSMsgType, web
from aiohttp.test_utils import TestClient, TestServer
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from robot_system_interfaces.msg import DomainReadiness, ErrorInfo

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.diag.feed import DiagnosticsFeed  # noqa: E402
from integ_operator_web.diag.page import DiagPage, self_check_reports  # noqa: E402
from integ_operator_web.session import Sessions  # noqa: E402

WEB = Path(__file__).resolve().parents[1] / "web/diag"


class Clock:
    now = 100.0

    def __call__(self):
        return self.now


def readiness_message():
    message = DomainReadiness()
    message.readiness_name = "arms_only"
    message.ready = False
    message.status = "UNAVAILABLE"
    message.operational_state = "self_check_done"
    message.blockers = ["device_fault"]
    message.producer_instance_id = "abc"
    error = ErrorInfo()
    error.code = 1028
    error.message = "RT_DEVICE_FAULT"
    error.severity = ErrorInfo.FAULT
    error.source = "arms/left_joint3/drive"
    error.detail = "0x603F: 0x2214 电机电流过流（ZeroErr 手册 p120）"
    message.errors = [error]
    return message


def diagnostics_message():
    status = DiagnosticStatus()
    status.name = "/robot/rt_control/enable_manager"
    status.level = DiagnosticStatus.ERROR
    status.message = "failed"
    status.values = [KeyValue(key="state", value="FAILED")]
    message = DiagnosticArray()
    message.status = [status]
    return message


def test_feed_converts_readiness_and_keeps_the_latest_row_per_name():
    clock = Clock()
    feed = DiagnosticsFeed(clock)
    assert feed.snapshot() == {"readiness": None, "diagnostics": []}
    feed.on_readiness(readiness_message())
    feed.on_diagnostics(diagnostics_message())
    clock.now += 2.5
    snapshot = feed.snapshot()
    readiness = snapshot["readiness"]
    assert (readiness["ready"], readiness["status"], readiness["blockers"]) == (
        False,
        "UNAVAILABLE",
        ["device_fault"],
    )
    assert readiness["age_s"] == 2.5
    [error] = readiness["errors"]
    assert (error["code"], error["name"], error["severity"]) == (1028, "RT_DEVICE_FAULT", "FAULT")
    assert error["source"] == "arms/left_joint3/drive" and "电机电流过流" in error["detail"]
    [row] = snapshot["diagnostics"]
    assert (row["level"], row["message"], row["values"]) == ("ERROR", "failed", {"state": "FAILED"})


def test_reports_are_newest_first_and_unreadable_ones_are_named(tmp_path):
    (tmp_path / "old.json").write_text(json.dumps({"control_scope": "old", "entries": []}))
    os.utime(tmp_path / "old.json", (time.time() - 60, time.time() - 60))
    (tmp_path / "new.json").write_text(json.dumps({"control_scope": "new", "entries": []}))
    (tmp_path / "broken.json").write_text("{")
    os.utime(tmp_path / "broken.json", (time.time() - 120, time.time() - 120))
    reports = self_check_reports(tmp_path)
    assert [report["control_scope"] for report in reports] == ["new", "old", "broken"]
    assert "无法读取" in reports[2]["error"]
    assert self_check_reports(tmp_path / "missing") == []


def test_page_needs_login_and_pushes_reports_readiness_and_rows(tmp_path):
    report = {
        "control_scope": "arms_only",
        "stage": "before_bus",
        "passed": False,
        "entries": [{"source": "ethercat/master0", "dree": "RT_DEVICE_OFFLINE", "blocking": True}],
    }
    (tmp_path / "arms_only.json").write_text(json.dumps(report))
    feed = DiagnosticsFeed()
    feed.on_readiness(readiness_message())
    feed.on_diagnostics(diagnostics_message())
    sessions = Sessions("secret", failure_delay=0.0)
    page = DiagPage(feed, sessions, WEB, lambda: tmp_path, status_period=0.05)
    app = web.Application()
    app.router.add_post("/api/login", sessions.login)
    page.add_routes(app)

    async def main():
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            assert "ALFA 诊断" in await (await client.get("/diag")).text()
            ws = await client.ws_connect("/ws/diag")
            closing = await ws.receive(timeout=2)
            assert closing.type in (WSMsgType.CLOSE, WSMsgType.CLOSED) and ws.close_code == 4401
            assert (await client.post("/api/login", json={"token": "secret"})).status == 200
            ws = await client.ws_connect("/ws/diag")
            first = await ws.receive_json(timeout=2)
            assert first["type"] == "status" and first["self_checks"] == [report]
            assert first["readiness"]["errors"][0]["code"] == 1028
            assert first["diagnostics"][0]["name"] == "/robot/rt_control/enable_manager"
            report["passed"] = True
            (tmp_path / "arms_only.json").write_text(json.dumps(report))
            second = await ws.receive_json(timeout=2)
            assert second["self_checks"][0]["passed"]
            await ws.close()
        finally:
            await client.close()

    asyncio.run(main())
