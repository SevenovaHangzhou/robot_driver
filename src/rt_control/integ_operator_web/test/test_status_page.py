"""Status page: snapshot shape, staleness handling, and the login-protected read-only routes."""

import asyncio
import json
from pathlib import Path
import sys

from aiohttp import web
from aiohttp.client_exceptions import WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.session import Sessions  # noqa: E402
from integ_operator_web.status import feed as status_feed  # noqa: E402
from integ_operator_web.status.page import StatusPage  # noqa: E402

WEB = Path(__file__).resolve().parents[1] / "web/status"


def battery(present=True, soc=82.0, current=6.1, temperature=31.0, at=0.0):
    return status_feed.Sample(
        {
            "present": present,
            "soc_pct": soc,
            "current_a": current,
            "voltage_v": 51.2 if present else None,
            "temperature_c": temperature,
            "health": 0,
        },
        at,
    )


ROLES = {
    "updown": "updown",
    "head_yaw": "yaw_joint",
    "head_pitch": "pitch_joint",
    "steer1": "steer_a",
}


def feed_with(base=None):
    """A feed whose newest sample is `base`; the fake clock keeps the ages deterministic."""
    import time as _time

    base = _time.monotonic() if base is None else base
    feed = status_feed.Feed(clock=lambda: base)
    feed.joints = status_feed.Sample(
        {"updown": 0.34, "yaw_joint": 0.2, "pitch_joint": -0.1, "steer_a": 0.5, "left1": 0.1},
        base - 0.5,
    )
    feed.batteries = [
        battery(at=base - 1.0),
        battery(present=False, soc=None, current=None, temperature=None, at=base - 50.0),
    ]
    feed.odometry = status_feed.Sample({"vx": 0.3, "vy": -0.05, "wz": 0.0}, base - 0.1)
    feed.sonar = [status_feed.Sample(1.2, base - 0.2)] + [None] * 7
    feed.diagnostics = status_feed.Sample(
        [
            {
                "name": "periph_bms/0x01/pack",
                "level": "ERROR",
                "message": "2 protocol alarm(s)",
                "values": {"hardware_id": "0x01"},
            }
        ],
        base - 1.0,
    )
    feed.readiness = status_feed.Sample(
        {
            "ready": False,
            "status": "UNAVAILABLE",
            "operational_state": "",
            "blockers": ["device_offline"],
            "errors": [
                {
                    "code": "RT_DEVICE_OFFLINE",
                    "name": "右臂驱动离线",
                    "severity": "FAULT",
                    "source": "ethercat",
                    "detail": "slave 4",
                    "retryable": False,
                }
            ],
        },
        base - 1.0,
    )
    return feed


def test_document_reports_fresh_values_and_alarms():
    document = status_feed.document(feed_with(100.0), state_period_s=1.0, roles=ROLES, now=100.0)
    assert document["state"] == "fault"
    assert document["pose"]["updown_m"] == 0.34
    assert abs(document["pose"]["head_yaw_deg"] - 11.5) < 0.1
    assert document["batteries"][0]["soc_pct"] == 82.0
    # A battery frame older than the timeout reports no values, not its last ones.
    assert document["batteries"][1]["present"] is False
    assert document["batteries"][1]["current_a"] is None
    assert document["chassis"]["vx"] == 0.3
    assert abs(document["chassis"]["steer_deg"][0] - 28.6) < 0.1
    assert document["chassis"]["sonar"][0] == 1.2 and document["chassis"]["sonar"][1] is None
    # The readiness error already explains the blocker, so it is not repeated.
    assert {entry["code"] for entry in document["alerts"]} == {
        "periph_bms/0x01/pack",
        "RT_DEVICE_OFFLINE",
    }
    assert [entry["level"] for entry in document["alerts"]] == ["fault", "fault"]


def test_stale_joints_make_the_page_offline():
    feed = feed_with(100.0)
    feed.joints = status_feed.Sample({"updown": 1.0}, 90.0)
    document = status_feed.document(feed, state_period_s=1.0, roles=ROLES, now=100.0)
    assert document["state"] == "offline"
    assert document["joints"] == {}
    assert document["pose"]["updown_m"] is None


def test_without_readiness_and_diagnostics_nothing_is_reported_as_healthy_or_broken():
    feed = feed_with(100.0)
    feed.readiness = None
    feed.diagnostics = None
    document = status_feed.document(feed, state_period_s=1.0, roles=ROLES, now=100.0)
    assert document["state"] == "ok"
    assert document["readiness"] is None
    assert document["alerts"] == []


def test_warnings_alone_do_not_raise_a_fault():
    feed = feed_with(100.0)
    feed.readiness = None
    feed.diagnostics = status_feed.Sample(
        [
            {
                "name": "periph_ultrasonic/ch5",
                "level": "WARN",
                "message": "interference",
                "values": {},
            }
        ],
        99.0,
    )
    document = status_feed.document(feed, state_period_s=1.0, roles=ROLES, now=100.0)
    assert document["state"] == "warn"
    assert len(document["alerts"]) == 1


def test_cpu_reads_proc_stat_and_isolated_cores():
    data = status_feed.cpu()
    assert data["count"] == len(data["cores"])
    assert all(isinstance(core["id"], int) for core in data["cores"])
    assert isinstance(data["isolated"], list)
    assert data["usage_pct"] is None or 0 <= data["usage_pct"] <= 100


def test_snapshot_is_json_serialisable_and_carries_the_camera_base():
    page = StatusPage(
        feed_with(), Sessions("token"), WEB, camera_base="http://nx:8120/", roles=ROLES
    )
    snapshot = page.snapshot()
    json.dumps(snapshot)
    assert snapshot["camera_base"] == "http://nx:8120"
    assert snapshot["model"] is False


def test_page_requires_the_web_assets(tmp_path):
    try:
        StatusPage(feed_with(), Sessions("token"), tmp_path)
    except ValueError as error:
        assert "missing web asset" in str(error)
    else:
        raise AssertionError("a page without index.html must be rejected")


def test_routes_need_login_and_push_a_snapshot():
    sessions = Sessions("secret", failure_delay=0.0)
    page = StatusPage(
        feed_with(), sessions, WEB, camera_base="http://nx:8120", roles=ROLES, status_period=0.05
    )
    app = web.Application()
    app.router.add_post("/api/login", sessions.login)
    page.add_routes(app)

    async def main():
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            # The shell is public like the other pages; the data stream and the model are not.
            assert (await client.get("/")).status == 200
            try:
                await client.ws_connect("/ws/status", max_msg_size=0)
            except WSServerHandshakeError as error:
                assert error.status == 401
            else:
                raise AssertionError("the stream must require login")
            assert (await client.get("/api/status/model")).status == 401
            assert (await client.post("/api/login", json={"token": "secret"})).status == 200
            ws = await client.ws_connect("/ws/status", max_msg_size=0)
            first = await ws.receive_json(timeout=2)
            assert first["state"] == "fault"
            assert first["camera_base"] == "http://nx:8120"
            assert first["batteries"][0]["soc_pct"] == 82.0
            await ws.close()
        finally:
            await client.close()

    asyncio.run(main())


# ---------------------------------------------------------------- log station
from integ_operator_web.status import logstation  # noqa: E402

STATION = {
    "runs": [
        {
            "id": "20261113-101500-d195-bench",
            "running": True,
            "update_age_s": 4.2,
            "report_url": "/20261113-101500-d195-bench/",
            "cycle": {
                "nominal_period_us": 999.9,
                "max_jitter_us": 204.5,
                "overruns": 1,
                "cycles": 97231027,
                "overrun_rule": "period > 1.5 x nominal",
            },
            "blackbox": [
                {
                    "id": "20261113-101000.500-drive_fault",
                    "time": 1763000000.0,
                    "reasons": ["drive_fault"],
                    "subjects": ["right_joint4"],
                    "url": "/20261113-101500-d195-bench/blackbox/20261113-101000.500-drive_fault/",
                },
            ],
        }
    ]
}


def test_station_numbers_carry_their_age():
    data = logstation.normalise(STATION, "http://log:18200", now=1000.0)
    assert (
        data["run"]["running"] is True
        and data["run"]["report_url"] == "http://log:18200/20261113-101500-d195-bench/"
    )
    assert data["cycle"]["overruns"] == 1 and data["cycle"]["stale"] is False
    assert data["records"][0]["url"].endswith("-drive_fault/")
    stale = logstation.normalise(
        {"runs": [dict(STATION["runs"][0], update_age_s=40.0)]}, "http://log:18200", now=1000.0
    )
    assert stale["cycle"]["stale"] is True


def test_evidence_links_the_matching_window_and_lists_the_rest_as_recovered():
    log = logstation.normalise(STATION, "http://log:18200", now=1000.0)
    alerts = [
        {
            "level": "fault",
            "code": "rt_control/enable_manager",
            "title": "右臂关节 4 过流",
            "part": "右臂",
            "values": {"detail": "right_joint4 fault"},
        },
        {
            "level": "warn",
            "code": "periph_bms/0x01/pack",
            "title": "温度偏高",
            "part": "0x01",
            "values": {},
        },
    ]
    merged = logstation.evidence(alerts, log)
    assert merged[0]["evidence"]["kind"] == "waveform"
    assert merged[0]["evidence"]["url"].endswith("-drive_fault/")
    assert merged[1]["evidence"]["kind"] == "run"  # no window matches: offer the run report
    assert [entry["level"] for entry in merged] == ["fault", "warn"]


def test_unexplained_windows_are_offered_as_recovered():
    log = logstation.normalise(STATION, "http://log:18200", now=1000.0)
    merged = logstation.evidence([], log)
    assert len(merged) == 1
    assert merged[0]["level"] == "recovered" and merged[0]["code"] == "drive_fault"
    assert merged[0]["evidence"]["kind"] == "waveform"


def test_page_merges_the_station_into_the_snapshot():
    class FakeStation:
        def snapshot(self):
            return logstation.normalise(STATION, "http://log:18200", now=1000.0)

        def start(self):
            pass

    page = StatusPage(feed_with(), Sessions("token"), WEB, roles=ROLES, log_station=FakeStation())
    snapshot = page.snapshot()
    assert snapshot["log"]["cycle"]["overruns"] == 1
    assert snapshot["log"]["run"]["running"] is True
    assert any(entry["level"] == "recovered" for entry in snapshot["alerts"])


def test_station_without_runs_is_available_but_empty():
    data = logstation.normalise({"runs": []}, "http://log:18200", now=1000.0)
    assert data["available"] is True and data["run"] is None and data["cycle"] is None


# ---------------------------------------------------------------- chassis geometry from the model
from integ_operator_web.status import model_geometry  # noqa: E402

MODEL = """<?xml version="1.0"?>
<robot name="alfa_robot_v3">
  <link name="base_link"/>
  %s
</robot>
"""


def urdf_with(entries):
    body = []
    for name, xyz, rpy in entries:
        body.append(
            f'<link name="{name}_link"/><joint name="{name}" type="fixed">'
            f'<origin xyz="{xyz}" rpy="{rpy}"/><parent link="base_link"/><child link="{name}_link"/></joint>'
        )
    return MODEL % "".join(body)


WHEELS = [(f"steer{i}_joint", "0.305 0.305 0.01", "0 0 0") for i in range(1, 5)]
WHEELS += [(f"steer{i}_joint", "0.305 -0.305 0.01", "0 0 0") for i in range(2, 5)]

BOARD = [
    ("ultrasonic_channel_1", "0.3145 0.485 0.019", "0 0 1.5707963267948966"),
    ("ultrasonic_channel_2", "0.485 0.3145 0.019", "0 0 0"),
    ("ultrasonic_channel_6", "-0.485 -0.3145 0.019", "0 0 3.141592653589793"),
]


def test_model_geometry_reads_sonar_frames_and_footprint():
    geometry = model_geometry.frames(urdf_with(WHEELS + BOARD))
    assert [item["channel"] for item in geometry["sonar"]] == [1, 2, 6]
    first = geometry["sonar"][0]
    assert (first["x_m"], first["y_m"], first["yaw_deg"]) == (0.3145, 0.485, 90.0)
    assert geometry["sonar"][2]["yaw_deg"] == 180.0
    assert geometry["body"] == {"half_length_m": 0.305, "half_width_m": 0.305}
    assert len(geometry["wheels"]) == 4


def test_model_without_sonar_frames_leaves_the_page_on_placeholders():
    geometry = model_geometry.frames(urdf_with(WHEELS))
    assert geometry["sonar"] == []
    assert geometry["body"] == {"half_length_m": 0.305, "half_width_m": 0.305}


def test_unreadable_model_is_not_an_error():
    assert model_geometry.frames("not xml") == {"sonar": [], "wheels": [], "body": None}
    assert model_geometry.parse(None) == {"sonar": [], "wheels": [], "body": None}
    assert model_geometry.parse("/nonexistent/model.urdf") == {
        "sonar": [],
        "wheels": [],
        "body": None,
    }


def test_snapshot_carries_the_geometry_of_the_served_model(tmp_path):
    model = tmp_path / "robot_description.xml"
    model.write_text(urdf_with(WHEELS + BOARD), encoding="utf-8")
    page = StatusPage(feed_with(), Sessions("token"), WEB, roles=ROLES, model_file=model)
    snapshot = page.snapshot()
    assert [item["channel"] for item in snapshot["model_geometry"]["sonar"]] == [1, 2, 6]
    assert snapshot["model"] is True
