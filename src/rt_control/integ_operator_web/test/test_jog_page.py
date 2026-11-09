"""Real arm/jog handlers share one authority; no ROS or motor transport."""

import asyncio
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from integ_operator_web.arm.jog import JogPage
from integ_operator_web.arm.console import ConsoleError
from test_arm_console import setup as setup
from test_arm_page import build, run, login, connect, next_of, command, TARGET


def both(console):
    app, arm = build(console)
    JogPage(arm).add_routes(app)
    app.router.add_post("/api/logout", arm._sessions.logout)
    console.release("s1")
    return app


async def jog(client):
    ws = await client.ws_connect("/ws/jog")
    auth = await next_of(ws, "auth_ok")
    assert auth["entry"] == "jog" and auth["protocol"] == "shared_arm_authority_v1"
    await next_of(ws, "status")
    return ws


def test_same_login_two_pages_cannot_preempt_each_other(setup):
    console, driver, _ = setup
    app = both(console)

    async def scenario(client):
        await login(client)
        arm, _ = await connect(client)
        end = await jog(client)
        assert (await command(end, "acquire"))["ok"]
        assert (await command(arm, "acquire"))["error"] == "lease_busy"
        assert (await command(arm, "execute", **TARGET))["error"] == "lease_required"
        for kind in ("execute", "enable", "reset_fault", "velocity", "arm"):
            assert (await command(end, kind, **TARGET))["error"] == "jog_motion_disabled"
        assert not driver.sent
        await command(end, "release")
        assert (await command(arm, "acquire"))["ok"]
        assert (await command(end, "acquire"))["error"] == "lease_busy"
        await command(arm, "release")
        assert (await command(end, "acquire"))["ok"]
        await end.close()
        for _ in range(100):
            if console.authority.owner() is None:
                break
            await asyncio.sleep(0.01)
        assert console.authority.owner() is None

    run(app, scenario)


def test_simultaneous_acquire_has_one_winner(setup):
    console, _, _ = setup
    app = both(console)

    async def scenario(client):
        await login(client)
        arm, _ = await connect(client)
        end = await jog(client)
        # Separate IDs avoid the shared helper's counter being changed mid-await.
        await asyncio.gather(
            arm.send_json({"type": "acquire", "id": 1}), end.send_json({"type": "acquire", "id": 2})
        )
        replies = await asyncio.gather(next_of(arm, "result"), next_of(end, "result"))
        assert sorted(r["ok"] for r in replies) == [False, True]

    run(app, scenario)


def test_disconnected_move_cannot_handover_before_actual_stop(setup):
    console, driver, _ = setup
    app = both(console)

    async def scenario(client):
        await login(client)
        arm, _ = await connect(client)
        end = await jog(client)
        await command(arm, "acquire")
        await command(arm, "execute", **TARGET)
        console.idle_probe = lambda _since: False
        await arm.close()
        for _ in range(100):
            if len(driver.sent) == 2:
                break
            await asyncio.sleep(0.01)
        assert len(driver.sent) == 2
        driver.sent[0][2](False, "superseded")
        driver.sent[1][2](True, "")
        assert (await command(end, "acquire"))["error"] == "lease_busy"
        assert console.authority.poll()["phase"] == "stopping"
        console.idle_probe = lambda _since: True
        assert (await command(end, "acquire"))["ok"]
        # A delayed result from the old move must not clear the new reservation.
        driver.sent[0][2](True, "")
        assert console.authority.poll()["owner_entry"] == "jog"

    run(app, scenario)


def test_logout_revokes_live_socket_and_releases_idle_reservation(setup):
    console, _, _ = setup
    app = both(console)

    async def scenario(client):
        await login(client)
        end = await jog(client)
        await command(end, "acquire")
        await client.post("/api/logout")
        for _ in range(100):
            message = await end.receive(timeout=2)
            if message.type.name in ("CLOSE", "CLOSED"):
                break
        assert end.close_code == 4401
        for _ in range(100):
            if console.authority.owner() is None:
                break
            await asyncio.sleep(0.01)
        assert console.authority.owner() is None

    run(app, scenario)


def test_scope_enable_operations_share_arm_resource_and_unknown_outcome_retains_it(setup):
    console, _, _ = setup
    console.release("s1")
    from integ_operator_web.arm.jog import JogAccess

    jog_access = JogAccess(console)
    jog_access.acquire("jog-tab")
    for entry in ("head", "chassis"):
        with pytest.raises(ConsoleError):
            console.begin_scope_operation(entry, entry + "-tab")
    jog_access.release("jog-tab")
    permit = console.begin_scope_operation("head", "head-tab")
    console.end_scope_operation(permit, confirmed=False)
    with pytest.raises(ConsoleError):
        jog_access.acquire("jog-tab")
    assert console.authority.poll()["phase"] == "stopping"


def test_scope_without_arms_does_not_require_arm_feedback(setup):
    console, _, _ = setup
    console.update(scope_has_arms=False)
    with pytest.raises(ConsoleError):
        console.begin_scope_operation("head", "head-tab")  # Existing arm owner still fences.
    console.release("s1")
    console.update(scope_has_arms=None)
    console.idle_probe = lambda _since: False
    # Unknown capability must still refuse a shared operation.
    with pytest.raises(ConsoleError):
        console.begin_scope_operation("head", "head-tab")
    console.update(scope_has_arms=False)  # Explicit parsed head-only capability declaration.
    assert console.begin_scope_operation("head", "head-tab") is None
    assert console.authority.owner() is None
