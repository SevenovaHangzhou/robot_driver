"""Automatic entry selection with the real console, authority and idle evidence."""

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.arm.console_types import ConsoleError
from integ_operator_web.arm.idle import ArmIdleEvidence
from integ_operator_web.arm.jog import JogAccess
from integ_operator_web.arm.modes import ArmModes, ModeError
from integ_operator_web.arm.rolling_output import RollingOutput
from test_arm_console import setup  # noqa: F401
from test_rolling_output import Port

TARGET = {"left_joint1": 0.3, "left_joint2": 0.2}


class Feedback:
    """External controller snapshots; all reconciliation uses ArmIdleEvidence."""

    def __init__(self, console, clock):
        self.c, self.clock = console, clock
        self.e = ArmIdleEvidence(0.2, 2.5, clock=clock)
        self.calls = []
        self.c.idle_probe = lambda since: self.e.idle(self.c.joint_names, since)
        self.c.modes = ArmModes(console, self.switch, lambda: True, self.e.mode_matches)
        self.current = "jtc"
        self.feed()

    def feed(self):
        c, e = self.c, self.e
        names = c.joint_names
        q = [c.robot.positions[n] for n in names]
        e.set_stationary_policy(names, [0.01] * len(names), 20)
        for _ in range(21):
            self.clock.now += 0.008
            e.measured(names, q, round(self.clock.now * 1e9))
        e.controllers(
            [
                NS(
                    name=c.controller,
                    type="JointTrajectoryController",
                    state="active" if self.current == "jtc" else "inactive",
                ),
                NS(
                    name="rolling",
                    type="RollingTrajectoryController",
                    state="active" if self.current == "rolling" else "inactive",
                ),
            ],
            c.controller,
        )
        e.controller_state(names, q, [0.0] * len(names), [0.0] * len(names))
        e.goal_status([])
        e.set_tolerance(0.01)
        e.rolling_state(NS(epoch=2, state=0, last_sequence=0, last_result=0, stop_reason=0))
        c.update(
            controller_active=self.current == "jtc",
            controller_at=self.clock(),
            feedback_at=self.clock(),
            enable_at=self.clock(),
        )

    def switch(self, target):
        self.calls.append(target)
        self.current = target
        self.feed()


def arm_rig(setup, source="rolling"):
    c, driver, clock = setup
    io = Feedback(c, clock)
    io.current = source
    io.feed()
    return c, driver, clock, io


def jog_rig(setup):
    c, driver, clock, io = arm_rig(setup, "jtc")
    c.release("s1")
    access = JogAccess(c)
    access.acquire("jog")
    bridge = NS(
        _readback=NS(evidence=io.e),
        rolling_parameters={"joints": list(c.joint_names)},
        rolling_parameters_at=clock(),
        rolling_hold=NS(query=lambda names: {n: c.robot.positions[n] for n in names}),
    )
    output = c.jog_output = RollingOutput(c, bridge, Port())
    identity = {"model_sha256": c.model.source_sha256, "joint_names": list(c.joint_names)}
    return c, driver, clock, io, access, output, identity


def test_joint_execution_selects_jtc_before_sending(setup):
    c, driver, _, io = arm_rig(setup)
    assert c.status("s1")["motion_ready"]
    c.execute("s1", TARGET, "leisure")
    assert io.calls == ["jtc"] and len(driver.sent) == 1
    assert c.authority.poll()["phase"] == "moving" and not c._mode_busy


def test_current_mode_does_not_call_the_service_again(setup):
    c, driver, _, io = arm_rig(setup, "jtc")
    c.execute("s1", TARGET, "leisure")
    assert not io.calls and len(driver.sent) == 1


def test_new_mode_reservation_rearms_disconnect_stop_on_the_same_lease(setup):
    c, driver, _, io = arm_rig(setup)
    c._authority_stop_sent = c.authority.owner().generation
    c.execute("s1", TARGET, "leisure")
    assert c._authority_stop_sent is None
    c.disconnect("s1")
    assert driver.sent[-1][1][0].time >= 0
    assert c._execution.kind == "stop"


@pytest.mark.parametrize("command", ["goto", "sequence"])
def test_saved_pose_and_sequence_select_jtc_once(setup, command):
    c, driver, _, io = arm_rig(setup)
    c._pose_store().save("goal", [TARGET[n] for n in c.joint_names])
    if command == "goto":
        c.goto_pose("s1", "goal", "leisure")
    else:
        c.run_sequence("s1", ["goal"], "leisure")
    assert io.calls == ["jtc"] and len(driver.sent) == 1


def test_jog_selects_rolling_before_reserving_its_first_window(setup):
    c, driver, _, io, _, output, identity = jog_rig(setup)
    assert output.ready()  # JTC is currently active; this entry can select Rolling.
    output.begin("jog", identity)
    assert io.calls == ["rolling"] and io.e.mode_matches("rolling")
    assert c._jog_busy and not output.sender.sent and not driver.sent
    assert c.authority.poll()["phase"] == "moving"


@pytest.mark.parametrize("bad", ["target", "gear", "model", "disabled", "moving"])
def test_invalid_request_never_changes_mode_or_publishes(setup, bad):
    if bad == "model":
        c, driver, _, io, _, output, identity = jog_rig(setup)
        action = lambda: output.begin("jog", {**identity, "model_sha256": "wrong"})
    else:
        c, driver, _, io = arm_rig(setup)
        if bad == "disabled":
            c.robot.enable_state = "IDLE"
        elif bad == "moving":
            io.e.rolling_state(NS(state=1))
        action = lambda: c.execute(
            "s1",
            {**TARGET, "left_joint1": 20} if bad == "target" else TARGET,
            "wrong" if bad == "gear" else "leisure",
        )
    with pytest.raises(ConsoleError):
        action()
    assert not io.calls and not driver.sent and not c._mode_busy


@pytest.mark.parametrize("uncertain", [False, True])
def test_failed_switch_never_sends_and_unknown_outcome_stays_fenced(setup, uncertain):
    c, driver, _, io = arm_rig(setup)

    def fail(_):
        raise ModeError(
            "mode_timeout" if uncertain else "mode_rejected", "switch failed", uncertain=uncertain
        )

    c.modes.switch = fail
    with pytest.raises(ModeError):
        c.execute("s1", TARGET, "leisure")
    assert not driver.sent and not c._mode_busy
    io.feed()
    c.poll_authority()
    assert c._authority_unknown is uncertain
    assert c.authority.poll()["phase"] == ("stopping" if uncertain else "leased")


@pytest.mark.parametrize("cancel", ["stop", "release", "disconnect", "expire"])
def test_switch_in_flight_cannot_be_preempted_or_publish_after_cancellation(setup, cancel):
    c, driver, clock, io = arm_rig(setup)
    entered, finish = threading.Event(), threading.Event()

    def wait_switch(target):
        entered.set()
        assert finish.wait(2)
        io.switch(target)

    c.modes.switch = wait_switch
    with ThreadPoolExecutor(max_workers=1) as pool:
        pending = pool.submit(c.execute, "s1", TARGET, "leisure")
        assert entered.wait(2)
        assert c._mode_busy and c.status("s1")["control_mode"]["switching"]
        assert not driver.sent
        with pytest.raises(ConsoleError):
            c.begin_enable("s1")
        with pytest.raises(ConsoleError):
            JogAccess(c).acquire("other")
        if cancel == "expire":
            clock.now += 4
            c.poll_authority()
        else:
            getattr(c, cancel)("s1") if cancel != "stop" else c.stop()
        assert c.authority.poll()["phase"] == "stopping"
        finish.set()
        with pytest.raises(ConsoleError):
            pending.result(2)
    assert not driver.sent and not c._mode_busy


def test_release_during_jog_switch_prevents_first_window(setup):
    c, driver, _, io, access, output, identity = jog_rig(setup)

    def release(target):
        access.release("jog")
        io.switch(target)

    c.modes.switch = release
    with pytest.raises(ConsoleError):
        output.begin("jog", identity)
    assert not c._jog_busy and not output.sender.sent and not driver.sent
