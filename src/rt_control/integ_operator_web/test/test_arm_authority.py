import concurrent.futures
import threading

import pytest

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.arm.authority import ArmAuthority
from integ_operator_web.arm.console_types import ConsoleError as JogError


def ready_authority(*args, **kwargs):
    authority = ArmAuthority(*args, **kwargs)
    authority.confirm_runtime_idle()
    return authority


def test_gateway_restart_cannot_grant_before_controller_reconciliation():
    authority = ArmAuthority(3.0)
    with pytest.raises(JogError, match="启动对账"):
        authority.acquire("18131", "page")
    authority.confirm_runtime_idle()
    permit = authority.acquire("18131", "page")
    authority.begin_motion(permit)
    with pytest.raises(JogError):
        authority.confirm_runtime_idle()


def test_joint_and_jog_console_share_one_resource_even_for_same_login():
    authority = ready_authority(3.0)
    joint = authority.acquire("18121", "joint-page")
    with pytest.raises(JogError, match="已持有"):
        authority.acquire("18131", "jog-page")
    authority.release(joint)
    jog = authority.acquire("18131", "jog-page")
    with pytest.raises(JogError):
        authority.acquire("18121", "joint-page")
    authority.require(jog)


def test_simultaneous_acquisition_has_exactly_one_winner():
    authority, barrier = ready_authority(3.0), threading.Barrier(2)

    def acquire(entry):
        barrier.wait()
        try:
            return authority.acquire(entry, entry)
        except JogError:
            return None

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        winners = list(pool.map(acquire, ("18121", "18131")))
    assert sum(p is not None for p in winners) == 1


def test_motion_or_sequence_reservation_blocks_other_entry_until_confirmed_stopped():
    authority = ready_authority(3.0)
    joint = authority.acquire("18121", "joint-page")
    authority.begin_motion(joint)
    with pytest.raises(JogError):
        authority.release(joint)
    assert authority.poll(joint)["stop_required"]
    with pytest.raises(JogError):
        authority.acquire("18131", "jog-page")
    authority.finish_motion(joint)
    jog = authority.acquire("18131", "jog-page")
    with pytest.raises(JogError):
        authority.require(joint)
    authority.require(jog)


def test_lost_heartbeat_fences_publishing_and_retains_resource_indefinitely_while_stop_unknown():
    clock = [0.0]
    authority = ready_authority(3.0, clock=lambda: clock[0])
    permit = authority.acquire("18131", "page")
    authority.begin_motion(permit)
    clock[0] = 4.0
    for _ in range(3):
        with pytest.raises(JogError):
            authority.require(permit)
        with pytest.raises(JogError):
            authority.renew(permit)
        with pytest.raises(JogError):
            authority.acquire("18121", "page")
        assert authority.poll()["phase"] == "stopping"
        clock[0] += 1000
    authority.finish_motion(permit)
    assert authority.acquire("18121", "page")


def test_idle_timeout_and_reacquisition_never_revive_old_permit():
    clock = [0.0]
    authority = ready_authority(3.0, clock=lambda: clock[0])
    old = authority.acquire("18131", "page")
    clock[0] = 4.0
    new = authority.acquire("18131", "page")
    assert new != old
    for operation in (
        authority.require,
        authority.renew,
        authority.release,
        authority.finish_motion,
    ):
        with pytest.raises(JogError):
            operation(old)
    authority.require(new)


def test_old_stop_confirmation_cannot_release_a_new_owner():
    authority = ready_authority(3.0)
    old = authority.acquire("18131", "page")
    authority.begin_motion(old)
    authority.request_stop(old)
    authority.finish_motion(old)
    new = authority.acquire("18121", "page")
    with pytest.raises(JogError):
        authority.finish_motion(old)
    assert authority.poll(new)["lease"] == "you"
