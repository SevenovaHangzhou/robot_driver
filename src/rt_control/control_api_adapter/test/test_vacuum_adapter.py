import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from control_api_adapter.vacuum_adapter import (  # noqa: E402
    ATTACHED_VERIFIED,
    GRIP,
    RELEASE,
    UNVERIFIED,
    PlcCommandResult,
    PlcVacuumSnapshot,
    PumpErrorCode,
    VacuumAdapterCore,
    build_vacuum_channel_states,
)


class FakeVacuumIo:
    def __init__(self, snapshot: PlcVacuumSnapshot) -> None:
        self.snapshot = snapshot
        self.calls = []

    def set_output(self, output_name: str, enabled: bool) -> PlcCommandResult:
        self.calls.append((output_name, enabled))
        return PlcCommandResult(True, "")

    def read_snapshot(self) -> PlcVacuumSnapshot:
        return self.snapshot

    def wait_for_attachment(self, channels, timeout_s) -> PlcVacuumSnapshot:
        self.calls.append(("wait_for_attachment", tuple(channels), timeout_s))
        return self.snapshot


def test_vacuum_state_projection_keeps_fixed_channel_order() -> None:
    channels = build_vacuum_channel_states(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=True,
            right_attached=False,
            left_valve_open=True,
            right_valve_open=False,
            pump_enabled=True,
        )
    )

    assert [item.channel for item in channels] == ["left", "right"]
    assert channels[0].attached
    assert not channels[1].attached
    assert all(item.data_fresh for item in channels)


def test_grip_succeeds_only_after_every_target_channel_is_attached() -> None:
    io = FakeVacuumIo(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=True,
            right_attached=True,
            left_valve_open=True,
            right_valve_open=True,
            pump_enabled=True,
        )
    )
    result = VacuumAdapterCore(io, grip_verify_timeout_s=2.0).execute_goal(
        GRIP, ["left", "right"], "default", "pick"
    )

    assert result.succeeded
    assert result.overall_verification_level == ATTACHED_VERIFIED
    assert io.calls == [
        ("pump", True),
        ("left", True),
        ("right", True),
        ("wait_for_attachment", ("left", "right"), 2.0),
    ]


def test_release_does_not_turn_off_common_pump() -> None:
    io = FakeVacuumIo(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=False,
            right_attached=True,
            left_valve_open=False,
            right_valve_open=True,
            pump_enabled=True,
        )
    )
    result = VacuumAdapterCore(io).execute_goal(RELEASE, ["left"], "default", "release")

    assert result.succeeded
    assert result.overall_verification_level == UNVERIFIED
    assert io.calls == [("left", False)]


def test_unknown_grip_profile_is_rejected_without_plc_writes() -> None:
    io = FakeVacuumIo(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=False,
            right_attached=False,
            left_valve_open=False,
            right_valve_open=False,
            pump_enabled=False,
        )
    )
    result = VacuumAdapterCore(io).execute_goal(GRIP, ["left"], "unknown", "pick")

    assert not result.succeeded
    assert not result.accepted
    assert "unsupported grip_profile_id" in result.error
    assert io.calls == []


def test_concurrent_vacuum_goal_rejects_busy_without_plc_writes() -> None:
    io = FakeVacuumIo(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=False,
            right_attached=False,
            left_valve_open=False,
            right_valve_open=False,
            pump_enabled=False,
        )
    )
    core = VacuumAdapterCore(io)
    core._active_operation = True

    result = core.execute_goal(GRIP, ["left"], "default", "pick")

    assert not result.succeeded
    assert not result.accepted
    assert result.overall_verification_level == UNVERIFIED
    assert "already running" in result.error
    assert io.calls == []


def test_pump_disable_rejects_possible_load_held() -> None:
    io = FakeVacuumIo(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=True,
            right_attached=False,
            left_valve_open=True,
            right_valve_open=False,
            pump_enabled=True,
        )
    )

    result = VacuumAdapterCore(io).set_pump_enabled(False, "maintenance")

    assert not result.accepted
    assert result.error_code == PumpErrorCode.POSSIBLE_LOAD_HELD
    assert io.calls == []
