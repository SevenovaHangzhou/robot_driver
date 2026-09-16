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
    VacuumAdapterCore,
    build_vacuum_channel_states,
)
from control_api_adapter.public_error import PublicErrorCode  # noqa: E402


class FakeVacuumIo:
    def __init__(self, snapshot: PlcVacuumSnapshot) -> None:
        self.snapshot = snapshot
        self.calls = []

    def set_output(self, output_name: str, enabled: bool) -> PlcCommandResult:
        self.calls.append((output_name, enabled))
        return PlcCommandResult(True, "", PublicErrorCode.SUCCESS)

    def read_snapshot(self) -> PlcVacuumSnapshot:
        return self.snapshot

    def wait_for_attachment(
        self, channels, timeout_s, cancel_requested, state_callback
    ) -> PlcVacuumSnapshot:
        self.calls.append(("wait_for_attachment", tuple(channels), timeout_s))
        state_callback(self.snapshot)
        return self.snapshot

    def wait_for_release(
        self, timeout_s, cancel_requested, state_callback
    ) -> PlcVacuumSnapshot:
        self.calls.append(("wait_for_release", timeout_s))
        state_callback(self.snapshot)
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
    assert channels[0].valve_commanded_open
    assert channels[1].valve_commanded_open
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
        ("wait_for_attachment", ("left", "right"), 2.0),
    ]


def test_grip_publishes_state_while_waiting_for_attachment() -> None:
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
    feedback = []

    result = VacuumAdapterCore(io).execute_goal(
        GRIP,
        ["left"],
        "default",
        "pick",
        feedback_callback=feedback.append,
    )

    assert result.succeeded
    assert len(feedback) >= 3
    assert feedback[-1][0].attached


def test_release_turns_relay_off_and_waits_for_zero_kpa() -> None:
    io = FakeVacuumIo(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=False,
            right_attached=True,
            left_valve_open=False,
            right_valve_open=True,
            pump_enabled=False,
            vacuum_pressure_valid=True,
            vacuum_pressure_kpa=0.0,
            vacuum_released=True,
        )
    )
    result = VacuumAdapterCore(io, release_verify_timeout_s=2.0).execute_goal(
        RELEASE, ["left"], "default", "release"
    )

    assert result.succeeded
    assert result.overall_verification_level == UNVERIFIED
    assert io.calls == [("pump", False), ("wait_for_release", 2.0)]


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
    assert result.error.code == PublicErrorCode.INVALID_GOAL
    assert not result.error.retryable
    assert "unsupported grip_profile_id" in result.error.message
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
    assert result.error.code == PublicErrorCode.RT_OPERATION_IN_PROGRESS
    assert result.error.retryable
    assert "already running" in result.error.message
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
    assert result.error.code == PublicErrorCode.RT_POSSIBLE_LOAD_HELD
    assert not result.error.retryable
    assert io.calls == []


def test_grip_reports_retryable_error_when_attachment_is_not_established() -> None:
    io = FakeVacuumIo(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=False,
            right_attached=False,
            left_valve_open=True,
            right_valve_open=False,
            pump_enabled=True,
        )
    )

    result = VacuumAdapterCore(io).execute_goal(
        GRIP, ["left"], "default", "pick"
    )

    assert not result.succeeded
    assert result.accepted
    assert result.error.code == PublicErrorCode.RT_VACUUM_NOT_ESTABLISHED
    assert result.error.retryable


def test_grip_cancel_leaves_pump_and_valve_outputs_unchanged() -> None:
    io = FakeVacuumIo(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=False,
            right_attached=True,
            left_valve_open=True,
            right_valve_open=True,
            pump_enabled=True,
        )
    )
    cancel_checks = iter((False, False, True))

    result = VacuumAdapterCore(io).execute_goal(
        GRIP,
        ["left"],
        "default",
        "pick",
        cancel_requested=lambda: next(cancel_checks, True),
    )

    assert not result.succeeded
    assert result.accepted
    assert result.error.code == PublicErrorCode.CANCELED
    assert ("pump", False) not in io.calls
    assert ("left", False) not in io.calls
    assert ("right", False) not in io.calls


def test_release_timeout_keeps_relay_off_and_reports_pressure() -> None:
    io = FakeVacuumIo(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=False,
            right_attached=False,
            left_valve_open=False,
            right_valve_open=False,
            pump_enabled=False,
            vacuum_pressure_valid=True,
            vacuum_pressure_kpa=-8.5,
            vacuum_released=False,
        )
    )

    result = VacuumAdapterCore(io, release_verify_timeout_s=1.0).execute_goal(
        RELEASE, ["left"], "default", "release"
    )

    assert not result.succeeded
    assert result.accepted
    assert result.error.code == PublicErrorCode.TIMEOUT
    assert "-8.500 kPa" in result.error.message
    assert io.calls == [("pump", False), ("wait_for_release", 1.0)]


def test_output_echo_without_attachment_never_reports_grip_success() -> None:
    io = FakeVacuumIo(
        PlcVacuumSnapshot(
            connected=True,
            data_fresh=True,
            left_attached=False,
            right_attached=False,
            left_valve_open=True,
            right_valve_open=False,
            pump_enabled=True,
        )
    )

    result = VacuumAdapterCore(io).execute_goal(
        GRIP, ["left"], "default", "pick"
    )

    assert not result.succeeded
    assert result.error.code == PublicErrorCode.RT_VACUUM_NOT_ESTABLISHED
    assert result.channel_results[0].valve_actuation_completed
    assert result.channel_results[0].verification_level == UNVERIFIED
    assert ("pump", False) not in io.calls
    assert ("left", False) not in io.calls
