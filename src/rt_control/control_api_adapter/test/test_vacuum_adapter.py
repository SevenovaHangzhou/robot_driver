import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from control_api_adapter.public_error import PublicErrorCode  # noqa: E402
from control_api_adapter.vacuum_adapter import (  # noqa: E402
    LEFT,
    OUTCOME_CONFIRMED,
    OUTCOME_NOT_EXECUTED,
    OUTCOME_UNKNOWN,
    RIGHT,
    TARGET_RELEASE,
    TARGET_SUCTION,
    VACUUM_ATTACHED,
    VACUUM_INTERMEDIATE,
    VACUUM_RELEASED,
    VACUUM_UNKNOWN,
    ChannelSnapshot,
    PlcCommandResult,
    PlcVacuumSnapshot,
    VacuumAdapterCore,
    classify_vacuum,
)


def channel(
    name,
    *,
    pressure=-1.0,
    pressure_valid=True,
    valve_on=False,
    valve_valid=True,
):
    return ChannelSnapshot(
        name,
        pressure_raw=1000,
        pressure_valid=pressure_valid,
        pressure_kpa=pressure,
        valve_valid=valve_valid,
        valve_on=valve_on,
    )


def snapshot(
    *,
    configured=True,
    fresh=True,
    pump_on=True,
    pump_valid=True,
    left=None,
    right=None,
):
    return PlcVacuumSnapshot(
        configured,
        configured,
        fresh,
        pump_valid,
        pump_on,
        left or channel(LEFT),
        right or channel(RIGHT),
    )


class FakeVacuumIo:
    def __init__(self, state: PlcVacuumSnapshot) -> None:
        self.state = state
        self.calls = []
        self.results = {}

    def set_output(self, output_name: str, enabled: bool) -> PlcCommandResult:
        self.calls.append((output_name, enabled))
        result = self.results.get(output_name)
        if isinstance(result, list):
            return result.pop(0)
        if result is not None:
            return result
        return PlcCommandResult(
            True,
            OUTCOME_CONFIRMED,
            enabled,
            True,
            "confirmed",
            PublicErrorCode.SUCCESS,
        )

    def read_snapshot(self) -> PlcVacuumSnapshot:
        return self.state


def failed(outcome, message="failed"):
    code = (
        PublicErrorCode.RT_PLC_UNAVAILABLE
        if outcome == OUTCOME_UNKNOWN
        else PublicErrorCode.RT_PUMP_COMMAND_REJECTED
    )
    return PlcCommandResult(True, outcome, False, False, message, code)


def test_vacuum_thresholds_use_single_fresh_sample_and_exact_boundaries() -> None:
    assert classify_vacuum(channel(LEFT, pressure=-60.0), data_fresh=True) == VACUUM_ATTACHED
    assert classify_vacuum(channel(LEFT, pressure=-59.999), data_fresh=True) == VACUUM_INTERMEDIATE
    assert classify_vacuum(channel(LEFT, pressure=-1.001), data_fresh=True) == VACUUM_INTERMEDIATE
    assert classify_vacuum(channel(LEFT, pressure=-1.0), data_fresh=True) == VACUUM_RELEASED
    assert classify_vacuum(channel(LEFT, pressure_valid=False), data_fresh=True) == VACUUM_UNKNOWN
    assert classify_vacuum(channel(LEFT), data_fresh=False) == VACUUM_UNKNOWN


def test_suction_confirms_only_output_and_does_not_wait_for_pressure() -> None:
    io = FakeVacuumIo(
        snapshot(left=channel(LEFT, pressure=-5.0, valve_on=False))
    )
    result = VacuumAdapterCore(io).execute_valves(
        TARGET_SUCTION, [LEFT], bytes(16)
    )

    assert result.succeeded
    assert result.channel_results[0].outcome == OUTCOME_CONFIRMED
    assert io.calls == [(LEFT, True)]


def test_suction_is_rejected_when_pump_is_not_valid_and_enabled() -> None:
    io = FakeVacuumIo(snapshot(pump_on=False))
    result = VacuumAdapterCore(io).execute_valves(
        TARGET_SUCTION, [LEFT], bytes(16)
    )

    assert not result.accepted
    assert result.error.code == PublicErrorCode.RT_PUMP_UNAVAILABLE
    assert io.calls == []


def test_release_does_not_require_pressure_observation() -> None:
    io = FakeVacuumIo(
        snapshot(
            left=channel(LEFT, pressure_valid=False, valve_on=True, valve_valid=True),
            right=channel(RIGHT, pressure_valid=False, valve_on=False, valve_valid=True),
        )
    )
    result = VacuumAdapterCore(io).execute_valves(
        TARGET_RELEASE, [LEFT], bytes(16)
    )

    assert result.succeeded
    assert io.calls == [(LEFT, False)]


def test_dual_goal_with_busy_channel_is_rejected_before_any_write() -> None:
    io = FakeVacuumIo(snapshot())
    core = VacuumAdapterCore(io)
    core._active_channels.add(LEFT)

    result = core.execute_valves(TARGET_RELEASE, [LEFT, RIGHT], bytes(16))

    assert not result.accepted
    assert all(item.outcome == OUTCOME_NOT_EXECUTED for item in result.channel_results)
    assert io.calls == []


def test_other_channel_can_execute_while_one_channel_is_busy() -> None:
    io = FakeVacuumIo(snapshot())
    core = VacuumAdapterCore(io)
    core._active_channels.add(LEFT)

    result = core.execute_valves(TARGET_RELEASE, [RIGHT], bytes(16))

    assert result.succeeded
    assert io.calls == [(RIGHT, False)]


def test_dual_partial_success_is_preserved_without_rollback() -> None:
    io = FakeVacuumIo(snapshot())
    io.results[RIGHT] = failed(OUTCOME_UNKNOWN)

    result = VacuumAdapterCore(io).execute_valves(
        TARGET_RELEASE, [LEFT, RIGHT], bytes(16)
    )

    assert not result.succeeded
    assert [item.outcome for item in result.channel_results] == [
        OUTCOME_CONFIRMED,
        OUTCOME_UNKNOWN,
    ]
    assert io.calls == [(LEFT, False), (RIGHT, False)]
    assert (LEFT, True) not in io.calls


def test_cancel_after_first_side_keeps_completed_output_and_skips_second() -> None:
    io = FakeVacuumIo(snapshot())
    checks = iter((False, True))

    result = VacuumAdapterCore(io).execute_valves(
        TARGET_RELEASE,
        [LEFT, RIGHT],
        bytes(16),
        cancel_requested=lambda: next(checks, True),
    )

    assert result.canceled
    assert [item.outcome for item in result.channel_results] == [
        OUTCOME_CONFIRMED,
        OUTCOME_NOT_EXECUTED,
    ]
    assert io.calls == [(LEFT, False)]


def test_same_request_id_is_trace_only_and_does_not_deduplicate() -> None:
    io = FakeVacuumIo(snapshot())
    core = VacuumAdapterCore(io)
    request_id = bytes(range(16))

    first = core.execute_valves(TARGET_RELEASE, [LEFT], request_id)
    second = core.execute_valves(TARGET_RELEASE, [LEFT], request_id)

    assert first.succeeded and second.succeeded
    assert io.calls == [(LEFT, False), (LEFT, False)]
    assert first.channel_results[0].attempt_id != second.channel_results[0].attempt_id


def test_pump_disable_requires_both_valves_and_both_pressures_released() -> None:
    io = FakeVacuumIo(
        snapshot(
            left=channel(LEFT, pressure=-1.0, valve_on=False),
            right=channel(RIGHT, pressure=-1.0, valve_on=False),
        )
    )
    core = VacuumAdapterCore(io)

    accepted = core.set_pump_enabled(False, "shutdown")
    assert accepted.accepted
    assert accepted.output.outcome == OUTCOME_CONFIRMED

    io.calls.clear()
    io.state = snapshot(left=channel(LEFT, pressure=-1.001, valve_on=False))
    rejected = core.set_pump_enabled(False, "shutdown")
    assert not rejected.accepted
    assert rejected.error.code == PublicErrorCode.RT_POSSIBLE_LOAD_HELD
    assert io.calls == []


def test_pump_disable_rejects_open_or_unknown_valve_state() -> None:
    io = FakeVacuumIo(snapshot(left=channel(LEFT, valve_on=True)))
    result = VacuumAdapterCore(io).set_pump_enabled(False)
    assert not result.accepted
    assert io.calls == []

    io.state = snapshot(left=channel(LEFT, valve_valid=False))
    result = VacuumAdapterCore(io).set_pump_enabled(False)
    assert not result.accepted
    assert io.calls == []


def test_unknown_write_needs_explicit_output_confirmation_not_periodic_readback() -> None:
    io = FakeVacuumIo(snapshot())
    io.results[LEFT] = failed(OUTCOME_UNKNOWN)
    core = VacuumAdapterCore(io)
    action = core.execute_valves(TARGET_RELEASE, [LEFT], bytes(16))
    assert action.channel_results[0].outcome == OUTCOME_UNKNOWN

    io.state = snapshot(left=channel(LEFT, valve_valid=False))
    blocked = core.set_pump_enabled(False)
    assert not blocked.accepted

    # Even a nominally fresh periodic sample may precede the timed-out write.
    io.state = snapshot(left=channel(LEFT, pressure=-1.0, valve_on=False, valve_valid=True))
    calls_before = list(io.calls)
    blocked_by_unknown = core.set_pump_enabled(False)
    assert not blocked_by_unknown.accepted
    assert io.calls == calls_before

    # Only a new, caller-requested write with its own confirmation resolves it.
    io.results.pop(LEFT)
    resolved = core.execute_valves(TARGET_RELEASE, [LEFT], bytes(16))
    assert resolved.succeeded
    allowed = core.set_pump_enabled(False)
    assert allowed.accepted


def test_unconfigured_hardware_fails_closed_without_io_writes() -> None:
    io = FakeVacuumIo(snapshot(configured=False, fresh=False))
    core = VacuumAdapterCore(io)

    valve = core.execute_valves(TARGET_RELEASE, [LEFT], bytes(16))
    pump = core.set_pump_enabled(True)

    assert not valve.accepted
    assert not pump.accepted
    assert io.calls == []
