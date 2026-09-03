import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rt_control_operator_ui.commands import SingleFlightCommands  # noqa: E402


def test_double_click_starts_exactly_one_operation() -> None:
    commands = SingleFlightCommands()

    first = commands.begin("reset_fault")
    second = commands.begin("reset_fault")

    assert first is not None
    assert second is None
    assert commands.pending == first


def test_completion_clears_matching_operation_and_ignores_stale_result() -> None:
    commands = SingleFlightCommands()
    first = commands.begin("enable")

    assert first is not None
    assert commands.complete(first.operation_id + 1) is False
    assert commands.pending == first
    assert commands.complete(first.operation_id) is True
    assert commands.pending is None


def test_invalid_command_is_rejected() -> None:
    commands = SingleFlightCommands()

    try:
        commands.begin("reboot")
    except ValueError as exc:
        assert "reboot" in str(exc)
    else:
        raise AssertionError("invalid command was accepted")
