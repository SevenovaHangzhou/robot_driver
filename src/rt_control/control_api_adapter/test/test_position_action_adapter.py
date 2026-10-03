import math
import sys
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from control_api_adapter.position_action_adapter import (  # noqa: E402
    EXECUTION_EXECUTING,
    EXECUTION_HOLDING,
    EXECUTION_IDLE,
    PositionResourceCore,
    require_preview_only,
)


def test_draft_cannot_enable_an_unimplemented_actuator_backend() -> None:
    require_preview_only(False)
    with pytest.raises(ValueError, match="not admitted"):
        require_preview_only(True)


def resource(configured=True):
    return PositionResourceCore(
        "axis",
        ("joint",),
        configured=configured,
        tolerance=0.01,
        velocity_tolerance=0.02,
        feedback_timeout_s=0.5,
        settle_samples=2,
        command_timeout_s=5.0,
        minimum_position=-2.0,
        maximum_position=2.0,
    )


def test_unconfigured_and_nonfinite_targets_fail_closed() -> None:
    with pytest.raises(ValueError, match="not configured"):
        resource(False).validate_target((0.0,))
    with pytest.raises(ValueError, match="finite"):
        resource().validate_target((math.nan,))
    with pytest.raises(ValueError, match="dimension"):
        resource().validate_target((0.0, 1.0))
    with pytest.raises(ValueError, match="limits"):
        PositionResourceCore(
            "axis",
            ("joint",),
            configured=True,
            tolerance=0.01,
            velocity_tolerance=0.02,
            feedback_timeout_s=0.5,
            settle_samples=2,
            command_timeout_s=5.0,
        )
    with pytest.raises(ValueError, match="outside"):
        resource().validate_target((3.0,))


def test_busy_resource_rejects_second_goal_without_queue() -> None:
    core = resource()
    assert core.try_start(bytes(16))
    assert not core.try_start(bytes([1]) * 16)
    assert core.state()[0] == EXECUTION_EXECUTING
    core.finish(EXECUTION_HOLDING)
    assert core.try_start(bytes([2]) * 16)


def test_target_requires_fresh_position_and_velocity_tolerance() -> None:
    core = resource()
    core.update_feedback((1.0,), (0.0,), 10.0, 20_000_000_000)
    assert core.feedback_is_fresh(10.4)
    assert core.target_reached((1.005,), 10.4)
    assert not core.target_reached((1.02,), 10.4)

    core.update_feedback((1.0,), (0.03,), 10.4, 20_400_000_000)
    assert not core.target_reached((1.0,), 10.5)
    assert not core.feedback_is_fresh(11.0)


def test_finish_clears_goal_and_reports_terminal_state() -> None:
    core = resource()
    assert core.state() == (EXECUTION_IDLE, None)
    assert core.try_start(bytes([3]) * 16)
    core.finish(EXECUTION_HOLDING)
    assert core.state() == (EXECUTION_HOLDING, None)
