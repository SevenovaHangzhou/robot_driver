"""The standard ROS response must describe the exact stationary command."""

from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
from control_msgs.srv import QueryTrajectoryState

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integ_operator_web.arm.rolling_hold import RollingHold
from integ_operator_web.arm.console_types import ConsoleError
from test_mode_service import ServiceClient


@pytest.mark.parametrize("bad", [None, "order", "position", "velocity", "acceleration", "inactive"])
def test_only_an_exact_stationary_command_response_can_seed_a_trajectory(bad):
    response = QueryTrajectoryState.Response(
        success=True,
        name=["a", "b"],
        position=[0.1, 0.2],
        velocity=[0.0, 0.0],
        acceleration=[0.0, 0.0],
    )
    if bad == "order":
        response.name = ["b", "a"]
    elif bad == "position":
        response.position = [float("nan"), 0.2]
    elif bad == "velocity":
        response.velocity = [0.0, 0.001]
    elif bad == "acceleration":
        response.acceleration = []
    elif bad == "inactive":
        response.success = False
    client = ServiceClient(response)
    query = RollingHold(NS(create_client=lambda *a, **kw: client), "controller", None)
    if bad is None:
        assert query.query(["a", "b"]) == {"a": 0.1, "b": 0.2}
    else:
        with pytest.raises(ConsoleError):
            query.query(["a", "b"])
    assert client.requests[0].time.sec == client.requests[0].time.nanosec == 0


def test_missing_service_does_not_fall_back_to_measured_positions():
    client = ServiceClient(None)
    client.service_is_ready = lambda: False
    query = RollingHold(NS(create_client=lambda *a, **kw: client), "controller", None)
    with pytest.raises(ConsoleError, match="不可用"):
        query.query(["a", "b"])
    assert not client.requests
