"""Chassis status as the console sees a ready, idle scope runtime."""

import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integ_operator_web.chassis_connection import ChassisConnection  # noqa: E402
from std_msgs.msg import String  # noqa: E402

STEERING = [f"steer{i}" for i in range(1, 5)]
DRIVE = [f"wheel{i}" for i in range(1, 5)]
TOPIC = "/rt/base/cmd_vel"


class ReadyClient:
    def service_is_ready(self):
        return True


class FakeNode:
    def __init__(self):
        self.config = SimpleNamespace(controller_node="/swerve_controller", command_topic=TOPIC)

    def create_subscription(self, *args, **kwargs):
        return None

    def create_client(self, *args, **kwargs):
        return ReadyClient()

    def create_timer(self, *args, **kwargs):
        return None


def ready_connection():
    connection = ChassisConnection(FakeNode(), software_test_only=False, tuning=None)
    connection.capabilities(
        String(
            data=json.dumps(
                {
                    "schema_version": 1,
                    "capabilities": [
                        {"name": "enable"},
                        {"name": "chassis_velocity", "topic": TOPIC},
                    ],
                }
            )
        )
    )
    now = time.monotonic()
    connection.hardware, connection.hardware_at = True, now
    connection.steering, connection.drive, connection.topic = STEERING, DRIVE, TOPIC
    connection.parameters_at = now
    connection.managed = STEERING + DRIVE
    connection.state, connection.diag_at = "IDLE", now
    return connection


def test_ready_idle_chassis_is_connected_and_can_enable():
    status = ready_connection().status()

    assert status["connected"] is True
    assert status["can_enable"] is True
    assert status["message"] == "未使能"
