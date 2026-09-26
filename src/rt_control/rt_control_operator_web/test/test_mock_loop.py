"""ELECTRI-109 Mock closed loop: web protocol -> operator node -> swerve controller.

Software-only: mock_components hardware, synthetic sensors, no device access,
isolated ROS domain set by CMake (ROS_DOMAIN_ID=109, ROS_LOCALHOST_ONLY=1).
Passing this test is Mock evidence only, never hardware or HIL acceptance.
"""

import asyncio
import math
import os
import re
import signal
import subprocess
import threading
import time
from pathlib import Path

import pytest
import rclpy
from aiohttp import ClientSession
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

import robot_interfaces_qos

HERE = Path(__file__).resolve().parent
LAUNCH = HERE / "mock" / "mock_chassis.launch.py"
URL = "http://127.0.0.1:18109"
TOKEN = "electri109-mock-loop-token"
# Module order FL, FR, RL, RR (V3 Robot Model joint names).
STEER = ["caster04_joint", "caster01_joint", "caster03_joint", "caster02_joint"]
DRIVE = ["wheel04_joint", "wheel01_joint", "wheel03_joint", "wheel02_joint"]
MODULES = [(0.115, 0.305), (0.115, -0.305), (-0.495, 0.305), (-0.495, -0.305)]
COMMAND_TOPIC = "/swerve_controller/cmd_vel"
MAX_FIRST_COMMAND_LATENCY_S = 0.1
SEND_PERIOD_S = 0.033


class Monitor:
    def __init__(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node("operator_web_mock_loop_monitor")
        self.lock = threading.Lock()
        self.joints: dict[str, tuple[float, float]] = {}
        self.commands: list[tuple[float, float, float, float]] = []
        self.node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.node.create_subscription(
            Twist, COMMAND_TOPIC, self._on_command, robot_interfaces_qos.control()
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _on_joints(self, message: JointState) -> None:
        with self.lock:
            for i, name in enumerate(message.name):
                position = message.position[i] if i < len(message.position) else math.nan
                velocity = message.velocity[i] if i < len(message.velocity) else math.nan
                self.joints[name] = (position, velocity)

    def _on_command(self, message: Twist) -> None:
        with self.lock:
            self.commands.append(
                (time.monotonic(), message.linear.x, message.linear.y, message.angular.z)
            )

    def joint_state(self) -> dict[str, tuple[float, float]]:
        with self.lock:
            return dict(self.joints)

    def commands_since(self, start: float) -> list[tuple[float, float, float, float]]:
        with self.lock:
            return [c for c in self.commands if c[0] >= start]

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()


@pytest.fixture(scope="module")
def stack(tmp_path_factory):
    token_file = tmp_path_factory.mktemp("token") / "token"
    token_file.write_text(TOKEN, encoding="utf-8")
    os.chmod(token_file, 0o600)
    env = dict(os.environ, RT_OPERATOR_WEB_TOKEN_FILE=str(token_file))
    log = open(tmp_path_factory.mktemp("log") / "launch.log", "w+", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "launch", str(LAUNCH)],
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    monitor = Monitor()
    try:
        yield monitor
    finally:
        monitor.close()
        # Signal only launch; it forwards SIGINT once to every child.
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.seek(0)
        output = log.read()
        log.close()
        print(output[-6000:])
        assert re.search(
            r"rt_control_operator_web-\d+\]: process has finished cleanly", output
        ), "operator web process did not exit cleanly"


async def wait_for(predicate, timeout: float, interval: float = 0.02) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        await asyncio.sleep(interval)
    return predicate()


async def open_session(http: ClientSession):
    for _ in range(300):
        try:
            ws = await http.ws_connect(f"{URL}/ws")
            break
        except OSError:
            await asyncio.sleep(0.1)
    else:
        raise AssertionError("operator web server did not start")
    await ws.send_json({"type": "auth", "token": TOKEN})
    reply = await ws.receive_json(timeout=5)
    assert reply["type"] == "auth_ok"
    return ws


async def wait_status(ws, predicate, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = await ws.receive_json(timeout=max(0.05, deadline - time.monotonic()))
        if message["type"] == "status" and predicate(message):
            return message
    raise AssertionError("expected status not observed")


async def wait_type(ws, kind: str, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = await ws.receive_json(timeout=max(0.05, deadline - time.monotonic()))
        if message["type"] == kind:
            return message
    raise AssertionError(f"no {kind} message")


async def hold(ws, x: float, y: float, yaw: int, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        await ws.send_json({"type": "drive", "x": x, "y": y, "yaw": yaw})
        await asyncio.sleep(SEND_PERIOD_S)


def drives_moving(monitor: Monitor) -> bool:
    state = monitor.joint_state()
    return all(name in state and abs(state[name][1]) > 0.1 for name in DRIVE)


def drives_stopped(monitor: Monitor) -> bool:
    state = monitor.joint_state()
    return all(name in state and abs(state[name][1]) < 1e-3 for name in DRIVE)


def steering_near(monitor: Monitor, targets, tol: float = 0.05) -> bool:
    state = monitor.joint_state()
    for name, target in zip(STEER, targets):
        if name not in state:
            return False
        error = abs(state[name][0] - target)
        # A swerve module may reach the equivalent reversed heading.
        if min(error, abs(error - math.pi)) > tol:
            return False
    return True


async def wait_command_zero(monitor: Monitor, since: float, timeout: float = 3.0) -> None:
    assert await wait_for(
        lambda: (lambda c: bool(c) and c[-1][1:] == (0.0, 0.0, 0.0))(monitor.commands_since(since)),
        timeout,
    ), "published command did not ramp to zero"


def assert_smooth_rampdown(samples, index: int, decel: float) -> None:
    values = [abs(c[index]) for c in samples]
    assert values, "no commands during ramp-down"
    assert values[0] > 0.1, "release must not cut the command to zero"
    steps = [b - a for a, b in zip(values, values[1:])]
    assert all(s <= 1e-9 for s in steps), "ramp-down must be monotonic"
    # 50 Hz nominal; allow scheduling jitter up to 2x the nominal period.
    assert max(-s for s in steps) <= decel * 0.04 + 1e-6
    assert values[-1] == 0.0


def test_mock_loop_joystick_ramps_watchdog_softstop_disconnect(stack: Monitor) -> None:
    monitor = stack

    async def scenario() -> None:
        async with ClientSession() as http:
            ws = await open_session(http)
            status = await wait_status(
                ws,
                lambda s: s["robot"]["controller_connected"]
                and s["robot"]["geometry"] is not None
                and s["robot"]["battery"] is not None
                and any(u is not None for u in s["robot"]["ultrasonic"]),
                60,
            )
            assert status["robot"]["geometry"]["module_x"] == [x for x, _ in MODULES]
            # Top view derived from the shared V3 Robot Model, matching the controller.
            status = await wait_status(ws, lambda s: s["robot"]["model"]["version"], 30)
            assert status["robot"]["model"]["controller_mismatch"] is False
            await ws.send_json({"type": "get_model"})
            model = await wait_type(ws, "model", 5)
            assert [m["joint"] for m in model["modules"]] == STEER
            for module, (x, y) in zip(model["modules"], MODULES):
                assert (module["x"], module["y"]) == pytest.approx((x, y), abs=1e-3)
                assert {p["role"] for p in module["parts"]} == {"steer", "wheel"}
            assert any(not p["hidden"] for p in model["parts"])
            assert model["missing_meshes"] == []
            # V1 boundary: nothing may publish the Motion-owned N-04 topic.
            assert monitor.node.count_publishers("/cmd_vel_safe") == 0

            await ws.send_json({"type": "acquire"})
            await wait_status(ws, lambda s: s["lease"] == "you", 5)
            await ws.send_json({"type": "gear", "gear": "sport"})
            await wait_status(ws, lambda s: s["gear"] == "sport", 5)

            # Latency: stick input to the first non-zero published command.
            sent = time.monotonic()
            await ws.send_json({"type": "drive", "x": 0.0, "y": 1.0, "yaw": 0})
            assert await wait_for(
                lambda: any(c[2] > 0.0 for c in monitor.commands_since(sent)), 1.0
            )
            first = next(c for c in monitor.commands_since(sent) if c[2] > 0.0)
            latency = first[0] - sent
            print(f"first non-zero command latency: {latency * 1000:.1f} ms")
            assert latency <= MAX_FIRST_COMMAND_LATENCY_S

            # Crab left: all modules steer to +/-90 deg and the wheels turn.
            await hold(ws, 0.0, 1.0, 0, 3.0)
            assert steering_near(monitor, [math.pi / 2] * 4)
            assert drives_moving(monitor)

            # Release: smooth S-curve ramp-down, never a step to zero.
            released = time.monotonic()
            await hold(ws, 0.0, 0.0, 0, 1.5)
            await wait_command_zero(monitor, released)
            assert_smooth_rampdown(monitor.commands_since(released), 2, 0.5)
            assert await wait_for(lambda: drives_stopped(monitor), 2.0)

            # Yaw CCW: modules steer tangentially (atan2 of module position).
            yaw_targets = [math.atan2(x, -y) for x, y in MODULES]
            await hold(ws, 0.0, 0.0, 1, 3.0)
            assert steering_near(monitor, yaw_targets)
            assert drives_moving(monitor)
            # Stop sending: the 200 ms watchdog revokes the lease and ramps down.
            silent = time.monotonic()
            status = await wait_status(ws, lambda s: s["lease"] == "none", 2)
            assert status["stop_reason"] == "command_timeout"
            assert await wait_for(lambda: drives_stopped(monitor), 3.0)
            await wait_command_zero(monitor, silent)
            assert_smooth_rampdown(monitor.commands_since(silent), 3, 1.0)

            # Soft stop while moving: faster ramp, lease revoked.
            await ws.send_json({"type": "acquire"})
            await wait_status(ws, lambda s: s["lease"] == "you", 5)
            await hold(ws, 1.0, 0.0, 0, 2.5)
            assert drives_moving(monitor)
            await ws.send_json({"type": "soft_stop"})
            status = await wait_status(ws, lambda s: s["lease"] == "none", 2)
            assert status["stop_reason"] == "soft_stop"
            assert await wait_for(lambda: drives_stopped(monitor), 2.0)

            # Disconnect while moving: lease revoked, chassis ramps to a stop.
            await ws.send_json({"type": "acquire"})
            await wait_status(ws, lambda s: s["lease"] == "you", 5)
            await hold(ws, -1.0, 0.0, 0, 2.5)
            assert drives_moving(monitor)
            await ws.close()
            assert await wait_for(lambda: drives_stopped(monitor), 3.0)

            ws2 = await open_session(http)
            status = await wait_status(ws2, lambda s: True, 2)
            assert status["lease"] == "none"
            assert status["stop_reason"] == "client_disconnected"
            await ws2.close()

    asyncio.run(scenario())
