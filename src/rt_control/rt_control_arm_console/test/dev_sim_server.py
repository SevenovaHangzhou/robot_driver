#!/usr/bin/env python3
"""Test-only page check without ROS: the console with a simulated arm.

The simulated driver plays each trajectory in real time and writes the sampled
positions back as feedback. Enable only flips the simulated state.
Usage: dev_sim_server.py <robot_description.xml> <description share> <port> <token file>
"""

from pathlib import Path
import sys
import threading
import time

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rt_control_arm_console.console import ArmConsole, config_from_mapping  # noqa: E402
from rt_control_arm_console.model import load_arm_model  # noqa: E402
from rt_control_arm_console.server import ConsoleServer  # noqa: E402

PACKAGE = Path(__file__).resolve().parents[1]
JOINTS = [f"left_joint{index}" for index in range(1, 8)]


class SimulatedArm:
    def __init__(self):
        self.console = None
        self.generation = 0
        self.lock = threading.Lock()

    def send(self, joints, points, done, start):
        with self.lock:
            self.generation += 1
            generation = self.generation

        def play():
            for point in points:
                while time.monotonic() - start < point.time:
                    time.sleep(0.002)
                with self.lock:
                    if generation != self.generation:
                        done(False, "preempted")
                        return
                self.console.update(positions=dict(zip(joints, point.positions)))
            done(True, "")

        threading.Thread(target=play, daemon=True).start()

    def cancel(self):
        with self.lock:
            self.generation += 1

    def set_enabled(self, enabled):
        time.sleep(0.5)
        self.console.update(
            enable_state="ENABLED" if enabled else "IDLE", controller_active=enabled
        )
        return {"accepted": True, "enabled": enabled, "message": ""}


def main():
    urdf, share, port, token_file = sys.argv[1:5]
    config = config_from_mapping(yaml.safe_load((PACKAGE / "config/console.yaml").read_text()))
    arm = SimulatedArm()
    console = ArmConsole(config, "left_arm_jtc", arm, Path("/tmp/arm-console-sim-poses.json"))
    arm.console = console
    console.set_model(load_arm_model(Path(urdf).read_text(), JOINTS))
    console.update(
        positions={name: 0.0 for name in JOINTS},
        enable_state="IDLE",
        capability={"topic": "/left_arm_jtc/follow_joint_trajectory"},
        mock=True,
    )

    def heartbeat():
        while True:
            now = time.monotonic()
            console.update(feedback_at=now, enable_at=now, controller_at=now)
            time.sleep(0.02)

    threading.Thread(target=heartbeat, daemon=True).start()
    server = ConsoleServer(
        ("127.0.0.1", int(port)),
        console,
        Path(token_file).read_text().strip(),
        PACKAGE / "web",
        lambda name: share if name == "robot_description" else "/nonexistent",
        arm.set_enabled,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
