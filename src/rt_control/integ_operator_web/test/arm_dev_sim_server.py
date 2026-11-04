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
from integ_operator_web.arm.console import ArmConsole, config_from_mapping  # noqa: E402
from integ_operator_web.arm.model import load_arm_model  # noqa: E402
from integ_operator_web.arm.page import ArmPage  # noqa: E402
from integ_operator_web.operators import Operators  # noqa: E402
from integ_operator_web.session import Sessions  # noqa: E402

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
    config = config_from_mapping(yaml.safe_load((PACKAGE / "config/arm.yaml").read_text()))
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
    from aiohttp import web

    sessions = Sessions(Path(token_file).read_text().strip())
    page = ArmPage(
        console,
        sessions,
        Operators(),
        PACKAGE / "web/arm",
        lambda name: share if name == "robot_description" else "/nonexistent",
        arm.set_enabled,
        view_model_share=Path(share),
    )
    app = web.Application()
    app.router.add_post("/api/login", sessions.login)
    app.router.add_static("/static/", PACKAGE / "web", follow_symlinks=True)
    page.add_routes(app)
    print(f"arm page simulation on http://127.0.0.1:{port}/arm")
    web.run_app(app, host="127.0.0.1", port=int(port), print=None)


if __name__ == "__main__":
    main()
