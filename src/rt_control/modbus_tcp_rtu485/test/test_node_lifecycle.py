"""Offline node startup and signal exit; never access hardware."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def run_and_stop(command: list[str], environment: dict[str, str]) -> None:
    process = subprocess.Popen(
        command, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    try:
        time.sleep(0.5)
        if process.poll() is not None:
            raise RuntimeError(process.communicate()[0].decode())
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=5)
        if process.returncode != 0:
            raise RuntimeError(output.decode())
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()


def check(led_executable: str, led_config: str, ultrasonic_executable: str, ultrasonic_config: str) -> None:
    with tempfile.TemporaryDirectory(prefix="robot-led-test-") as directory:
        environment = dict(
            os.environ,
            ROS_DOMAIN_ID="143",
            ROS_LOCALHOST_ONLY="1",
            ROS_LOG_DIR=str(Path(directory) / "ros-log"),
        )
        invalid_led = subprocess.run(
            [led_executable, "--ros-args", "-p", "response_timeout_ms:=0"],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10,
        )
        if invalid_led.returncode != 1 or b"response_timeout_ms must be" not in invalid_led.stdout:
            raise RuntimeError(invalid_led.stdout.decode())
        invalid_ultrasonic = subprocess.run(
            [ultrasonic_executable, "--ros-args", "-p", "gateway_port:=0"],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10,
        )
        if invalid_ultrasonic.returncode != 1 or b"gateway port must be" not in invalid_ultrasonic.stdout:
            raise RuntimeError(invalid_ultrasonic.stdout.decode())
        for _ in range(2):
            run_and_stop(
                [led_executable, "--ros-args", "--params-file", led_config], environment,
            )
            run_and_stop(
                [ultrasonic_executable, "--ros-args", "--params-file", ultrasonic_config,
                 "-p", "poll_enabled:=false"],
                environment,
            )
    print("PASS: invalid configs rejected; repeat starts and SIGINT exits; no hardware access")


if __name__ == "__main__":
    check(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
