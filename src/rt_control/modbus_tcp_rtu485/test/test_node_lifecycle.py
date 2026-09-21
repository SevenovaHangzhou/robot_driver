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


def require_rejected(
    command: list[str], environment: dict[str, str], expected_message: bytes,
) -> None:
    result = subprocess.run(
        command, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10,
    )
    if result.returncode != 1 or expected_message not in result.stdout:
        raise RuntimeError(result.stdout.decode())


def check(led_executable: str, led_config: str, ultrasonic_executable: str, ultrasonic_config: str) -> None:
    with tempfile.TemporaryDirectory(prefix="robot-led-test-") as directory:
        environment = dict(
            os.environ,
            ROS_DOMAIN_ID="143",
            ROS_LOCALHOST_ONLY="1",
            ROS_LOG_DIR=str(Path(directory) / "ros-log"),
        )
        require_rejected(
            [led_executable, "--ros-args", "-p", "response_timeout_ms:=0"], environment,
            b"response_timeout_ms must be",
        )
        require_rejected(
            [
                led_executable,
                "--ros-args",
                "-p",
                "gateway_ports:=[502,502,502,502]",
            ],
            environment,
            b"gateway_ports and controller_addresses require six values",
        )
        require_rejected(
            [ultrasonic_executable, "--ros-args", "-p", "gateway_port:=0"], environment,
            b"gateway port must be",
        )
        require_rejected(
            [ultrasonic_executable, "--ros-args", "-p", "unit_ids:=[1,2]"], environment,
            b"E08 unit IDs 2..5 are reserved",
        )
        require_rejected(
            [ultrasonic_executable, "--ros-args", "-p", "max_range_m:=1.5"], environment,
            b"max_range_m must be 3.5",
        )
        require_rejected(
            [ultrasonic_executable, "--ros-args", "-p", "field_of_view_rad:=0.5"], environment,
            b"field_of_view_rad must be 1.0471975512",
        )
        for _ in range(2):
            run_and_stop(
                [led_executable, "--ros-args", "--params-file", led_config], environment,
            )
            run_and_stop(
                [ultrasonic_executable, "--ros-args", "--params-file", ultrasonic_config,
                 "-p", "poll_enabled:=false"],
                environment,
            )
    print(
        "PASS: invalid endpoint and A22 metadata rejected; repeat starts and SIGINT exits; "
        "no hardware access"
    )


if __name__ == "__main__":
    check(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
