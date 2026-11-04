"""Offline node startup and signal exit; never access hardware."""

import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time


def run_and_stop(
    command: list[str], environment: dict[str, str], stop_signal: signal.Signals
) -> None:
    process = subprocess.Popen(
        command, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    try:
        # Signal only after construction finished: a signal during startup is a different case.
        startup = b""
        deadline = time.monotonic() + 10.0
        while b"ready:" not in startup:
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError((startup + process.stdout.read()).decode())
            if select.select([process.stdout], [], [], 0.1)[0]:
                startup += os.read(process.stdout.fileno(), 4096)
        process.send_signal(stop_signal)
        output, _ = process.communicate(timeout=5)
        if process.returncode != 0:
            raise RuntimeError((startup + output).decode())
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
            RCUTILS_LOGGING_BUFFERED_STREAM="0",
        )
        require_rejected(
            [led_executable, "--ros-args", "-p", "response_timeout_ms:=0"], environment,
            b"response_timeout_ms must be",
        )
        require_rejected(
            [led_executable, "--ros-args", "-p", "exit_response_timeout_ms:=0"],
            environment,
            b"exit_response_timeout_ms must be",
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
            b"field_of_view_rad must be 0.6981317008",
        )
        for stop_signal in (signal.SIGINT, signal.SIGTERM):
            run_and_stop(
                [
                    led_executable,
                    "--ros-args",
                    "--params-file",
                    led_config,
                    "-p",
                    "exit_color_enabled:=false",
                ],
                environment,
                stop_signal,
            )
            run_and_stop(
                [ultrasonic_executable, "--ros-args", "--params-file", ultrasonic_config,
                 "-p", "poll_enabled:=false"],
                environment,
                stop_signal,
            )
    print(
        "PASS: invalid endpoint and A22 metadata rejected; SIGINT/SIGTERM exits; "
        "no hardware access"
    )


if __name__ == "__main__":
    check(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
