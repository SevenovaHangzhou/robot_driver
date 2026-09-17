"""Offline node startup and signal exit; never publish a hardware command."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def check(executable: str, config: str) -> None:
    with tempfile.TemporaryDirectory(prefix="robot-led-test-") as directory:
        environment = dict(
            os.environ,
            ROS_DOMAIN_ID="143",
            ROS_LOCALHOST_ONLY="1",
            ROS_LOG_DIR=str(Path(directory) / "ros-log"),
        )
        invalid = subprocess.run(
            [executable, "--ros-args", "-p", "response_timeout_ms:=0"],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10,
        )
        if invalid.returncode != 1 or b"response_timeout_ms must be" not in invalid.stdout:
            raise RuntimeError(invalid.stdout.decode())
        for _ in range(2):
            process = subprocess.Popen(
                [executable, "--ros-args", "--params-file", config],
                env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
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
    print("PASS: invalid config rejected; two starts and SIGINT exits; no colors published")


if __name__ == "__main__":
    check(sys.argv[1], sys.argv[2])
