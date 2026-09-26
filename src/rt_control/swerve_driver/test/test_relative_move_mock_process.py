"""Offline executable lifecycle only; no hardware plugins or device paths."""
import os
import signal
import subprocess
import time


BINARY = os.environ["SWERVE_RELATIVE_MOCK_BINARY"]
CONFIG = os.environ["SWERVE_RELATIVE_MOCK_CONFIG"]
ENV = {**os.environ, "ROS_LOCALHOST_ONLY": "1"}


def test_missing_mock_selection_and_missing_config_refuse():
    for arguments, diagnostic in (
        ([], "explicit_no_device_mock=true required"),
        (["--ros-args", "-p", "explicit_no_device_mock:=true"], "base_frame is required"),
    ):
        completed = subprocess.run(
            [BINARY, *arguments], env=ENV, capture_output=True, text=True, timeout=10
        )
        assert completed.returncode == 1, completed.stderr
        assert diagnostic in completed.stderr


def test_sigint_during_startup_and_after_start_then_restart():
    for delay in (0.1, 0.25, 1.0):
        process = subprocess.Popen(
            [BINARY, "--ros-args", "--params-file", CONFIG], env=ENV,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        try:
            time.sleep(delay)
            assert process.poll() is None
            process.send_signal(signal.SIGINT)
            stdout, stderr = process.communicate(timeout=10)
            assert process.returncode == 0, (process.returncode, stdout, stderr)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)
