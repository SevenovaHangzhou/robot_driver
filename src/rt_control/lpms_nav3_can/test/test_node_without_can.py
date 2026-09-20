"""Node/launch regression using only a deliberately nonexistent interface."""

from pathlib import Path
import os
import signal
import selectors
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest


def _executable():
    return str(
        Path(get_package_prefix("lpms_nav3_can"))
        / "lib/lpms_nav3_can/lpms_nav3_can_node"
    )


def _command(*parameters):
    result = [_executable(), "--ros-args"]
    for parameter in parameters:
        result += ["-p", parameter]
    return result


@pytest.mark.parametrize(
    "parameters,reason",
    [
        ([], "node_id must be in [1, 127]"),
        (["node_id:=1"], "can_interface must be explicitly configured"),
        (["node_id:=1", "can_interface:=can0"], "refusing reserved interface"),
        (["node_id:=1", "can_interface:=can1"], "refusing reserved interface"),
        (["node_id:=1", "can_interface:=lpms_absent"], "frame_id must be explicitly configured"),
        (
            ["node_id:=1", "can_interface:=lpms_absent", "frame_id:=/imu_link"],
            "frame_id must be explicitly configured",
        ),
    ],
)
def test_rejects_incomplete_configuration_before_connecting(parameters, reason):
    result = subprocess.run(_command(*parameters), capture_output=True, text=True, timeout=10)
    assert result.returncode == 1
    assert reason in result.stderr


def test_missing_interface_logs_failure_and_exits_cleanly_on_signal():
    assert "lpms_absent" not in os.listdir("/sys/class/net")
    process = subprocess.Popen(
        _command("node_id:=1", "can_interface:=lpms_absent", "frame_id:=imu_link"),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    output = b""
    try:
        deadline = time.monotonic() + 5.0
        while b"lpms_absent is unavailable" not in output and time.monotonic() < deadline:
            assert process.poll() is None
            if selector.select(timeout=0.05):
                output += os.read(process.stdout.fileno(), 4096)
        assert b"lpms_absent is unavailable" in output
        process.send_signal(signal.SIGINT)
        remaining_output, _ = process.communicate(timeout=5)
        output += remaining_output
        assert process.returncode == 0, output.decode(errors="replace")
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        selector.close()
        process.stdout.close()


@pytest.mark.parametrize("runtime", [False, True])
def test_launch_stays_static_or_rejects_default_runtime(runtime):
    arguments = ["ros2", "launch", "lpms_nav3_can", "lpms_nav3_can.launch.py"]
    if runtime:
        arguments += ["validation_only:=false"]
    result = subprocess.run(arguments, capture_output=True, text=True, timeout=15)
    output = result.stdout + result.stderr
    if runtime:
        assert result.returncode != 0
        assert "LPMS runtime requires explicit dedicated interface" in output
    else:
        assert result.returncode == 0
        assert "no CAN node started" in output
    assert "process started with pid" not in output
