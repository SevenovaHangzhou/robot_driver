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
        (["node_id:=0"], "node_id must be in [1, 127]"),
        (["can_interface:=TBD"], "can_interface must be explicitly configured"),
        (["node_id:=1", "can_interface:=can1"], "refusing reserved interface"),
        (["frame_id:=TBD"], "frame_id must be explicitly configured"),
        (["orientation_covariance:=[1.0]"], "orientation_covariance must contain 9 values"),
        (["angular_velocity_covariance:=[1.0]"], "angular_velocity_covariance must contain 9 values"),
        (
            ["angular_velocity_covariance:=[-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"],
            "angular_velocity_covariance diagonal values must be nonnegative",
        ),
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


def test_launch_static_validation_without_starting_node():
    arguments = ["ros2", "launch", "lpms_nav3_can", "lpms_nav3_can.launch.py"]
    arguments += ["validation_only:=true"]
    result = subprocess.run(arguments, capture_output=True, text=True, timeout=15)
    output = result.stdout + result.stderr
    assert result.returncode == 0
    assert "no CAN node started" in output
    assert "process started with pid" not in output


def test_can0_config_is_selected_by_default():
    import yaml

    package = Path(__file__).resolve().parents[1]
    config = yaml.safe_load(
        (package / "config/lpms_nav3_can.yaml").read_text()
    )["/**"]["ros__parameters"]
    assert config["can_interface"] == "can0"
    assert config["node_id"] == 1
    assert config["frame_id"] == "imu_link"
    assert config["mount_yaw_deg"] == 90.0
    covariance = config["angular_velocity_covariance"]
    assert len(covariance) == 9
    assert covariance[0] > 0.0
    assert covariance[4] > 0.0
    assert covariance[8] > 0.0
    assert covariance[1] == covariance[3]
    assert covariance[2] == covariance[6]
    assert covariance[5] == covariance[7]
    assert "orientation_covariance" not in config
    assert "linear_acceleration_covariance" not in config
    config_text = (package / "config/lpms_nav3_can.yaml").read_text()
    assert "# orientation_covariance:" in config_text
    assert "# linear_acceleration_covariance:" in config_text
    launch = (package / "launch/lpms_nav3_can.launch.py").read_text()
    assert 'DeclareLaunchArgument("validation_only", default_value="false")' in launch
