import sys
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rt_control_thread_affinity as affinity


def test_rt_control_bringup_controller_node_is_found():
    controller = "/opt/rt_control_ws/install/lib/rt_control_bringup/ros2_control_node"
    with (
        patch.object(affinity.Path, "iterdir", return_value=[Path("/proc/10")]),
        patch.object(affinity, "process_cmdline", return_value=controller + " --ros-args"),
        patch.object(affinity.Path, "readlink", return_value=Path(controller)),
    ):
        assert affinity.find_ros2_control_pid() == 10
