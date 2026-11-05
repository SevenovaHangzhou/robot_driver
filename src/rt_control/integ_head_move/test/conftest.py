from pathlib import Path

import pytest
import yaml

URDF = """<robot name="r"><joint name="H1_joint" type="revolute"><limit lower="-3" upper="3"/></joint>
<ros2_control name="damiao_head" type="system">
<joint name="H1_joint"><param name="velocity_limit">0.2</param>
<command_interface name="position"><param name="min">-1.57</param><param name="max">1.57</param>
</command_interface></joint>
<joint name="H2_joint"><param name="velocity_limit">0.4</param>
<command_interface name="position"><param name="min">-1.0</param><param name="max">1.0</param>
</command_interface></joint></ros2_control></robot>"""


@pytest.fixture
def generated_dir(tmp_path) -> Path:
    lifecycle = {
        "capabilities": [
            {"name": "enable", "service": "/rt/enable", "type": "RtEnable"},
            {
                "name": "head_move",
                "topic": "/rt/head/move",
                "type": "robot_rt_control_interfaces/action/MoveHead",
                "controller": "head_position_controller",
            },
        ]
    }
    controllers = {
        "head_position_controller": {"ros__parameters": {"joints": ["H1_joint", "H2_joint"]}}
    }
    (tmp_path / "lifecycle.yaml").write_text(yaml.safe_dump(lifecycle))
    (tmp_path / "controllers.yaml").write_text(yaml.safe_dump(controllers))
    (tmp_path / "robot_description.xml").write_text(URDF)
    return tmp_path
