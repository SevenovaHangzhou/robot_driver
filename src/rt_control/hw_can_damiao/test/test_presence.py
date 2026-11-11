"""Only a missing reply or absent/down link is optional; probe errors remain errors."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from hw_can_damiao.presence import probe_module

MACHINE = {
    "spec": {
        "configure_timeout_ms": 100,
        "joints": [
            {"can_id": 2, "master_id": 18, "robot_model_joint": "yaw"},
            {"can_id": 1, "master_id": 17, "robot_model_joint": "pitch"},
        ],
    }
}


@pytest.mark.parametrize(
    "codes, present, missing",
    [([0, 0], True, []), ([0, 3], False, ["pitch"]), ([3, 3], False, ["yaw", "pitch"])],
)
def test_response_selection(codes, present, missing):
    with (
        patch.object(Path, "exists", return_value=True),
        patch.object(Path, "read_text", return_value="0x1"),
        patch("hw_can_damiao.presence.get_package_prefix", return_value="/test"),
        patch(
            "hw_can_damiao.presence.subprocess.run",
            side_effect=[SimpleNamespace(returncode=c, stderr="") for c in codes],
        ) as run,
    ):
        result = probe_module(
            MACHINE, {"can_interface_key": "head"}, {"can_interfaces": {"head": "can2"}}
        )
        assert result["present"] == present
        assert result["missing_joints"] == missing
        assert run.call_count == 2
        assert run.call_args_list[0].args[0][-3:] == ["2", "18", "100"]


def test_probe_error_does_not_become_missing():
    with (
        patch.object(Path, "exists", return_value=True),
        patch.object(Path, "read_text", return_value="0x1"),
        patch("hw_can_damiao.presence.get_package_prefix", return_value="/test"),
        patch(
            "hw_can_damiao.presence.subprocess.run",
            return_value=SimpleNamespace(returncode=1, stderr="identity mismatch"),
        ),
    ):
        with pytest.raises(ValueError, match="identity mismatch"):
            probe_module(
                MACHINE, {"can_interface_key": "head"}, {"can_interfaces": {"head": "can2"}}
            )


def test_missing_interface_does_not_send():
    with (
        patch.object(Path, "exists", return_value=False),
        patch("hw_can_damiao.presence.subprocess.run") as run,
    ):
        assert not probe_module(
            MACHINE, {"can_interface_key": "head"}, {"can_interfaces": {"head": "can2"}}
        )["present"]
        run.assert_not_called()
