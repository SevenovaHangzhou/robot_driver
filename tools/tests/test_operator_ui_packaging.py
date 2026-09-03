from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_control_image_builds_operator_ui_package() -> None:
    dockerfile = (ROOT / "docker" / "rt-control" / "Dockerfile").read_text(
        encoding="utf-8"
    )

    assert "rt_control_operator_ui" in dockerfile
    assert "from python_qt_binding.QtWidgets import QApplication" in dockerfile


def test_operator_ui_compose_has_no_hardware_or_privileged_access() -> None:
    compose = yaml.safe_load(
        (ROOT / "docker" / "operator-ui.compose.yaml").read_text(encoding="utf-8")
    )
    service = compose["services"]["rt-control-operator-ui"]

    assert service["network_mode"] == "host"
    assert service["ipc"] == "host"
    assert service["user"] == "1000:1000"
    assert service["restart"] == "no"
    assert service["cap_drop"] == ["ALL"]
    assert service["security_opt"] == ["no-new-privileges:true"]
    assert "devices" not in service
    assert "cap_add" not in service
    assert "/var/run/docker.sock" not in str(service)
    assert service["cpuset"].startswith("${RT_CONTROL_UI_CPUSET:")
    assert service["environment"]["ROS_DOMAIN_ID"].startswith(
        "${RT_CONTROL_ROS_DOMAIN_ID:"
    )
