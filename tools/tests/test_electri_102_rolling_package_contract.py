"""ELECTRI-102 rolling controller public-contract integration gate."""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "src/rt_control/rolling_trajectory_controller"


def _source_text() -> str:
    source_files = sorted(
        path
        for path in PACKAGE.rglob("*")
        if path.suffix in {".cpp", ".hpp", ".xml"} or path.name in {"CMakeLists.txt", "package.xml"}
    )
    return "\n".join(path.read_text(encoding="utf-8") for path in source_files)


def test_rolling_controller_consumes_provider_owned_public_packages() -> None:
    manifest = ET.parse(PACKAGE / "package.xml").getroot()
    dependencies = {element.text for element in manifest.findall("depend")}

    assert "robot_motion_interfaces" in dependencies
    assert "robot_rt_control_interfaces" in dependencies
    assert "robot_interfaces_qos" in dependencies
    assert "robot_interfaces" not in dependencies

    source = _source_text()
    assert "robot_motion_interfaces::msg::RollingJointTargetBatch" in source
    assert "robot_rt_control_interfaces::srv::OpenRollingJointSession" in source
    assert "robot_rt_control_interfaces::srv::CloseRollingJointSession" in source
    assert "robot_rt_control_interfaces::msg::RollingJointControlState" in source
    assert "robot_interfaces::" not in source


def test_rolling_endpoints_use_named_contract_qos() -> None:
    source = _source_text()

    assert "robot_interfaces_qos::rolling_command()" in source
    assert "robot_interfaces_qos::rolling_state()" in source
    assert "makePrototypeQRollingCommand" not in source
    assert "makePrototypeQRollingState" not in source


def test_rolling_plugin_is_exported_from_driver_tree() -> None:
    plugin = ET.parse(PACKAGE / "rolling_trajectory_controller_plugins.xml").getroot()
    classes = plugin.findall("class")

    assert any(
        item.attrib.get("type")
        == "rolling_trajectory_controller::RollingTrajectoryController"
        for item in classes
    )
