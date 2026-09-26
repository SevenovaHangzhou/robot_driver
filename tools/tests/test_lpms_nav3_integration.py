"""ELECTRI-105 source integration, without accessing a CAN interface."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "src/rt_control/lpms_nav3_can"


def test_lpms_is_built_without_being_enabled_in_production():
    manifest = ET.parse(PACKAGE / "package.xml").getroot()
    assert manifest.findtext("name") == "lpms_nav3_can"
    bringup = ET.parse(ROOT / "src/rt_control/rt_control_bringup/package.xml")
    assert "lpms_nav3_can" not in {
        entry.text for entry in bringup.findall("exec_depend")
    }
    workflow = (ROOT / ".github/workflows/rt-control-ci.yml").read_text()
    assert workflow.count("lpms_nav3_can") == 3
    assert "lpms_nav3_can" in (ROOT / "tools/bootstrap_native_dev.sh").read_text()
    launch = ROOT / "src/rt_control/rt_control_bringup/launch/rt_control_module.launch.py"
    assert 'package="lpms_nav3_can"' not in launch.read_text()


def test_lpms_config_and_launch_fail_closed():
    config = yaml.safe_load((PACKAGE / "config/lpms_nav3_can.yaml").read_text())
    parameters = config["/**"]["ros__parameters"]
    assert parameters["can_interface"] == ""
    assert parameters["node_id"] == 0
    assert parameters["frame_id"] == ""
    assert "send_nmt_start" not in parameters
    launch = (PACKAGE / "launch/lpms_nav3_can.launch.py").read_text()
    assert 'DeclareLaunchArgument("validation_only", default_value="true")' in launch


def test_lpms_has_no_bus_write_or_host_deployment_configuration():
    source = (PACKAGE / "src/lpms_nav3_can_node.cpp").read_text()
    assert "::write(" not in source
    assert "::send(" not in source
    assert "make_nmt_start_frame" not in source
    assert '"~/data"' in source
    assert '"~/mag"' in source
    assert '"~/diagnostics"' in source
    assert not list((PACKAGE / "config").glob("*.service"))
    assert not list((PACKAGE / "config").glob("*.rules"))
