"""ELECTRI-105 source integration, without accessing a CAN interface."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "src/rt_control/lpms_nav3_can"


def test_lpms_is_started_only_through_a_deployment_declared_sensor_node():
    manifest = ET.parse(PACKAGE / "package.xml").getroot()
    assert manifest.findtext("name") == "lpms_nav3_can"
    bringup = ET.parse(ROOT / "src/rt_control/rt_control_bringup/package.xml")
    assert "lpms_nav3_can" in {entry.text for entry in bringup.findall("exec_depend")}
    # The scope runtime starts it from the artifact's sensor_nodes, never by a fixed name.
    scope = ROOT / "src/rt_control/rt_control_bringup/launch/rt_control_scope_runtime.launch.py"
    assert "lpms_nav3_can" not in scope.read_text()
    assert 'build.lifecycle.get("sensor_nodes", [])' in scope.read_text()
    workflow = (ROOT / ".github/workflows/rt-control-ci.yml").read_text()
    assert workflow.count("lpms_nav3_can") == 3
    assert "lpms_nav3_can" in (ROOT / "tools/bootstrap_native_dev.sh").read_text()
    launch = ROOT / "src/rt_control/rt_control_bringup/launch/rt_control_module.launch.py"
    assert 'package="lpms_nav3_can"' not in launch.read_text()


def test_lpms_config_has_valid_receive_only_identity():
    config = yaml.safe_load((PACKAGE / "config/lpms_nav3_can.yaml").read_text())
    parameters = config["/**"]["ros__parameters"]
    assert parameters["can_interface"] not in ("", "TBD", "can1")
    assert 1 <= parameters["node_id"] <= 127
    assert parameters["frame_id"] and not parameters["frame_id"].startswith("/")
    assert "send_nmt_start" not in parameters


def test_lpms_has_no_bus_write_or_host_deployment_configuration():
    source = (PACKAGE / "src/lpms_nav3_can_node.cpp").read_text()
    assert "::write(" not in source
    assert "::send(" not in source
    assert "make_nmt_start_frame" not in source
    assert '"imu_topic", "/imu/data"' in source
    transport = (PACKAGE / "src/socket_can.cpp").read_text()
    assert "::write(" not in transport
    assert "::send(" not in transport
    assert '"diagnostics_topic", "~/diagnostics"' in source
    assert '"sensor_status_topic", "/rt_control/sensors/status"' in source
    assert not list((PACKAGE / "config").glob("*.service"))
    assert not list((PACKAGE / "config").glob("*.rules"))
