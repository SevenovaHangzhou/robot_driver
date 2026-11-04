"""ELECTRI-105 source integration, without accessing a CAN interface."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "src/rt_control/periph_imu_lpms"


def test_lpms_is_started_only_through_a_deployment_declared_sensor_node():
    manifest = ET.parse(PACKAGE / "package.xml").getroot()
    assert manifest.findtext("name") == "periph_imu_lpms"
    bringup = ET.parse(ROOT / "src/rt_control/integ_bringup/package.xml")
    assert "periph_imu_lpms" in {entry.text for entry in bringup.findall("exec_depend")}
    workflow = (ROOT / ".github/workflows/rt-control-ci.yml").read_text()
    assert workflow.count("periph_imu_lpms") == 3
    assert "periph_imu_lpms" in (ROOT / "tools/bootstrap_native_dev.sh").read_text()
    launch = ROOT / "src/rt_control/integ_bringup/launch/rt_control_module.launch.py"
    assert 'package="periph_imu_lpms"' not in launch.read_text()


def test_lpms_config_has_valid_receive_only_identity():
    config = yaml.safe_load((PACKAGE / "config/lpms_nav3_can.yaml").read_text())
    parameters = config["/**"]["ros__parameters"]
    assert parameters["can_interface"] not in ("", "TBD")
    assert 1 <= parameters["node_id"] <= 127
    assert parameters["frame_id"] and not parameters["frame_id"].startswith("/")
    assert "send_nmt_start" not in parameters


# Topic defaults are checked on the live node (periph_imu_lpms test_node). The bus-write
# absence stays a static guard: a node that only listens has no behaviour to observe for it.
def test_lpms_has_no_bus_write_or_host_deployment_configuration():
    source = (PACKAGE / "src/lpms_nav3_can_node.cpp").read_text()
    assert "::write(" not in source
    assert "::send(" not in source
    assert "make_nmt_start_frame" not in source
    assert not list((PACKAGE / "config").glob("*.service"))
    assert not list((PACKAGE / "config").glob("*.rules"))
