"""Launcher checks that run before any ROS node exists."""

import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integ_operator_web import launcher  # noqa: E402
from integ_operator_web.launcher import check_dds, parse_cpus, view_model  # noqa: E402

LOOPBACK = """<?xml version="1.0"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors><transport_descriptor><transport_id>u</transport_id><type>UDPv4</type>
    <interfaceWhiteList><address>{address}</address></interfaceWhiteList>
  </transport_descriptor></transport_descriptors>
  <participant profile_name="p" is_default_profile="true"><rtps>
    <userTransports><transport_id>u</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps></participant>
</profiles>"""


def test_cpu_list_must_be_allowed_and_well_formed():
    allowed = sorted(os.sched_getaffinity(0))
    assert parse_cpus(str(allowed[0])) == {allowed[0]}
    for bad in ("", "3-1", "a", "-1", str(max(allowed) + 1000)):
        with pytest.raises(ValueError):
            parse_cpus(bad)


def test_ros_domain_must_be_explicit():
    for domain in ("", "0", "233", "x"):
        with pytest.raises(ValueError, match="ROS_DOMAIN_ID"):
            check_dds({"ROS_DOMAIN_ID": domain, "ROS_LOCALHOST_ONLY": "1"})
    check_dds({"ROS_DOMAIN_ID": "173", "ROS_LOCALHOST_ONLY": "1"})


def test_dds_must_stay_on_loopback(tmp_path):
    good = tmp_path / "good.xml"
    good.write_text(LOOPBACK.format(address="127.0.0.1"))
    check_dds({"ROS_DOMAIN_ID": "173", "FASTRTPS_DEFAULT_PROFILES_FILE": str(good)})
    lan = tmp_path / "lan.xml"
    lan.write_text(LOOPBACK.format(address="192.168.1.2"))
    for environ in (
        {"ROS_DOMAIN_ID": "173"},
        {"ROS_DOMAIN_ID": "173", "FASTRTPS_DEFAULT_PROFILES_FILE": str(lan)},
    ):
        with pytest.raises(ValueError, match="DDS"):
            check_dds(environ)


def test_lan_dds_requires_explicit_opt_in(tmp_path):
    lan = tmp_path / "lan.xml"
    lan.write_text(LOOPBACK.format(address="192.168.1.2"))
    environ = {"ROS_DOMAIN_ID": "173", "FASTRTPS_DEFAULT_PROFILES_FILE": str(lan)}
    for value in ("", "0", "true"):
        with pytest.raises(ValueError, match="DDS"):
            check_dds({**environ, "RT_CONTROL_OPERATOR_WEB_ALLOW_LAN_DDS": value})
    check_dds({**environ, "RT_CONTROL_OPERATOR_WEB_ALLOW_LAN_DDS": "1"})
    with pytest.raises(ValueError, match="ROS_DOMAIN_ID"):
        check_dds({"ROS_DOMAIN_ID": "0", "RT_CONTROL_OPERATOR_WEB_ALLOW_LAN_DDS": "1"})


def test_view_model_is_read_from_the_robot_description_share(tmp_path):
    model = tmp_path / "urdf/robot.urdf"
    model.parent.mkdir()
    model.write_text("<robot name='r'/>")
    assert view_model(tmp_path, "urdf/robot.urdf") == (tmp_path, model)
    with pytest.raises(ValueError, match="view model not found"):
        view_model(tmp_path, "urdf/other.urdf")
    assert view_model(tmp_path, "") == (None, None)


def test_view_model_defaults_to_the_installed_package(monkeypatch, tmp_path):
    (tmp_path / "urdf").mkdir()
    (tmp_path / "urdf/robot.urdf").write_text("<robot name='r'/>")
    monkeypatch.setattr(launcher, "get_package_share_directory", lambda package: str(tmp_path))
    assert view_model(None, "urdf/robot.urdf") == (tmp_path, tmp_path / "urdf/robot.urdf")

    def missing(package):
        raise launcher.PackageNotFoundError(package)

    monkeypatch.setattr(launcher, "get_package_share_directory", missing)
    assert view_model(None, "urdf/robot.urdf") == (None, None)
