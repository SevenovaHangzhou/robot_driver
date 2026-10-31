"""Launcher checks that run before any ROS node exists."""

import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from integ_operator_web.launcher import check_dds, parse_cpus  # noqa: E402

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
