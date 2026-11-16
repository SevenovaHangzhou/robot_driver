"""Entry point of the chassis operator console. Never starts hardware or enables axes.

Checks before any ROS node exists: an explicit ROS domain, localhost-only DDS (or the
explicit loopback UDP profile, or the explicit LAN opt-in), explicit non-realtime CPUs and a
readable token file.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import tempfile
from typing import Optional
import xml.etree.ElementTree as ET

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import yaml

from .audit import DEFAULT_AUDIT
from .config import ConfigError, load_token

# Arm page state carried over from rt_control_arm_console (same files on existing hosts).
DEFAULT_POSES = Path.home() / ".local/state/rt-control/arm_console_poses.json"


def parse_cpus(value: str) -> set[int]:
    cpus: set[int] = set()
    for item in value.split(","):
        ends = item.split("-")
        if len(ends) == 1:
            cpus.add(int(ends[0]))
        elif len(ends) == 2 and int(ends[0]) <= int(ends[1]):
            cpus.update(range(int(ends[0]), int(ends[1]) + 1))
        else:
            raise ValueError("invalid CPU list")
    if not cpus or min(cpus) < 0 or not cpus <= os.sched_getaffinity(0):
        raise ValueError("CPU list must be a non-empty subset of this process's allowed CPUs")
    return cpus


def view_model(share: Optional[Path], relative: str) -> tuple[Optional[Path], Optional[Path]]:
    """Browser geometry: the deployment-chosen model file inside a robot_description share.

    The share defaults to the installed robot_description package, whose version the
    deployment pins. No model or no package means no 3-D view; a chosen model that is
    missing from the share is an error.
    """
    if not relative:
        return None, None
    if share is None:
        try:
            share = Path(get_package_share_directory("robot_description"))
        except PackageNotFoundError:
            return None, None
    model = share / relative
    if not model.is_file():
        raise ValueError(f"view model not found: {model}")
    return share, model


def check_dds(environ=os.environ) -> None:
    domain = environ.get("ROS_DOMAIN_ID", "")
    if not domain.isdecimal() or not 1 <= int(domain) <= 232:
        raise ValueError("set an explicit isolated ROS_DOMAIN_ID in 1..232")
    if environ.get("ROS_LOCALHOST_ONLY") == "1":
        return
    if environ.get("RT_CONTROL_OPERATOR_WEB_ALLOW_LAN_DDS") == "1":
        # Explicit deployment opt-in when the chassis runtime itself is LAN-visible on this
        # isolated domain; a loopback-only console cannot reach a LAN-visible runtime.
        return
    profile = environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    if not profile:
        raise ValueError("DDS must use ROS_LOCALHOST_ONLY=1 or the explicit loopback UDP profile")
    root = ET.parse(profile).getroot()
    transports = root.findall(".//{*}transport_descriptor")
    local_udp = bool(transports) and all(
        t.findtext("{*}type") == "UDPv4"
        and [a.text for a in t.findall("{*}interfaceWhiteList/{*}address")] == ["127.0.0.1"]
        for t in transports
    )
    builtin = root.findall(".//{*}useBuiltinTransports")
    if not local_udp or not builtin or any(x.text != "false" for x in builtin):
        raise ValueError("DDS must use localhost-only or the explicit loopback UDP profile")


def main(argv=None) -> int:
    share = Path(get_package_share_directory("integ_operator_web"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-host", default="127.0.0.1", help="explicit interface IP")
    parser.add_argument("--port", type=int, default=18120)
    parser.add_argument("--cpus", required=True, help="validated housekeeping CPUs, e.g. 0-6")
    parser.add_argument("--config", type=Path, default=share / "config/operator.yaml")
    parser.add_argument("--arm-config", type=Path, default=share / "config/arm.yaml")
    parser.add_argument("--poses", type=Path, default=DEFAULT_POSES)
    parser.add_argument("--head-config", type=Path, default=share / "config/head.yaml")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument(
        "--vacuum-script",
        type=Path,
        default=Path.home() / ".local/bin/vacuum",
        help="optional operator-installed vacuum CLI; absent file disables the IO buttons",
    )
    parser.add_argument(
        "--view-model-share",
        type=Path,
        default=None,
        help="robot_description share for browser geometry (default: the installed package); "
        "control uses the live model",
    )
    parser.add_argument(
        "--view-model",
        default="",
        help="model file of the deployed machine, relative to the robot_description share "
        "(empty: no 3-D view)",
    )
    parser.add_argument(
        "--audit-log",
        default=str(DEFAULT_AUDIT),
        help="append-only JSON-lines operator audit log; empty string disables it",
    )
    parser.add_argument("--software-test-only", action="store_true")
    parser.add_argument("--jog-motion-enabled", action="store_true",
                        help="allow the shared jog entry to publish via public RollingSender; never enables hardware")
    args = parser.parse_args(argv)
    try:
        check_dds()
        os.sched_setaffinity(0, parse_cpus(args.cpus))
        load_token()  # credentials before creating any publisher
        values = yaml.safe_load(args.config.read_text(encoding="utf-8"))
        params = values["integ_operator_web"]["ros__parameters"]
        view_share, view_file = view_model(args.view_model_share, args.view_model)
        params.update(
            bind_host=args.bind_host,
            port=args.port,
            static_dir=str(share / "web"),
            jog_motion_enabled=args.jog_motion_enabled,
        )
        from .node import main as run_node

        with tempfile.TemporaryDirectory(prefix="operator-web-") as directory:
            config = Path(directory) / "operator.yaml"
            config.write_text(yaml.safe_dump(values, allow_unicode=True), encoding="utf-8")
            return run_node(
                ["--ros-args", "--params-file", str(config)],
                software_test_only=args.software_test_only,
                audit_path=Path(args.audit_log).expanduser() if args.audit_log else None,
                arm={
                    "config": args.arm_config,
                    "poses": args.poses,
                    "joint_state_topic": args.joint_state_topic,
                    "vacuum_script": args.vacuum_script,
                    "view_model_share": view_share,
                    "view_model_file": view_file,
                },
                head={"config": args.head_config, "joint_state_topic": args.joint_state_topic},
            )
    except (ConfigError, OSError, ValueError) as exc:
        print(f"integ_operator_web: {exc}", file=sys.stderr)
        return 2
