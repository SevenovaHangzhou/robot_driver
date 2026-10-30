"""Entry point: ROS bridge + HTTP console. Never enables or moves on its own."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml

from .audit import DEFAULT_AUDIT, AuditLog
from .console import ArmConsole, config_from_mapping
from .ros_bridge import RosBridge, spin_in_background
from .server import ConsoleServer
from .vacuum_control import VacuumControl

DEFAULT_POSES = Path.home() / ".local/state/rt-control/arm_console_poses.json"
DEFAULT_VIEW_SHARE = Path.home() / ".local/share/rt-control/view_robot_description"


def parse_arguments(argv=None):
    share = Path(get_package_share_directory("rt_control_arm_console"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18120)
    parser.add_argument(
        "--token-file",
        type=Path,
        required=True,
        help="file that holds the operator token (mode 0600)",
    )
    parser.add_argument("--config", type=Path, default=share / "config/console.yaml")
    parser.add_argument("--poses", type=Path, default=DEFAULT_POSES)
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--web-dir", type=Path, default=share / "web")
    parser.add_argument(
        "--vacuum-script", type=Path, default=Path.home() / ".local/bin/vacuum",
        help="optional operator-installed vacuum CLI; absent file disables the IO buttons",
    )
    parser.add_argument(
        "--view-model-share",
        type=Path,
        default=DEFAULT_VIEW_SHARE
        if DEFAULT_VIEW_SHARE.exists() or DEFAULT_VIEW_SHARE.is_symlink()
        else None,
        help="authoritative robot_description share for browser geometry; control uses the live model",
    )
    parser.add_argument(
        "--audit-log",
        default=str(DEFAULT_AUDIT),
        help="append-only JSON-lines operator audit log; empty string disables it",
    )
    return parser.parse_args(argv)


def load_token(path: Path) -> str:
    if path.stat().st_mode & 0o077:
        raise SystemExit(f"token file must not be group/world readable: {path}")
    token = path.read_text(encoding="utf-8").strip()
    if not token:
        raise SystemExit(f"token file is empty: {path}")
    return token


def main(argv=None) -> None:
    arguments = parse_arguments(argv)
    token = load_token(arguments.token_file)
    config = config_from_mapping(yaml.safe_load(arguments.config.read_text(encoding="utf-8")))
    rclpy.init()
    bridge = RosBridge(arguments.joint_state_topic)
    console = ArmConsole(config, "", bridge, arguments.poses)
    bridge.console = console
    spin_in_background(bridge)
    server = ConsoleServer(
        (arguments.bind, arguments.port),
        console,
        token,
        arguments.web_dir,
        get_package_share_directory,
        bridge.set_enabled,
        AuditLog(Path(arguments.audit_log).expanduser() if arguments.audit_log else None),
        bridge.reset_fault,
        view_model_share=arguments.view_model_share,
        vacuum=VacuumControl(arguments.vacuum_script) if arguments.vacuum_script.is_file() else None,
    )

    def shutdown(*_):
        console.stop()
        server.stopping.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    bridge.get_logger().info(
        f"arm console on http://{arguments.bind}:{arguments.port} (pid {os.getpid()})"
    )
    try:
        server.serve_forever()
    finally:
        server.server_close()
        bridge.destroy_node()
        rclpy.try_shutdown()
