#!/usr/bin/env python3
"""Validate an ELECTRI-118 machine manifest without touching hardware."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

# ``ament_cmake_python`` installs the package below the prefix's
# ``local/lib/python*/dist-packages`` directory. Keep the validator directly
# executable from an installed prefix as well as from the source tree.
INSTALL_PREFIX = Path(__file__).resolve().parents[2]
for python_path in sorted(INSTALL_PREFIX.glob("local/lib/python*/dist-packages")):
    if python_path.is_dir() and str(python_path) not in sys.path:
        sys.path.insert(0, str(python_path))

from rt_control_bringup.machine_profile import (  # noqa: E402
    MachineProfileError,
    load_machine_manifest,
    select_hardware,
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--all", action="store_true", dest="all_selections")
    parser.add_argument("--physical-profile")
    parser.add_argument("--control-scope")
    parser.add_argument(
        "--require-runtime-ready",
        action="store_true",
        help="reject draft/TBD hardware facts instead of only inspecting them",
    )
    return parser.parse_args()


def _validate_all(manifest, *, require_runtime_ready: bool) -> int:
    selections = 0
    for profile in manifest.profiles.values():
        for scope_name in profile.allowed_scopes:
            select_hardware(
                manifest,
                physical_profile=profile.name,
                control_scope=scope_name,
                require_runtime_ready=require_runtime_ready,
            )
            selections += 1
    print(
        f"validated {manifest.variant}: {len(manifest.modules)} modules, "
        f"{len(manifest.profiles)} physical profiles, {selections} profile/scope selections"
    )
    return 0


def main() -> int:
    arguments = _arguments()
    try:
        manifest = load_machine_manifest(arguments.manifest)
        if arguments.all_selections:
            if arguments.physical_profile or arguments.control_scope:
                raise MachineProfileError(
                    "--all cannot be combined with --physical-profile or --control-scope"
                )
            return _validate_all(
                manifest, require_runtime_ready=arguments.require_runtime_ready
            )
        if not arguments.physical_profile or not arguments.control_scope:
            raise MachineProfileError(
                "--physical-profile and --control-scope are required unless --all is used"
            )
        selected = select_hardware(
            manifest,
            physical_profile=arguments.physical_profile,
            control_scope=arguments.control_scope,
            require_runtime_ready=arguments.require_runtime_ready,
        )
        print(
            f"validated {manifest.variant}/{selected.physical_profile}/"
            f"{selected.control_scope}: actuators={selected.actuator_count}"
        )
        return 0
    except (MachineProfileError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
