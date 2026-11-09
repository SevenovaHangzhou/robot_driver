#!/usr/bin/env python3
"""Read the artifacts of an existing ros2_control process. No ROS commands."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path)
    parser.add_argument('--scope', default='bench_dual_arm_head')
    parser.add_argument('--allow-stopped', action='store_true',
                        help='Static preparation only; requires an explicit workspace')
    args = parser.parse_args()
    candidates = []
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            argv = proc.joinpath('cmdline').read_bytes().decode().split('\0')
        except (OSError, UnicodeError):
            continue
        if not argv or Path(argv[0]).name != 'ros2_control_node':
            continue
        for word in argv[1:]:
            path = Path(word)
            if path.name != 'controllers.yaml' or path.parent.name != args.scope or path.parent.parent.name != 'artifacts':
                continue
            workspace = path.parents[2]
            if args.workspace and workspace != args.workspace:
                continue
            candidates.append((int(proc.name), workspace, path.parent))
    if not candidates and args.allow_stopped and args.workspace:
        workspace = args.workspace
        candidates.append((None, workspace, workspace / 'artifacts' / args.scope))
    if len(candidates) != 1:
        parser.error(f'Expected one running artifact-backed controller, found {len(candidates)}; specify --workspace')
    pid, workspace, directory = candidates[0]
    files = {}
    for name in ('robot_description.xml', 'controllers.yaml', 'artifact.yaml', 'lifecycle.yaml'):
        path = directory / name
        if name == 'lifecycle.yaml' and not path.is_file():
            continue
        if path.stat().st_size > 2_000_000:
            parser.error(f'Artifact exceeds read limit: {name}')
        files[name] = path.read_text()
    print(json.dumps({'workspace': str(workspace), 'controller_pid': pid,
                      'runtime_running': pid is not None,
                      'artifact_dir': str(directory), 'files': files}))


if __name__ == '__main__':
    main()
