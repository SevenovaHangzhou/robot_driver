"""Verify the reviewed Git revisions without reading a dirty working tree.

This checks source identity only. It never fetches, updates a reference,
connects to ROS, or grants permission to move hardware.
"""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[1]


def git(repo, *args):
    return subprocess.check_output(['git', '-C', str(repo), *args], stderr=subprocess.PIPE)


def blob(repo, revision, path):
    return git(repo, 'show', f'{revision}:{path}')


def verify_files(repo, revision, files):
    for entry in files:
        if sha256(blob(repo, revision, entry['path'])).hexdigest() != entry['sha256']:
            raise ValueError(f"Source fingerprint differs: {entry['path']}")


def check(reference, repo, interfaces):
    revision = reference['reference_commit']
    current = git(repo, 'rev-parse', reference['reference_ref']).decode().strip()
    if current != revision:
        raise ValueError(f"{reference['reference_ref']} changed to {current}; review the new main first")
    verify_files(repo, revision, reference['files'])
    public = reference['public_interfaces']
    lock = yaml.safe_load(blob(repo, revision, 'src/interfaces/source-lock.yaml'))
    if lock['commit'] != public['commit']:
        raise ValueError('Public interface pin differs from the driver source lock')
    verify_files(interfaces, public['commit'], public['files'])
    changed = git(interfaces, 'diff', '--name-only', public['previous_commit'], public['commit'])
    if changed.decode().splitlines() != public['changed_paths']:
        raise ValueError('Public interface delta differs from the reviewed delta')
    # ErrorCode has new constants. All other recorded Rolling dependencies and
    # both public QoS implementations must remain byte-identical to the old pin.
    for entry in public['files']:
        path = entry['path']
        if path != 'robot_system_interfaces/msg/ErrorCode.msg' and \
                blob(interfaces, public['previous_commit'], path) != blob(interfaces, public['commit'], path):
            raise ValueError(f'Rolling dependency changed: {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=ROOT / 'integration/main-reference.json')
    parser.add_argument('--repo', type=Path, help='robot_driver Git repository or worktree')
    parser.add_argument('--interfaces-repo', required=True, type=Path, help='robot_interfaces Git repository')
    args = parser.parse_args()
    try:
        reference = json.loads(args.reference.read_text())
        check(reference, args.repo or Path(reference['reference_repository']), args.interfaces_repo)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f'Source verification failed: {exc}\n')
    print(f"driver main {reference['reference_commit'][:7]}: {len(reference['files'])} files verified")
    print(f"public pin {reference['public_interfaces']['commit'][:7]}: "
          f"{len(reference['public_interfaces']['files'])} files verified")
    print('Source identity only; hardware motion and shared authority remain unverified.')


if __name__ == '__main__':
    main()
