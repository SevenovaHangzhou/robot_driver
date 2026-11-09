#!/usr/bin/env python3
"""Verify a reviewed integration patch and its files without changing a runtime."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent


def digest(data):
    return sha256(data).hexdigest()


def command(args, cwd):
    return subprocess.run(args, cwd=cwd, check=True, capture_output=True).stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', type=Path,
                        default=ROOT.parents[1] / 'frame/robot_driver_jog_current')
    args = parser.parse_args()
    manifest = json.loads((ROOT / 'integration/shared-authority-reference.json').read_text())
    patch = ROOT / manifest['patch']
    if digest(patch.read_bytes()) != manifest['patch_sha256']:
        parser.error('Patch digest differs from the reviewed manifest')
    base = manifest['driver_base']
    with tempfile.TemporaryDirectory(prefix='jog-authority-check-') as directory:
        temporary = Path(directory)
        for name, hashes in manifest['driver_files'].items():
            path = temporary / name
            if hashes['before'] is not None:
                data = command(['git', 'show', base + ':' + name], args.driver)
                if digest(data) != hashes['before']:
                    parser.error('Base file differs: ' + name)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
            current = args.driver / name
            if digest(current.read_bytes()) != hashes['after']:
                parser.error('Working file differs: ' + name)
        command(['git', 'apply', '--check', str(patch)], temporary)
        command(['git', 'apply', str(patch)], temporary)
        for name, hashes in manifest['driver_files'].items():
            if digest((temporary / name).read_bytes()) != hashes['after']:
                parser.error('Applied patch differs: ' + name)
    for name in manifest.get('removed_client_files', []):
        if (ROOT / name).exists():
            parser.error('Obsolete independent authority still exists: ' + name)
    for name, expected in manifest['client_files'].items():
        if digest((ROOT / name).read_bytes()) != expected:
            parser.error('Client file differs: ' + name)
    print(f"PASS: patch applies to {base[:7]}, {len(manifest['driver_files'])} driver files match")
    print('PASS: client files match; no runtime or hardware state changed')


if __name__ == '__main__':
    main()
