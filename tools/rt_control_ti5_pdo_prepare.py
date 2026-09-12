#!/usr/bin/env python3
"""Prepare the verified IPC Ti5 position-PDO assignments while buses are idle."""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from pathlib import Path

import yaml

from rt_control_thread_affinity import find_ros2_control_pid


POSITIONS = (2, 3, 8, 9)
MOTION = (*range(1, 13), 16, 17)
TARGET = (0x1601, 0x1A01)
DEFAULT = (0x1600, 0x1A00)
MAPPINGS = {0x1601: (0x60400010, 0x607A0020), 0x1A01: (0x60410010, 0x60640020)}


def validate_profiles(directory: Path) -> None:
    for name in ('ti5_j2', 'ti5_j3', 'ti5_left_joint3', 'ti5_right_joint2'):
        profile = yaml.safe_load((directory / (name + '.yaml')).read_text())
        if (profile.get('vendor_id') != 0x00522227 or profile.get('product_id') != 0x00009253
                or profile.get('assign_activate') != 0x0300
                or profile.get('use_slave_pdo_defaults') is not True
                or profile.get('auto_state_transitions') is not False
                or profile.get('auto_fault_reset') is not False):
            raise RuntimeError(f'Installed profile {name} is not the approved fixed-PDO configuration')
        widths = {'uint16': 16, 'int32': 32}
        for direction, index in zip(('rpdo', 'tpdo'), TARGET):
            pdos = profile.get(direction, [])
            if len(pdos) != 1 or pdos[0].get('index') != index:
                raise RuntimeError(f'Installed profile {name} requires {direction} {index:04x}')
            entries = tuple((item['index'] << 16) | (item['sub_index'] << 8) | widths.get(item['type'], 0)
                            for item in pdos[0].get('channels', []))
            if entries != MAPPINGS[index]:
                raise RuntimeError(f'Installed profile {name} has an unexpected {direction} mapping')


class Ethercat:
    def command(self, *args: str) -> str:
        result = subprocess.run(['ethercat', *map(str, args)], capture_output=True,
                                text=True, timeout=15)
        if result.returncode:
            raise RuntimeError(f'ethercat {args}: {result.stderr.strip()}')
        return result.stdout

    def read(self, position: int, index: int, subindex: int, kind: str) -> int:
        result = self.command('upload', '-p', str(position), '-t', kind, hex(index), str(subindex))
        match = re.fullmatch(r'0x([0-9a-fA-F]+)\s+(\d+)\s*', result)
        if not match or int(match[1], 16) != int(match[2]):
            raise RuntimeError(f'Invalid SDO response at {position}/{index:04x}:{subindex}')
        return int(match[2])

    def write(self, position: int, index: int, subindex: int, kind: str, value: int) -> None:
        self.command('download', '-p', str(position), '-t', kind, hex(index), str(subindex), str(value))

    def idle(self) -> None:
        if find_ros2_control_pid() is not None:
            raise RuntimeError('Cannot prepare Ti5 PDOs while a controller is running')
        master = self.command('master')
        if not all(text in master for text in ('Phase: Idle', 'Active: no', 'Slaves: 18', 'Link: UP',
                                               '8c:59:3c:15:01:f8')):
            raise RuntimeError('Expected the verified IPC master idle with 18 linked slaves')
        slaves = self.command('slaves').splitlines()
        if len(slaves) != 18 or any(line.split()[2] != 'PREOP' for line in slaves):
            raise RuntimeError('All 18 slaves must be PREOP before PDO preparation')
        for position in MOTION:
            if self.read(position, 0x6041, 0, 'uint16') & 4:
                raise RuntimeError(f'Axis {position} reports Operation Enabled')

    def snapshot(self, position: int) -> dict:
        identity = self.command('slaves', '-p', str(position), '-v')
        for label, expected in [('Vendor Id', 0x00522227), ('Product code', 0x00009253)]:
            match = re.search(label + r':\s+(0x[0-9a-fA-F]+)', identity)
            if not match or int(match[1], 16) != expected:
                raise RuntimeError(f'Unexpected Ti5 {label} at position {position}')
        counts = tuple(self.read(position, idx, 0, 'uint8') for idx in (0x1C12, 0x1C13))
        assigned = tuple(self.read(position, idx, 1, 'uint16') for idx in (0x1C12, 0x1C13))
        mapping = {}
        for index, entries in MAPPINGS.items():
            count = self.read(position, index, 0, 'uint8')
            if count != len(entries):
                raise RuntimeError(f'Unexpected fixed PDO count at {position}/{index:04x}')
            actual = tuple(self.read(position, index, sub, 'uint32') for sub in range(1, count + 1))
            if actual != entries:
                raise RuntimeError(f'Fixed position mapping differs at {position}/{index:04x}')
            mapping[hex(index)] = actual
        if counts != (1, 1) or assigned not in (DEFAULT, TARGET):
            raise RuntimeError(f'Unrecognized Ti5 assignment at {position}: {counts}/{assigned}')
        cached = self.command('pdos', '-p', str(position))
        rx = re.findall(r'RxPDO\s+(0x[0-9a-fA-F]+)', cached)
        tx = re.findall(r'TxPDO\s+(0x[0-9a-fA-F]+)', cached)
        cache_target = [int(item, 16) for item in rx] == [TARGET[0]] and [
            int(item, 16) for item in tx] == [TARGET[1]]
        return {'identity': identity, 'assignment': assigned, 'mapping': mapping,
                'cache_target': cache_target}

    def rescan(self) -> None:
        self.command('rescan')
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            time.sleep(0.2)
            try:
                self.idle()
                if all(self.snapshot(pos)['cache_target'] for pos in POSITIONS):
                    return
            except RuntimeError:
                continue
        raise RuntimeError('IgH rescan did not discover all verified Ti5 assignments')


def prepare(bus: Ethercat, output: Path, restore: bool) -> None:
    bus.idle()
    before = {pos: bus.snapshot(pos) for pos in POSITIONS}
    (output / 'before.json').write_text(json.dumps(before, indent=2) + '\n')
    changes = [pos for pos, item in before.items() if item['assignment'] != TARGET]
    refresh = changes or any(not item['cache_target'] for item in before.values())
    if refresh and not restore:
        raise RuntimeError(f'Ti5 assignment/cache needs preparation at positions {changes}')
    if refresh:
        bus.idle()
        for pos in changes:
            for index, target in zip((0x1C12, 0x1C13), TARGET):
                # Only select already verified PDOs. Never write 160x/1Axx mappings.
                bus.write(pos, index, 0, 'uint8', 0)
                bus.write(pos, index, 1, 'uint16', target)
                bus.write(pos, index, 0, 'uint8', 1)
            if bus.snapshot(pos)['assignment'] != TARGET:
                raise RuntimeError(f'Ti5 {pos} assignment write did not read back')
        bus.rescan()
    bus.idle()
    after = {pos: bus.snapshot(pos) for pos in POSITIONS}
    (output / 'after.json').write_text(json.dumps(after, indent=2) + '\n')
    if any(item['assignment'] != TARGET or not item['cache_target'] for item in after.values()):
        raise RuntimeError('Ti5 assignments and IgH cache must both match before startup')
    print(f'PASS: Ti5 1601/1A01 verified; restored={changes}; rescanned={bool(refresh)}', flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--restore', action='store_true', help='Restore known default assignments')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profiles', type=Path, required=True, help='Actual installed slave profiles')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        validate_profiles(args.profiles)
        prepare(Ethercat(), args.output, args.restore)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError, RuntimeError,
            subprocess.SubprocessError) as error:
        (args.output / 'failure.txt').write_text(str(error) + '\n')
        print(f'FAIL: {error}', flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
