import sys
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rt_control_ti5_pdo_prepare as module
import rt_control_thread_affinity as affinity


def test_installed_full_pdo_profile_is_rejected_before_device_preparation(tmp_path):
    source = Path(__file__).resolve().parents[2] / 'src/rt_control/robot_hw_ethercat/config/slaves'
    for name in ('ti5_j2', 'ti5_j3', 'ti5_left_joint3', 'ti5_right_joint2'):
        (tmp_path / (name + '.yaml')).write_bytes((source / (name + '.yaml')).read_bytes())
    module.validate_profiles(tmp_path)
    path = tmp_path / 'ti5_j2.yaml'
    data = yaml.safe_load(path.read_text())
    data['rpdo'][0]['index'] = 0x1600
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(RuntimeError, match='Installed profile'):
        module.validate_profiles(tmp_path)


class Bus:
    def __init__(self, assignment=module.DEFAULT, cache=False):
        self.assignments = {pos: list(assignment) for pos in module.POSITIONS}
        self.cache = cache
        self.writes = []
        self.scans = 0
        self.reject = None

    def idle(self):
        pass

    def snapshot(self, pos):
        if pos == self.reject:
            raise RuntimeError('Unverified mapping')
        return {'assignment': tuple(self.assignments[pos]), 'cache_target': self.cache}

    def write(self, pos, index, sub, kind, value):
        self.writes.append((pos, index, sub, kind, value))
        if sub == 1:
            self.assignments[pos][index - 0x1C12] = value

    def rescan(self):
        self.scans += 1
        self.cache = True


def test_restore_selects_verified_pdos_and_refreshes_cached_assignment(tmp_path):
    bus = Bus()
    module.prepare(bus, tmp_path, restore=True)
    assert len(bus.writes) == 24
    assert {row[1] for row in bus.writes} == {0x1C12, 0x1C13}
    assert {row[4] for row in bus.writes if row[2] == 1} == set(module.TARGET)
    assert bus.scans == 1


def test_warm_start_never_writes_or_rescans(tmp_path):
    bus = Bus(module.TARGET, True)
    module.prepare(bus, tmp_path, restore=True)
    assert bus.writes == [] and bus.scans == 0


def test_stale_master_cache_is_rescanned_without_device_writes(tmp_path):
    bus = Bus(module.TARGET)
    module.prepare(bus, tmp_path, restore=True)
    assert bus.writes == [] and bus.scans == 1


def test_any_unverified_node_prevents_all_writes(tmp_path):
    bus = Bus()
    bus.reject = 9
    with pytest.raises(RuntimeError, match='Unverified'):
        module.prepare(bus, tmp_path, restore=True)
    assert bus.writes == []


def test_readonly_audit_cannot_restore(tmp_path):
    bus = Bus()
    with pytest.raises(RuntimeError):
        module.prepare(bus, tmp_path, restore=False)
    assert bus.writes == [] and bus.scans == 0


def test_failed_assignment_write_aborts_before_scan_or_launch(tmp_path):
    bus = Bus()
    with patch.object(bus, 'write', side_effect=RuntimeError('write failed')):
        with pytest.raises(RuntimeError, match='write failed'):
            module.prepare(bus, tmp_path, restore=True)
    assert bus.scans == 0


def test_crash_reporter_argument_does_not_become_a_second_controller():
    controller = '/opt/ros/humble/lib/controller_manager/ros2_control_node'
    cmdlines = {10: controller + ' --ros-args', 11: '/usr/bin/python3 /usr/share/apport/apport -- ' + controller,
                12: '/bin/sh -c ' + controller}
    with patch.object(affinity.Path, 'iterdir', return_value=[Path('/proc') / str(p) for p in cmdlines]), \
            patch.object(affinity, 'process_cmdline', side_effect=cmdlines.get), \
            patch.object(affinity.Path, 'readlink', return_value=Path(controller)):
        assert affinity.find_ros2_control_pid() == 10


def test_two_actual_controllers_still_fail():
    controller = '/opt/ros/humble/lib/controller_manager/ros2_control_node'
    with patch.object(affinity.Path, 'iterdir', return_value=[Path('/proc/10'), Path('/proc/11')]), \
            patch.object(affinity, 'process_cmdline', return_value=controller + ' --ros-args'), \
            patch.object(affinity.Path, 'readlink', return_value=Path(controller)):
        with pytest.raises(RuntimeError, match='multiple'):
            affinity.find_ros2_control_pid()
