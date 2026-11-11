"""Optional tail selection must preserve actuator identity, calibration and motor admission."""

import re
import xml.etree.ElementTree as ET

import pytest
import yaml

from integ_composer.optional_sensors import write_sensor_artifact
from test_bench_left_arm import scan, shares as shares


def observed_scan(missing=()):
    text = scan()(['ethercat', 'slaves', '-m', '0', '-v'])
    blocks = re.findall(r'=== Master 0, Slave (\d+) ===(.*?)(?==== Master|\Z)', text, re.S)
    mapping = {int(p): int(p) - sum(m < int(p) for m in missing) for p, _ in blocks if int(p) not in missing}
    result = ''
    for pos, body in blocks:
        pos = int(pos)
        if pos not in mapping:
            continue
        if pos == 0:
            body = re.sub(r'(yes\s+)(\d+)', lambda m: m[1] + str(mapping[int(m[2])]), body)
        result += f'=== Master 0, Slave {mapping[pos]} ===' + body
    return result


@pytest.mark.parametrize('missing', [(), (8,), (16,), (8, 16)])
@pytest.mark.parametrize('side', ['left', 'right', 'dual'])
def test_auto_selects_present_scope_sensors_and_preserves_motor_calibration(shares, tmp_path, missing, side):
    selection = dict(robot_variant='Alfa_V3', machine_unit='Alfa_V3-bench-01',
                     physical_profile=f'bench_{side}_arm', control_scope=f'bench_{side}_arm_head',
                     use_mock_hardware=False)
    result, report = write_sensor_artifact(tmp_path / 'artifact', workspace=tmp_path / 'selection',
        scan=observed_scan(missing), package_share=lambda name: str(shares[name]), **selection)
    expected = {s for s, p in [('left', 8), ('right', 16)] if p not in missing and side in (s, 'dual')}
    assert set(report['selected']) == expected
    robot = ET.fromstring(result.robot_description)
    arm = robot.find("ros2_control[@name='ecat_arms']")
    params = result.controllers['enable_manager']['ros__parameters']
    motor_domains = {1, 2} if side == 'dual' else {1 if side == 'left' else 2}
    ages = {f'ethercat_domain_{d}/process_data_age_ms' for d in motor_domains}
    assert ages.issubset(params['feedback_age_interfaces'])
    assert not any('ethercat_domain_3/' in x or 'ethercat_domain_4/' in x for x in params['feedback_age_interfaces'])
    assert 'ethercat_domain/process_data_age_ms' not in params['feedback_age_interfaces']
    for joint in arm.findall('joint'):
        domain = int(joint.find("param[@name='ec_module.domain_id']").text)
        assert domain in motor_domains
    for sensor_side in ('left', 'right'):
        controller = f'{sensor_side}_wrist_force_sensor_broadcaster'
        assert (controller in result.controllers) == (sensor_side in expected)
    before = yaml.safe_load((shares['hwcfg_machines'] / f"units/Alfa_V3-bench-01/bench_{side}_arm_zero.yaml").read_text())
    after = yaml.safe_load((tmp_path / f"selection/hwcfg_machines/units/Alfa_V3-bench-01/bench_{side}_arm_zero.yaml").read_text())
    for old, new in zip(before['spec']['axes'], after['spec']['axes']):
        assert {k:v for k,v in old.items() if k != 'ring_position'} == {k:v for k,v in new.items() if k != 'ring_position'}
    assert [x['ring_position'] for x in result.systems[0]['result']['expected_topology']['responders']] == list(range(18-len(missing)))


def make(shares, tmp_path, scan_text, choice='auto'):
    return write_sensor_artifact(tmp_path / 'artifact', workspace=tmp_path / 'selection',
        scan=scan_text, sensors=choice, package_share=lambda name: str(shares[name]),
        robot_variant='Alfa_V3', machine_unit='Alfa_V3-bench-01',
        physical_profile='bench_dual_arm', control_scope='bench_dual_arm_head', use_mock_hardware=True)


def test_explicit_none_keeps_connected_sensors_in_topology_but_excludes_pdos(shares, tmp_path):
    result, report = make(shares, tmp_path, observed_scan(), 'none')
    assert report['selected'] == []
    assert not any('wrist_force_sensor_broadcaster' in name for name in result.controllers)
    assert len(result.systems[0]['result']['expected_topology']['responders']) == 18


def test_missing_requested_sensor_is_an_error(shares, tmp_path):
    with pytest.raises(ValueError, match='unavailable'):
        make(shares, tmp_path, observed_scan((16,)), 'right')


@pytest.mark.parametrize('damage', ['motor', 'wiring', 'identity'])
def test_does_not_treat_motor_loss_or_bad_wiring_as_optional(shares, tmp_path, damage):
    text = observed_scan((16,))
    if damage == 'motor':
        text = observed_scan((5, 16))
    elif damage == 'wiring':
        text = text.replace('yes             9', 'yes             1', 1)
    else:
        text = text.replace('0x00029252', '0x12345678', 1)
    with pytest.raises(ValueError):
        make(shares, tmp_path, text)


@pytest.mark.parametrize('side', ['left', 'right', 'dual'])
@pytest.mark.parametrize('present', [True, False])
def test_optional_head_removes_all_claims_when_absent(shares, tmp_path, side, present):
    result, report = write_sensor_artifact(tmp_path / 'artifact', workspace=tmp_path / 'selection',
        scan=observed_scan((16,)), head='auto', head_probe=lambda *args: {'present': present},
        package_share=lambda name: str(shares[name]), robot_variant='Alfa_V3',
        machine_unit='Alfa_V3-bench-01', physical_profile=f'bench_{side}_arm',
        control_scope=f'bench_{side}_arm_head', use_mock_hardware=True)
    params = result.controllers['enable_manager']['ros__parameters']
    assert ('head_position_controller' in result.controllers) == present
    assert ('damiao_head_control' in params.get('handshake_devices', [])) == present
    assert ('head_position_controller' in params.get('companion_controllers', [])) == present
    assert any('H1_joint/' in name for name in params['feedback_age_interfaces']) == present
    robot = ET.fromstring(result.robot_description)
    assert (robot.find("ros2_control[@name='damiao_head']") is not None) == present
    assert len(params['managed_joints']) == (14 if side == 'dual' else 7)


@pytest.mark.parametrize('change', ['required_sensor', 'required_sensor_excluded', 'required_head',
                                  'exclude_arm', 'unknown', 'duplicate_alias', 'ambiguous_alias'])
def test_declarations_control_admission(shares, tmp_path, change):
    from integ_composer.startup_selection import write_startup_artifact
    root = shares['hwcfg_machines']
    path = root / 'machines/Alfa_V3/manifest/modules.yaml'
    data = yaml.safe_load(path.read_text())
    choices = {}
    if change in ('required_sensor', 'required_sensor_excluded'):
        data['modules']['wrist_force_sensors']['startup']['policy'] = 'required'
        if change == 'required_sensor_excluded':
            choices = {'wrist_force': 'none'}
    elif change == 'required_head':
        data['modules']['head_gimbal']['startup']['policy'] = 'required'
    elif change == 'exclude_arm':
        choices = {'bench_dual_arm': 'none'}
    elif change == 'duplicate_alias':
        choices = {'head': 'none', 'head_gimbal': 'none'}
    elif change == 'ambiguous_alias':
        data['modules']['wrist_force_sensors']['startup']['option'] = 'head'
        choices = {'head': 'none'}
    else:
        choices = {'not_declared': 'none'}
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    with pytest.raises(ValueError):
        write_startup_artifact(tmp_path / 'artifact', workspace=tmp_path / 'selection',
            scan=observed_scan((16,)), choices=choices, head_probe=lambda *args: {'present': False},
            package_share=lambda name: str(shares[name]), robot_variant='Alfa_V3',
            machine_unit='Alfa_V3-bench-01', physical_profile='bench_dual_arm',
            control_scope='bench_dual_arm_head', use_mock_hardware=True)


def test_module_report_is_part_of_verified_artifact(shares, tmp_path):
    from integ_composer.artifact import load_artifact
    result, report = make(shares, tmp_path, observed_scan((16,)))
    loaded = load_artifact(tmp_path / 'artifact', robot_variant='Alfa_V3',
        machine_unit='Alfa_V3-bench-01', physical_profile='bench_dual_arm',
        control_scope='bench_dual_arm_head', use_mock_hardware=True)
    metadata = yaml.safe_load(loaded.files['module_selection.yaml'])
    assert len(metadata['modules']) == 5
    assert [r['label'] for r in metadata['modules'] if r['reason'] == 'absent'] == ['右腕力']
    assert all(r['selected'] for r in metadata['modules'] if r['policy'] == 'required')
