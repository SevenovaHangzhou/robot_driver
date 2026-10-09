from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from rt_control_bringup.force_sensor_runtime import build_force_sensor_runtime


def build(tmp_path, **kwargs):
    return build_force_sensor_runtime(
        hardware_share=PACKAGE.parent / "robot_hw_ethercat",
        bringup_share=PACKAGE, runtime_dir=tmp_path, **kwargs)


def scan(argv):
    if "master" in argv:
        return "Phase: Idle\nActive: no\nSlaves: 6\nLink: UP\n"
    if "slaves" in argv:
        return "State: PREOP\nFlag: +\nVendor Id: 0xA1\nProduct code: 0x8081\nRevision number: 0x1\n"
    return ('RxPDO 0x1600\n' + ''.join(
        f'PDO entry 0x7000:{i:02x}, 32 bit\n' for i in range(1, 9)) +
        'TxPDO 0x1a00\n' + ''.join(
        f'PDO entry 0x6000:{i:02x}, 32 bit\n' for i in range(1, 10)))


def test_mock_does_not_access_hardware_and_is_raw_only(tmp_path):
    def forbidden(_):
        pytest.fail("mock must not query EtherCAT")
    runtime = build(tmp_path, run=forbidden)
    root = ET.fromstring(runtime.robot_description)
    assert root.findall('.//joint') == []
    assert root.findall('.//command_interface') == []
    assert root.findall('.//param[@name="ec_module.plugin"]') == []
    for side in ('left', 'right'):
        params = runtime.controllers[f'{side}_wrist_force_sensor_broadcaster']['ros__parameters']
        assert params['raw_topic'] == f'/rt_control/{side}_wrist/raw'
        assert params['calibration_valid'] is False
        assert params['al_state_interface'] == f'ethercat_slave_{2 if side == "left" else 4}/al_state'


def test_real_only_loads_two_passive_modules(tmp_path):
    runtime = build(tmp_path, use_mock_hardware=False, sync_mode='sm', run=scan)
    root = ET.fromstring(runtime.robot_description)
    assert root.findall('.//joint') == []
    assert root.findall('.//command_interface') == []
    assert [p.text for p in root.findall('.//param[@name="ec_module.position"]')] == ['2', '4']
    assert [p.text for p in root.findall('.//param[@name="ec_module.plugin"]')] == [
        'ethercat_generic_plugins/GenericEcSlave'] * 2


@pytest.mark.parametrize('bad', ['Slaves: 6', 'Revision number: 0x1', '0x6000:09'])
def test_real_rejects_topology_identity_or_pdo_mismatch(tmp_path, bad):
    def wrong(argv):
        return scan(argv).replace(bad, 'mismatch')
    with pytest.raises(ValueError):
        build(tmp_path, use_mock_hardware=False, sync_mode='dc', run=wrong)


def test_real_requires_explicit_sync_mode(tmp_path):
    with pytest.raises(ValueError, match='sync_mode'):
        build(tmp_path, use_mock_hardware=False)


def test_real_rejects_previous_pdf_layout_for_docx_selected_profiles(tmp_path):
    def old_scan(argv):
        if "pdos" not in argv:
            return scan(argv)
        return ('RxPDO 0x1600\n' + ''.join(
            f'PDO entry 0x{i:04x}:00, 32 bit\n' for i in [0x2000]) +
            'TxPDO 0x1a00\n' + ''.join(
            f'PDO entry 0x{i:04x}:00, 32 bit\n' for i in [*range(0x4000, 0x4006), 0x4007, 0x4008]))
    with pytest.raises(ValueError, match="PDO layout mismatch"):
        build(tmp_path, use_mock_hardware=False, sync_mode='dc', run=old_scan)
