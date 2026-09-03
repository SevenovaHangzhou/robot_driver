from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
ROOT_DIR = PACKAGE_DIR.parents[2]
PROFILE_DIR = PACKAGE_DIR / "config/slaves"
XACRO_PATH = PACKAGE_DIR / "urdf/ecat.ros2_control.xacro"
MOCK_PATH = ROOT_DIR / "src/rt_control/rt_control_bringup/urdf/mock.ros2_control.xacro"
DIAGNOSTICS_PATH = ROOT_DIR / "src/rt_control/rt_diagnostics/src/rt_diagnostics_node.cpp"
NATIVE_PATH = ROOT_DIR / "tools/rt_control_native.sh"
IPC_PATH = ROOT_DIR / "tools/rt_control_ipc.sh"
DOCKERFILE_PATH = ROOT_DIR / "docker/rt-control/Dockerfile"
ECAT_PATCH_PATH = ROOT_DIR / "patches/ecat_icube/0004-preserve-fixed-pdo-config.patch"
IGH_PATCH_PATH = ROOT_DIR / "patches/igh/0001-preserve-verified-pdo-config.patch"


def load_profile(side: str) -> dict:
    return yaml.safe_load(
        (PROFILE_DIR / f"x503_{side}.yaml").read_text(encoding="utf-8")
    )


def test_eighteen_position_topology_keeps_fourteen_motion_axes():
    xacro = XACRO_PATH.read_text(encoding="utf-8")

    assert 'joint_name="right_joint1" ring_position="1"' in xacro
    assert 'joint_name="right_joint6" ring_position="6"' in xacro
    assert 'joint_name="left_joint1" ring_position="7"' in xacro
    assert 'joint_name="left_joint6" ring_position="12"' in xacro
    assert (
        'sensor_name="right_force_sensor" ring_position="14" profile="x503_right"'
        in xacro
    )
    assert (
        'sensor_name="left_force_sensor" ring_position="15" profile="x503_left"'
        in xacro
    )
    assert 'joint_name="turn" ring_position="16"' in xacro
    assert 'joint_name="updown" ring_position="17"' in xacro
    assert "Hub 13 OUT8 -> right X503 at 14 -> left X503 at 15" in xacro
    assert 'joint_name="right_joint7"' not in xacro
    assert 'joint_name="left_joint7"' not in xacro
    assert xacro.index('sensor_name="right_force_sensor" ring_position="14"') < xacro.index(
        'sensor_name="left_force_sensor" ring_position="15"'
    )
    assert 'ethercat_slave_13' not in xacro
    for position in (*range(1, 13), 14, 15, 16, 17):
        assert f'ethercat_slave_{position}' in xacro


def test_x503_profiles_match_confirmed_fixed_online_layout():
    for side, position in (("right", 14), ("left", 15)):
        profile = load_profile(side)

        assert profile["vendor_id"] == 0x00000503
        assert profile["product_id"] == 0x26483052
        assert profile["assign_activate"] == 0x0300
        assert profile["use_slave_pdo_defaults"] is True
        assert "sdo" not in profile
        assert profile["metadata"]["motion_ready"] is False
        assert profile["metadata"]["ring_position"] == position
        assert profile["metadata"]["topology"] == "daisy_chain_from_hub_position_13_out8"
        assert profile["metadata"]["standard_ethercat_serial_is_zero"] is True
        assert profile["metadata"]["online_txpdo_size_bytes"] == 100
        assert profile["metadata"]["esi_size_conflict_unresolved"] is True

        rx = profile["rpdo"]
        assert len(rx) == 1 and rx[0]["index"] == 0x1601
        assert [(item["index"], item["sub_index"]) for item in rx[0]["channels"][:8]] == [
            (0x7010, sub_index) for sub_index in range(1, 9)
        ]
        assert all(
            item["type"] == "bool" and item["default"] == 0
            for item in rx[0]["channels"][:8]
        )
        assert rx[0]["channels"][8] == {"index": 0, "sub_index": 0, "type": "bit8"}

        tx = profile["tpdo"]
        assert len(tx) == 1 and tx[0]["index"] == 0x1A00
        assert len(tx[0]["channels"]) == 25
        assert [
            (item["index"], item["sub_index"], item["type"])
            for item in tx[0]["channels"]
        ] == [(0x6000, sub_index, "int32") for sub_index in range(1, 26)]


def test_x503_ros_interfaces_are_state_only_and_present_in_mock():
    xacro = XACRO_PATH.read_text(encoding="utf-8")
    start = xacro.index('<xacro:macro name="x503_sensor"')
    stop = xacro.index("</xacro:macro>", start)
    macro = xacro[start:stop]
    mock = MOCK_PATH.read_text(encoding="utf-8")

    assert "ethercat_generic_plugins/GenericEcSlave" in macro
    assert "${profile}.yaml" in macro
    assert "command_interface" not in macro
    assert "force." not in macro and "torque." not in macro
    assert 'mock_x503_sensor sensor_name="right_force_sensor"' in mock
    assert 'mock_x503_sensor sensor_name="left_force_sensor"' in mock
    assert '<param name="initial_value">18.0</param>' in mock


def test_diagnostics_and_startup_require_both_x503_devices():
    diagnostics = DIAGNOSTICS_PATH.read_text(encoding="utf-8")
    native = NATIVE_PATH.read_text(encoding="utf-8")
    ipc = IPC_PATH.read_text(encoding="utf-8")

    assert "static_cast<int>(slaves_responding) != 18" in diagnostics
    assert "kRingPositions = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 17}" in diagnostics
    assert "kSensorRingPositions = {14, 15}" in diagnostics
    assert '"right_force_sensor", "left_force_sensor"' in diagnostics
    for script in (native, ipc):
        assert "Slaves: 18" in script
        assert "DST_X503" in script
        assert "verify_x503_identity_and_pdos" in script
        assert "Vendor Id:" in script
        assert "Product code:" in script
        assert "Revision number:" in script
        assert "RxPDO[[:space:]]+0x1601" in script
        assert "TxPDO[[:space:]]+0x1a00" in script
    assert "EtherCAT Idle/Inactive 且 18 个从站全 PREOP" in ipc


def test_fixed_pdo_policy_reaches_patched_igh_master():
    ecat_patch = ECAT_PATCH_PATH.read_text(encoding="utf-8")
    igh_patch = IGH_PATCH_PATH.read_text(encoding="utf-8")
    dockerfile = DOCKERFILE_PATH.read_text(encoding="utf-8")

    assert "use_slave_pdo_defaults" in ecat_patch
    assert "useSlavePdoDefaults()" in ecat_patch
    assert '"PreservePdoConfig", 1' in ecat_patch
    assert "int pdos_status = ecrt_slave_config_pdos(" in ecat_patch
    assert "without writing CoE mapping objects" in ecat_patch
    assert "ec_fsm_pdo_conf_preserve_config" in igh_patch
    assert "ec_pdo_equal_entries" in igh_patch
    assert "ec_pdo_list_equal" in igh_patch
    assert "0004-preserve-fixed-pdo-config.patch" in dockerfile
    assert "0001-preserve-verified-pdo-config.patch" in dockerfile
