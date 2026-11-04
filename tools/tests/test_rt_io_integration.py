import os
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / "src/rt_control/integ_bringup"
HOSTSETUP = ROOT / "hostsetup"
COMPOSE_WRAPPER = ROOT / "tools/rt_control_compose.sh"
sys.path.insert(0, str(ROOT / "src/rt_control/integ_api_adapter"))

from integ_api_adapter.public_error import PublicErrorCode


def test_public_and_private_interface_packages_have_distinct_ownership() -> None:
    interface_root = ROOT / "src/interfaces"
    public_packages = (
        "robot_rt_control_interfaces",
        "robot_system_interfaces",
        "robot_interfaces_qos",
    )
    source_lock = yaml.safe_load((interface_root / "source-lock.yaml").read_text())
    dependencies = yaml.safe_load((ROOT / "deps.repos").read_text())["repositories"]

    assert source_lock == {
        "schema_version": 1,
        "repository": "https://github.com/SevenovaHangzhou/robot_interfaces.git",
        "commit": "c82f0e40937453f1f8fb104822c2e91e9fb8d742",
        "contract_version": "1.0.0",
        "vendor_path": "src/vendor/robot_interfaces",
        "vendored_packages": list(public_packages),
    }
    assert dependencies["src/vendor/robot_interfaces"] == {
        "type": "git",
        "url": "https://github.com/SevenovaHangzhou/robot_interfaces.git",
        "version": source_lock["commit"],
    }

    assert not (interface_root / "alfa_control_interfaces").exists()
    assert not (interface_root / "alfa_system_interfaces").exists()
    assert not (interface_root / "robot_interfaces").exists()
    for package_name in public_packages:
        assert not (interface_root / package_name).exists()

    private_manifest = (interface_root / "rt_control_interfaces/package.xml").read_text()
    assert "<name>rt_control_interfaces</name>" in private_manifest
    assert {
        path.relative_to(interface_root / "rt_control_interfaces").as_posix()
        for path in (interface_root / "rt_control_interfaces").glob("**/*")
        if path.is_file() and path.suffix in {".msg", ".srv", ".action"}
    } == {
        "msg/JointControlModeResult.msg",
        "msg/PlcIoState.msg",
        "srv/RtEnable.srv",
        "action/ChassisRelativeMove.action",
        "msg/ChassisMoveLimits.msg",
        "msg/ChassisMoveState.msg",
        "msg/ChassisState.msg",
        "msg/SwerveExecutionState.msg",
        "srv/ChassisSetMode.srv",
        "srv/ChassisResetFault.srv",
        "srv/SetDigitalOutput.srv",
    }


def test_rt_io_uses_one_central_hardware_configuration() -> None:
    plc = yaml.safe_load(
        (ROOT / "src/rt_control/periph_plc_io/config/plc_io_modbus.yaml").read_text()
    )["plc_io_modbus"]["ros__parameters"]
    bms = yaml.safe_load((ROOT / "src/rt_control/periph_bms/config/bms_node.yaml").read_text())[
        "bms_node"
    ]["ros__parameters"]
    assert plc["digital"]["module"]["host"] == "192.168.1.12"
    assert plc["digital"]["inputs"]["infrared_laser"] == {
        "configured": False,
        "active_high": True,
        "di_address": 0,
    }
    assert plc["vacuum_system"]["configured"] is False
    assert plc["digital"]["outputs"]["vacuum_pump_relay"]["do_address"] == -1
    assert plc["digital"]["outputs"]["left_vacuum_valve"]["do_address"] == -1
    assert plc["digital"]["outputs"]["right_vacuum_valve"]["do_address"] == -1
    assert plc["battery_relays"]["configured"] is True
    assert plc["digital"]["outputs"]["primary_battery_relay"]["do_address"] == 0
    assert plc["digital"]["outputs"]["secondary_battery_relay"]["do_address"] == 1
    assert plc["analog"]["module"]["host"] == "192.168.1.13"
    left_sensor = plc["analog"]["inputs"]["left_vacuum_sensor"]
    right_sensor = plc["analog"]["inputs"]["right_vacuum_sensor"]
    assert left_sensor["register"] == -1
    assert right_sensor["register"] == -1
    assert left_sensor["sensor_id"] == "TBD"
    assert right_sensor["sensor_id"] == "TBD"
    assert bms["transport"] == "socketcan"
    assert bms["can_interface"] == "can1"
    assert bms["expected_adapter_serial"] == "004D00675230500720333159"
    assert bms["protocol"] == "golden_phoenix_v1_1"
    assert bms["can_bitrate"] == 250000
    assert bms["primary_bms_address"] == 1
    assert bms["secondary_bms_address"] == 2
    assert bms["host_address"] == 64
    assert bms["multi_byte_order"] == "auto"
    assert bms["request_period_s"] == 0.2
    assert bms["publish_period_s"] == 5.0
    assert bms["frame_timeout_s"] == 3.0
    assert bms["mos_closed_raw"] == -1
    assert bms["mos_open_raw"] == -1


def test_bms_public_parameters_match_current_contract() -> None:
    bms_params = yaml.safe_load(
        (ROOT / "src/rt_control/periph_bms/config/bms_node.yaml").read_text()
    )["bms_node"]["ros__parameters"]
    assert bms_params["primary_battery_state_topic"] == "/battery_state"
    assert bms_params["primary_status_chinese_topic"] == "/bms/primary/status_chinese"
    assert bms_params["primary_fault_chinese_topic"] == "/bms/primary/fault_chinese"
    assert bms_params["publish_period_s"] == 5.0


def test_v3_jtc_runtime_uses_adapter_without_installing_legacy_launch() -> None:
    bringup_manifest = (BRINGUP / "package.xml").read_text()
    bringup_cmake = (BRINGUP / "CMakeLists.txt").read_text()
    bootstrap = (ROOT / "tools/bootstrap_native_dev.sh").read_text()

    assert "<exec_depend>integ_api_adapter</exec_depend>" in bringup_manifest
    assert (
        'executable="control_enable_adapter"'
        in (BRINGUP / "launch/rt_control_scope_runtime.launch.py").read_text()
    )
    assert "launch/rt_control.launch.py" not in bringup_cmake
    assert "integ_api_adapter" in bootstrap
    assert int(PublicErrorCode.RT_ENABLE_MANAGER_NOT_READY) == 1101


def test_public_vacuum_and_readiness_interfaces_are_in_runtime_package_lists() -> None:
    dockerfile = (ROOT / "docker/rt-control/Dockerfile").read_text()
    bootstrap = (ROOT / "tools/bootstrap_native_dev.sh").read_text()

    assert "robot_interfaces_qos" in dockerfile
    assert "robot_interfaces_qos" in bootstrap

    assert "      robot_rt_control_interfaces \\\n" in dockerfile
    assert "      robot_system_interfaces \\\n" in dockerfile
    assert "      integ_api_adapter \\\n" in dockerfile
    assert "\n  robot_rt_control_interfaces\n" in bootstrap
    assert "\n  robot_system_interfaces\n" in bootstrap
    assert "\n  integ_api_adapter\n" in bootstrap
    assert "test -d src/vendor/robot_interfaces/robot_rt_control_interfaces" in dockerfile
    assert "test -d src/vendor/robot_interfaces/robot_system_interfaces" in dockerfile
    assert "src/vendor/robot_interfaces" in bootstrap


# BMS and enable_manager QoS are checked by their package tests on the live publishers.
def test_ros2_controllers_patch_uses_named_robot_interfaces_qos_profiles() -> None:
    controller_patch = (
        ROOT / "patches/ros2_controllers/0002-use-contract-qos-profiles.patch"
    ).read_text()

    assert "robot_interfaces_qos::control()" in controller_patch
    assert controller_patch.count("robot_interfaces_qos::fast_state()") == 2
    assert "diff_drive_controller/test/test_diff_drive_controller.cpp" in controller_patch
    assert 'controller_name + "/cmd_vel", robot_interfaces_qos::control()' in controller_patch


def test_compose_starts_rt_io_in_same_rt_control_container() -> None:
    compose = yaml.safe_load((ROOT / "docker/compose.yaml").read_text())

    assert set(compose["services"]) == {"rt-control"}
    service = compose["services"]["rt-control"]
    assert set(service["cap_add"]) == {"SYS_NICE", "IPC_LOCK", "NET_RAW"}
    assert service["cpuset"].startswith("${RT_CONTROL_CPUSET:?")
    assert service["environment"]["ROS_DOMAIN_ID"] == "${RT_CONTROL_ROS_DOMAIN_ID:-0}"
    assert service["environment"]["ROS_LOCALHOST_ONLY"] == "0"
    assert service["environment"]["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert "CYCLONEDDS_URI" not in service["environment"]
    assert "volumes" not in service
    assert service["environment"]["RT_CONTROL_START_CPUSET"].startswith(
        "${RT_CONTROL_START_CPUSET:?"
    )
    assert "RT_CONTROL_START_PLC" not in service["environment"]
    assert "RT_CONTROL_START_BMS" not in service["environment"]
    assert "command" not in service
    assert "entrypoint" not in service


def _run_compose_wrapper(tmp_path: Path, domain_id: str | None) -> subprocess.CompletedProcess[str]:
    fake_docker = tmp_path / "docker"
    fake_docker.write_text(
        "#!/usr/bin/env bash\n"
        "printf 'domain=%s\\n' \"${RT_CONTROL_ROS_DOMAIN_ID-unset}\"\n"
        "printf 'args=%s\\n' \"$*\"\n",
        encoding="utf-8",
    )
    fake_docker.chmod(0o755)

    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{tmp_path}:{environment['PATH']}",
            "RT_CONTROL_CPUSET": "14",
            "RT_CONTROL_IMAGE_TAG": "domain-contract-test",
            "RT_CONTROL_PROJECT_ROOT": str(ROOT),
        }
    )
    if domain_id is None:
        environment.pop("RT_CONTROL_ROS_DOMAIN_ID", None)
    else:
        environment["RT_CONTROL_ROS_DOMAIN_ID"] = domain_id

    return subprocess.run(
        [str(COMPOSE_WRAPPER), "config"],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )


def test_compose_wrapper_defaults_ros_domain_to_zero(tmp_path: Path) -> None:
    result = _run_compose_wrapper(tmp_path, None)

    assert result.returncode == 0
    assert "domain=0" in result.stdout


def test_compose_wrapper_accepts_explicit_safe_ros_domain(tmp_path: Path) -> None:
    result = _run_compose_wrapper(tmp_path, "12")

    assert result.returncode == 0
    assert "domain=12" in result.stdout


def test_compose_wrapper_rejects_invalid_ros_domains_before_docker(tmp_path: Path) -> None:
    for invalid_domain in ("-1", "08", "233", "not-a-number"):
        result = _run_compose_wrapper(tmp_path, invalid_domain)

        assert result.returncode == 2
        assert "RT_CONTROL_ROS_DOMAIN_ID must be a decimal integer in 0..232" in result.stderr
        assert "args=" not in result.stdout


def test_docker_build_contains_required_io_packages() -> None:
    dockerfile = (ROOT / "docker/rt-control/Dockerfile").read_text()

    assert "      robot_rt_control_interfaces \\\n" in dockerfile
    assert "      periph_bms \\\n" in dockerfile
    assert "      integ_api_adapter \\\n" in dockerfile
    assert "      periph_plc_io \\\n" in dockerfile
    assert "ros-humble-rmw-fastrtps-cpp" in dockerfile
    assert "ros-humble-rmw-cyclonedds-cpp" not in dockerfile
    assert "      util-linux \\\n" in dockerfile
    assert "0001-shared-canopen-lifecycle.patch" in dockerfile
    assert "0005-use-component-parameters-for-ec-modules.patch" in dockerfile
    assert "0006-validate-component-module-parameters.patch" in dockerfile
    assert "0005-derive-motor-topology-from-hardware-info.patch" not in dockerfile
    assert "can_bus_guard" not in dockerfile
    assert not (ROOT / "src/rt_control/can_bus_guard").exists()


def test_bms_can_is_configured_and_started_by_its_own_host_unit() -> None:
    can1_unit = (HOSTSETUP / "can1.service").read_text()
    naming_unit = (HOSTSETUP / "rt-control-can-names.service").read_text()
    installer = (HOSTSETUP / "can-install.sh").read_text()
    verifier = (HOSTSETUP / "verify-host.sh").read_text()

    assert "Requires=rt-control-can-names.service" in can1_unit
    assert "After=rt-control-can-names.service" in can1_unit
    assert "ip link set dev can1 type can bitrate 250000" in can1_unit
    assert "ip link set dev can1 txqueuelen 128" in can1_unit
    assert "ip link set dev can1 up" in can1_unit
    assert "Before=can1.service" in naming_unit
    assert "Before=can0.service" not in naming_unit
    assert '"${script_dir}/can1.service" /etc/systemd/system/can1.service' in installer
    assert '"${script_dir}/can0.service"' not in installer
    assert "disable rt-control-can-names.service can1.service" in installer
    assert "/usr/local/sbin/rt-control-can-names --wait 30 --configure" in installer
    assert "rt-control-can-names.service must not be enabled at boot" in verifier
    assert "can0.service must not be enabled at boot" not in verifier
    assert "can1.service must not be enabled at boot" in verifier
    assert "legacy can0.service is still installed" in installer
    assert "legacy can0.service remains installed" in verifier
    naming_script = (HOSTSETUP / "rt-control-can-names.sh").read_text()
    assert 'readonly BITRATE="250000"' in naming_script
    assert "verify_reserved_name_not_unknown can1" in naming_script
    assert 'wait_for_serial "BMS/can1" "${BMS_SERIAL}"' in naming_script
    assert 'BMS_SERIAL="004D00675230500720333159"' in naming_script
    assert "can0" not in naming_script
