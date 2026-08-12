import xml.etree.ElementTree as ET
from pathlib import Path
import re
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / "src/rt_control/rt_control_bringup"
HOSTSETUP = ROOT / "hostsetup"
sys.path.insert(0, str(ROOT / "src/rt_control/control_api_adapter"))

from control_api_adapter.public_error import PublicErrorCode


def test_public_and_private_interface_packages_have_distinct_ownership() -> None:
    interface_root = ROOT / "src/interfaces"
    public_packages = ("robot_rt_control_interfaces", "robot_system_interfaces")
    source_lock = yaml.safe_load((interface_root / "source-lock.yaml").read_text())

    assert source_lock == {
        "schema_version": 1,
        "repository": "https://github.com/SevenovaHangzhou/robot_interfaces.git",
        "commit": "e19d1450339d6bce598f664eb18fb093e02097ff",
        "contract_version": "0.6.0",
        "mirrored_packages": list(public_packages),
    }

    assert not (interface_root / "alfa_control_interfaces").exists()
    assert not (interface_root / "alfa_system_interfaces").exists()
    assert not (interface_root / "robot_interfaces").exists()

    for package_name in public_packages:
        manifest = ET.parse(interface_root / package_name / "package.xml").getroot()
        assert manifest.findtext("name") == package_name
        assert manifest.findtext("version") == "0.6.0"

    private_manifest = ET.parse(
        interface_root / "rt_control_interfaces/package.xml"
    ).getroot()
    assert private_manifest.findtext("name") == "rt_control_interfaces"
    assert {
        path.relative_to(interface_root / "rt_control_interfaces").as_posix()
        for path in (interface_root / "rt_control_interfaces").glob("**/*")
        if path.is_file() and path.suffix in {".msg", ".srv", ".action"}
    } == {"msg/PlcIoState.msg", "srv/RtEnable.srv"}


def test_rt_io_uses_one_central_hardware_configuration() -> None:
    document = yaml.safe_load((BRINGUP / "config/rt_io.yaml").read_text())
    plc = document["plc_node"]["ros__parameters"]
    bms = document["bms_node"]["ros__parameters"]

    assert plc["host"] == "192.168.1.88"
    assert plc["interface"] == "enp4s0"
    assert plc["poll_period_s"] == 0.5
    assert plc["io_control_register"] == 201
    assert plc["di_status_register"] == 210
    assert plc["do_status_register"] == 211
    assert plc["io_alarm_register"] == 212
    assert bms["can_interface"] == "can1"
    assert bms["publish_period_s"] == 5.0
    assert bms["frame_timeout_s"] == 3.0


def test_public_rt_control_interfaces_match_current_contract() -> None:
    controllers = yaml.safe_load((BRINGUP / "config/controllers.yaml").read_text())
    launch_text = (BRINGUP / "launch/rt_control.launch.py").read_text()
    rt_io = yaml.safe_load((BRINGUP / "config/rt_io.yaml").read_text())

    expected_joints = [
        "right_joint1",
        "right_joint2",
        "right_joint3",
        "right_joint4",
        "right_joint5",
        "right_joint6",
        "left_joint1",
        "left_joint2",
        "left_joint3",
        "left_joint4",
        "left_joint5",
        "left_joint6",
        "turn",
        "updown",
    ]

    manager_params = controllers["controller_manager"]["ros__parameters"]
    assert "whole_body_jtc" in manager_params
    assert "dual_arm_jtc" not in manager_params
    assert (
        manager_params["rt_internal_state_broadcaster"]["type"]
        == "joint_state_broadcaster/JointStateBroadcaster"
    )

    joint_state_params = controllers["joint_state_broadcaster"]["ros__parameters"]
    assert joint_state_params["update_rate"] == 100
    assert joint_state_params["joints"] == expected_joints
    assert joint_state_params["interfaces"] == ["position"]
    assert joint_state_params["publish_dynamic_joint_states"] is False

    internal_state_params = controllers["rt_internal_state_broadcaster"]["ros__parameters"]
    assert internal_state_params["update_rate"] == 50
    assert internal_state_params["use_local_topics"] is True
    assert internal_state_params["publish_dynamic_joint_states"] is True
    assert "joints" not in internal_state_params
    assert "interfaces" not in internal_state_params

    jtc_params = controllers["whole_body_jtc"]["ros__parameters"]
    assert jtc_params["joints"] == expected_joints
    assert jtc_params["allow_partial_joints_goal"] is False

    diff_drive_params = controllers["diff_drive_controller"]["ros__parameters"]
    assert diff_drive_params["cmd_vel_timeout"] == 0.5
    assert diff_drive_params["use_stamped_vel"] is False
    assert '("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel_safe")' in launch_text
    assert '"/cmd_vel"' not in launch_text

    bms_params = rt_io["bms_node"]["ros__parameters"]
    assert bms_params["battery_state_topic"] == "/battery_state"
    assert bms_params["publish_period_s"] == 5.0

    vacuum_params = rt_io["vacuum_adapter"]["ros__parameters"]
    assert vacuum_params["vacuum_state_topic"] == "/vacuum/state"
    assert vacuum_params["pump_set_enabled_service"] == "/vacuum/pump/set_enabled"
    assert vacuum_params["grip_action_name"] == "/vacuum/grip"
    assert vacuum_params["publish_period_s"] == 0.05
    assert vacuum_params["accepted_grip_profile_ids"] == ["default"]

    status_params = rt_io["rt_status_adapter"]["ros__parameters"]
    assert status_params["safety_state_topic"] == "/control/safety_state"
    assert status_params["readiness_topic"] == "/rt_control/readiness"
    assert status_params["safety_publish_period_s"] == 0.1
    assert status_params["readiness_publish_period_s"] == 1.0


def test_public_control_enable_adapter_is_started_with_rt_control() -> None:
    launch_text = (BRINGUP / "launch/rt_control.launch.py").read_text()
    bringup_manifest = (BRINGUP / "package.xml").read_text()
    interface_srv = (
        ROOT
        / "src/interfaces/robot_rt_control_interfaces/srv/SetControlEnabled.srv"
    ).read_text()

    assert 'package="control_api_adapter"' in launch_text
    assert 'executable="control_enable_adapter"' in launch_text
    assert "<exec_depend>control_api_adapter</exec_depend>" in bringup_manifest
    interface_fields = [
        line.strip()
        for line in interface_srv.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    assert interface_fields == [
        "bool enabled",
        "string reason",
        "---",
        "bool accepted",
        "bool enabled",
        "robot_system_interfaces/ErrorInfo error",
    ]


def test_all_public_rt_command_results_use_error_info() -> None:
    interface_root = ROOT / "src/interfaces/robot_rt_control_interfaces"
    for relative_path in (
        "srv/SetControlEnabled.srv",
        "srv/SetPumpEnabled.srv",
        "action/VacuumGrip.action",
    ):
        interface_text = (interface_root / relative_path).read_text()
        assert interface_text.count("robot_system_interfaces/ErrorInfo error") == 1
        assert "uint16 error_code" not in interface_text


def test_public_error_mapping_matches_contract_constants() -> None:
    error_code_idl = (
        ROOT / "src/interfaces/robot_system_interfaces/msg/ErrorCode.msg"
    ).read_text()
    contract_constants = {
        name: int(value)
        for name, value in re.findall(
            r"^uint32\s+([A-Z][A-Z0-9_]*)=(\d+)\s*$",
            error_code_idl,
            flags=re.MULTILINE,
        )
    }

    assert {
        item.name: int(item.value) for item in PublicErrorCode
    }.items() <= contract_constants.items()


def test_public_vacuum_and_state_adapters_are_started_with_rt_control() -> None:
    launch_text = (BRINGUP / "launch/rt_control.launch.py").read_text()
    adapter_cmake = (
        ROOT / "src/rt_control/control_api_adapter/CMakeLists.txt"
    ).read_text()
    adapter_manifest = (
        ROOT / "src/rt_control/control_api_adapter/package.xml"
    ).read_text()

    assert 'executable="vacuum_adapter"' in launch_text
    assert 'executable="rt_status_adapter"' in launch_text
    assert '"rt_internal_state_broadcaster"' in launch_text
    assert '"/rt_internal_state_broadcaster/dynamic_joint_states"' in launch_text
    assert "parameters=[rt_io_file, {\"use_sim_time\": use_sim_time}]" in launch_text
    assert "scripts/vacuum_adapter" in adapter_cmake
    assert "scripts/rt_status_adapter" in adapter_cmake
    assert "<exec_depend>robot_system_interfaces</exec_depend>" in adapter_manifest
    assert "<exec_depend>sensor_msgs</exec_depend>" in adapter_manifest
    assert "<exec_depend>std_srvs</exec_depend>" in adapter_manifest


def test_internal_dynamic_state_is_used_only_for_rt_diagnostics() -> None:
    diagnostics_source = (
        ROOT / "src/rt_control/rt_diagnostics/src/rt_diagnostics_node.cpp"
    ).read_text()
    native_launcher = (ROOT / "tools/rt_control_native.sh").read_text()
    ipc_launcher = (ROOT / "tools/rt_control_ipc.sh").read_text()

    internal_topic = "/rt_internal_state_broadcaster/dynamic_joint_states"
    assert "declare_parameter<std::string>" in diagnostics_source
    assert '"dynamic_joint_states_topic"' in diagnostics_source
    assert internal_topic in diagnostics_source
    assert internal_topic in native_launcher
    assert internal_topic in ipc_launcher


def test_public_vacuum_and_readiness_interfaces_are_in_runtime_package_lists() -> None:
    dockerfile = (ROOT / "docker/rt-control/Dockerfile").read_text()
    bootstrap = (ROOT / "tools/bootstrap_native_dev.sh").read_text()

    assert "      robot_rt_control_interfaces \\\n" in dockerfile
    assert "      robot_system_interfaces \\\n" in dockerfile
    assert "      control_api_adapter \\\n" in dockerfile
    assert "\n  robot_rt_control_interfaces\n" in bootstrap
    assert "\n  robot_system_interfaces\n" in bootstrap
    assert "\n  control_api_adapter\n" in bootstrap


def test_main_launch_owns_both_nodes_with_safe_direct_launch_defaults() -> None:
    launch_text = (BRINGUP / "launch/rt_control.launch.py").read_text()

    assert '"RT_CONTROL_START_PLC", default_value="false"' in launch_text
    assert '"RT_CONTROL_START_BMS", default_value="false"' in launch_text
    assert 'package="plc_node"' in launch_text
    assert 'package="bms_node"' in launch_text
    assert "rt_io.yaml" in launch_text


def test_rt_io_nodes_are_not_respawned_during_container_shutdown() -> None:
    launch_text = (BRINGUP / "launch/rt_control.launch.py").read_text()

    assert "respawn=True" not in launch_text


def test_compose_starts_rt_io_in_same_rt_control_container() -> None:
    compose = yaml.safe_load((ROOT / "docker/compose.yaml").read_text())

    assert set(compose["services"]) == {"rt-control"}
    service = compose["services"]["rt-control"]
    assert set(service["cap_add"]) == {"SYS_NICE", "IPC_LOCK", "NET_RAW"}
    assert service["cpuset"].startswith("${RT_CONTROL_CPUSET:?")
    assert service["environment"]["ROS_DOMAIN_ID"] == "0"
    assert service["environment"]["ROS_LOCALHOST_ONLY"] == "0"
    assert service["environment"]["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert "CYCLONEDDS_URI" not in service["environment"]
    assert "volumes" not in service
    assert service["environment"]["RT_CONTROL_START_CPUSET"].startswith(
        "${RT_CONTROL_START_CPUSET:?"
    )
    assert service["environment"]["RT_CONTROL_START_PLC"] == "true"
    assert service["environment"]["RT_CONTROL_START_BMS"] == "true"
    assert "command" not in service
    assert "entrypoint" not in service


def test_docker_build_contains_only_the_two_required_io_packages() -> None:
    dockerfile = (ROOT / "docker/rt-control/Dockerfile").read_text()

    assert "      robot_rt_control_interfaces \\\n" in dockerfile
    assert "      bms_node \\\n" in dockerfile
    assert "      control_api_adapter \\\n" in dockerfile
    assert "      plc_node \\\n" in dockerfile
    assert "ros-humble-rmw-fastrtps-cpp" in dockerfile
    assert "ros-humble-rmw-cyclonedds-cpp" not in dockerfile
    assert "      util-linux \\\n" in dockerfile
    assert "0004-name-canopen-master-loop-thread.patch" in dockerfile
    assert "can_bus_guard" not in dockerfile
    assert not (ROOT / "src/rt_control/can_bus_guard").exists()


def test_removed_duplicate_and_unused_ros_interfaces_do_not_return() -> None:
    plc_source = (
        ROOT / "src/rt_control/plc_node/plc_node/plc_node.py"
    ).read_text()
    bms_source = (
        ROOT / "src/rt_control/bms_node/bms_node/bms_node.py"
    ).read_text()
    bms_manifest = (ROOT / "src/rt_control/bms_node/package.xml").read_text()

    assert "create_subscription" not in plc_source
    assert '"/plc/command"' not in plc_source
    assert "/command" not in plc_source
    assert bms_source.count("create_publisher") == 1
    assert "can_bus_guard" not in bms_source
    assert "can_bus_guard" not in bms_manifest


def test_bms_can_is_configured_and_started_by_its_own_host_unit() -> None:
    can1_unit = (HOSTSETUP / "can1.service").read_text()
    naming_unit = (HOSTSETUP / "rt-control-can-names.service").read_text()
    installer = (HOSTSETUP / "can-install.sh").read_text()
    verifier = (HOSTSETUP / "verify-host.sh").read_text()
    launcher = (ROOT / "tools/rt_control_ipc.sh").read_text()

    assert "Requires=rt-control-can-names.service" in can1_unit
    assert "After=rt-control-can-names.service" in can1_unit
    assert "ip link set dev can1 type can bitrate 500000" in can1_unit
    assert "ip link set dev can1 txqueuelen 128" in can1_unit
    assert "ip link set dev can1 up" in can1_unit
    assert "Before=can0.service can1.service" in naming_unit
    assert '"${script_dir}/can1.service" /etc/systemd/system/can1.service' in installer
    assert "disable rt-control-can-names.service can0.service can1.service" in installer
    assert "/usr/local/sbin/rt-control-can-names --wait 30 --configure" in installer
    assert "rt-control-can-names.service must not be enabled at boot" in verifier
    assert "can0.service must not be enabled at boot" in verifier
    assert "can1.service must not be enabled at boot" in verifier
    assert 'can_setup_tool="${repository_root}/hostsetup/rt-control-can-names.sh"' in launcher
    assert 'readonly expected_container_cpuset="0,2,4,6,8,10,12,14,16-27"' in launcher
    assert 'RT_CONTROL_START_CPUSET="${expected_housekeeping_cpuset}"' in launcher
    assert "pin_controller_update_thread" in launcher
    assert "controller_manager_running_in_container" in launcher
    assert '"${can_setup_tool}" --wait 30 --configure' in launcher
    assert "/control/set_enabled" in launcher
