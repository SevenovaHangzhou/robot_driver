from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / "src/rt_control/rt_control_bringup"
HOSTSETUP = ROOT / "hostsetup"


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

    assert "      bms_node \\\n" in dockerfile
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
