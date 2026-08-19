#!/usr/bin/env python3

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "tools" / "rt_control_native.sh"
BOOTSTRAP = ROOT / "tools" / "bootstrap_native_dev.sh"
IGH_INSTALLER = ROOT / "hostsetup" / "igh-install.sh"
HOST_VERIFIER = ROOT / "hostsetup" / "verify-host.sh"
RT_LAUNCH = ROOT / "src" / "rt_control" / "rt_control_bringup" / "launch" / "rt_control.launch.py"
SIGNAL_GATE = ROOT / "src/rt_control/rt_control_bringup/scripts/rt_control_start"
ONECLICK = ROOT / "tools" / "rt_control_native_oneclick.sh"
CPU_GUARD = ROOT / "tools" / "rt_cpu_contamination_check.sh"
THREAD_AFFINITY = ROOT / "tools" / "rt_control_thread_affinity.py"
HEARTBEAT_WATCH = ROOT / "tools" / "canopen_heartbeat_watch.sh"
CAN_SETUP = ROOT / "hostsetup" / "rt-control-can-names.sh"
CANOPEN_MASTER_THREAD_PATCH = (
    ROOT / "patches" / "ros2_canopen" / "0004-name-canopen-master-loop-thread.patch"
)


class NativeLauncherContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = LAUNCHER.read_text(encoding="utf-8")

    def test_runtime_is_fixed_to_shared_domain_and_default_fastdds_transports(self):
        self.assertIn('readonly expected_ros_domain_id="0"', self.text)
        self.assertIn('ROS_DOMAIN_ID="${expected_ros_domain_id}"', self.text)
        self.assertIn('RMW_IMPLEMENTATION="rmw_fastrtps_cpp"', self.text)
        self.assertIn('-u FASTRTPS_DEFAULT_PROFILES_FILE', self.text)
        self.assertIn('-u FASTDDS_DEFAULT_PROFILES_FILE', self.text)
        self.assertIn('-u CYCLONEDDS_URI', self.text)
        self.assertNotIn("fastdds_udp_only.xml", self.text)

    def test_native_runtime_cannot_overlap_the_container_runtime(self):
        start = self.text.index("start_native()")
        stop = self.text.index("enable_native()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("reject_running_container"), body.index("launch_native"))

    def test_plain_start_does_not_pin_before_controllers_exist(self):
        start = self.text.index("start_native()")
        stop = self.text.index("enable_native()", start)
        body = self.text[start:stop]
        self.assertNotIn("pin_controller_update_thread", body)

    def test_plain_start_does_not_enable_or_reset_faults(self):
        start = self.text.index("start_native()")
        stop = self.text.index("enable_native()", start)
        body = self.text[start:stop]
        self.assertNotIn("call_rt_service enable", body)
        self.assertNotIn("call_rt_service reset_fault", body)

    def test_start_and_enable_requires_confirmation_and_never_resets_faults(self):
        start = self.text.index("start_and_enable_native()")
        stop = self.text.index("recover_power_loss_native()", start)
        body = self.text[start:stop]
        self.assertLess(
            body.index("confirm_enable_authorization"),
            body.index("start_native"),
        )
        self.assertLess(body.index("start_native"), body.index("call_rt_service enable"))
        self.assertNotIn("call_rt_service reset_fault", body)

    def test_reset_fault_is_only_used_by_the_explicit_recovery_entrypoint(self):
        self.assertIn("enable|disable|reset_fault", self.text)
        self.assertIn("recover-power-loss) recover_power_loss_native", self.text)

        start = self.text.index("recover_power_loss_native()")
        stop = self.text.index("stop_native()", start)
        body = self.text[start:stop]
        self.assertNotIn("confirm_recovery_authorization", body)
        self.assertLess(
            body.index("wait_for_enable_manager_reset_ready"),
            body.index("clear_resettable_faults_before_enable"),
        )
        self.assertLess(
            body.index("clear_resettable_faults_before_enable"),
            body.index("verify_enable_manager_diagnostic_state IDLE success"),
        )
        self.assertLess(
            body.index("verify_enable_manager_diagnostic_state IDLE success"),
            body.index("call_rt_service enable"),
        )
        self.assertLess(
            body.index("call_rt_service enable"),
            body.index("verify_enable_manager_diagnostic_state ENABLED success"),
        )
        self.assertLess(
            body.index("verify_enable_manager_diagnostic_state ENABLED success"),
            body.index("monitor_native_warning_logs"),
        )

    def test_recovery_fault_reset_reports_unclearable_fault_context(self):
        self.assertIn("clear_resettable_faults_before_enable()", self.text)
        start = self.text.index("clear_resettable_faults_before_enable()")
        stop = self.text.index("verify_controllers_before_enable()", start)
        body = self.text[start:stop]
        self.assertIn("call_rt_service reset_fault", body)
        self.assertIn("print_fault_recovery_context", body)
        self.assertIn("power-cycle the main contactor", body)

    def test_reset_ready_gate_rejects_restart_required_before_idle(self):
        start = self.text.index("wait_for_enable_manager_reset_ready()")
        stop = self.text.index("enable_manager_diagnostic_snapshot()", start)
        body = self.text[start:stop]
        self.assertIn("value: restart_required", body)
        self.assertLess(body.index("value: FAILED"), body.index("value: IDLE"))
        self.assertLess(body.index("value: restart_required"), body.index("value: IDLE"))

    def test_jtc_deactivate_is_idempotent_before_strict_switch(self):
        source = (
            ROOT
            / "src"
            / "rt_control"
            / "enable_manager"
            / "src"
            / "enable_manager_controller.cpp"
        ).read_text(encoding="utf-8")
        start = source.index("EnableManagerController::SwitchResult EnableManagerController::switchJtc")
        stop = source.index("void EnableManagerController::handleNonRtFaultStop", start)
        body = source[start:stop]
        self.assertIn("ListControllers", body)
        self.assertLess(body.index('controller.state != "active"'), body.index("SwitchController::Request"))
        self.assertIn("return SwitchResult::kSuccess", body)

    def test_emergency_jtc_deactivate_only_runs_after_enable_requested_jtc(self):
        source = (
            ROOT
            / "src"
            / "rt_control"
            / "enable_manager"
            / "src"
            / "enable_manager_controller.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("jtc_deactivation_required_.store(false", source)
        enable_start = source.index("const SwitchResult activation_result = switchJtc(true)")
        self.assertLess(
            source.rindex("jtc_deactivation_required_.store(true", 0, enable_start),
            enable_start,
        )

        start = source.index("void EnableManagerController::handleNonRtFaultStop")
        stop = source.index("void EnableManagerController::publishDiagnostics", start)
        body = source[start:stop]
        self.assertLess(
            body.index("jtc_deactivation_required_.load"),
            body.index("switchJtc(false)"),
        )

    def test_reset_fault_uses_standard_disable_sequence_after_fault_clears(self):
        source = (
            ROOT
            / "src"
            / "rt_control"
            / "enable_manager"
            / "src"
            / "enable_manager_controller.cpp"
        ).read_text(encoding="utf-8")
        start = source.index("void EnableManagerController::updateReset")
        stop = source.index("void EnableManagerController::setAllControlWords", start)
        body = source[start:stop]
        self.assertIn("startDownward(Phase::kResetDisabling", body)
        self.assertNotIn("DriveState::kOperationEnabled", body)
        self.assertNotIn("DriveState::kSwitchedOn", body)

        header = (
            ROOT
            / "src"
            / "rt_control"
            / "enable_manager"
            / "include"
            / "enable_manager"
            / "enable_manager_controller.hpp"
        ).read_text(encoding="utf-8")
        self.assertIn("kResetDisabling", header)

        downward_start = source.index("void EnableManagerController::updateDownward")
        downward_stop = source.index("void EnableManagerController::startEmergency", downward_start)
        downward_body = source[downward_start:downward_stop]
        self.assertIn("case DriveState::kReadyToSwitchOn", downward_body)
        self.assertIn("command = 0x0000U", downward_body)

        finish_start = source.index("void EnableManagerController::finishDownward")
        finish_stop = source.index("void EnableManagerController::updateEnable", finish_start)
        finish_body = source[finish_start:finish_stop]
        self.assertIn("completed_phase == Phase::kResetDisabling", finish_body)
        self.assertIn("publishResult(reset_result_, true, Stage::kSuccess)", finish_body)
        self.assertIn("publishResult(reset_result_, false, Stage::kFaultResetTimeout", finish_body)

    def test_native_launcher_self_raises_rtprio_and_memlock_limits(self):
        self.assertIn("ensure_runtime_limits", self.text)
        start = self.text.index("verify_realtime_host()")
        stop = self.text.index("verify_igh_run_on_cpu_config()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("ensure_runtime_limits"), body.index("ulimit -r"))
        self.assertIn("sudo prlimit --pid", self.text)

    def test_axis_state_checker_is_read_as_python_not_executed(self):
        start = self.text.index("check_axis_states()")
        stop = self.text.index("verify_operational_ethercat()", start)
        body = self.text[start:stop]
        self.assertIn('[[ -f "${axis_state_checker}" && -r "${axis_state_checker}" ]]', body)
        self.assertIn('python3 "${axis_state_checker}"', body)
        self.assertNotIn('-x "${axis_state_checker}"', body)

    def test_axis_state_checker_retries_ros2_echo_sampling(self):
        start = self.text.index("check_axis_states()")
        stop = self.text.index("verify_operational_ethercat()", start)
        body = self.text[start:stop]
        self.assertIn("local deadline", body)
        self.assertIn("while (( SECONDS < deadline )); do", body)
        self.assertIn("sleep 1", body)
        self.assertIn("last_error", body)

    def test_stop_requests_disable_before_signalling_runtime(self):
        start = self.text.index("stop_native()")
        stop = self.text.index("status_native()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("call_rt_service disable"), body.index("kill -TERM"))

    def test_verified_realtime_cpu_is_applied_to_the_runtime(self):
        self.assertIn('readonly expected_cpuset="14"', self.text)
        self.assertIn('readonly expected_housekeeping_cpuset="0,2,4,6,16-27"', self.text)
        self.assertIn("cpu_list_contains", self.text)
        self.assertIn("cpu_list_overlaps", self.text)
        self.assertIn('CPU ${expected_cpuset} is not included in isolated CPUs', self.text)
        self.assertIn('housekeeping CPUs ${expected_housekeeping_cpuset} overlap isolated CPUs', self.text)
        self.assertIn('command -v setsid', self.text)
        self.assertIn('nohup setsid env', self.text)
        self.assertIn('taskset --cpu-list "${expected_housekeeping_cpuset}"', self.text)
        self.assertNotIn('taskset --cpu-list "${expected_cpuset}" \\\n    "${installed_start}"', self.text)
        self.assertNotIn("RT_CONTROL_ENABLE_NODE_AFFINITY", self.text)

    def test_native_start_rejects_realtime_thread_pollution_on_cpu14(self):
        self.assertIn('realtime_cpu_guard="${repository_root}/tools/rt_cpu_contamination_check.sh"', self.text)
        self.assertIn("verify_realtime_cpu_guard", self.text)
        start = self.text.index("start_native()")
        stop = self.text.index("enable_native()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("verify_realtime_cpu_guard"), body.index("launch_native"))

    def test_native_preflight_requires_igh_to_run_on_the_isolated_cpu(self):
        self.assertIn("verify_igh_run_on_cpu_config", self.text)
        self.assertIn('options ec_master run_on_cpu=${expected_cpuset}', self.text)
        self.assertIn("verify_ethercat_op_thread_cpu", self.text)

    def test_native_pins_controller_update_and_canopen_master_loop_after_startup(self):
        self.assertIn('thread_affinity_tool="${repository_root}/tools/rt_control_thread_affinity.py"', self.text)
        self.assertIn("pin_controller_update_thread", self.text)
        self.assertIn("--required-rt-thread-name rtcan-master", self.text)
        start = self.text.index("call_rt_service()")
        stop = self.text.index("verify_target_identity()", start)
        body = self.text[start:stop]
        self.assertLess(body.index('[[ "${operation}" == "enable" ]]'), body.index("pin_controller_update_thread"))
        self.assertLess(body.index("pin_controller_update_thread"), body.index('"/rt/${operation}"'))

    def test_native_start_forces_can_preflight_after_authorization_before_launch(self):
        self.assertIn('can_setup_tool="${repository_root}/hostsetup/rt-control-can-names.sh"', self.text)
        self.assertIn("prepare_can_interfaces", self.text)
        self.assertIn("--wait 30 --configure", self.text)
        self.assertIn("EtherCAT service must be active", self.text)
        self.assertNotIn("CAN naming, can0 and can1 services must be active", self.text)

        start = self.text.index("start_native()")
        stop = self.text.index("enable_native()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("confirm_start_authorization"), body.index("prepare_can_interfaces"))
        self.assertLess(body.index("prepare_can_interfaces"), body.index("launch_native"))

        recovery_start = self.text.index("recover_power_loss_native()")
        recovery_stop = self.text.index("stop_native()", recovery_start)
        recovery_body = self.text[recovery_start:recovery_stop]
        self.assertLess(
            recovery_body.index("best_effort_stop_existing_native_for_recovery"),
            recovery_body.index("prepare_can_interfaces"),
        )
        self.assertLess(
            recovery_body.index("prepare_can_interfaces"),
            recovery_body.index("start_native preauthorized"),
        )

    def test_boot_failure_does_not_wait_full_enable_timeout_after_controller_manager_exit(self):
        self.assertIn("controller_manager_died_during_boot", self.text)
        self.assertIn("print_first_boot_error", self.text)
        self.assertIn("terminate_native_launch_tree", self.text)
        start = self.text.index("wait_for_enable_service()")
        stop = self.text.index("terminate_failed_start()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("controller_manager_died_during_boot"), body.index("run_ros2_timeout 3 ros2 service type /rt/enable"))
        self.assertIn("ros2_control_node exited during boot", body)

    def test_stop_skips_disable_when_controller_manager_has_already_exited(self):
        start = self.text.index("stop_native()")
        stop = self.text.index("status_native()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("controller_manager_running_under_native"), body.index("call_rt_service disable"))
        self.assertIn("without waiting for /rt/disable", body)
        self.assertIn("terminate_native_launch_tree", body)

    def test_every_native_enable_service_call_is_hard_gated_by_thread_pin(self):
        start = self.text.index("call_rt_service()")
        stop = self.text.index("verify_target_identity()", start)
        body = self.text[start:stop]
        self.assertIn('if [[ "${operation}" == "enable" ]]; then', body)
        self.assertLess(body.index('if [[ "${operation}" == "enable" ]]; then'), body.index("run_ros2_timeout 40 ros2 service call"))
        self.assertLess(body.index("pin_controller_update_thread"), body.index("run_ros2_timeout 40 ros2 service call"))

    def test_oneclick_recovery_preserves_enable_pin_gate_after_fault_reset(self):
        start = self.text.index("recover_power_loss_native()")
        stop = self.text.index("stop_native()", start)
        body = self.text[start:stop]
        self.assertLess(
            body.index("clear_resettable_faults_before_enable"),
            body.index("verify_enable_manager_diagnostic_state IDLE success"),
        )
        self.assertLess(
            body.index("verify_enable_manager_diagnostic_state IDLE success"),
            body.index("call_rt_service enable"),
        )
        self.assertIn("call_rt_service enable", body)

    def test_oneclick_recovery_monitors_logs_by_default_after_successful_enable(self):
        self.assertIn("should_monitor_native_logs()", self.text)
        self.assertIn("RT_CONTROL_NATIVE_MONITOR", self.text)
        start = self.text.index("recover_power_loss_native()")
        stop = self.text.index("stop_native()", start)
        body = self.text[start:stop]
        self.assertLess(
            body.index("verify_enable_manager_diagnostic_state ENABLED success"),
            body.index("should_monitor_native_logs"),
        )
        self.assertLess(body.index("should_monitor_native_logs"), body.index("monitor_native_warning_logs"))

    def test_runtime_sources_ros_setup_without_nounset(self):
        start = self.text.index("source_runtime_environment()")
        stop = self.text.index("runtime_env()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("set +u"), body.index("source /opt/ros/humble/setup.bash"))
        self.assertLess(body.index('source "${install_root}/setup.bash"'), body.index("set -u"))

    def test_axis_policy_comes_from_the_selected_install_root(self):
        self.assertIn(
            'enable_manager_config="${install_root}/share/'
            'rt_control_bringup/config/controllers.yaml"',
            self.text,
        )
        self.assertNotIn(
            'enable_manager_config="${repository_root}/src/rt_control/'
            'rt_control_bringup/config/controllers.yaml"',
            self.text,
        )

    def test_native_entrypoints_verify_public_adapter_dependency_closure_before_hardware(self):
        self.assertIn("verify_runtime_dependency_closure()", self.text)
        helper_start = self.text.index("verify_runtime_dependency_closure()")
        helper_stop = self.text.index("verify_realtime_cpu_guard()", helper_start)
        helper = self.text[helper_start:helper_stop]

        for package in (
            "robot_interfaces_qos",
            "robot_rt_control_interfaces",
            "robot_system_interfaces",
            "rt_control_interfaces",
            "bms_node",
            "control_api_adapter",
            "plc_node",
            "rt_diagnostics",
            "rt_control_bringup",
        ):
            self.assertIn(package, helper)
        for profile in ("control", "fast_state", "state", "latched", "diagnostic"):
            self.assertIn(profile, helper)
        self.assertIn("runtime_env python3 -", helper)

        start = self.text.index("start_native()")
        stop = self.text.index("enable_native()", start)
        start_body = self.text[start:stop]
        self.assertLess(
            start_body.index("source_runtime_environment"),
            start_body.index("verify_runtime_dependency_closure"),
        )
        self.assertLess(
            start_body.index("verify_runtime_dependency_closure"),
            start_body.index("verify_bus_services"),
        )

        recovery_start = self.text.index("recover_power_loss_native()")
        recovery_stop = self.text.index("stop_native()", recovery_start)
        recovery_body = self.text[recovery_start:recovery_stop]
        self.assertLess(
            recovery_body.index("verify_runtime_dependency_closure"),
            recovery_body.index("verify_bus_services"),
        )

        doctor_start = self.text.index("doctor_native()")
        doctor_stop = self.text.index('case "${1:-}"', doctor_start)
        doctor_body = self.text[doctor_start:doctor_stop]
        self.assertIn("verify_runtime_dependency_closure", doctor_body)

    def test_symlink_install_signal_gate_source_is_executable(self):
        self.assertTrue(
            SIGNAL_GATE.stat().st_mode & 0o111,
            "rt_control_start must be executable because native symlink-install points to it",
        )
        text = SIGNAL_GATE.read_text(encoding="utf-8")
        self.assertIn("RT_CONTROL_START_CPUSET", text)
        self.assertIn("taskset --cpu-list", text)
        self.assertIn("setsid", text)


class NativeBootstrapContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = BOOTSTRAP.read_text(encoding="utf-8")

    def test_bootstrap_uses_frozen_dependency_manifest_and_refuses_partial_vendor_tree(self):
        self.assertIn('dependency_manifest="${repository_root}/deps.repos"', self.text)
        self.assertIn("refuse_partial_vendor_tree", self.text)
        self.assertIn("verify_vendor_heads", self.text)
        self.assertNotIn("git reset", self.text)
        self.assertNotIn("git checkout", self.text)

    def test_patches_are_idempotent_and_never_rewrite_vendor_history(self):
        self.assertIn('apply --reverse --check "${patch_file}"', self.text)
        self.assertIn('apply --check "${patch_file}"', self.text)
        self.assertIn("verify_frozen_vendor_trees", self.text)
        self.assertIn('GIT_INDEX_FILE="${actual_index}"', self.text)
        self.assertNotIn("git am", self.text)

        start = self.text.index("prepare_sources()")
        stop = self.text.index("source_build_environment()", start)
        body = self.text[start:stop]
        self.assertIn("if (verify_frozen_vendor_trees)", body)
        self.assertLess(
            body.index("if (verify_frozen_vendor_trees)"),
            body.index("apply_frozen_patches"),
        )

    def test_bootstrap_applies_the_canopen_master_thread_identity_patch(self):
        self.assertIn(
            "patches/ros2_canopen/0004-name-canopen-master-loop-thread.patch",
            self.text,
        )

    def test_bootstrap_applies_typed_hardware_topology_patches(self):
        for patch in (
            "patches/ecat_icube/0004-use-component-parameters-for-ec-modules.patch",
            "patches/ecat_icube/0005-validate-component-module-parameters.patch",
            "patches/ros2_canopen/0005-derive-motor-topology-from-hardware-info.patch",
        ):
            self.assertGreaterEqual(self.text.count(patch), 2, patch)

    def test_bootstrap_applies_the_cross_domain_qos_patch(self):
        self.assertIn(
            "patches/ros2_controllers/0002-use-contract-qos-profiles.patch",
            self.text,
        )

    def test_build_is_incremental_and_kept_outside_the_repository(self):
        self.assertIn("--symlink-install", self.text)
        self.assertIn('build_base="${workspace_root}/build"', self.text)
        self.assertIn('install_base="${workspace_root}/install"', self.text)
        self.assertIn('log_base="${workspace_root}/log"', self.text)
        self.assertIn("robot_rt_control_interfaces", self.text)
        self.assertIn("control_api_adapter", self.text)

    def test_bootstrap_sources_ros_setup_without_nounset(self):
        start = self.text.index("source_build_environment()")
        stop = self.text.index("dependency_paths()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("set +u"), body.index("source /opt/ros/humble/setup.bash"))
        self.assertLess(body.index("source /opt/ros/humble/setup.bash"), body.index("set -u"))

    def test_full_runtime_build_verifies_installed_dependency_closure(self):
        self.assertIn("verify_runtime_install()", self.text)
        helper_start = self.text.index("verify_runtime_install()")
        helper_stop = self.text.index("build_workspace()", helper_start)
        helper = self.text[helper_start:helper_stop]
        self.assertIn('for package in "${runtime_packages[@]}"', helper)
        self.assertIn("ros2 pkg prefix", helper)
        self.assertIn("runtime_qos_profile_smoke", helper)

        build_start = self.text.index("build_workspace()")
        build_stop = self.text.index("doctor()", build_start)
        build_body = self.text[build_start:build_stop]
        self.assertIn("verify_full_runtime=1", build_body)
        custom_selection = build_body.index("if (( $# > 0 ))")
        default_selection = build_body.index("else", custom_selection)
        selection_done = build_body.index("fi", default_selection)
        self.assertNotIn(
            "--cmake-clean-cache",
            build_body[custom_selection:default_selection],
        )
        self.assertIn(
            "--cmake-clean-cache",
            build_body[default_selection:selection_done],
        )
        self.assertLess(
            build_body.index("colcon --log-base"),
            build_body.index("verify_runtime_install"),
        )


class NativeHostSetupContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.installer_text = IGH_INSTALLER.read_text(encoding="utf-8")
        cls.verifier_text = HOST_VERIFIER.read_text(encoding="utf-8")
        cls.bootstrap_text = BOOTSTRAP.read_text(encoding="utf-8")
        cls.launcher_text = LAUNCHER.read_text(encoding="utf-8")

    def test_host_igh_install_writes_the_dependency_identity_used_by_native_checks(self):
        expected_path = "/usr/local/share/rt-control/dependency-versions.env"
        self.assertIn(expected_path, self.installer_text)
        self.assertIn(expected_path, self.bootstrap_text)
        self.assertIn(expected_path, self.launcher_text)
        self.assertIn("IGH_VERSION=%s\\nIGH_COMMIT=%s\\n", self.installer_text)
        self.assertIn('install -d -m 0755 /usr/local/share/rt-control', self.installer_text)

    def test_host_igh_install_pins_master_thread_to_the_isolated_cpu(self):
        self.assertIn("/etc/modprobe.d/ec_master.conf", self.installer_text)
        self.assertIn("options ec_master run_on_cpu=14", self.installer_text)
        self.assertIn("/etc/modprobe.d/ec_master.conf", self.verifier_text)
        self.assertIn("options ec_master run_on_cpu=14", self.verifier_text)

    def test_can_units_are_installed_but_not_enabled_at_boot(self):
        can_installer = (ROOT / "hostsetup" / "can-install.sh").read_text(encoding="utf-8")
        self.assertIn("systemctl disable rt-control-can-names.service can0.service can1.service", can_installer)
        self.assertNotIn("systemctl enable rt-control-can-names.service can0.service can1.service", can_installer)
        self.assertIn("rt-control-can-names.service must not be enabled at boot", self.verifier_text)
        self.assertIn("can0.service must not be enabled at boot", self.verifier_text)
        self.assertIn("can1.service must not be enabled at boot", self.verifier_text)

    def test_can_setup_keeps_gs_usb_autoload_and_forces_runtime_configuration(self):
        self.assertTrue(CAN_SETUP.exists(), "CAN setup helper is missing")
        self.assertTrue(CAN_SETUP.stat().st_mode & 0o111, "CAN setup helper must be executable")
        text = CAN_SETUP.read_text(encoding="utf-8")
        self.assertIn('readonly RT_CONTROL_SERIAL="004D00675230500720333159"', text)
        self.assertIn('readonly BMS_SERIAL="003000265230500720333159"', text)
        self.assertIn("--wait SECONDS", text)
        self.assertIn("verify_reserved_name_not_unknown can0", text)
        self.assertIn("verify_reserved_name_not_unknown can1", text)
        self.assertIn('"${RT_CONTROL_SERIAL}"|"${BMS_SERIAL}")', text)
        self.assertIn("unapproved USB serial", text)
        self.assertIn("deadline=$((SECONDS + 5))", text)
        self.assertNotIn("verify_name_not_foreign", text)
        self.assertIn('ip link set dev "${interface}" type can bitrate "${BITRATE}"', text)
        self.assertIn('ip link set dev "${interface}" txqueuelen "${TXQUEUELEN}"', text)
        self.assertIn('ip link set dev "${interface}" up', text)
        self.assertIn('missing ${label} CAN adapter', text)
        self.assertNotIn("modprobe gs_usb", text)


class RtControlLaunchAffinityContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = RT_LAUNCH.read_text(encoding="utf-8")

    def test_launch_does_not_bind_whole_ros2_processes_to_the_rt_cpu(self):
        self.assertNotIn("RT_CONTROL_ENABLE_NODE_AFFINITY", self.text)
        self.assertNotIn("RT_CONTROL_RT_CPU", self.text)
        self.assertNotIn("RT_CONTROL_HOUSEKEEPING_CPUS", self.text)
        self.assertNotIn("taskset --cpu-list", self.text)
        self.assertNotIn("prefix=", self.text)
        self.assertIn('package="control_api_adapter"', self.text)
        self.assertIn('executable="control_enable_adapter"', self.text)

    def test_start_waits_for_internal_and_public_control_services(self):
        launcher_text = LAUNCHER.read_text(encoding="utf-8")
        start = launcher_text.index("wait_for_enable_service()")
        stop = launcher_text.index("terminate_failed_start()", start)
        body = launcher_text[start:stop]
        self.assertIn("local deadline=$((SECONDS + 150))", body)
        self.assertIn("ros2 service type /rt/enable", body)
        self.assertIn("rt_control_interfaces/srv/RtEnable", body)
        self.assertIn("ros2 service type /control/set_enabled", body)
        self.assertIn("robot_rt_control_interfaces/srv/SetControlEnabled", body)
        self.assertIn("ros2 service call", body)
        self.assertIn("/controller_manager/list_controllers", body)
        self.assertIn("controller_manager_msgs/srv/ListControllers", body)
        self.assertIn("ListControllers_Response", body)

    def test_plain_start_reports_ready_only_after_runtime_initialization(self):
        launcher_text = LAUNCHER.read_text(encoding="utf-8")
        start = launcher_text.index("start_native()")
        stop = launcher_text.index("enable_native()", start)
        body = launcher_text[start:stop]
        self.assertLess(
            body.index("wait_for_enable_service"),
            body.index("verify_controllers_before_enable"),
        )
        self.assertLess(
            body.index("verify_controllers_before_enable"),
            body.index("verify_operational_ethercat"),
        )
        self.assertLess(
            body.index("verify_operational_ethercat"),
            body.index('info "READY:'),
        )

    def test_controller_state_gate_waits_for_spawners_before_enable(self):
        launcher_text = LAUNCHER.read_text(encoding="utf-8")
        start = launcher_text.index("verify_controllers_before_enable()")
        stop = launcher_text.index("verify_enabled_controllers()", start)
        body = launcher_text[start:stop]
        self.assertIn("deadline=$((SECONDS + 30))", body)
        self.assertIn("while (( SECONDS < deadline ))", body)
        self.assertIn("sleep 1", body)
        self.assertIn("whole_body_jtc", body)


class NativeOneClickRecoveryContractTest(unittest.TestCase):
    def test_oneclick_script_delegates_to_the_explicit_recovery_entrypoint(self):
        self.assertTrue(ONECLICK.exists(), "native one-click recovery script is missing")
        self.assertTrue(
            ONECLICK.stat().st_mode & 0o111,
            "native one-click recovery script must be executable",
        )
        text = ONECLICK.read_text(encoding="utf-8")
        self.assertIn('exec "${script_dir}/rt_control_native.sh" recover-power-loss "$@"', text)


class RtControlDiagnosticScriptContractTest(unittest.TestCase):
    def test_cpu_contamination_guard_is_executable_and_targets_realtime_threads(self):
        self.assertTrue(CPU_GUARD.exists(), "CPU contamination guard is missing")
        self.assertTrue(CPU_GUARD.stat().st_mode & 0o111, "CPU guard must be executable")
        text = CPU_GUARD.read_text(encoding="utf-8")
        self.assertIn("samples=10", text)
        self.assertIn("--samples", text)
        self.assertIn("sleep", text)
        self.assertIn("SCHED_FIFO", text)
        self.assertIn("SCHED_RR", text)
        self.assertIn("Cpus_allowed_list", text)
        self.assertIn("PSR", text)
        self.assertIn("tight affinity", text)
        self.assertIn("rcub/*", text)
        self.assertIn("warn_records", text)
        self.assertIn("WARN:", text)
        self.assertIn("rtprio >= 80", text)
        self.assertNotIn("LOG-ONLY:", text)

    def test_thread_affinity_helper_is_executable_and_pins_required_named_rt_threads(self):
        self.assertTrue(THREAD_AFFINITY.exists(), "thread affinity helper is missing")
        self.assertTrue(THREAD_AFFINITY.stat().st_mode & 0o111, "thread helper must be executable")
        text = THREAD_AFFINITY.read_text(encoding="utf-8")
        self.assertIn("os.sched_setaffinity", text)
        self.assertIn("SCHED_FIFO", text)
        self.assertIn("rt_priority", text)
        self.assertIn("housekeeping", text)
        self.assertIn("ros2_control_node", text)
        self.assertIn("expected exactly one matching update thread", text)
        self.assertIn("current_processor", text)
        self.assertIn("Phase 0", text)
        self.assertIn("--sample", text)
        self.assertIn("SCHED_RR", text)
        self.assertIn("--required-rt-thread-name", text)
        self.assertIn("expected exactly one matching required RT thread", text)
        self.assertIn("rtcan-master", LAUNCHER.read_text(encoding="utf-8"))

    def test_canopen_master_loop_patch_names_the_lely_thread_for_affinity_gate(self):
        self.assertTrue(
            CANOPEN_MASTER_THREAD_PATCH.exists(),
            "CANopen master thread identity patch is missing",
        )
        text = CANOPEN_MASTER_THREAD_PATCH.read_text(encoding="utf-8")
        self.assertIn("pthread_setname_np", text)
        self.assertIn("rtcan-master", text)
        self.assertIn("node_canopen_master.hpp", text)

    def test_canopen_heartbeat_watcher_is_read_only_and_tracks_master_gap(self):
        self.assertTrue(HEARTBEAT_WATCH.exists(), "CANopen heartbeat watcher is missing")
        self.assertTrue(HEARTBEAT_WATCH.stat().st_mode & 0o111, "heartbeat watcher must be executable")
        text = HEARTBEAT_WATCH.read_text(encoding="utf-8")
        self.assertIn("candump", text)
        self.assertIn("0x764", text)
        self.assertIn("0x702", text)
        self.assertIn("0x703", text)
        self.assertIn("max_gap", text)
        self.assertNotIn("cansend", text)


if __name__ == "__main__":
    unittest.main()
