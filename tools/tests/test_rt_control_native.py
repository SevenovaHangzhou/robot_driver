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
        self.assertLess(body.index("clear_resettable_faults_before_enable"), body.index("check_axis_states disabled"))
        self.assertLess(body.index("check_axis_states disabled"), body.index("call_rt_service enable"))
        self.assertLess(body.index("call_rt_service enable"), body.index("check_axis_states enabled"))
        self.assertLess(body.index("check_axis_states enabled"), body.index("monitor_native_warning_logs"))

    def test_recovery_fault_reset_reports_unclearable_fault_context(self):
        self.assertIn("clear_resettable_faults_before_enable()", self.text)
        start = self.text.index("clear_resettable_faults_before_enable()")
        stop = self.text.index("verify_controllers_before_enable()", start)
        body = self.text[start:stop]
        self.assertIn("call_rt_service reset_fault", body)
        self.assertIn("print_fault_recovery_context", body)
        self.assertIn("power-cycle the main contactor", body)

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
        self.assertIn('readonly expected_housekeeping_cpuset="0,2,4,6,8,10,12,16-27"', self.text)
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

    def test_native_pins_only_controller_update_thread_after_startup(self):
        self.assertIn('thread_affinity_tool="${repository_root}/tools/rt_control_thread_affinity.py"', self.text)
        self.assertIn("pin_controller_update_thread", self.text)
        start = self.text.index("start_native()")
        stop = self.text.index("enable_native()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("wait_for_enable_service"), body.index("pin_controller_update_thread"))
        self.assertLess(body.index("pin_controller_update_thread"), body.index("READY: native stack"))

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
        self.assertLess(body.index("clear_resettable_faults_before_enable"), body.index("check_axis_states disabled"))
        self.assertLess(body.index("check_axis_states disabled"), body.index("call_rt_service enable"))
        self.assertIn("call_rt_service enable", body)

    def test_runtime_sources_ros_setup_without_nounset(self):
        start = self.text.index("source_runtime_environment()")
        stop = self.text.index("runtime_env()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("set +u"), body.index("source /opt/ros/humble/setup.bash"))
        self.assertLess(body.index('source "${install_root}/setup.bash"'), body.index("set -u"))

    def test_symlink_install_signal_gate_source_is_executable(self):
        self.assertTrue(
            SIGNAL_GATE.stat().st_mode & 0o111,
            "rt_control_start must be executable because native symlink-install points to it",
        )


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

    def test_build_is_incremental_and_kept_outside_the_repository(self):
        self.assertIn("--symlink-install", self.text)
        self.assertIn('build_base="${workspace_root}/build"', self.text)
        self.assertIn('install_base="${workspace_root}/install"', self.text)
        self.assertIn('log_base="${workspace_root}/log"', self.text)

    def test_bootstrap_sources_ros_setup_without_nounset(self):
        start = self.text.index("source_build_environment()")
        stop = self.text.index("dependency_paths()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("set +u"), body.index("source /opt/ros/humble/setup.bash"))
        self.assertLess(body.index("source /opt/ros/humble/setup.bash"), body.index("set -u"))


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
        self.assertIn("warn_records", text)
        self.assertIn("WARN:", text)
        self.assertIn("rtprio >= 80", text)
        self.assertNotIn("LOG-ONLY:", text)

    def test_thread_affinity_helper_is_executable_and_keeps_canopen_off_rt_cpu(self):
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
