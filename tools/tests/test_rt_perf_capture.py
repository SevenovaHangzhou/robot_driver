"""ELECTRI-80 T2 性能采集脚本测试（TDD：先测后实现）。

固件文本取自既有实测记录，不臆造格式：
- pidstat/mpstat/vmstat 口径见 domains/rt_control/docs/fjt-14axis-low-speed-commissioning-20260727.md
- cyclictest 行取自 domains/rt_control/docs/host-setup-record.md（已退役 alfa-two 主机）
"""
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import rt_perf_capture as rpc

BASELINE_V0 = ROOT / "domains/rt_control/testing/baselines/v0.yaml"

PIDSTAT_CPU_TEXT = """Linux 5.15.0-1032-realtime (ar-Default-string) \t08/13/2026 \t_x86_64_\t(20 CPU)

09:18:44      UID       PID    %usr %system  %guest   %wait    %CPU   CPU  Command
09:18:45     1000      4321    2.00    1.00    0.00    0.00    3.00    14  ros2_control_no
09:18:46     1000      4321    2.50    1.50    0.00    0.00    4.00    14  ros2_control_no
09:18:47     1000      4321    3.00    0.49    0.00    0.00    3.49    14  ros2_control_no
Average:     1000      4321    2.50    1.00    0.00    0.00    3.50    14  ros2_control_no
"""

PIDSTAT_RSS_TEXT = """Linux 5.15.0-1032-realtime (ar-Default-string) \t08/13/2026 \t_x86_64_\t(20 CPU)

09:18:44      UID       PID  minflt/s  majflt/s     VSZ     RSS   %MEM  Command
09:18:45     1000      4321      0.00      0.00 2317412  591157   1.81  ros2_control_no
09:18:46     1000      4321      0.00      0.00 2317412  591157   1.81  ros2_control_no
Average:     1000      4321      0.00      0.00 2317412  591157   1.81  ros2_control_no
"""

MPSTAT_TEXT = """Linux 5.15.0-1032-realtime (ar-Default-string) \t08/13/2026 \t_x86_64_\t(20 CPU)

09:18:44     CPU    %usr   %nice    %sys %iowait    %irq   %soft  %steal  %guest  %gnice   %idle
09:18:45      14    3.37    0.00    1.00    0.00    0.00    0.00    0.00    0.00    0.00   95.63
09:18:46      14   20.20    0.00    3.03    0.00    0.00    0.00    0.00    0.00    0.00   76.77
Average:      14   11.79    0.00    2.02    0.00    0.00    0.00    0.00    0.00    0.00   95.055
"""

VMSTAT_TEXT = """procs -----------memory---------- ---swap-- -----io---- -system-- ------cpu-----
 r  b   swpd   free   buff  cache   si   so    bi    bo   in   cs us sy id wa st
 6  0      0 6899364  46756 6099564    0    0    96    14   36   46  0  0 100  0  0
 3  0      0 6899136  46756 6099576    0    0     0     0  390  406  1  0 99  0  0
"""

FREE_TEXT = """               total        used        free      shared  buff/cache   available
Mem:           15704        2964        6737           4        6002       12494
Swap:           4096           0        4096
"""

PROC_STAT_TEXT = """cpu  1 2 3 4 5 6 7 8 9 10
intr 12345
ctxt 987654
btime 1755100000
processes 44221
procs_running 2
procs_blocked 0
"""

IP_CAN_TEXT = """3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0
    can <TRIPLE-SAMPLING> state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0
\t  bitrate 500000 sample-point 0.875
\t  re-started bus-errors arbit-lost error-warn error-pass bus-off
\t  0          0          0          0          0          0
    numtxqueues 1 numrxqueues 1 gso_max_size 65536 gso_max_segs 65535
    RX:  bytes packets errors dropped  missed   mcast
         22551     216      0       0       0       0
    TX:  bytes packets errors dropped carrier collsns
         22551     216      0       0       0       0
4: can1: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0
    can <TRIPLE-SAMPLING> state ERROR-ACTIVE (berr-counter tx 0 rx 1) restart-ms 0
\t  re-started bus-errors arbit-lost error-warn error-pass bus-off
\t  0          2          0          1          0          0
    RX:  bytes packets errors dropped  missed   mcast
         1000      50       3       0       0       0
    TX:  bytes packets errors dropped carrier collsns
         900       45       0       0       0       0
"""

CYCLICTEST_LOG = """# /dev/cpu_dma_latency set to 0us
policy: fifo: loadavg: 0.51 0.32 0.19 2/1453 30412

T: 0 (12700) P:90 I:1000 C:1800000 Min:      1 Avg:      4 Max:      18 us
"""

DIAGNOSTICS_BEFORE = """  values:
  - key: link_up
    value: 'true'
  - key: wc_error_count
    value: '705'
"""

DIAGNOSTICS_AFTER = """  values:
  - key: link_up
    value: 'true'
  - key: wc_error_count
    value: '712'
"""


def _write(text: str, suffix: str = ".txt") -> str:
    handle = tempfile.NamedTemporaryFile("w", suffix=suffix, delete=False, encoding="utf-8")
    handle.write(text)
    handle.close()
    return handle.name


class PidstatParserTest(unittest.TestCase):
    def test_parses_process_cpu_average_and_peak_from_samples(self):
        # Arrange / Act
        result = rpc.parse_pidstat_cpu(PIDSTAT_CPU_TEXT)

        # Assert
        self.assertEqual(result["samples"], 3)
        self.assertAlmostEqual(result["avg_percent"], 3.5, places=3)
        self.assertAlmostEqual(result["peak_percent"], 4.0, places=3)
        self.assertEqual(result["pid"], 4321)

    def test_computes_cpu_average_from_samples_when_average_row_absent(self):
        text = "\n".join(
            line for line in PIDSTAT_CPU_TEXT.splitlines() if not line.startswith("Average:")
        )
        result = rpc.parse_pidstat_cpu(text + "\n")
        self.assertAlmostEqual(result["avg_percent"], (3.00 + 4.00 + 3.49) / 3, places=3)

    def test_parses_rss_kib_and_converts_to_mib(self):
        result = rpc.parse_pidstat_rss(PIDSTAT_RSS_TEXT)
        self.assertEqual(result["rss_kib_peak"], 591157.0)
        self.assertAlmostEqual(result["rss_mib_peak"], 577.30, places=2)

    def test_raises_when_pidstat_text_has_no_samples(self):
        with self.assertRaises(rpc.ParseError):
            rpc.parse_pidstat_cpu("Linux 5.15.0 (host)\n\nno samples here\n")

    def test_rss_parser_raises_on_text_without_rss_column(self):
        with self.assertRaises(rpc.ParseError):
            rpc.parse_pidstat_rss(PIDSTAT_CPU_TEXT)


class MpstatParserTest(unittest.TestCase):
    def test_derives_cpu_busy_average_and_peak_from_idle_column(self):
        result = rpc.parse_mpstat_cpu_busy(MPSTAT_TEXT, cpu=14)
        self.assertAlmostEqual(result["avg_percent"], 4.945, places=3)
        self.assertAlmostEqual(result["peak_percent"], 23.230, places=3)
        self.assertEqual(result["samples"], 2)

    def test_raises_when_requested_cpu_is_absent(self):
        with self.assertRaises(rpc.ParseError):
            rpc.parse_mpstat_cpu_busy(MPSTAT_TEXT, cpu=3)


class VmstatAndMemoryParserTest(unittest.TestCase):
    def test_reports_peak_blocked_and_swap_activity(self):
        result = rpc.parse_vmstat(VMSTAT_TEXT)
        self.assertEqual(result["blocked_processes"], 0)
        self.assertEqual(result["runnable_peak"], 6)
        self.assertEqual(result["swap_in"], 0)
        self.assertEqual(result["swap_out"], 0)

    def test_reports_nonzero_blocked_and_swap_when_present(self):
        text = VMSTAT_TEXT.replace(
            " 3  0      0 6899136  46756 6099576    0    0",
            " 3  2      0 6899136  46756 6099576    5    7",
        )
        result = rpc.parse_vmstat(text)
        self.assertEqual(result["blocked_processes"], 2)
        self.assertEqual(result["swap_in"], 5)
        self.assertEqual(result["swap_out"], 7)

    def test_vmstat_parser_raises_without_data_rows(self):
        with self.assertRaises(rpc.ParseError):
            rpc.parse_vmstat("procs -----------memory----------\n r  b   swpd   free\n")

    def test_parses_free_swap_usage_in_mib(self):
        result = rpc.parse_free(FREE_TEXT)
        self.assertEqual(result["swap_total_mib"], 4096.0)
        self.assertEqual(result["swap_used_mib"], 0.0)
        self.assertEqual(result["mem_available_mib"], 12494.0)

    def test_free_parser_raises_without_swap_row(self):
        with self.assertRaises(rpc.ParseError):
            rpc.parse_free("               total        used        free\nMem: 1 2 3\n")

    def test_parses_procs_blocked_from_proc_stat(self):
        self.assertEqual(rpc.parse_proc_stat(PROC_STAT_TEXT)["blocked_processes"], 0)

    def test_proc_stat_parser_raises_when_field_missing(self):
        with self.assertRaises(rpc.ParseError):
            rpc.parse_proc_stat("cpu 1 2 3\nprocs_running 2\n")


class CanErrorParserTest(unittest.TestCase):
    def test_parses_per_interface_can_error_counters(self):
        stats = rpc.parse_ip_link_stats(IP_CAN_TEXT)

        self.assertEqual(sorted(stats), ["can0", "can1"])
        self.assertEqual(stats["can0"]["state"], "ERROR-ACTIVE")
        self.assertEqual(stats["can0"]["berr_tx"], 0)
        self.assertEqual(stats["can0"]["rx_errors"], 0)
        self.assertEqual(stats["can1"]["berr_rx"], 1)
        self.assertEqual(stats["can1"]["bus_errors"], 2)
        self.assertEqual(stats["can1"]["error_warn"], 1)
        self.assertEqual(stats["can1"]["rx_errors"], 3)
        self.assertEqual(stats["can1"]["tx_errors"], 0)

    def test_sums_all_error_counters_per_interface(self):
        stats = rpc.parse_ip_link_stats(IP_CAN_TEXT)
        self.assertEqual(rpc.can_error_total(stats["can0"]), 0)
        self.assertEqual(rpc.can_error_total(stats["can1"]), 7)

    def test_can_error_delta_is_second_reading_minus_first(self):
        clean = rpc.parse_ip_link_stats(IP_CAN_TEXT.split("4: can1:")[0])
        dirty = rpc.parse_ip_link_stats(IP_CAN_TEXT.replace("4: can1:", "4: can0x:"))

        delta = rpc.diff_can_errors(clean, dirty, interfaces=["can0"])

        self.assertEqual(delta["can_error_delta"], 0)
        self.assertEqual(delta["per_interface"]["can0"], 0)

    def test_can_error_delta_counts_growth_and_ignores_unknown_interfaces(self):
        first = rpc.parse_ip_link_stats(IP_CAN_TEXT)
        second = rpc.parse_ip_link_stats(IP_CAN_TEXT.replace("         1000      50       3", "         1000      50       9"))

        delta = rpc.diff_can_errors(first, second, interfaces=["can0", "can1", "can9"])

        self.assertEqual(delta["can_error_delta"], 6)
        self.assertEqual(delta["missing"], ["can9"])

    def test_ip_parser_raises_when_no_interface_block_found(self):
        with self.assertRaises(rpc.ParseError):
            rpc.parse_ip_link_stats("no interfaces here\n")


class CyclictestParserTest(unittest.TestCase):
    def test_parses_historical_alfa_two_summary_line(self):
        result = rpc.parse_cyclictest("T: 0 (12700) P:90 I:1000 C:1800000 Min: 1 Avg: 4 Max: 18 us\n")

        self.assertEqual(result["cyclictest_min_us"], 1.0)
        self.assertEqual(result["cyclictest_avg_us"], 4.0)
        self.assertEqual(result["cyclictest_max_us"], 18.0)
        self.assertEqual(result["cycles"], 1800000)
        self.assertEqual(result["priority"], 90)
        self.assertEqual(result["interval_us"], 1000)

    def test_parses_padded_summary_line_from_full_log(self):
        result = rpc.parse_cyclictest(CYCLICTEST_LOG)
        self.assertEqual(result["cyclictest_max_us"], 18.0)
        self.assertEqual(result["threads"], 1)

    def test_uses_worst_thread_when_multiple_threads_reported(self):
        text = (
            "T: 0 (100) P:90 I:1000 C:  1000 Min:      1 Avg:      4 Max:      18 us\n"
            "T: 1 (101) P:90 I:1000 C:  1000 Min:      2 Avg:      5 Max:      37 us\n"
        )
        result = rpc.parse_cyclictest(text)
        self.assertEqual(result["cyclictest_max_us"], 37.0)
        self.assertEqual(result["cyclictest_min_us"], 1.0)
        self.assertEqual(result["threads"], 2)

    def test_verdict_fails_above_bq064_limit_and_passes_at_or_below(self):
        self.assertEqual(rpc.cyclictest_verdict(18.0), "PASS")
        self.assertEqual(rpc.cyclictest_verdict(100.0), "PASS")
        self.assertEqual(rpc.cyclictest_verdict(100.1), "FAIL")

    def test_raises_when_summary_line_absent(self):
        with self.assertRaises(rpc.ParseError):
            rpc.parse_cyclictest("# /dev/cpu_dma_latency set to 0us\nno summary\n")


class WcErrorDiffTest(unittest.TestCase):
    def test_parses_wc_error_count_from_diagnostics_key_value_dump(self):
        self.assertEqual(rpc.parse_wc_error_count(DIAGNOSTICS_BEFORE), 705.0)

    def test_parses_wc_error_count_from_inline_mapping_form(self):
        self.assertEqual(rpc.parse_wc_error_count("ethercat_master/wc_error_count: 8\n"), 8.0)

    def test_wc_error_delta_is_window_growth(self):
        delta = rpc.diff_wc_error(DIAGNOSTICS_BEFORE, DIAGNOSTICS_AFTER)
        self.assertEqual(delta["wc_error_delta"], 7.0)
        self.assertEqual(delta["wc_error_first"], 705.0)
        self.assertEqual(delta["wc_error_last"], 712.0)

    def test_raises_when_wc_error_count_missing(self):
        with self.assertRaises(rpc.ParseError):
            rpc.parse_wc_error_count("  values:\n  - key: link_up\n    value: 'true'\n")


class MetricsDocumentTest(unittest.TestCase):
    def test_metric_keys_match_registered_baseline_v0_names(self):
        baseline = yaml.safe_load(BASELINE_V0.read_text(encoding="utf-8"))

        document = rpc.build_metrics_document(
            cpu_busy=rpc.parse_mpstat_cpu_busy(MPSTAT_TEXT, cpu=14),
            process=rpc.parse_pidstat_rss(PIDSTAT_RSS_TEXT),
            vm=rpc.parse_vmstat(VMSTAT_TEXT),
            window_s=180,
            cpu=14,
        )

        for key in baseline["metrics"]:
            self.assertIn(key, document["metrics"], key)
        self.assertEqual(document["schema"], "rt-control-candidate/v1")
        self.assertAlmostEqual(document["metrics"]["cpu14_busy_avg_percent"], 4.945, places=3)
        self.assertAlmostEqual(document["metrics"]["ros2_control_node_rss_mib"], 577.30, places=2)
        self.assertEqual(document["window_s"], 180)

    def test_optional_bus_and_cyclictest_metrics_are_merged_when_supplied(self):
        document = rpc.build_metrics_document(
            cpu_busy=rpc.parse_mpstat_cpu_busy(MPSTAT_TEXT, cpu=14),
            process=rpc.parse_pidstat_rss(PIDSTAT_RSS_TEXT),
            vm=rpc.parse_vmstat(VMSTAT_TEXT),
            wc=rpc.diff_wc_error(DIAGNOSTICS_BEFORE, DIAGNOSTICS_AFTER),
            can=rpc.diff_can_errors(
                rpc.parse_ip_link_stats(IP_CAN_TEXT),
                rpc.parse_ip_link_stats(IP_CAN_TEXT),
                interfaces=["can0", "can1"],
            ),
            cyclictest=rpc.parse_cyclictest(CYCLICTEST_LOG),
            window_s=180,
            cpu=14,
        )

        metrics = document["metrics"]
        self.assertEqual(metrics["wc_error_delta"], 7.0)
        self.assertEqual(metrics["can_error_delta"], 0)
        self.assertEqual(metrics["cyclictest_max_us"], 18.0)
        self.assertEqual(metrics["cyclictest_min_us"], 1.0)
        self.assertEqual(metrics["cyclictest_avg_us"], 4.0)

    def test_metrics_omit_unsupplied_optional_groups_and_record_unverified(self):
        document = rpc.build_metrics_document(
            cpu_busy=rpc.parse_mpstat_cpu_busy(MPSTAT_TEXT, cpu=14),
            process=rpc.parse_pidstat_rss(PIDSTAT_RSS_TEXT),
            vm=rpc.parse_vmstat(VMSTAT_TEXT),
            window_s=180,
            cpu=14,
        )

        self.assertNotIn("cyclictest_max_us", document["metrics"])
        self.assertNotIn("wc_error_delta", document["metrics"])
        self.assertTrue(any("cyclictest" in note for note in document["unverified"]))
        self.assertTrue(any("wc_error" in note for note in document["unverified"]))

    def test_document_is_consumable_by_release_test_runner_delta(self):
        import release_test_runner as rtr

        baseline = yaml.safe_load(BASELINE_V0.read_text(encoding="utf-8"))
        thresholds = yaml.safe_load(
            (ROOT / "domains/rt_control/testing/thresholds.yaml").read_text(encoding="utf-8")
        )
        document = rpc.build_metrics_document(
            cpu_busy=rpc.parse_mpstat_cpu_busy(MPSTAT_TEXT, cpu=14),
            process=rpc.parse_pidstat_rss(PIDSTAT_RSS_TEXT),
            vm=rpc.parse_vmstat(VMSTAT_TEXT),
            wc=rpc.diff_wc_error(DIAGNOSTICS_BEFORE, DIAGNOSTICS_BEFORE),
            window_s=180,
            cpu=14,
        )

        rows = rtr.compute_delta(baseline, document, thresholds=thresholds)

        by_metric = {row["metric"]: row for row in rows}
        self.assertEqual(by_metric["cpu14_busy_avg_percent"]["verdict"], "OK")
        self.assertEqual(by_metric["ros2_control_node_rss_mib"]["verdict"], "OK")
        self.assertEqual(by_metric["wc_error_delta"]["verdict"], "OK")
        self.assertNotIn("MISSING", [row["verdict"] for row in rows])


class DockerCommandTest(unittest.TestCase):
    def test_emits_runbook_conformant_diagnostic_container_command(self):
        command = rpc.cyclictest_docker_command(duration_s=1800, cpu=14, image="rt-control:frozen")

        self.assertIn("--read-only", command)
        self.assertIn("/usr/bin/cyclictest:/usr/bin/cyclictest:ro", command)
        self.assertIn("/dev/cpu_dma_latency", command)
        self.assertIn("--priority 90", command)
        self.assertIn("--interval 1000", command)
        self.assertIn("--mlockall", command)
        self.assertIn("--duration 1800", command)
        self.assertIn("--affinity 14", command)
        self.assertIn("rt-control:frozen", command)
        self.assertNotIn("docker run -d", command)


class CaptureOrchestrationTest(unittest.TestCase):
    def _runner(self):
        def runner(argv):
            joined = " ".join(argv)
            if argv[0] == "mpstat":
                return MPSTAT_TEXT
            if argv[0] == "pidstat" and "-r" in argv:
                return PIDSTAT_RSS_TEXT
            if argv[0] == "pidstat":
                return PIDSTAT_CPU_TEXT
            if argv[0] == "vmstat":
                return VMSTAT_TEXT
            if argv[0] == "free":
                return FREE_TEXT
            if argv[0] == "ip":
                return IP_CAN_TEXT
            raise AssertionError(f"unexpected command: {joined}")

        return runner

    def test_capture_collects_all_window_metrics_through_injected_runner(self):
        document = rpc.capture(window_s=180, cpu=14, pid=4321, runner=self._runner())

        self.assertAlmostEqual(document["metrics"]["cpu14_busy_peak_percent"], 23.230, places=3)
        self.assertEqual(document["metrics"]["blocked_processes"], 0)
        self.assertEqual(document["metrics"]["can_error_delta"], 0)
        self.assertEqual(document["pid"], 4321)

    def test_capture_resolves_pid_by_process_name_when_pid_not_given(self):
        calls = []
        base = self._runner()

        def runner(argv):
            calls.append(argv[0])
            if argv[0] == "pgrep":
                return "4321\n"
            return base(argv)

        document = rpc.capture(window_s=5, cpu=14, process="ros2_control_node", runner=runner)

        self.assertIn("pgrep", calls)
        self.assertEqual(document["pid"], 4321)

    def test_capture_raises_when_process_cannot_be_resolved(self):
        def runner(argv):
            if argv[0] == "pgrep":
                return "\n"
            raise AssertionError("must not sample without a pid")

        with self.assertRaises(rpc.CaptureError):
            rpc.capture(window_s=5, cpu=14, process="ros2_control_node", runner=runner)

    def test_capture_skips_can_sampling_when_no_interfaces_requested(self):
        document = rpc.capture(
            window_s=5, cpu=14, pid=4321, can_interfaces=[], runner=self._runner()
        )
        self.assertNotIn("can_error_delta", document["metrics"])


class WcErrorMergeCliTest(unittest.TestCase):
    """窗口结束后才拿到第二次 /diagnostics 读数，必须能事后合并进 metrics.yaml。"""

    def test_wc_error_subcommand_merges_delta_into_existing_capture(self):
        capture_doc = rpc.capture(
            window_s=5, cpu=14, pid=4321, runner=CaptureOrchestrationTest()._runner()
        )
        self.assertNotIn("wc_error_delta", capture_doc["metrics"])
        out = _write(yaml.safe_dump(capture_doc), suffix=".yaml")
        first = _write(DIAGNOSTICS_BEFORE)
        last = _write(DIAGNOSTICS_AFTER)

        code = rpc.main(
            ["wc-error", "--first", first, "--last", last, "--merge-into", out, "--out", out]
        )

        merged = yaml.safe_load(Path(out).read_text(encoding="utf-8"))
        self.assertEqual(code, 0)
        self.assertEqual(merged["metrics"]["wc_error_delta"], 7.0)
        self.assertEqual(merged["metrics"]["cpu14_busy_peak_percent"], capture_doc["metrics"]["cpu14_busy_peak_percent"])
        self.assertFalse([note for note in merged["unverified"] if "wc_error" in note])

    def test_wc_error_subcommand_emits_standalone_document_without_merge(self):
        first = _write(DIAGNOSTICS_BEFORE)
        last = _write(DIAGNOSTICS_BEFORE)
        out = _write("", suffix=".yaml")

        code = rpc.main(["wc-error", "--first", first, "--last", last, "--out", out])

        document = yaml.safe_load(Path(out).read_text(encoding="utf-8"))
        self.assertEqual(code, 0)
        self.assertEqual(document["metrics"]["wc_error_delta"], 0.0)

    def test_wc_error_subcommand_reports_parse_failure_as_exit_code_two(self):
        first = _write("no counter here\n")
        last = _write(DIAGNOSTICS_AFTER)
        self.assertEqual(rpc.main(["wc-error", "--first", first, "--last", last]), 2)


class CliTest(unittest.TestCase):
    def test_parse_subcommand_prints_named_parser_result(self):
        path = _write(CYCLICTEST_LOG)
        code = rpc.main(["parse", "--kind", "cyclictest", "--file", path])
        self.assertEqual(code, 0)

    def test_parse_subcommand_reports_parse_failure_as_exit_code_two(self):
        path = _write("nothing useful\n")
        self.assertEqual(rpc.main(["parse", "--kind", "vmstat", "--file", path]), 2)

    def test_parse_subcommand_supports_every_registered_kind(self):
        cases = {
            "pidstat-cpu": PIDSTAT_CPU_TEXT,
            "pidstat-rss": PIDSTAT_RSS_TEXT,
            "mpstat": MPSTAT_TEXT,
            "vmstat": VMSTAT_TEXT,
            "free": FREE_TEXT,
            "proc-stat": PROC_STAT_TEXT,
            "ip-link": IP_CAN_TEXT,
            "cyclictest": CYCLICTEST_LOG,
            "wc-error": DIAGNOSTICS_BEFORE,
        }
        for kind, text in cases.items():
            path = _write(text)
            self.assertEqual(rpc.main(["parse", "--kind", kind, "--file", path]), 0, kind)

    def test_cyclictest_subcommand_prints_command_when_no_log_given(self):
        code = rpc.main(["cyclictest", "--duration", "1800", "--print-command"])
        self.assertEqual(code, 0)

    def test_cyclictest_subcommand_writes_metrics_from_log(self):
        log = _write(CYCLICTEST_LOG)
        out = _write("", suffix=".yaml")
        code = rpc.main(["cyclictest", "--duration", "1800", "--from-log", log, "--out", out])
        document = yaml.safe_load(Path(out).read_text(encoding="utf-8"))
        self.assertEqual(code, 0)
        self.assertEqual(document["metrics"]["cyclictest_max_us"], 18.0)
        self.assertEqual(document["verdict"], "PASS")

    def test_cyclictest_subcommand_exits_nonzero_when_latency_breaches_bq064(self):
        log = _write("T: 0 (14700) P:90 I:1000 C:1800000 Min: 2 Avg: 9 Max: 141 us\n")
        out = _write("", suffix=".yaml")
        code = rpc.main(["cyclictest", "--from-log", log, "--out", out])
        self.assertEqual(code, 1)
        self.assertEqual(yaml.safe_load(Path(out).read_text(encoding="utf-8"))["verdict"], "FAIL")

    def test_cyclictest_subcommand_merges_into_existing_capture_metrics(self):
        capture_doc = rpc.capture(window_s=5, cpu=14, pid=4321, runner=CaptureOrchestrationTest()._runner())
        out = _write(yaml.safe_dump(capture_doc), suffix=".yaml")
        log = _write(CYCLICTEST_LOG)

        code = rpc.main(["cyclictest", "--from-log", log, "--merge-into", out, "--out", out])

        merged = yaml.safe_load(Path(out).read_text(encoding="utf-8"))
        self.assertEqual(code, 0)
        self.assertEqual(merged["metrics"]["cyclictest_max_us"], 18.0)
        self.assertEqual(merged["metrics"]["blocked_processes"], 0)

    def test_capture_subcommand_dry_run_prints_plan_without_sampling(self):
        self.assertEqual(rpc.main(["capture", "--window", "180", "--dry-run"]), 0)

    def test_capture_subcommand_writes_yaml_document(self):
        out = _write("", suffix=".yaml")
        code = rpc.main(
            ["capture", "--window", "180", "--out", out, "--pid", "4321", "--no-can"],
            runner=CaptureOrchestrationTest()._runner(),
        )
        document = yaml.safe_load(Path(out).read_text(encoding="utf-8"))
        self.assertEqual(code, 0)
        self.assertIn("cpu14_busy_avg_percent", document["metrics"])

    def test_capture_subcommand_reports_capture_error_as_exit_code_two(self):
        def runner(argv):
            raise rpc.CaptureError("pidstat missing")

        self.assertEqual(
            rpc.main(["capture", "--window", "5", "--pid", "4321"], runner=runner), 2
        )

    def test_default_runner_executes_subprocess_and_returns_stdout(self):
        self.assertIn("hello", rpc.run_command(["echo", "hello"]))

    def test_default_runner_raises_capture_error_for_missing_binary(self):
        with self.assertRaises(rpc.CaptureError):
            rpc.run_command(["definitely-not-a-real-binary-xyz"])

    def test_default_runner_raises_capture_error_on_nonzero_exit(self):
        with self.assertRaises(rpc.CaptureError):
            rpc.run_command(["false"])


if __name__ == "__main__":
    unittest.main()
