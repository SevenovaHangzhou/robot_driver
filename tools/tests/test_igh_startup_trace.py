import sys
import argparse
import contextlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import igh_startup_trace as trace


def event(time, name, tid=12, **fields):
    return trace.Event(time, tid, name, fields)


class TimingAnalysisTest(unittest.TestCase):
    def test_state_change_is_timed_at_previous_execution_return(self):
        symbols = {1: "start", 2: "sdo_conf", 3: "dc_sync_check", 4: "safeop", 5: "op", 6: "end"}
        events = [event(0, "activate", master=10), event(10, "activate_ret", result=0),
                  event(100, "cfg_begin", fsm=42, pos=2, requested=8)]
        for enter, leave, state in [(200, 220, 1), (4200, 4210, 2), (8200, 8210, 2),
                                    (12200, 12210, 3), (16100, 16120, 3),
                                    (20100, 20120, 4), (24100, 24120, 5)]:
            events.extend([event(enter, "cfg_exec", fsm=42, pos=2, state=state),
                           event(leave, "cfg_ret", result=int(state != 5))])
        events.append(event(24200, "cfg_end", fsm=42, pos=2, state=6))
        report = trace.analyze_events(events, symbols)
        row = report["slaves"][0]
        self.assertEqual(row["position"], 2)
        self.assertEqual(row["outcome"], "end")
        self.assertEqual(row["duration_ns"], 24020)
        self.assertEqual(row["phases_ns"]["sdo_conf"], 7990)
        self.assertEqual(row["phases_ns"]["dc_sync_check"], 7910)
        self.assertEqual(sum(row["phases_ns"].values()), row["duration_ns"])

    def test_offsets_preserve_width_sign_and_return_value(self):
        events = [event(0, "offset32_enter", fsm=7, pos=2, system=0xfffffff0,
                        old=42, app=0x100000010),
                  event(10, "offset32_exit", new=42),
                  event(20, "offset64_enter", fsm=7, pos=14, system=2000100,
                        old=100, app=100),
                  event(30, "offset64_exit", new=0xffffffffffE17BE4)]
        rows = trace.analyze_events(events, {})["offsets"]
        self.assertEqual(rows[0]["difference_ns"], 32)
        self.assertFalse(rows[0]["changed"])
        self.assertEqual(rows[1]["difference_ns"], -2000000)
        self.assertTrue(rows[1]["changed"])

    def test_dc_samples_decode_sign_magnitude(self):
        report = trace.analyze_events([
            event(1, "dc_sample", fsm=1, pos=14, raw=0x800003e8, dstate=3),
            event(2, "dc_sample", fsm=1, pos=14, raw=50, dstate=3)], {})
        self.assertEqual(report["dc_samples"][0]["difference_ns"], -1000)
        self.assertEqual(report["dc_samples"][1]["difference_ns"], 50)

    def test_missing_return_or_terminal_state_is_not_reported_complete(self):
        report = trace.analyze_events([
            event(0, "cfg_begin", fsm=1, pos=2, requested=8),
            event(1, "cfg_exec", fsm=1, pos=2, state=1)], {1: "dc_sync_check"})
        self.assertFalse(report["complete"])
        self.assertEqual(report["slaves"], [])
        self.assertTrue(report["errors"])

    def test_unknown_state_and_unpaired_return_fail_explicitly(self):
        for events in [[event(1, "cfg_ret", result=1)],
                       [event(0, "cfg_begin", fsm=1, pos=2, requested=8),
                        event(1, "cfg_exec", fsm=1, pos=2, state=123)]]:
            with self.subTest(events=events):
                self.assertFalse(trace.analyze_events(events, {})["complete"])

    def test_parallel_slave_durations_are_not_added_as_wall_clock_time(self):
        self.assertEqual(trace.interval_union_ns([(0, 10), (5, 15), (30, 40)]), 25)

    def test_blocking_sdo_download_is_separate_from_activation_timing(self):
        report = trace.analyze_events([
            event(100, "sdo_enter", pos=2, index=0x6060, sub=0, size=1),
            event(5100, "sdo_exit", result=0),
            event(6000, "activate", master=0),
            event(6010, "activate_ret", result=0)], {})
        self.assertEqual(report["sdo_downloads"][0]["duration_ns"], 5000)
        self.assertEqual(report["sdo_downloads"][0]["index"], 0x6060)
        self.assertEqual(report["pre_activation_sdo_ns"], 5000)

    def test_initial_offset_return_is_checked_against_the_recorded_inputs(self):
        report = trace.analyze_events([
            event(1, "offset64_enter", pos=1, fsm=1, system=0, app=2000000, old=10),
            event(2, "offset64_exit", new=99)], {})
        self.assertFalse(report["complete"])
        self.assertIn("offset", " ".join(report["errors"]))


class TraceParsingTest(unittest.TestCase):
    def test_parse_kernel_trace_with_flags_and_hex_values(self):
        events = trace.parse_trace(
            " EtherCAT-OP-123 [014] d..2. 12.123456: e97_test_cfg_exec: "
            "(ec_fsm_slave_config_exec+0x0/0x60 [ec_master]) fsm=0xffff pos=2 state=0x123\n",
            "e97_test")
        self.assertEqual(events, [event(12123456000, "cfg_exec", tid=123,
                                       fsm=65535, pos=2, state=291)])

    def test_lost_events_and_malformed_own_events_are_rejected(self):
        for text in ["CPU:14 [LOST 3 EVENTS]\n", "garbled e97_test_cfg_exec: fsm=0\n"]:
            with self.assertRaises(ValueError):
                trace.parse_trace(text, "e97_test")

    def test_other_events_and_comments_are_ignored(self):
        self.assertEqual(trace.parse_trace("# tracer: nop\n unrelated_event\n", "e97_test"), [])

    def test_dwarf_layout_requires_expected_sizes(self):
        with self.assertRaises(ValueError):
            trace.parse_layout("config_slave=48,4\n")

    def test_complete_dwarf_layout_is_decoded(self):
        text = "".join(f"{name}={index * 8},{size}\n" for index, (name, (_, _, size)) in enumerate(trace.FIELDS.items()))
        layout = trace.parse_layout(text + "received=3\n")
        self.assertEqual(layout["received"], 3)
        self.assertEqual(layout["config_slave"], 0)
        with self.assertRaisesRegex(ValueError, "Incomplete"):
            trace.parse_layout(text)

    def test_probe_offsets_come_from_verified_layout(self):
        layout = {key: index * 8 for index, key in enumerate(trace.FIELDS)}
        layout["received"] = 3
        probes = trace.probe_definitions("e97_test", layout)
        self.assertEqual(len(probes), 13)
        self.assertIn("fsm=$arg1:x64", probes["cfg_exec"])
        self.assertIn("result=$retval:s32", probes["cfg_ret"])
        self.assertNotIn("$arg1", probes["cfg_ret"])
        self.assertIn("system=$arg2:x64 old=$arg3:x64 app=$arg4:x64", probes["offset64_enter"])

    def test_al_probes_include_send_receive_ticks_and_application_stamp(self):
        layout = {key: index * 8 for index, key in enumerate(trace.FIELDS)}
        probes = trace.probe_definitions("e97_test", layout, al_details=True)
        self.assertEqual(len(probes), 18)
        for name in ["al_ack", "al_poll"]:
            for field in ["tx_jiffies=", "rx_jiffies=", "tx_app_ns=", "wc=", "dstate="]:
                self.assertIn(field, probes[name])


class AlTimingTest(unittest.TestCase):
    def test_transfer_ticks_separate_round_trip_from_slave_transition_wait(self):
        events = [event(100, "al_begin", fsm=1, pos=2, requested=8),
                  event(200, "al_ack", fsm=1, pos=2, requested=8, wc=1,
                        tx_jiffies=1000, rx_jiffies=1004),
                  event(300, "al_poll", fsm=1, pos=2, requested=8, wc=1, raw=4,
                        tx_jiffies=1008, rx_jiffies=1012),
                  event(400, "al_poll", fsm=1, pos=2, requested=8, wc=1, raw=8,
                        tx_jiffies=1280, rx_jiffies=1284),
                  event(500, "al_end", fsm=1, pos=2, requested=8, state=42)]
        result = trace.analyze_al_events(events, {42: "end"}, 1000)
        self.assertFalse(result["errors"])
        row = result["transitions"][0]
        self.assertEqual(row["write_to_ready_ms"], 284)
        self.assertEqual(row["polls"][0]["round_trip_ms"], 4)
        self.assertEqual(row["poll_tx_intervals_ms"], [272])

    def test_incomplete_or_inconsistent_transition_is_not_valid(self):
        for events in [
            [event(0, "al_begin", fsm=1, pos=2, requested=8)],
            [event(0, "al_begin", fsm=1, pos=2, requested=8),
             event(1, "al_end", fsm=1, pos=3, requested=8, state=42)],
        ]:
            self.assertTrue(trace.analyze_al_events(events, {42: "end"}, 1000)["errors"])

    def test_error_terminal_is_not_confused_with_ready(self):
        result = trace.analyze_al_events([
            event(0, "al_begin", fsm=1, pos=2, requested=8),
            event(1, "al_end", fsm=1, pos=2, requested=8, state=43)], {43: "error"}, 1000)
        self.assertFalse(result["errors"])
        self.assertEqual(result["transitions"][0]["outcome"], "error")
        self.assertIsNone(result["transitions"][0]["write_to_ready_ms"])

    def test_loss_counters_are_checked(self):
        self.assertEqual(trace.loss_errors({"cpu14": "overrun: 0\ndropped events: 0\n"},
                                           {"cfg_ret": {"hits": 3, "missed": 0}}), [])
        self.assertTrue(trace.loss_errors({"cpu14": "overrun: 2\n"}, {}))
        self.assertTrue(trace.loss_errors({}, {"cfg_ret": {"hits": 3, "missed": 1}}))


class TraceCleanupTest(unittest.TestCase):
    def test_release_only_capture_does_not_require_a_dc_filter(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "instances").mkdir()
            with mock.patch.object(trace, "write_control") as writer:
                with trace.TraceSession(root, "e97_release") as session:
                    clock_file = session.instance / "trace_clock"
                    clock_file.write_text("mono")
                    session.arm({"release": "definition"}, received=3)
                    self.assertFalse(any(call.args[0].name == "filter" for call in writer.call_args_list))
                    clock_file.unlink()

    def test_partial_registration_cleans_up_only_owned_probes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "instances").mkdir()
            writes = []

            def write(path, value, append=False):
                writes.append((path.name, value, append))
                if value == "bad probe":
                    raise OSError("probe rejected")

            with mock.patch.object(trace, "write_control", side_effect=write):
                with self.assertRaises(OSError):
                    with trace.TraceSession(root, "e97_test") as session:
                        session.arm({"one": "first probe", "two": "bad probe"}, received=3)
            self.assertIn(("kprobe_events", "-:e97_test/e97_test_one", True), writes)
            self.assertNotIn(("kprobe_events", "-:e97_test/e97_test_two", True), writes)
            self.assertFalse((root / "instances/e97_test").exists())

    def test_non_owned_group_names_are_rejected(self):
        with self.assertRaises(ValueError):
            trace.TraceSession(Path("/unused"), "kprobes")

    def test_control_write_never_seeks_in_append_mode(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "control"
            path.touch()
            with mock.patch.object(trace.os, "lseek", side_effect=AssertionError("seek forbidden")):
                trace.write_control(path, "first", append=True)
                trace.write_control(path, "second", append=True)
                self.assertEqual(path.read_text(), "first\nsecond\n")
                trace.write_control(path, "new")
                self.assertEqual(path.read_text(), "new\n")
            with mock.patch.object(trace.os, "write", return_value=0):
                with self.assertRaisesRegex(OSError, "Short"):
                    trace.write_control(path, "bad")

    def test_arm_and_cleanup_keep_all_controls_inside_the_private_instance(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "instances").mkdir()
            session = trace.TraceSession(root, "e97_test")
            session.__enter__()
            (session.instance / "trace_clock").write_text("[local] mono\n")
            with mock.patch.object(trace, "write_control") as writer:
                session.arm({"dc_sample": "definition"}, received=3)
                session.freeze()
                self.assertIn(mock.call(session.instance / "trace_clock", "mono"), writer.call_args_list)
                with mock.patch.object(Path, "rmdir"):
                    session.__exit__()
                self.assertIn(mock.call(root / "kprobe_events", "-:e97_test/e97_test_dc_sample", append=True), writer.call_args_list)
            with self.assertRaises(FileExistsError):
                trace.TraceSession(root, "e97_test").__enter__()

    def test_cleanup_failure_is_reported(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "instances").mkdir()
            session = trace.TraceSession(root, "e97_test").__enter__()
            (session.instance / "tracing_on").write_text("1")
            session.registered = ["one"]
            with mock.patch.object(trace, "write_control", side_effect=OSError("denied")):
                with self.assertRaisesRegex(RuntimeError, "cleanup incomplete"):
                    session.__exit__()

    def test_loss_statistics_include_probe_return_misses(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            stats = root / "instances/e97_test/per_cpu/cpu14/stats"
            stats.parent.mkdir(parents=True)
            stats.write_text("overrun: 0\n")
            (root / "kprobe_profile").write_text("other 1 0\ne97_test_cfg_ret 5 2\n")
            cpu, profiles = trace.TraceSession(root, "e97_test").statistics()
            self.assertEqual(profiles["e97_test_cfg_ret"], {"hits": 5, "missed": 2})
            self.assertTrue(trace.loss_errors(cpu, profiles))


class CaptureWorkflowTest(unittest.TestCase):
    def test_module_identity_and_symbols_are_checked(self):
        layout = "".join(f"{name}={index * 8},{size}\n" for index, (name, (_, _, size)) in enumerate(trace.FIELDS.items())) + "received=3\n"
        symbols = "\n".join(f"{i + 1:016x} t {trace.STATE_PREFIX}{name} [ec_master]"
                            for i, name in enumerate(["start", "dc_sync_check", "op", "end", "error"]))
        with mock.patch.object(Path, "read_bytes", return_value=b"note"), \
                mock.patch.object(Path, "read_text", return_value=symbols), \
                mock.patch.object(trace, "run", side_effect=[b"note", layout.encode()]):
            metadata = trace.inspect_module(Path("/module"))
            self.assertEqual(metadata["layout"]["received"], 3)
            self.assertEqual(len(metadata["symbols"]), 5)
        with mock.patch.object(Path, "read_bytes", return_value=b"note"), \
                mock.patch.object(trace, "run", return_value=b"different"):
            with self.assertRaisesRegex(ValueError, "Build ID"):
                trace.inspect_module(Path("/module"))

    def capture_case(self, broken=False):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "capture"
            args = argparse.Namespace(module=Path("/module"), output=output,
                                      tracefs=Path("/tracefs"), seconds=1)
            metadata = {"layout": {name: i * 8 for i, name in enumerate(trace.FIELDS)}, "symbols": {1: "start"}}
            metadata["layout"]["received"] = 3
            session = mock.MagicMock()
            session.instance = Path("/private-trace")
            session.statistics.return_value = ({"cpu14": "overrun: 0\n"},
                                               {str(i): {"hits": 0, "missed": 0} for i in range(13)})
            with contextlib.ExitStack() as stack:
                stack.enter_context(mock.patch.object(trace.os, "geteuid", return_value=0))
                stack.enter_context(mock.patch.object(trace.platform, "machine", return_value="x86_64"))
                stack.enter_context(mock.patch.object(trace, "run", side_effect=[b"Master0\nActive: no\n", b"slaves"]))
                stack.enter_context(mock.patch.object(trace, "inspect_module", return_value=metadata))
                factory = stack.enter_context(mock.patch.object(trace, "TraceSession"))
                factory.return_value.__enter__.return_value = session
                stack.enter_context(mock.patch.object(Path, "read_text", return_value="format"))
                stack.enter_context(mock.patch.object(trace.os, "open", return_value=10))
                stack.enter_context(mock.patch.object(trace.os, "close"))
                stack.enter_context(mock.patch.object(trace.os, "chown"))
                stack.enter_context(mock.patch.object(trace.signal, "signal"))
                stack.enter_context(mock.patch.object(trace.time, "monotonic", side_effect=[0, 0, 2]))
                stack.enter_context(mock.patch.object(trace.select, "select", return_value=([10], [], [])))
                stack.enter_context(mock.patch("builtins.print"))
                reader = stack.enter_context(mock.patch.object(trace.os, "read", side_effect=[b"sample\n", BlockingIOError()]))
                if broken:
                    reader.side_effect = OSError("capture failed")
                    with self.assertRaisesRegex(OSError, "capture failed"):
                        trace.capture(args)
                else:
                    self.assertEqual(trace.capture(args), 0)
                    session.freeze.assert_called_once()
                factory.return_value.__exit__.assert_called_once()
            saved = json.loads((output / "metadata.json").read_text())
            self.assertEqual(bool(saved["capture_error"]), broken)

    def test_capture_drains_and_records_loss_counters(self):
        self.capture_case()

    def test_capture_errors_preserve_metadata_and_close_the_session(self):
        self.capture_case(broken=True)

    def test_capture_rejects_invalid_environment_before_tracing(self):
        args = argparse.Namespace(seconds=0)
        with mock.patch.object(trace.os, "geteuid", return_value=1):
            with self.assertRaisesRegex(ValueError, "requires root"):
                trace.capture(args)
        with mock.patch.object(trace.os, "geteuid", return_value=0), \
                mock.patch.object(trace.platform, "machine", return_value="x86_64"):
            with self.assertRaisesRegex(ValueError, "duration"):
                trace.capture(args)
            args.seconds = 1
            with mock.patch.object(trace, "run", return_value=b"Master0\nActive: yes\n"):
                with self.assertRaisesRegex(ValueError, "inactive"):
                    trace.capture(args)

    def test_analyze_exports_data_and_preserves_capture_failures(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "metadata.json").write_text(json.dumps({"symbols": {}, "group": "e97_test", "capture_error": "lost"}))
            (root / "trace.txt").write_text("")
            with mock.patch("builtins.print"):
                self.assertEqual(trace.analyze(argparse.Namespace(capture=root)), 1)
            self.assertTrue((root / "stages.csv").is_file())
            self.assertIn("lost", json.loads((root / "analysis.json").read_text())["errors"])
