#!/usr/bin/env python3
"""Capture a single-master IgH startup with isolated, removable kprobes."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import re
import select
import signal
import subprocess
import time
import uuid
from collections import defaultdict
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path


FIELDS = {
    "config_slave": ("ec_fsm_slave_config_t", "slave", 8),
    "config_state": ("ec_fsm_slave_config_t", "state", 8),
    "config_datagram": ("ec_fsm_slave_config_t", "datagram", 8),
    "slave_position": ("ec_slave_t", "ring_position", 2),
    "slave_requested": ("ec_slave_t", "requested_state", 4),
    "master_fsm_slave": ("ec_fsm_master_t", "slave", 8),
    "datagram_data": ("ec_datagram_t", "data", 8),
    "datagram_state": ("ec_datagram_t", "state", 4),
    "master_index": ("ec_master_t", "index", 4),
    "change_slave": ("ec_fsm_change_t", "slave", 8),
    "change_datagram": ("ec_fsm_change_t", "datagram", 8),
    "change_requested": ("ec_fsm_change_t", "requested_state", 4),
    "change_state": ("ec_fsm_change_t", "state", 8),
    "datagram_wc": ("ec_datagram_t", "working_counter", 2),
    "datagram_sent_jiffies": ("ec_datagram_t", "jiffies_sent", 8),
    "datagram_received_jiffies": ("ec_datagram_t", "jiffies_received", 8),
    "datagram_app_time_sent": ("ec_datagram_t", "app_time_sent", 8),
}
STATE_PREFIX = "ec_fsm_slave_config_state_"


def run(command):
    return subprocess.run(command, check=True, capture_output=True, timeout=30).stdout


def parse_layout(text):
    values = {}
    for name, offset, size in re.findall(r"^(\w+)=(\d+),(\d+)$", text, re.MULTILINE):
        if name not in FIELDS or int(size) != FIELDS[name][2]:
            raise ValueError(f"Unexpected DWARF field size: {name}/{size}")
        values[name] = int(offset)
    received = re.search(r"^received=(\d+)$", text, re.MULTILINE)
    if set(values) != set(FIELDS) or received is None:
        raise ValueError("Incomplete DWARF layout; refusing guessed offsets")
    values["received"] = int(received.group(1))
    return values


def inspect_module(module):
    notes = Path("/sys/module/ec_master/notes/.note.gnu.build-id").read_bytes()
    disk_notes = run(["objcopy", "--dump-section", ".note.gnu.build-id=/dev/stdout",
                      str(module), "/dev/null"])
    if not notes or disk_notes != notes:
        raise ValueError("Debug module does not match the loaded ec_master Build ID")
    command = ["gdb", "-nx", "-nh", "-batch", "-iex", "set auto-load off",
               "-iex", "set debuginfod enabled off", str(module)]
    for name, (typename, field, _size) in FIELDS.items():
        expression = f"(({typename} *)0)->{field}"
        command += ["-ex", f'printf "{name}=%lu,%lu\\n", (unsigned long)&{expression}, sizeof({expression})']
    command += ["-ex", 'printf "received=%d\\n", EC_DATAGRAM_RECEIVED']
    layout_text = run(command).decode()
    symbols = {}
    change_symbols = {}
    for line in Path("/proc/kallsyms").read_text().splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[3] == "[ec_master]" and fields[2].startswith(STATE_PREFIX):
            address = int(fields[0], 16)
            if address:
                symbols[address] = fields[2][len(STATE_PREFIX):].split(".")[0]
        if len(fields) == 4 and fields[3] == "[ec_master]" and fields[2].startswith("ec_fsm_change_state_"):
            address = int(fields[0], 16)
            if address:
                change_symbols[address] = fields[2][len("ec_fsm_change_state_"):].split(".")[0]
    if not {"start", "dc_sync_check", "op", "end", "error"} <= set(symbols.values()):
        raise ValueError("Loaded IgH state symbols are unavailable")
    return {"module": str(module), "module_sha256": hashlib.sha256(module.read_bytes()).hexdigest(),
            "build_note_hex": notes.hex(), "layout": parse_layout(layout_text),
            "layout_evidence": layout_text, "symbols": symbols, "change_symbols": change_symbols}


def probe_definitions(group, layout, al_details=False):
    slave = f'+{layout["config_slave"]}($arg1)'
    position = f'+{layout["slave_position"]}({slave})'
    master_slave = f'+{layout["master_fsm_slave"]}($arg1)'
    datagram = f'+{layout["config_datagram"]}($arg1)'
    data = f'+{layout["datagram_data"]}({datagram})'
    common = f'fsm=$arg1:x64 pos={position}:u16 state=+{layout["config_state"]}($arg1):x64'
    definitions = {
        "cfg_begin": ("p", "ec_fsm_slave_config_start",
                      f'fsm=$arg1:x64 pos=+{layout["slave_position"]}($arg2):u16 '
                      f'requested=+{layout["slave_requested"]}($arg2):u32'),
        "cfg_exec": ("p", "ec_fsm_slave_config_exec", common),
        "cfg_ret": ("r64", "ec_fsm_slave_config_exec", "result=$retval:s32"),
        "cfg_end": ("p", "ec_fsm_slave_config_success", common),
        "dc_sample": ("p", "ec_fsm_slave_config_state_dc_sync_check",
                      f'fsm=$arg1:x64 pos={position}:u16 raw=+0({data}):x32 '
                      f'dstate=+{layout["datagram_state"]}({datagram}):u32'),
        "activate": ("p", "ecrt_master_activate", f'master=+{layout["master_index"]}($arg1):u32'),
        "activate_ret": ("r64", "ecrt_master_activate", "result=$retval:s32"),
        "sdo_enter": ("p", "ecrt_master_sdo_download",
                      "pos=$arg2:u16 index=$arg3:x16 sub=$arg4:u8 size=$arg6:u64"),
        "sdo_exit": ("r64", "ecrt_master_sdo_download", "result=$retval:s32"),
    }
    for width in [32, 64]:
        function = f"ec_fsm_master_dc_offset{width}"
        definitions[f"offset{width}_enter"] = (
            "p", function, f'fsm=$arg1:x64 pos=+{layout["slave_position"]}({master_slave}):u16 '
            "system=$arg2:x64 old=$arg3:x64 app=$arg4:x64")
        definitions[f"offset{width}_exit"] = ("r64", function, "new=$retval:x64")
    if al_details:
        change_slave = f'+{layout["change_slave"]}($arg1)'
        change_datagram = f'+{layout["change_datagram"]}($arg1)'
        change_data = f'+{layout["datagram_data"]}({change_datagram})'
        change_common = (f'fsm=$arg1:x64 pos=+{layout["slave_position"]}({change_slave}):u16 '
                         f'requested=+{layout["change_requested"]}($arg1):u32')
        transfer = (f'dstate=+{layout["datagram_state"]}({change_datagram}):u32 '
                    f'wc=+{layout["datagram_wc"]}({change_datagram}):u16 '
                    f'tx_jiffies=+{layout["datagram_sent_jiffies"]}({change_datagram}):u64 '
                    f'rx_jiffies=+{layout["datagram_received_jiffies"]}({change_datagram}):u64 '
                    f'tx_app_ns=+{layout["datagram_app_time_sent"]}({change_datagram}):u64')
        definitions.update({
            "al_begin": ("p", "ec_fsm_change_start",
                         f'fsm=$arg1:x64 pos=+{layout["slave_position"]}($arg2):u16 requested=$arg3:u32'),
            "al_ack": ("p", "ec_fsm_change_state_check", f'{change_common} {transfer}'),
            "al_poll": ("p", "ec_fsm_change_state_status",
                        f'{change_common} {transfer} raw=+0({change_data}):u16'),
            "al_end": ("p", "ec_fsm_change_success",
                       f'{change_common} state=+{layout["change_state"]}($arg1):x64'),
            "dc_start_ack": ("p", "ec_fsm_slave_config_state_dc_start",
                             f'pos={position}:u16 start_ns=+0({data}):u64 '
                             f'dstate=+{layout["datagram_state"]}({datagram}):u32'),
        })
    return {name: f"{kind}:{group}/{group}_{name} ec_master:{function} {arguments}"
            for name, (kind, function, arguments) in definitions.items()}


def write_control(path, value, append=False):
    try:
        # tracefs rejects SEEK_END, which Python's buffered append mode attempts.
        descriptor = os.open(path, os.O_WRONLY | (os.O_APPEND if append else os.O_TRUNC))
        try:
            data = (value + "\n").encode()
            if os.write(descriptor, data) != len(data):
                raise OSError("Short trace-control write")
        finally:
            os.close(descriptor)
    except OSError as error:
        raise OSError(f"Trace control rejected {path}: {value!r}: {error}") from error


class TraceSession:
    def __init__(self, root, group):
        if not re.fullmatch(r"e97_[a-z0-9_]+", group):
            raise ValueError("Only private e97_* trace groups are permitted")
        self.root = root
        self.group = group
        self.instance = root / "instances" / group
        self.registered = []

    def __enter__(self):
        if (self.root / "events" / self.group).exists():
            raise ValueError("Trace group already exists")
        self.instance.mkdir()
        return self

    def arm(self, definitions, received):
        for name, definition in definitions.items():
            write_control(self.root / "kprobe_events", definition, append=True)
            self.registered.append(name)
        write_control(self.instance / "tracing_on", "0")
        write_control(self.instance / "current_tracer", "nop")
        write_control(self.instance / "buffer_size_kb", "1024")
        if "mono" not in self.instance.joinpath("trace_clock").read_text().replace("[", "").replace("]", "").split():
            raise ValueError("Monotonic trace clock is unavailable")
        write_control(self.instance / "trace_clock", "mono")
        if "dc_sample" in definitions:
            write_control(self.instance / "events" / self.group / f"{self.group}_dc_sample" / "filter",
                          f"dstate == {received}")
        for name in ["al_ack", "al_poll", "dc_start_ack"]:
            if name in definitions:
                write_control(self.instance / "events" / self.group / f"{self.group}_{name}" / "filter",
                              f"dstate == {received}")
        write_control(self.instance / "events" / self.group / "enable", "1")
        write_control(self.instance / "tracing_on", "1")

    def freeze(self):
        write_control(self.instance / "tracing_on", "0")

    def statistics(self):
        stats = {path.parent.name: path.read_text() for path in self.instance.glob("per_cpu/cpu*/stats")}
        profile = {}
        for line in (self.root / "kprobe_profile").read_text().splitlines():
            columns = line.split()
            if len(columns) == 3 and self.group in columns[0]:
                profile[columns[0]] = {"hits": int(columns[1]), "missed": int(columns[2])}
        return stats, profile

    def __exit__(self, *_exception):
        errors = []
        for path in [self.instance / "tracing_on", self.instance / "events" / self.group / "enable"]:
            try:
                write_control(path, "0")
            except OSError as error:
                if path.exists():
                    errors.append(str(error))
        try:
            self.instance.rmdir()
        except OSError as error:
            errors.append(str(error))
        for name in reversed(self.registered):
            try:
                write_control(self.root / "kprobe_events", f"-:{self.group}/{self.group}_{name}", append=True)
            except OSError as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError("Trace cleanup incomplete: " + "; ".join(errors))


def loss_errors(stats, profile):
    errors = []
    for cpu, text in stats.items():
        for counter, value in re.findall(r"^(overrun|commit overrun|dropped events):\s*(\d+)", text, re.MULTILINE):
            if int(value):
                errors.append(f"{cpu} {counter}={value}")
    errors.extend(f"{name} missed={values['missed']}" for name, values in profile.items() if values["missed"])
    return errors


@dataclass(frozen=True)
class Event:
    time_ns: int
    tid: int
    name: str
    fields: dict


def parse_trace(text, group):
    events = []
    pattern = re.compile(r"^\s*.*-(\d+)\s+\[\d+\]\s+.*?(\d+\.\d+):\s+" +
                         re.escape(group) + r"_(\w+):\s+(.*)$")
    for line in text.splitlines():
        if "LOST" in line and "EVENT" in line:
            raise ValueError("Kernel trace reports lost events")
        if group + "_" not in line:
            continue
        match = pattern.match(line)
        if not match:
            raise ValueError("Malformed trace event: " + line)
        tid, timestamp, name, payload = match.groups()
        fields = {}
        for key, value in re.findall(r"\b(\w+)=(\S+)", payload):
            fields[key] = int(value, 16 if value.startswith("0x") else 10)
        events.append(Event(int(Decimal(timestamp) * 1000000000), int(tid), name, fields))
    return events


def interval_union_ns(intervals):
    end = None
    total = 0
    for start, stop in sorted(intervals):
        if end is None or start > end:
            total += stop - start
        elif stop > end:
            total += stop - end
        end = stop if end is None else max(end, stop)
    return total


def analyze_al_events(events, symbols, kernel_hz):
    if kernel_hz <= 0:
        raise ValueError("Invalid kernel HZ")
    active, rows, errors, dc_starts = {}, [], [], []
    for event in events:
        fields = event.fields
        if event.name == "dc_start_ack":
            dc_starts.append({"time_ns": event.time_ns, "position": fields["pos"],
                              "start_ns": fields["start_ns"]})
            continue
        if not event.name.startswith("al_"):
            continue
        key = fields["fsm"]
        if event.name == "al_begin":
            if key in active:
                errors.append(f"AL transition restarted before completion: {key}")
            active[key] = {"position": fields["pos"], "requested_state": fields["requested"],
                           "start_ns": event.time_ns, "acks": [], "polls": []}
            continue
        if key not in active:
            # Error acknowledgements use a separate entry function, not al_begin.
            if event.name != "al_end":
                errors.append(f"Unpaired AL transfer: {event.name}/{key}")
            continue
        row = active[key]
        if (fields["pos"], fields["requested"]) != (row["position"], row["requested_state"]):
            errors.append(f"AL identity changed during transition: {key}")
        if event.name in {"al_ack", "al_poll"}:
            transfer = {"time_ns": event.time_ns, **fields}
            transfer["round_trip_ms"] = ((fields["rx_jiffies"] - fields["tx_jiffies"]) % (1 << 64)) * 1000 / kernel_hz
            row["acks" if event.name == "al_ack" else "polls"].append(transfer)
        elif event.name == "al_end":
            active.pop(key)
            row["end_ns"] = event.time_ns
            row["outcome"] = symbols.get(fields["state"], "unknown")
            if row["outcome"] not in {"end", "error"}:
                errors.append(f"Unknown AL terminal state: {fields['state']}")
            valid_acks = [item for item in row["acks"] if item["wc"] == 1]
            ready = [item for item in row["polls"]
                     if item["wc"] == 1 and item["raw"] == row["requested_state"]]
            row["write_to_ready_ms"] = None
            if valid_acks and ready:
                row["write_to_ready_ms"] = ((ready[0]["rx_jiffies"] - valid_acks[0]["tx_jiffies"]) % (1 << 64)) * 1000 / kernel_hz
            elif row["outcome"] == "end":
                errors.append(f"Successful AL transition lacks acknowledged write or ready response: {key}")
            row["poll_tx_intervals_ms"] = [
                ((right["tx_jiffies"] - left["tx_jiffies"]) % (1 << 64)) * 1000 / kernel_hz
                for left, right in zip(row["polls"], row["polls"][1:])]
            rows.append(row)
    if active:
        errors.append(f"Incomplete AL transitions: {len(active)}")
    return {"transitions": rows, "dc_starts": dc_starts, "errors": errors}


def analyze_events(events, symbols):
    active = {}
    pending = defaultdict(list)
    pending_offsets = defaultdict(list)
    pending_sdo = defaultdict(list)
    sdo_downloads = []
    rows, offsets, dc_samples, errors, activations = [], [], [], [], []

    def change(row, name, timestamp):
        if row["state"] is not None:
            elapsed = timestamp - row["phase_start"]
            if elapsed < 0:
                raise ValueError("State timing moved backwards")
            row["phases_ns"][row["state"]] += elapsed
        row["state"], row["phase_start"] = name, timestamp

    for item in events:
        fields = item.fields
        try:
            if item.name == "activate":
                activations.append(item.time_ns)
            elif item.name == "activate_ret" and fields["result"] != 0:
                errors.append("Master activation failed")
            elif item.name == "sdo_enter":
                pending_sdo[item.tid].append(item)
            elif item.name == "sdo_exit":
                entry = pending_sdo[item.tid].pop()
                sdo_downloads.append({**entry.fields, "start_ns": entry.time_ns,
                                      "end_ns": item.time_ns, "duration_ns": item.time_ns - entry.time_ns,
                                      "result": fields["result"]})
            elif item.name == "cfg_begin":
                if fields["fsm"] in active:
                    raise ValueError("Configuration restarted without a terminal event")
                active[fields["fsm"]] = {
                    "position": fields["pos"], "requested_state": fields["requested"],
                    "start_ns": item.time_ns, "phase_start": item.time_ns,
                    "state": "start", "last_exit": None, "phases_ns": defaultdict(int)}
            elif item.name == "cfg_exec":
                row = active[fields["fsm"]]
                state = symbols[fields["state"]]
                if state != row["state"]:
                    change(row, state, row["last_exit"] if row["last_exit"] is not None else item.time_ns)
                pending[item.tid].append(fields["fsm"])
            elif item.name == "cfg_ret":
                fsm = pending[item.tid].pop()
                active[fsm]["last_exit"] = item.time_ns
            elif item.name == "cfg_end":
                row = active.pop(fields["fsm"])
                terminal = symbols[fields["state"]]
                if terminal not in {"end", "error"} or row["last_exit"] is None:
                    raise ValueError("Missing terminal state or final execution return")
                end = row["last_exit"]
                change(row, None, end)
                rows.append({key: row[key] for key in ["position", "requested_state", "start_ns", "phases_ns"]} |
                            {"end_ns": end, "duration_ns": end - row["start_ns"], "outcome": terminal})
            elif item.name == "dc_sample":
                raw = fields["raw"]
                dc_samples.append({"time_ns": item.time_ns, "position": fields["pos"],
                                   "difference_ns": (raw & 0x7fffffff) * (-1 if raw & 0x80000000 else 1)})
            elif re.fullmatch(r"offset(32|64)_enter", item.name):
                width = int(item.name[6:8])
                pending_offsets[item.tid, width].append(item)
            elif re.fullmatch(r"offset(32|64)_exit", item.name):
                width = int(item.name[6:8])
                entry = pending_offsets[item.tid, width].pop()
                original = entry.fields
                difference = (original["app"] - original["system"]) & ((1 << width) - 1)
                if difference & (1 << (width - 1)):
                    difference -= 1 << width
                if fields["new"] != original["old"] and fields["new"] != ((original["old"] + difference) & ((1 << width) - 1)):
                    raise ValueError("Initial offset return does not match its captured inputs")
                offsets.append({"position": original["pos"], "width": width,
                                "time_ns": entry.time_ns, "system_time_ns": original["system"],
                                "app_time_sent_ns": original["app"], "old_offset": original["old"],
                                "new_offset": fields["new"], "difference_ns": difference,
                                "changed": fields["new"] != original["old"]})
        except (KeyError, IndexError, ValueError) as error:
            errors.append(f"{item.time_ns} {item.name}: {error}")
    if active or any(pending.values()) or any(pending_offsets.values()) or any(pending_sdo.values()):
        errors.append("Capture contains unfinished configurations or unmatched function entries")
    if len(activations) > 1:
        errors.append("Capture contains multiple master activations")
    phase_totals = defaultdict(int)
    for row in rows:
        for phase, duration in row["phases_ns"].items():
            phase_totals[phase] += duration
    span = None
    if activations and rows:
        span = max(row["end_ns"] for row in rows) - activations[0]
    return {"complete": not errors, "errors": errors, "slaves": rows, "offsets": offsets,
            "sdo_downloads": sdo_downloads,
            "pre_activation_sdo_ns": sum(row["duration_ns"] for row in sdo_downloads
                                         if activations and row["end_ns"] <= activations[0]),
            "dc_samples": dc_samples, "phase_totals_ns": phase_totals,
            "activation_to_last_config_ns": span,
            "configuration_union_ns": interval_union_ns([(row["start_ns"], row["end_ns"]) for row in rows])}


def capture(args):
    if os.geteuid() != 0 or platform.machine() != "x86_64":
        raise ValueError("Capture requires root on the verified x86_64 target")
    if not 1 <= args.seconds <= 300:
        raise ValueError("Capture duration must be between 1 and 300 seconds")
    master = run(["ethercat", "master"]).decode()
    if "Active: no" not in master or len(re.findall(r"^Master\d+$", master, re.MULTILINE)) != 1:
        raise ValueError("Arm only while the single EtherCAT master is inactive")
    metadata = inspect_module(args.module.resolve())
    group = "e97_" + uuid.uuid4().hex[:10]
    al_details = getattr(args, "al_details", False)
    definitions = probe_definitions(group, metadata["layout"], al_details=al_details)
    if al_details:
        config = Path(f"/boot/config-{platform.release()}").read_text()
        hz = re.findall(r"^CONFIG_HZ=([1-9][0-9]*)$", config, re.MULTILINE)
        if len(hz) != 1 or not {"end", "error"} <= set(metadata["change_symbols"].values()):
            raise ValueError("Kernel HZ or AL state symbols unavailable")
        metadata["kernel_hz"] = int(hz[0])
    metadata.update(group=group, definitions=definitions, master_before=master,
                    slaves_before=run(["ethercat", "slaves"]).decode(), capture_error=None)
    args.output.mkdir(mode=0o700, parents=True, exist_ok=False)
    stopped = []
    old_handlers = {sig: signal.signal(sig, lambda number, _frame: stopped.append(number))
                    for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]}
    try:
        with TraceSession(args.tracefs, group) as session:
            session.arm(definitions, metadata["layout"]["received"])
            metadata["formats"] = {name: (session.instance / "events" / group / f"{group}_{name}" / "format").read_text()
                                   for name in definitions}
            descriptor = os.open(session.instance / "trace_pipe", os.O_RDONLY | os.O_NONBLOCK)
            try:
                with (args.output / "trace.txt").open("wb") as output:
                    deadline = time.monotonic() + args.seconds
                    print(json.dumps({"status": "ARMED", "group": group, "pid": os.getpid(),
                                      "output": str(args.output)}), flush=True)
                    while not stopped and time.monotonic() < deadline:
                        readable, _, _ = select.select([descriptor], [], [], 0.2)
                        if readable:
                            output.write(os.read(descriptor, 65536))
                    session.freeze()
                    while True:
                        try:
                            block = os.read(descriptor, 65536)
                        except BlockingIOError:
                            break
                        if not block:
                            break
                        output.write(block)
            finally:
                os.close(descriptor)
            metadata["cpu_stats"], metadata["probe_profile"] = session.statistics()
            metadata["loss_errors"] = loss_errors(metadata["cpu_stats"], metadata["probe_profile"])
            if not metadata["cpu_stats"] or len(metadata["probe_profile"]) != len(definitions):
                metadata["loss_errors"].append("Missing trace or probe loss counters")
            metadata["stop_signals"] = stopped
    except BaseException as error:
        metadata["capture_error"] = str(error)
        raise
    finally:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        owner = int(os.environ.get("SUDO_UID", os.getuid()))
        group_id = int(os.environ.get("SUDO_GID", os.getgid()))
        for path in args.output.iterdir():
            path.chmod(0o600)
            os.chown(path, owner, group_id)
        os.chown(args.output, owner, group_id)
    print(json.dumps({"status": "CAPTURED", "loss_errors": metadata["loss_errors"]}), flush=True)
    return 1 if metadata["loss_errors"] else 0


def analyze(args):
    metadata = json.loads((args.capture / "metadata.json").read_text())
    symbols = {int(address): name for address, name in metadata["symbols"].items()}
    events = parse_trace((args.capture / "trace.txt").read_text(), metadata["group"])
    report = analyze_events(events, symbols)
    if "al_begin" in metadata.get("definitions", {}):
        change_symbols = {int(address): name for address, name in metadata["change_symbols"].items()}
        report["al"] = analyze_al_events(events, change_symbols, metadata["kernel_hz"])
        report["errors"].extend(report["al"]["errors"])
    report["sdo_instrumented"] = "sdo_enter" in metadata.get("definitions", {})
    report["errors"].extend(metadata.get("loss_errors", []))
    if metadata.get("capture_error"):
        report["errors"].append(metadata["capture_error"])
    if not report["slaves"] or not report["offsets"]:
        report["errors"].append("No complete slave configuration or initial offset capture")
    report["complete"] = not report["errors"]
    (args.capture / "analysis.json").write_text(json.dumps(report, indent=2) + "\n")
    phases = sorted(report["phase_totals_ns"])
    with (args.capture / "stages.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["position", "outcome", "requested_state", "total_ms", *[phase + "_ms" for phase in phases]])
        for row in report["slaves"]:
            writer.writerow([row["position"], row["outcome"], row["requested_state"], row["duration_ns"] / 1e6,
                             *[row["phases_ns"].get(phase, 0) / 1e6 for phase in phases]])
    print(json.dumps({"complete": report["complete"], "errors": report["errors"],
                      "pre_activation_sdo_ms": report["pre_activation_sdo_ns"] / 1e6
                      if report["sdo_instrumented"] else None,
                      "activation_to_last_config_ms": None if report["activation_to_last_config_ns"] is None
                      else report["activation_to_last_config_ns"] / 1e6,
                      "phase_totals_ms": {name: value / 1e6 for name, value in report["phase_totals_ns"].items()}}, indent=2))
    return 0 if report["complete"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    collector = commands.add_parser("capture")
    collector.add_argument("--module", type=Path, required=True, help="Unstripped module matching the loaded Build ID")
    collector.add_argument("--output", type=Path, required=True, help="New private output directory")
    collector.add_argument("--seconds", type=float, default=120)
    collector.add_argument("--tracefs", type=Path, default=Path("/sys/kernel/tracing"))
    collector.add_argument("--al-details", action="store_true", help="Capture AL write/status transfers and DC start times")
    analyzer = commands.add_parser("analyze")
    analyzer.add_argument("capture", type=Path)
    args = parser.parse_args()
    try:
        return capture(args) if args.command == "capture" else analyze(args)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"FAIL: {error}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
