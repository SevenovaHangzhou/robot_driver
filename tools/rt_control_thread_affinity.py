#!/usr/bin/env python3
"""Pin only the controller-manager realtime update thread to the RT CPU.

The native launcher starts the full ROS launch tree on housekeeping CPUs.  This
helper then finds the single ros2_control_node SCHED_FIFO thread with the
approved rt_priority and moves only that thread onto the isolated realtime CPU.
All other ros2_control_node threads, including DDS, services and CANopen/Lely
event-loop work, remain on housekeeping CPUs.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

SCHED_FIFO = 1
SCHED_RR = 2
SCHEDULER_NAMES = {
    0: "SCHED_OTHER",
    SCHED_FIFO: "SCHED_FIFO",
    SCHED_RR: "SCHED_RR",
}


@dataclass(frozen=True)
class ThreadSnapshot:
    tid: int
    name: str
    scheduler: int
    rt_priority: int
    processor: int
    affinity: set[int]


def parse_cpu_list(value: str) -> set[int]:
    cpus: set[int] = set()
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            first, last = part.split("-", 1)
            cpus.update(range(int(first), int(last) + 1))
        else:
            cpus.add(int(part))
    if not cpus:
        raise ValueError(f"empty CPU list: {value!r}")
    return cpus


def format_cpu_list(cpus: set[int]) -> str:
    return ",".join(str(cpu) for cpu in sorted(cpus))


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def process_cmdline(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode()
    except OSError:
        return ""


def find_ros2_control_pid() -> int | None:
    matches: list[int] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        cmdline = process_cmdline(pid)
        if "ros2_control_node" in cmdline and "controller_manager" in cmdline:
            matches.append(pid)
    if len(matches) > 1:
        raise RuntimeError(f"multiple ros2_control_node processes found: {matches}")
    return matches[0] if matches else None


def thread_name(tid: int) -> str:
    status = read_text(Path(f"/proc/{tid}/status"))
    for line in status.splitlines():
        if line.startswith("Name:"):
            return line.split(":", 1)[1].strip()
    return "unknown"


def list_threads(pid: int) -> list[int]:
    task_dir = Path(f"/proc/{pid}/task")
    return sorted(int(entry.name) for entry in task_dir.iterdir() if entry.name.isdigit())


def scheduler_and_priority(tid: int) -> tuple[int, int]:
    scheduler = os.sched_getscheduler(tid)
    priority = os.sched_getparam(tid).sched_priority
    return scheduler, priority


def current_processor(tid: int) -> int:
    stat = read_text(Path(f"/proc/{tid}/stat"))
    fields_after_comm = stat.rsplit(") ", 1)[1].split()
    # proc_pid_stat(5): processor is field 39.  fields_after_comm[0] is field 3.
    return int(fields_after_comm[36])


def thread_snapshot(tid: int) -> ThreadSnapshot:
    scheduler, priority = scheduler_and_priority(tid)
    return ThreadSnapshot(
        tid=tid,
        name=thread_name(tid),
        scheduler=scheduler,
        rt_priority=priority,
        processor=current_processor(tid),
        affinity=os.sched_getaffinity(tid),
    )


def realtime_thread_inventory(pid: int) -> list[ThreadSnapshot]:
    snapshots = [thread_snapshot(tid) for tid in list_threads(pid)]
    return [
        snapshot
        for snapshot in snapshots
        if snapshot.scheduler in {SCHED_FIFO, SCHED_RR}
    ]


def describe_snapshot(snapshot: ThreadSnapshot) -> str:
    scheduler_name = SCHEDULER_NAMES.get(snapshot.scheduler, str(snapshot.scheduler))
    return (
        f"tid={snapshot.tid} comm={snapshot.name} policy={scheduler_name} "
        f"rt_priority={snapshot.rt_priority} PSR={snapshot.processor} "
        f"affinity={format_cpu_list(snapshot.affinity)}"
    )


def describe_inventory(pid: int) -> str:
    inventory = realtime_thread_inventory(pid)
    if not inventory:
        return "Phase 0 realtime inventory: no SCHED_FIFO/SCHED_RR threads found"
    lines = ["Phase 0 realtime inventory:"]
    lines.extend(f"  {describe_snapshot(snapshot)}" for snapshot in inventory)
    return "\n".join(lines)


def select_update_thread(pid: int, rt_priority: int) -> int:
    inventory = realtime_thread_inventory(pid)
    matches = [
        snapshot.tid
        for snapshot in inventory
        if snapshot.scheduler == SCHED_FIFO and snapshot.rt_priority == rt_priority
    ]
    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one matching update thread "
            f"(SCHED_FIFO rt_priority={rt_priority}), found {len(matches)}\n"
            f"{describe_inventory(pid)}"
        )
    return matches[0]


def pin_threads(
    pid: int,
    rt_cpu: set[int],
    housekeeping_cpus: set[int],
    rt_priority: int,
) -> int:
    threads = list_threads(pid)
    target_tid = select_update_thread(pid, rt_priority)
    for tid in threads:
        try:
            if tid == target_tid:
                os.sched_setaffinity(tid, rt_cpu)
            else:
                os.sched_setaffinity(tid, housekeeping_cpus)
        except OSError as exc:
            raise RuntimeError(
                f"sched_setaffinity failed for tid={tid} ({thread_name(tid)}): {exc}"
            ) from exc
    return target_tid


def verify_affinity(
    pid: int,
    target_tid: int,
    rt_cpu: set[int],
    housekeeping_cpus: set[int],
) -> None:
    for tid in list_threads(pid):
        actual = os.sched_getaffinity(tid)
        if tid == target_tid:
            if actual != rt_cpu:
                raise RuntimeError(
                    f"update thread {tid} affinity is {format_cpu_list(actual)}, "
                    f"expected {format_cpu_list(rt_cpu)}"
                )
        elif actual != housekeeping_cpus:
            raise RuntimeError(
                f"non-realtime ros2_control_node thread {tid} "
                f"({thread_name(tid)}) affinity is {format_cpu_list(actual)}, "
                f"expected housekeeping {format_cpu_list(housekeeping_cpus)}"
            )


def wait_for_target_processor(tid: int, rt_cpu: int, deadline_seconds: float) -> None:
    deadline = time.monotonic() + deadline_seconds
    last_processor = -1
    while time.monotonic() < deadline:
        last_processor = current_processor(tid)
        if last_processor == rt_cpu:
            return
        time.sleep(0.02)
    raise RuntimeError(
        f"update thread {tid} PSR={last_processor}, expected PSR={rt_cpu} "
        "after sched_setaffinity"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rt-cpu", required=True)
    parser.add_argument("--housekeeping-cpus", required=True)
    parser.add_argument("--rt-priority", type=int, default=80)
    parser.add_argument("--deadline", type=float, default=30.0)
    parser.add_argument("--psr-deadline", type=float, default=2.0)
    parser.add_argument(
        "--sample",
        action="store_true",
        help="print the Phase 0 ros2_control_node FIFO/RR thread inventory and exit",
    )
    args = parser.parse_args()

    rt_cpu = parse_cpu_list(args.rt_cpu)
    housekeeping_cpus = parse_cpu_list(args.housekeeping_cpus)
    if rt_cpu & housekeeping_cpus:
        raise SystemExit("RT CPU and housekeeping CPUs must not overlap")
    if len(rt_cpu) != 1:
        raise SystemExit("this gate supports exactly one isolated RT CPU")
    rt_cpu_number = next(iter(rt_cpu))

    deadline = time.monotonic() + args.deadline
    last_error = ""
    while time.monotonic() < deadline:
        try:
            pid = find_ros2_control_pid()
            if pid is None:
                last_error = "ros2_control_node process not found"
                time.sleep(0.5)
                continue
            if args.sample:
                print(describe_inventory(pid))
                return 0
            target_tid = pin_threads(pid, rt_cpu, housekeeping_cpus, args.rt_priority)
            verify_affinity(pid, target_tid, rt_cpu, housekeeping_cpus)
            wait_for_target_processor(target_tid, rt_cpu_number, args.psr_deadline)
            print(
                "Pinned ros2_control_node update thread "
                f"pid={pid} tid={target_tid} name={thread_name(target_tid)} "
                f"to RT CPU {format_cpu_list(rt_cpu)} with verified PSR={rt_cpu_number}; "
                f"other threads remain on "
                f"housekeeping CPUs {format_cpu_list(housekeeping_cpus)}"
            )
            return 0
        except (FileNotFoundError, ProcessLookupError) as exc:
            last_error = str(exc)
            time.sleep(0.5)
        except (RuntimeError, OSError) as exc:
            print(f"FAIL: {exc}", file=sys.stderr)
            return 1

    print(f"FAIL: {last_error}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
