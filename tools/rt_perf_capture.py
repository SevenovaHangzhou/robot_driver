#!/usr/bin/env python3
"""T2 实时性能采集：产出 release_test_runner delta 可消费的候选指标 YAML（ELECTRI-80）。

覆盖 domains/rt_control/testing/cases/realtime-perf.yaml 的 TC-RT-01..03：

- `capture`：180 s 只读窗口，采 CPU14 busy 均值/峰值、ros2_control_node RSS、
  blocked/swap、CAN 错误增量、可选 /diagnostics `wc_error_count` 增量（TC-RT-02/03）。
- `cyclictest`：TC-RT-01。本工具**不自己起 docker**——按 runbook 第 8.1 节
  （BQ-093/096）只打印一次性诊断容器命令，实际运行由现场人员执行，日志用
  `--from-log` 回灌解析。判据 BQ-064：任何延迟 > 100 us 即 FAIL。
- `parse`：把任一采集文本喂给对应纯函数，便于离线复算与证据归档。

指标键与 domains/rt_control/testing/baselines/v0.yaml 同名，口径同
docs/docker-deployment-performance-summary.md 的 180 s 窗口；本文件只做测量与记录，
判定阈值属 thresholds.yaml（BQ-134），不在此复制第二份事实。

所有解析器都是纯函数（文本入、dict 出），不碰 ROS、不联网、不写控制对象。
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

import yaml

CANDIDATE_SCHEMA = "rt-control-candidate/v1"
KIB_PER_MIB = 1024.0

# BQ-064 冻结判据：任何一次 cyclictest 延迟超过该值即 FAIL。
CYCLICTEST_MAX_LATENCY_US = 100.0
# runbook 8.1 / BQ-096 固定的注入参数，改动须新 BQ 裁决。
CYCLICTEST_PRIORITY = 90
CYCLICTEST_INTERVAL_US = 1000
DEFAULT_CYCLICTEST_DURATION_S = 1800
DEFAULT_RT_CPU = 14
DEFAULT_WINDOW_S = 180
DEFAULT_PROCESS = "ros2_control_node"
DEFAULT_CAN_INTERFACES = ("can0", "can1")

CAN_COUNTER_KEYS = (
    "bus_errors", "error_warn", "error_pass", "bus_off",
    "arbit_lost", "re_started", "rx_errors", "tx_errors",
    "berr_tx", "berr_rx",
)


class ParseError(ValueError):
    """采集文本不含所需字段：拒绝猜测，交由调用方记 UNVERIFIED。"""


class CaptureError(RuntimeError):
    """采集命令不可用或失败：目标机环境问题，不得静默降级。"""


def _numbers(line: str) -> list[str]:
    return re.findall(r"-?\d+(?:\.\d+)?", line)


def _mean(values: list[float]) -> float:
    return sum(values) / len(values)


# ------------------------------------------------------------------ pidstat/mpstat

def _pidstat_rows(text: str, column: str) -> tuple[list[list[str]], int, float | None]:
    """返回 (样本行, 目标列下标, Average 行该列值)。列名如 '%CPU' / 'RSS'。"""
    header_fields: list[str] | None = None
    index = -1
    samples: list[list[str]] = []
    average: float | None = None
    for line in text.splitlines():
        fields = line.split()
        if not fields:
            continue
        if column in fields:
            header_fields = fields
            index = fields.index(column)
            continue
        if header_fields is None or index < 0 or len(fields) <= index:
            continue
        if fields[0] == "Average:":
            # "Average:" 占据时间戳列，因此列下标与样本行一致。
            try:
                average = float(fields[index])
            except ValueError:
                average = None
            continue
        try:
            float(fields[index])
        except ValueError:
            continue
        samples.append(fields)
    return samples, index, average


def parse_pidstat_cpu(text: str) -> dict:
    """解析 `pidstat -u -p <pid> 1 N`：进程 CPU 均值与 1 s 峰值。"""
    samples, index, average = _pidstat_rows(text, "%CPU")
    if not samples:
        raise ParseError("pidstat CPU 输出没有可用样本行（缺 %CPU 列或无采样）")
    values = [float(row[index]) for row in samples]
    pids = {int(row[2]) for row in samples if row[2].isdigit()}
    return {
        "samples": len(values),
        "avg_percent": average if average is not None else _mean(values),
        "peak_percent": max(values),
        "pid": pids.pop() if len(pids) == 1 else None,
    }


def parse_pidstat_rss(text: str) -> dict:
    """解析 `pidstat -r -p <pid> 1 N`：RSS 峰值（KiB 原值与 MiB 换算）。"""
    samples, index, _ = _pidstat_rows(text, "RSS")
    if not samples:
        raise ParseError("pidstat 内存输出没有可用样本行（缺 RSS 列或无采样）")
    peak_kib = max(float(row[index]) for row in samples)
    return {
        "samples": len(samples),
        "rss_kib_peak": peak_kib,
        "rss_mib_peak": peak_kib / KIB_PER_MIB,
    }


def parse_mpstat_cpu_busy(text: str, cpu: int = DEFAULT_RT_CPU) -> dict:
    """解析 `mpstat -P <cpu> 1 N`：由 %idle 反算 busy 的均值与峰值。

    与 baseline v0 同口径：busy = 100 - %idle，均值取全样本平均、峰值取 1 s 最差样本。
    """
    header: list[str] | None = None
    idle_index = -1
    cpu_index = -1
    idles: list[float] = []
    average_idle: float | None = None
    for line in text.splitlines():
        fields = line.split()
        if not fields:
            continue
        if "%idle" in fields and "CPU" in fields:
            header = fields
            idle_index = fields.index("%idle")
            cpu_index = fields.index("CPU")
            continue
        if header is None or len(fields) <= idle_index:
            continue
        if fields[cpu_index] != str(cpu):
            continue
        try:
            value = float(fields[idle_index])
        except ValueError:
            continue
        # "Average:" 占据时间戳列，列下标与样本行一致；均值优先用它，口径与
        # baseline v0 的 180 s 窗口一致。
        if fields[0] == "Average:":
            average_idle = value
            continue
        idles.append(value)
    if not idles:
        raise ParseError(f"mpstat 输出中没有 CPU {cpu} 的样本行")
    busy = [100.0 - value for value in idles]
    average_busy = 100.0 - average_idle if average_idle is not None else _mean(busy)
    return {
        "cpu": cpu,
        "samples": len(busy),
        "avg_percent": average_busy,
        "peak_percent": max(busy),
    }


# --------------------------------------------------------------- vmstat/free/proc

def parse_vmstat(text: str) -> dict:
    """解析 `vmstat 1 N`：blocked 峰值、runnable 峰值、swap-in/out 峰值。

    零容忍项按 BQ-134 只记录、不在此判 FAIL；峰值非零即需人工裁量。
    """
    rows: list[list[float]] = []
    for line in text.splitlines():
        fields = line.split()
        if len(fields) < 8 or not all(field.lstrip("-").isdigit() for field in fields[:8]):
            continue
        rows.append([float(field) for field in fields[:8]])
    if not rows:
        raise ParseError("vmstat 输出没有数值样本行")
    return {
        "samples": len(rows),
        "runnable_peak": int(max(row[0] for row in rows)),
        "blocked_processes": int(max(row[1] for row in rows)),
        "swap_in": int(max(row[6] for row in rows)),
        "swap_out": int(max(row[7] for row in rows)),
    }


def parse_free(text: str) -> dict:
    """解析 `free -m`：swap 总量/占用与可用内存（MiB）。"""
    swap: list[str] | None = None
    mem: list[str] | None = None
    for line in text.splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0].startswith("Swap"):
            swap = fields
        elif fields[0].startswith("Mem"):
            mem = fields
    if swap is None or len(swap) < 4:
        raise ParseError("free 输出缺少 Swap 行")
    result = {
        "swap_total_mib": float(swap[1]),
        "swap_used_mib": float(swap[2]),
        "swap_free_mib": float(swap[3]),
    }
    if mem is not None and len(mem) >= 7:
        result["mem_available_mib"] = float(mem[6])
    return result


def parse_proc_stat(text: str) -> dict:
    """解析 /proc/stat 的 procs_blocked（无 vmstat 时的备用口径）。"""
    for line in text.splitlines():
        fields = line.split()
        if len(fields) == 2 and fields[0] == "procs_blocked":
            return {"blocked_processes": int(fields[1])}
    raise ParseError("/proc/stat 中没有 procs_blocked 字段")


# ------------------------------------------------------------------- CAN counters

def parse_ip_link_stats(text: str) -> dict:
    """解析 `ip -details -statistics link show`：按接口取 CAN 与 RX/TX 错误计数。"""
    interfaces: dict[str, dict] = {}
    current: dict | None = None
    pending_labels: list[str] | None = None
    direction: str | None = None
    for line in text.splitlines():
        stripped = line.strip()
        header = re.match(r"^\d+:\s+([^:@]+)[:@]", stripped)
        if header:
            current = {key: 0 for key in CAN_COUNTER_KEYS}
            current["state"] = None
            interfaces[header.group(1).strip()] = current
            pending_labels = None
            direction = None
            continue
        if current is None:
            continue
        state = re.search(r"\bstate\s+(ERROR-\w+|BUS-OFF|STOPPED|SLEEPING)\b", stripped)
        if state:
            current["state"] = state.group(1)
        berr = re.search(r"berr-counter\s+tx\s+(\d+)\s+rx\s+(\d+)", stripped)
        if berr:
            current["berr_tx"] = int(berr.group(1))
            current["berr_rx"] = int(berr.group(2))
        if "re-started" in stripped and "bus-errors" in stripped:
            pending_labels = stripped.split()
            continue
        if pending_labels is not None:
            values = _numbers(stripped)
            if len(values) >= len(pending_labels):
                for label, value in zip(pending_labels, values):
                    key = label.replace("-", "_")
                    if key in CAN_COUNTER_KEYS:
                        current[key] = int(float(value))
            pending_labels = None
            continue
        if stripped.startswith("RX:"):
            direction = "rx"
            continue
        if stripped.startswith("TX:"):
            direction = "tx"
            continue
        if direction is not None:
            values = _numbers(stripped)
            if len(values) >= 3:
                current[f"{direction}_errors"] = int(float(values[2]))
            direction = None
    if not interfaces:
        raise ParseError("ip link 输出中没有可识别的接口块")
    return interfaces


def can_error_total(interface_stats: dict) -> int:
    """接口全部错误计数之和：任何一项增长都要被窗口增量看见。"""
    return sum(int(interface_stats.get(key, 0)) for key in CAN_COUNTER_KEYS)


def diff_can_errors(first: dict, second: dict, interfaces=DEFAULT_CAN_INTERFACES) -> dict:
    """窗口首尾 CAN 错误增量；缺失接口单列 missing，不折算成 0。"""
    per_interface: dict[str, int] = {}
    missing: list[str] = []
    for name in interfaces:
        if name not in first or name not in second:
            missing.append(name)
            continue
        per_interface[name] = can_error_total(second[name]) - can_error_total(first[name])
    return {
        "can_error_delta": sum(per_interface.values()),
        "per_interface": per_interface,
        "missing": missing,
    }


# -------------------------------------------------------------------- cyclictest

CYCLICTEST_LINE = re.compile(
    r"^T:\s*(?P<thread>\d+)\s*\((?P<tid>\d+)\)\s*"
    r"P:\s*(?P<prio>\d+)\s+I:\s*(?P<interval>\d+)\s+C:\s*(?P<cycles>\d+)\s+"
    r"Min:\s*(?P<min>\d+)\s+Avg:\s*(?P<avg>\d+)\s+Max:\s*(?P<max>\d+)",
    re.MULTILINE,
)


def parse_cyclictest(text: str) -> dict:
    """解析 cyclictest 汇总行（`T: 0 (...) P:90 I:1000 C:... Min/Avg/Max`）。

    多线程时取最差线程：Min 取全局最小、Avg 取最大、Max 取最大——门禁看最坏情况。
    """
    matches = list(CYCLICTEST_LINE.finditer(text))
    if not matches:
        raise ParseError("cyclictest 输出中没有 'T: <n> ... Min: .. Avg: .. Max: ..' 汇总行")
    threads = [
        {
            "min": float(match.group("min")),
            "avg": float(match.group("avg")),
            "max": float(match.group("max")),
            "cycles": int(match.group("cycles")),
            "priority": int(match.group("prio")),
            "interval_us": int(match.group("interval")),
        }
        for match in matches
    ]
    worst = max(threads, key=lambda thread: thread["max"])
    return {
        "threads": len(threads),
        "cyclictest_min_us": min(thread["min"] for thread in threads),
        "cyclictest_avg_us": max(thread["avg"] for thread in threads),
        "cyclictest_max_us": worst["max"],
        "cycles": max(thread["cycles"] for thread in threads),
        "priority": worst["priority"],
        "interval_us": worst["interval_us"],
    }


def cyclictest_verdict(max_us: float) -> str:
    """BQ-064：> 100 us 即 FAIL，等于 100 us 仍算通过。"""
    return "FAIL" if max_us > CYCLICTEST_MAX_LATENCY_US else "PASS"


def cyclictest_docker_command(
    duration_s: int = DEFAULT_CYCLICTEST_DURATION_S,
    cpu: int = DEFAULT_RT_CPU,
    image: str = "<frozen-production-image-id>",
) -> str:
    """按 runbook 8.1（BQ-093/096）拼一次性诊断容器命令。

    只返回字符串供人工审阅执行；本工具绝不自己调用 docker，也不把 cyclictest
    装进生产镜像或改冻结 Compose。
    """
    return (
        "docker run --rm -it --read-only "
        "--cap-add=SYS_NICE --cap-add=IPC_LOCK --ulimit rtprio=99 --ulimit memlock=-1 "
        f"--cpuset-cpus {cpu} "
        "--device /dev/cpu_dma_latency:/dev/cpu_dma_latency "
        "-v /usr/bin/cyclictest:/usr/bin/cyclictest:ro "
        f"{image} "
        f"cyclictest --affinity {cpu} --threads 1 "
        f"--priority {CYCLICTEST_PRIORITY} --policy fifo "
        f"--interval {CYCLICTEST_INTERVAL_US} --mlockall --duration {duration_s} "
        "--histogram 1000 --quiet"
    )


# ---------------------------------------------------------------- wc_error_count

def parse_wc_error_count(text: str) -> float:
    """从 /diagnostics 文本取 `wc_error_count`（key/value 列表或 inline 映射两种形态）。"""
    inline = re.search(r"wc_error_count\s*[:=]\s*'?(-?\d+(?:\.\d+)?)'?", text)
    if inline:
        return float(inline.group(1))
    lines = text.splitlines()
    for index, line in enumerate(lines):
        if "wc_error_count" not in line:
            continue
        for candidate in lines[index + 1: index + 3]:
            value = re.search(r"value:\s*'?(-?\d+(?:\.\d+)?)'?", candidate)
            if value:
                return float(value.group(1))
    raise ParseError("/diagnostics 文本中没有 wc_error_count 读数")


def diff_wc_error(first_text: str, last_text: str) -> dict:
    """窗口首尾 wc_error_count 增量（TC-RT-03；已知成组丢包现象另行登记，不静默）。"""
    first = parse_wc_error_count(first_text)
    last = parse_wc_error_count(last_text)
    return {"wc_error_first": first, "wc_error_last": last, "wc_error_delta": last - first}


# ------------------------------------------------------------- metrics document

def build_metrics_document(
    cpu_busy: dict,
    process: dict,
    vm: dict,
    wc: dict | None = None,
    can: dict | None = None,
    cyclictest: dict | None = None,
    window_s: int = DEFAULT_WINDOW_S,
    cpu: int = DEFAULT_RT_CPU,
    extra: dict | None = None,
) -> dict:
    """组装候选指标文档；键名与 baseline v0 一致，未采到的指标一律不臆造。"""
    metrics = {
        f"cpu{cpu}_busy_avg_percent": cpu_busy["avg_percent"],
        f"cpu{cpu}_busy_peak_percent": cpu_busy["peak_percent"],
        "ros2_control_node_rss_mib": process["rss_mib_peak"],
        "blocked_processes": vm["blocked_processes"],
        "swap_in": vm["swap_in"],
        "swap_out": vm["swap_out"],
    }
    unverified: list[str] = []
    if wc is not None:
        metrics["wc_error_delta"] = wc["wc_error_delta"]
    else:
        unverified.append("wc_error_delta：未提供 /diagnostics 首尾读数，TC-RT-03 该项未验证")
    if can is not None:
        metrics["can_error_delta"] = can["can_error_delta"]
        if can.get("missing"):
            unverified.append(
                "can_error_delta：未取到接口 " + ",".join(can["missing"]) + " 的读数"
            )
    else:
        unverified.append("can_error_delta：未采集 CAN 错误计数，TC-RT-03 该项未验证")
    if cyclictest is not None:
        for key in ("cyclictest_min_us", "cyclictest_avg_us", "cyclictest_max_us"):
            metrics[key] = cyclictest[key]
    else:
        unverified.append(
            "cyclictest_*：需按 runbook 8.1 一次性诊断容器单独运行（TC-RT-01），"
            "本窗口未包含"
        )
    document = {
        "schema": CANDIDATE_SCHEMA,
        "window_s": window_s,
        "cpu": cpu,
        "metrics": metrics,
        "unverified": unverified,
    }
    if extra:
        document.update(extra)
    return document


# ------------------------------------------------------------------ orchestration

def run_command(argv: list[str], timeout_s: int | None = None) -> str:
    """默认执行器：只读采集命令，失败一律抛 CaptureError，不静默返回空文本。"""
    try:
        completed = subprocess.run(
            argv, check=True, capture_output=True, text=True, timeout=timeout_s
        )
    except FileNotFoundError as exc:
        raise CaptureError(f"目标机缺少采集命令 {argv[0]}：{exc}") from exc
    except subprocess.TimeoutExpired as exc:
        raise CaptureError(f"采集命令超时：{' '.join(argv)}") from exc
    except subprocess.CalledProcessError as exc:
        raise CaptureError(
            f"采集命令失败（exit {exc.returncode}）：{' '.join(argv)}\n{exc.stderr}"
        ) from exc
    return completed.stdout


def resolve_pid(process: str, runner) -> int:
    """按进程名解析 PID；多实例或无实例都要报错，不猜第一个。"""
    output = runner(["pgrep", "-f", process])
    pids = [int(token) for token in output.split() if token.isdigit()]
    if not pids:
        raise CaptureError(f"目标机上没有匹配 {process} 的进程：栈未运行时不得采样")
    if len(pids) > 1:
        raise CaptureError(f"{process} 匹配到多个 PID {pids}：请用 --pid 明确指定")
    return pids[0]


def capture(
    window_s: int = DEFAULT_WINDOW_S,
    cpu: int = DEFAULT_RT_CPU,
    pid: int | None = None,
    process: str = DEFAULT_PROCESS,
    can_interfaces=DEFAULT_CAN_INTERFACES,
    diagnostics_first: str | None = None,
    diagnostics_last: str | None = None,
    runner=None,
) -> dict:
    """执行 TC-RT-02/03 的只读采样窗口，返回候选指标文档。

    CAN 计数在窗口首尾各取一次；wc_error_count 需调用方提供首尾 /diagnostics 文本
    （本工具不订阅 ROS 话题）。
    """
    execute = runner or run_command
    if pid is None:
        pid = resolve_pid(process, execute)

    can_first = None
    if can_interfaces:
        can_first = parse_ip_link_stats(execute(["ip", "-details", "-statistics", "link", "show"]))

    cpu_busy = parse_mpstat_cpu_busy(
        execute(["mpstat", "-P", str(cpu), "1", str(window_s)]), cpu=cpu
    )
    process_memory = parse_pidstat_rss(
        execute(["pidstat", "-r", "-p", str(pid), "1", "1"])
    )
    process_cpu = parse_pidstat_cpu(execute(["pidstat", "-u", "-p", str(pid), "1", "1"]))
    vm = parse_vmstat(execute(["vmstat", "1", "2"]))
    free_info = parse_free(execute(["free", "-m"]))

    can_delta = None
    if can_interfaces and can_first is not None:
        can_last = parse_ip_link_stats(execute(["ip", "-details", "-statistics", "link", "show"]))
        can_delta = diff_can_errors(can_first, can_last, interfaces=can_interfaces)

    wc_delta = None
    if diagnostics_first and diagnostics_last:
        wc_delta = diff_wc_error(diagnostics_first, diagnostics_last)

    return build_metrics_document(
        cpu_busy=cpu_busy,
        process=process_memory,
        vm=vm,
        wc=wc_delta,
        can=can_delta,
        window_s=window_s,
        cpu=cpu,
        extra={
            "pid": pid,
            "process": process,
            "process_cpu_avg_percent": process_cpu["avg_percent"],
            "process_cpu_peak_percent": process_cpu["peak_percent"],
            "swap_used_mib": free_info["swap_used_mib"],
            "runnable_peak": vm["runnable_peak"],
            "can": can_delta,
            "case_refs": ["TC-RT-02", "TC-RT-03"],
        },
    )


# --------------------------------------------------------------------------- CLI

PARSERS = {
    "pidstat-cpu": parse_pidstat_cpu,
    "pidstat-rss": parse_pidstat_rss,
    "mpstat": parse_mpstat_cpu_busy,
    "vmstat": parse_vmstat,
    "free": parse_free,
    "proc-stat": parse_proc_stat,
    "ip-link": parse_ip_link_stats,
    "cyclictest": parse_cyclictest,
    "wc-error": lambda text: {"wc_error_count": parse_wc_error_count(text)},
}


def _dump(document: dict, out: Path | None) -> None:
    text = yaml.safe_dump(document, allow_unicode=True, sort_keys=False)
    if out:
        out.write_text(text, encoding="utf-8")
        print(f"metrics -> {out}", file=sys.stderr)
    else:
        print(text, end="")


def _capture_plan(args) -> str:
    return (
        f"计划：CPU{args.cpu} busy {args.window} s 窗口、{DEFAULT_PROCESS} RSS、vmstat/free、"
        f"CAN {','.join(args.can_interfaces or []) or '(跳过)'} 首尾计数；"
        "cyclictest 需单独执行 cyclictest 子命令"
    )


def main(argv=None, runner=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)

    cap = sub.add_parser("capture", help="TC-RT-02/03 只读窗口采样")
    cap.add_argument("--window", type=int, default=DEFAULT_WINDOW_S, help="采样秒数（默认 180）")
    cap.add_argument("--cpu", type=int, default=DEFAULT_RT_CPU)
    cap.add_argument("--pid", type=int, default=None)
    cap.add_argument("--process", default=DEFAULT_PROCESS)
    cap.add_argument("--can", dest="can_interfaces", action="append", default=None)
    cap.add_argument("--no-can", action="store_true", help="跳过 CAN 错误计数采集")
    cap.add_argument("--diagnostics-first", type=Path, default=None)
    cap.add_argument("--diagnostics-last", type=Path, default=None)
    cap.add_argument("--out", type=Path, default=None)
    cap.add_argument("--dry-run", action="store_true", help="只打印采集计划，不执行命令")

    cyc = sub.add_parser("cyclictest", help="TC-RT-01：打印注入命令或解析既有日志")
    cyc.add_argument("--duration", type=int, default=DEFAULT_CYCLICTEST_DURATION_S)
    cyc.add_argument("--cpu", type=int, default=DEFAULT_RT_CPU)
    cyc.add_argument("--image", default="<frozen-production-image-id>")
    cyc.add_argument("--from-log", type=Path, default=None, help="解析已归档的 cyclictest 输出")
    cyc.add_argument("--merge-into", type=Path, default=None, help="合并进既有 capture 文档")
    cyc.add_argument("--out", type=Path, default=None)
    cyc.add_argument("--print-command", action="store_true")

    wce = sub.add_parser("wc-error", help="TC-RT-03：窗口首尾 /diagnostics 读数算 wc_error_delta")
    wce.add_argument("--first", type=Path, required=True, help="窗口开始时的 /diagnostics 文本")
    wce.add_argument("--last", type=Path, required=True, help="窗口结束时的 /diagnostics 文本")
    wce.add_argument("--merge-into", type=Path, default=None, help="合并进既有 capture 文档")
    wce.add_argument("--out", type=Path, default=None)

    par = sub.add_parser("parse", help="把采集文本喂给对应纯解析函数")
    par.add_argument("--kind", choices=sorted(PARSERS), required=True)
    par.add_argument("--file", type=Path, required=True)

    args = parser.parse_args(argv)

    if args.command == "parse":
        try:
            result = PARSERS[args.kind](args.file.read_text(encoding="utf-8"))
        except ParseError as exc:
            print(f"PARSE-ERROR: {exc}", file=sys.stderr)
            return 2
        print(yaml.safe_dump(result, allow_unicode=True, sort_keys=False), end="")
        return 0

    if args.command == "wc-error":
        try:
            result = diff_wc_error(
                args.first.read_text(encoding="utf-8"), args.last.read_text(encoding="utf-8")
            )
        except ParseError as exc:
            print(f"PARSE-ERROR: {exc}", file=sys.stderr)
            return 2
        if args.merge_into:
            document = yaml.safe_load(args.merge_into.read_text(encoding="utf-8")) or {}
            document.setdefault("metrics", {})
            document["unverified"] = [
                note for note in document.get("unverified", []) if "wc_error" not in note
            ]
        else:
            document = {"schema": CANDIDATE_SCHEMA, "metrics": {}, "unverified": []}
        document["metrics"]["wc_error_delta"] = result["wc_error_delta"]
        document["wc_error"] = result
        _dump(document, args.out)
        return 0

    if args.command == "cyclictest":
        command = cyclictest_docker_command(
            duration_s=args.duration, cpu=args.cpu, image=args.image
        )
        if args.from_log is None:
            print("# 按 runbook 8.1 / BQ-093/096 由现场人员执行，本工具不调用 docker：")
            print(command)
            print("# 归档输出后回灌：rt_perf_capture.py cyclictest --from-log <log> --out <yaml>")
            return 0
        try:
            result = parse_cyclictest(args.from_log.read_text(encoding="utf-8"))
        except ParseError as exc:
            print(f"PARSE-ERROR: {exc}", file=sys.stderr)
            return 2
        verdict = cyclictest_verdict(result["cyclictest_max_us"])
        if args.merge_into:
            document = yaml.safe_load(args.merge_into.read_text(encoding="utf-8")) or {}
            document.setdefault("metrics", {})
            document["metrics"].update(
                {key: result[key] for key in
                 ("cyclictest_min_us", "cyclictest_avg_us", "cyclictest_max_us")}
            )
            document["unverified"] = [
                note for note in document.get("unverified", []) if "cyclictest" not in note
            ]
        else:
            document = {
                "schema": CANDIDATE_SCHEMA,
                "metrics": {key: result[key] for key in
                            ("cyclictest_min_us", "cyclictest_avg_us", "cyclictest_max_us")},
                "unverified": [],
            }
        document["cyclictest"] = result
        document["verdict"] = verdict
        document["injection_command"] = command
        _dump(document, args.out)
        if verdict == "FAIL":
            print(
                f"FAIL: cyclictest Max {result['cyclictest_max_us']} us > "
                f"{CYCLICTEST_MAX_LATENCY_US} us（BQ-064 冻结判据）",
                file=sys.stderr,
            )
            return 1
        return 0

    interfaces = [] if args.no_can else (args.can_interfaces or list(DEFAULT_CAN_INTERFACES))
    args.can_interfaces = interfaces
    if args.dry_run:
        print(_capture_plan(args))
        return 0
    try:
        document = capture(
            window_s=args.window,
            cpu=args.cpu,
            pid=args.pid,
            process=args.process,
            can_interfaces=interfaces,
            diagnostics_first=(
                args.diagnostics_first.read_text(encoding="utf-8")
                if args.diagnostics_first else None
            ),
            diagnostics_last=(
                args.diagnostics_last.read_text(encoding="utf-8")
                if args.diagnostics_last else None
            ),
            runner=runner,
        )
    except (CaptureError, ParseError) as exc:
        print(f"CAPTURE-ERROR: {exc}", file=sys.stderr)
        return 2
    _dump(document, args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
