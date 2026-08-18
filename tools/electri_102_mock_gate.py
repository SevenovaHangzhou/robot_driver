#!/usr/bin/env python3
"""Run the no-hardware ELECTRI-102 rolling-control acceptance matrix.

The caller must source the ROS and robot_interfaces overlays used to build the
test binaries.  This runner never starts a controller manager, enables a drive,
or touches a bus.  It executes deterministic GoogleTest scenarios against the
fake 250 Hz controller loop and preserves JSON, JUnit, metrics, and text logs.
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FAKE_UPDATE_RATE_HZ = 250


@dataclass(frozen=True)
class Scenario:
    """One independently reported requirement-to-test binding."""

    id: str
    title: str
    requirement: str
    package: str
    binary: str
    gtest_filter: str
    soak: bool = False


def scenario_catalog() -> tuple[Scenario, ...]:
    """Return the frozen ELECTRI-102 mock acceptance catalog."""

    rolling = "rolling_trajectory_controller"
    return (
        Scenario(
            "frequency_jitter",
            "bounded publisher/control-period jitter",
            "frequency jitter",
            rolling,
            "test_rt_update",
            "RtUpdateTest.BoundedPeriodJitterAdvancesWithoutClockAnomaly",
        ),
        Scenario(
            "update_interruption_recovery",
            "update timeout followed by an explicit fresh session",
            "update interruption and recovery",
            rolling,
            "test_rt_update",
            "RtUpdateTest.TimedOutSessionCanFinalizeAndOpenAFreshSession",
        ),
        Scenario(
            "out_of_order_sequence",
            "deterministic randomized arrival order",
            "out-of-order sequence",
            rolling,
            "test_protocol_vectors",
            "ProtocolVectors.RandomArrivalOrderMatchesTheSequenceReferenceModel",
        ),
        Scenario(
            "duplicate_sequence",
            "duplicate sequence cannot mutate the accepted candidate",
            "duplicate sequence",
            rolling,
            "test_protocol_vectors",
            "ProtocolVectors.DuplicateSequenceIsStaleAndCannotMutatePendingTrajectory",
        ),
        Scenario(
            "late_replacement",
            "late suffix rejection consumes sequence but not trajectory",
            "late replacement",
            rolling,
            "test_protocol_vectors",
            "ProtocolVectors.LateSuffixIsRejectedAfterSequenceConsumption",
        ),
        Scenario(
            "capacity_exhaustion",
            "prefix plus suffix cannot exceed runtime capacity",
            "capacity exhaustion",
            rolling,
            "test_protocol_vectors",
            "ProtocolVectors.PrefixPlusSuffixCannotExceedRuntimeCapacity",
        ),
        Scenario(
            "low_water",
            "low-water equality stops before exhaustion",
            "low water",
            rolling,
            "test_rt_update",
            "RtUpdateTest.LowWaterEqualityStopsBeforeTheShortBufferCanExhaust",
        ),
        Scenario(
            "prime_timeout",
            "prime timeout preserves the original hold",
            "prime timeout",
            rolling,
            "test_rt_update",
            "RtUpdateTest.PrimeTimeoutStopsAtTheOriginalHold",
        ),
        Scenario(
            "clock_anomaly",
            "bad and non-positive periods latch clock anomaly",
            "clock anomaly",
            rolling,
            "test_rt_update",
            "RtUpdateTest.ClockAnomalyUsesNominalStepsInsteadOfTheBadPeriod:"
            "RtUpdateTest.NonPositivePeriodsLatchTheSameClockAnomaly",
        ),
        Scenario(
            "exit_realtime_mode",
            "graceful close stops continuously and rejects future updates",
            "exit realtime mode",
            rolling,
            "test_rt_update",
            "RtUpdateTest.MovingGracefulCloseStopsContinuouslyAndRejectsLaterUpdates",
        ),
        Scenario(
            "session_during_disable",
            "deactivation invalidates the current boot and rolling session",
            "session during disable",
            rolling,
            "test_controller_callbacks",
            "ControllerCallbacksTest.DeactivateInvalidatesOldBootAndSession",
        ),
        Scenario(
            "mode_switch_safety",
            "verified STRICT switch and group-fault preemption",
            "mode switch safety",
            "enable_manager",
            "test_enable_manager_state_machine",
            "EnableManagerFixture.ModeServiceExecutesVerifiedStrictSwitch:"
            "EnableManagerFixture.GroupFaultPreemptsModeAdmissionWithoutClobberingSafetyOwner",
        ),
        Scenario(
            "rt_zero_allocation",
            "RT update path performs no dynamic allocation",
            "zero RT allocation",
            rolling,
            "test_rt_update",
            "RtUpdateTest.UpdatePathDoesNotAllocate",
        ),
        Scenario(
            "continuous_update_soak",
            "10 Hz knots in 30 Hz batches remain stable in a fake 250 Hz loop",
            "continuous update and ten-minute soak",
            rolling,
            "test_rt_update",
            "RtUpdateTest.TenHertzKnotsAtThirtyHertzRemainStableAtFakeTwoHundredFiftyHertz",
            soak=True,
        ),
    )


def select_scenarios(requested_ids: Sequence[str] | None) -> list[Scenario]:
    catalog = scenario_catalog()
    if not requested_ids:
        return list(catalog)
    by_id = {scenario.id: scenario for scenario in catalog}
    unknown = sorted(set(requested_ids) - set(by_id))
    if unknown:
        raise ValueError("unknown scenario(s): " + ", ".join(unknown))
    requested = set(requested_ids)
    return [scenario for scenario in catalog if scenario.id in requested]


def build_command(
    scenario: Scenario,
    build_root: Path,
    evidence_directory: Path,
    *,
    soak_seconds: int,
    realtime: bool,
) -> tuple[list[str], dict[str, str]]:
    if soak_seconds <= 0:
        raise ValueError("soak_seconds must be positive")
    executable = Path(build_root) / scenario.package / scenario.binary
    junit_path = Path(evidence_directory) / f"{scenario.id}.junit.xml"
    command = [
        str(executable),
        "--gtest_filter=" + scenario.gtest_filter,
        "--gtest_output=xml:" + str(junit_path),
    ]
    environment: dict[str, str] = {}
    if scenario.soak:
        environment = {
            "E102_SOAK_CYCLES": str(soak_seconds * FAKE_UPDATE_RATE_HZ),
            "E102_SOAK_REALTIME": "1" if realtime else "0",
            "E102_SOAK_REPORT": str(
                Path(evidence_directory) / f"{scenario.id}.metrics.json"
            ),
        }
    return command, environment


def validate_soak_metrics(
    metrics: Mapping[str, object],
    *,
    expected_cycles: int | None = None,
    expected_realtime: bool | None = None,
) -> list[str]:
    """Return fail-closed findings for the soak report."""

    findings: list[str] = []
    required = (
        "passed",
        "cycles_requested",
        "cycles_completed",
        "late_replace_count",
        "rt_allocation_count",
        "invariant_failure_count",
        "rejected_batches",
        "published_batches",
        "accepted_batches",
        "max_point_count",
        "knot_interval_ns",
        "batch_rate_hz",
        "update_rate_hz",
        "realtime",
    )
    for field in required:
        if field not in metrics:
            findings.append(f"missing metric '{field}'")
    if findings:
        return findings

    if metrics["passed"] is not True:
        findings.append("passed is not true")
    requested = metrics["cycles_requested"]
    completed = metrics["cycles_completed"]
    if not isinstance(requested, int) or requested <= 0:
        findings.append(f"cycles_requested={requested}")
    if completed != requested:
        findings.append(f"cycles_completed={completed}, requested={requested}")
    if expected_cycles is not None and requested != expected_cycles:
        findings.append(f"cycles_requested={requested}, expected={expected_cycles}")
    if expected_realtime is not None and metrics["realtime"] is not expected_realtime:
        findings.append(
            f"realtime={metrics['realtime']}, expected={expected_realtime}"
        )
    for counter in (
        "late_replace_count",
        "rt_allocation_count",
        "invariant_failure_count",
        "rejected_batches",
    ):
        if metrics[counter] != 0:
            findings.append(f"{counter}={metrics[counter]}")
    if metrics["accepted_batches"] != metrics["published_batches"]:
        findings.append(
            "accepted_batches="
            f"{metrics['accepted_batches']}, published_batches={metrics['published_batches']}"
        )
    if not isinstance(metrics["max_point_count"], int) or not (
        1 <= metrics["max_point_count"] <= 64
    ):
        findings.append(f"max_point_count={metrics['max_point_count']}")
    expected_values = {
        "knot_interval_ns": 100_000_000,
        "batch_rate_hz": 30,
        "update_rate_hz": FAKE_UPDATE_RATE_HZ,
    }
    for field, expected in expected_values.items():
        if metrics[field] != expected:
            findings.append(f"{field}={metrics[field]}, expected={expected}")
    return findings


def validate_junit(junit_path: Path) -> list[str]:
    """Reject missing, malformed, empty, or failing GoogleTest XML."""

    if not junit_path.is_file():
        return [f"missing JUnit: {junit_path}"]
    try:
        root = ET.parse(junit_path).getroot()
    except (ET.ParseError, OSError) as error:
        return [f"invalid JUnit: {error}"]
    test_cases = root.findall(".//testcase")
    if not test_cases:
        return ["JUnit executed zero tests"]
    failures = root.findall(".//failure")
    errors = root.findall(".//error")
    findings = []
    if failures:
        findings.append(f"JUnit failures={len(failures)}")
    if errors:
        findings.append(f"JUnit errors={len(errors)}")
    return findings


def run_scenario(
    scenario: Scenario,
    build_root: Path,
    evidence_directory: Path,
    *,
    soak_seconds: int,
    realtime: bool,
    inherited_environment: Mapping[str, str],
) -> dict:
    evidence_directory = Path(evidence_directory)
    evidence_directory.mkdir(parents=True, exist_ok=True)
    command, overrides = build_command(
        scenario,
        build_root,
        evidence_directory,
        soak_seconds=soak_seconds,
        realtime=realtime,
    )
    executable = Path(command[0])
    log_path = evidence_directory / f"{scenario.id}.log"
    junit_path = evidence_directory / f"{scenario.id}.junit.xml"
    metrics_path = evidence_directory / f"{scenario.id}.metrics.json"
    base_result = {
        "id": scenario.id,
        "title": scenario.title,
        "requirement": scenario.requirement,
        "status": "FAIL",
        "exit_code": None,
        "duration_ms": 0,
        "detail": "",
        "command": command,
        "environment_overrides": overrides,
        "artifacts": [],
    }
    if not executable.is_file() or not os.access(executable, os.X_OK):
        base_result["detail"] = f"missing executable: {executable}"
        return base_result

    environment = dict(inherited_environment)
    environment.update(overrides)
    started = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            env=environment,
            timeout=(soak_seconds + 120 if scenario.soak and realtime else 120),
            check=False,
        )
        elapsed_ms = round((time.monotonic() - started) * 1000)
        base_result["exit_code"] = completed.returncode
        base_result["duration_ms"] = elapsed_ms
        log_path.write_text(
            "$ "
            + shlex.join(command)
            + "\n"
            + f"exit={completed.returncode}\n"
            + "--- stdout ---\n"
            + completed.stdout
            + "\n--- stderr ---\n"
            + completed.stderr
        )
        base_result["artifacts"].append(log_path.name)
        if junit_path.is_file():
            base_result["artifacts"].append(junit_path.name)
        if completed.returncode != 0:
            base_result["detail"] = f"gtest exit={completed.returncode}"
            return base_result
    except subprocess.TimeoutExpired as error:
        elapsed_ms = round((time.monotonic() - started) * 1000)
        base_result["duration_ms"] = elapsed_ms
        log_path.write_text(
            "$ "
            + shlex.join(command)
            + "\nTIMEOUT\n--- stdout ---\n"
            + (error.stdout or "")
            + "\n--- stderr ---\n"
            + (error.stderr or "")
        )
        base_result["artifacts"].append(log_path.name)
        base_result["detail"] = "scenario timeout"
        return base_result

    junit_findings = validate_junit(junit_path)
    if junit_findings:
        base_result["detail"] = "; ".join(junit_findings)
        return base_result

    if scenario.soak:
        if not metrics_path.is_file():
            base_result["detail"] = f"missing soak metrics: {metrics_path}"
            return base_result
        try:
            metrics = json.loads(metrics_path.read_text())
        except (json.JSONDecodeError, OSError) as error:
            base_result["detail"] = f"invalid soak metrics: {error}"
            return base_result
        base_result["metrics"] = metrics
        base_result["artifacts"].append(metrics_path.name)
        findings = validate_soak_metrics(
            metrics,
            expected_cycles=soak_seconds * FAKE_UPDATE_RATE_HZ,
            expected_realtime=realtime,
        )
        if findings:
            base_result["detail"] = "; ".join(findings)
            return base_result

    base_result["status"] = "PASS"
    base_result["detail"] = "all assertions passed"
    return base_result


def build_summary(results: Sequence[dict], source_commit: str) -> dict:
    passed = sum(result["status"] == "PASS" for result in results)
    failed = sum(result["status"] != "PASS" for result in results)
    return {
        "schema_version": 1,
        "requirement": "ELECTRI-102",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_commit": source_commit,
        "source_dirty": bool(_git("status", "--porcelain")),
        "verdict": "PASS" if failed == 0 and results else "FAIL",
        "counts": {"pass": passed, "fail": failed},
        "results": list(results),
    }


def write_aggregate_junit(results: Sequence[dict], output_path: Path) -> None:
    failures = sum(result["status"] != "PASS" for result in results)
    total_seconds = sum(result.get("duration_ms", 0) for result in results) / 1000.0
    suite = ET.Element(
        "testsuite",
        {
            "name": "ELECTRI-102 mock acceptance",
            "tests": str(len(results)),
            "failures": str(failures),
            "errors": "0",
            "time": f"{total_seconds:.3f}",
        },
    )
    for result in results:
        case = ET.SubElement(
            suite,
            "testcase",
            {
                "classname": "electri_102.mock_gate",
                "name": result["id"],
                "time": f"{result.get('duration_ms', 0) / 1000.0:.3f}",
            },
        )
        if result["status"] != "PASS":
            failure = ET.SubElement(case, "failure", {"message": result.get("detail", "")})
            failure.text = result.get("detail", "")
    ET.ElementTree(suite).write(output_path, encoding="utf-8", xml_declaration=True)


def _git(*arguments: str) -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(REPOSITORY_ROOT), *arguments],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (FileNotFoundError, subprocess.CalledProcessError):
        return "unknown"


def parse_arguments(arguments: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--build-root",
        type=Path,
        required=True,
        help="colcon build base containing <package>/<test_binary>",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="new or empty evidence directory",
    )
    parser.add_argument(
        "--soak-seconds",
        type=int,
        default=600,
        help="wall/simulated seconds for the continuous-update scenario (default: 600)",
    )
    parser.add_argument(
        "--accelerated",
        action="store_true",
        help="do not pace the fake 250 Hz loop against wall time",
    )
    parser.add_argument(
        "--scenario",
        action="append",
        dest="scenarios",
        help="run only this scenario id; may be repeated",
    )
    parser.add_argument("--list", action="store_true", help="list scenario ids and exit")
    return parser.parse_args(arguments)


def main(arguments: Sequence[str] | None = None) -> int:
    options = parse_arguments(arguments)
    if options.list:
        for scenario in scenario_catalog():
            print(f"{scenario.id}: {scenario.title}")
        return 0
    if options.soak_seconds <= 0:
        print("ERROR: --soak-seconds must be positive", file=sys.stderr)
        return 2
    try:
        scenarios = select_scenarios(options.scenarios)
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    output_directory = options.output_dir.resolve()
    if output_directory.exists() and (
        not output_directory.is_dir() or any(output_directory.iterdir())
    ):
        print(
            f"ERROR: --output-dir must be empty: {output_directory}",
            file=sys.stderr,
        )
        return 2
    output_directory.mkdir(parents=True, exist_ok=True)
    results = []
    for scenario in scenarios:
        print(f"[{scenario.id}] {scenario.title}", flush=True)
        result = run_scenario(
            scenario,
            options.build_root.resolve(),
            output_directory,
            soak_seconds=options.soak_seconds,
            realtime=not options.accelerated,
            inherited_environment=os.environ,
        )
        results.append(result)
        print(f"  {result['status']}: {result['detail']}", flush=True)

    summary = build_summary(results, _git("rev-parse", "HEAD"))
    summary_path = output_directory / "electri-102-mock-gate.json"
    junit_path = output_directory / "electri-102-mock-gate.junit.xml"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    write_aggregate_junit(results, junit_path)
    print(f"summary: {summary_path}")
    print(f"junit: {junit_path}")
    print(f"verdict: {summary['verdict']}")
    return 0 if summary["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
