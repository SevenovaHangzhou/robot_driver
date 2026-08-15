#!/usr/bin/env python3
"""RT-Control 发布前测试 runner（ELECTRI-76 首版骨架）。

职责：读取 domains/rt_control/testing/cases/*.yaml，校验 schema，按门禁/层级选择
用例；采集发布身份六元组（TC-ST-04）；执行 T0–T2 且四个授权位全 no 的 auto 用例；
合并人工结果目录；产出 JSON 报告与人类可读摘要；为 T3/T4 生成人工执行卡；
对照 baseline 输出 delta。

安全边界（不可放宽）：
- 只允许自动执行 risk 属于 {T0,T1,T2} 且 writes 四位全 no 的用例；
- 绝不代替现场人员输入任何硬件授权确认短语；
- delta 判定阈值未裁决（ELECTRI-80），verdict 一律 REVIEW，不给 PASS/FAIL。
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

VALID_RISKS = ("T0", "T1", "T2", "T3", "T4")
AUTO_RISKS = ("T0", "T1", "T2")
VALID_STATUSES = ("automated", "manual", "planned", "blocked", "retired")
VALID_GATES = ("run", "reference", "no")
WRITE_KEYS = ("reset", "enable", "motion", "plc")
RESULT_STATUSES = ("PASS", "FAIL", "UNVERIFIED")
REQUIRED_FIELDS = (
    "id", "title", "purpose", "area", "risk", "writes", "env", "covers",
    "traces", "preconditions", "entry", "pass", "evidence", "cleanup",
    "status", "v010_gate",
)
FULL_SHA = re.compile(r"^[0-9a-f]{40}$")


class AutoExecutionRefused(RuntimeError):
    """请求自动执行一个不满足安全边界的用例。"""


# ---------------------------------------------------------------- catalog

def validate_case(case: dict, source: str) -> list[str]:
    errors = []
    for field in REQUIRED_FIELDS:
        if field not in case:
            errors.append(f"{source}: {case.get('id', '?')} missing field '{field}'")
    if case.get("risk") not in VALID_RISKS:
        errors.append(f"{source}: {case.get('id', '?')} invalid risk '{case.get('risk')}'")
    if case.get("status") not in VALID_STATUSES:
        errors.append(f"{source}: {case.get('id', '?')} invalid status '{case.get('status')}'")
    if case.get("v010_gate") not in VALID_GATES:
        errors.append(f"{source}: {case.get('id', '?')} invalid v010_gate '{case.get('v010_gate')}'")
    writes = case.get("writes")
    if not isinstance(writes, dict) or set(writes) != set(WRITE_KEYS):
        errors.append(f"{source}: {case.get('id', '?')} writes must have exactly keys {WRITE_KEYS}")
    return errors


def load_cases(cases_dir: Path) -> tuple[list[dict], list[str]]:
    cases, errors, seen = [], [], {}
    for path in sorted(Path(cases_dir).glob("*.yaml")):
        data = yaml.safe_load(path.read_text())
        if not isinstance(data, dict) or "cases" not in data:
            errors.append(f"{path.name}: no 'cases' list")
            continue
        for case in data["cases"]:
            case = dict(case)
            case.setdefault("category", data.get("category"))
            errors.extend(validate_case(case, source=path.name))
            cid = case.get("id")
            if cid in seen:
                errors.append(f"{path.name}: duplicate case id {cid} (also in {seen[cid]})")
            seen[cid] = path.name
            cases.append(case)
    return cases, errors


def select_cases(cases, gate=None, tiers=None, ids=None) -> list[dict]:
    out = []
    for case in cases:
        if case.get("status") == "retired":
            continue
        if gate == "v010" and case.get("v010_gate") not in ("run", "reference"):
            continue
        if tiers and case.get("risk") not in tiers:
            continue
        if ids and case.get("id") not in ids:
            continue
        out.append(case)
    return out


# ---------------------------------------------------------------- auto execution

def case_writes_anything(case: dict) -> bool:
    return any(bool(v) for v in case.get("writes", {}).values())


def can_auto_execute(case: dict) -> bool:
    return case.get("risk") in AUTO_RISKS and not case_writes_anything(case)


def run_auto(case: dict, evidence_dir: Path, execute: bool) -> dict | None:
    command = case.get("auto")
    if not command:
        return None
    if not can_auto_execute(case):
        raise AutoExecutionRefused(
            f"{case['id']}: risk={case.get('risk')} writes={case.get('writes')} "
            "violates the auto-execution boundary (T0-T2, all writes no)"
        )
    if not execute:
        return {"status": "UNVERIFIED", "detail": "auto available; run with --execute", "evidence": []}
    evidence_dir = Path(evidence_dir)
    evidence_dir.mkdir(parents=True, exist_ok=True)
    log_path = evidence_dir / f"{case['id']}.log"
    proc = subprocess.run(
        ["bash", "-c", command], capture_output=True, text=True
    )
    log_path.write_text(
        f"$ {command}\nexit={proc.returncode}\n--- stdout ---\n{proc.stdout}"
        f"\n--- stderr ---\n{proc.stderr}\n"
    )
    return {
        "status": "PASS" if proc.returncode == 0 else "FAIL",
        "detail": f"exit={proc.returncode}",
        "evidence": [str(log_path)],
    }


# ---------------------------------------------------------------- release identity

def _git(repo_root: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(repo_root), *args], capture_output=True, text=True, check=True
    ).stdout.strip()


def collect_release_identity(repo_root: Path, image: str | None = None):
    repo_root = Path(repo_root)
    findings: list[str] = []
    identity = {
        "source_commit": _git(repo_root, "rev-parse", "HEAD"),
        "source_dirty": bool(_git(repo_root, "status", "--porcelain")),
        "contract_version": None,
        "contract_commit": None,
        "vendor_commits": {},
        "igh_commit": None,
        "image_tag": image,
        "image_id": None,
        "image_digest": None,
        "image_igh_label": None,
    }
    if identity["source_dirty"]:
        findings.append("source working tree is dirty; identity is not release-grade")

    lock = repo_root / "src/interfaces/source-lock.yaml"
    if lock.exists():
        data = yaml.safe_load(lock.read_text())
        identity["contract_commit"] = data.get("commit")
        identity["contract_version"] = str(data.get("contract_version"))
        if not FULL_SHA.match(str(data.get("commit", ""))):
            findings.append("source-lock.yaml commit is not a full 40-hex SHA")
    else:
        findings.append("src/interfaces/source-lock.yaml missing")

    deps = repo_root / "deps.repos"
    if deps.exists():
        for name, repo in (yaml.safe_load(deps.read_text()) or {}).get("repositories", {}).items():
            version = str(repo.get("version", ""))
            identity["vendor_commits"][name] = version
            if not FULL_SHA.match(version):
                findings.append(f"deps.repos '{name}' version is not a full 40-hex SHA")
        robot_interfaces_commit = identity["vendor_commits"].get(
            "src/vendor/robot_interfaces"
        )
        if robot_interfaces_commit is None:
            findings.append("deps.repos is missing src/vendor/robot_interfaces")
        elif robot_interfaces_commit != identity["contract_commit"]:
            findings.append(
                "source-lock contract commit does not match "
                "deps.repos src/vendor/robot_interfaces"
            )
    else:
        findings.append("deps.repos missing")

    versions = repo_root / "versions.env"
    if versions.exists():
        for line in versions.read_text().splitlines():
            if line.startswith("IGH_COMMIT="):
                identity["igh_commit"] = line.split("=", 1)[1].strip()
        if not FULL_SHA.match(identity["igh_commit"] or ""):
            findings.append("versions.env IGH_COMMIT is not a full 40-hex SHA")
    else:
        findings.append("versions.env missing")

    if image:
        try:
            fmt = '{{.Id}}|{{index .Config.Labels "io.rt-control.igh.commit"}}|{{index .RepoDigests 0}}'
            out = subprocess.run(
                ["docker", "inspect", "--format", fmt, image],
                capture_output=True, text=True, check=True,
            ).stdout.strip()
            image_id, igh_label, digest = (out.split("|") + [None, None, None])[:3]
            identity.update(image_id=image_id, image_igh_label=igh_label, image_digest=digest)
            tag = image.split(":", 1)[1] if ":" in image else image
            if tag != identity["source_commit"]:
                findings.append("image tag does not equal the source commit SHA")
            if igh_label and igh_label != identity["igh_commit"]:
                findings.append("image IgH label does not match versions.env IGH_COMMIT")
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            findings.append(f"docker inspect failed for '{image}': {exc}")
    return identity, findings


# ---------------------------------------------------------------- results and report

def load_results(results_dir: Path | None) -> tuple[dict, list[str]]:
    results, errors = {}, []
    if not results_dir:
        return results, errors
    for path in sorted(Path(results_dir).glob("*.yaml")):
        data = yaml.safe_load(path.read_text()) or {}
        if data.get("status") not in RESULT_STATUSES:
            errors.append(f"{path.name}: status must be one of {RESULT_STATUSES}")
            continue
        results[path.stem] = data
    return results, errors


def evaluate_case(case: dict, result: dict | None, auto_outcome: dict | None) -> dict:
    row = {
        "id": case["id"],
        "title": case.get("title"),
        "category": case.get("category"),
        "area": case.get("area"),
        "risk": case.get("risk"),
        "writes": case.get("writes"),
        "v010_gate": case.get("v010_gate"),
        "case_status": case.get("status"),
        "status": "UNVERIFIED",
        "source": "missing",
        "evidence": [],
        "notes": None,
    }
    if auto_outcome and auto_outcome.get("evidence"):
        row.update(
            status=auto_outcome["status"], source="auto",
            evidence=auto_outcome["evidence"], notes=auto_outcome.get("detail"),
        )
        return row
    if result:
        if case.get("v010_gate") == "reference":
            ref = result.get("reference") or {}
            if not (ref.get("record") and ref.get("commit")):
                row["notes"] = "reference result must provide record coordinate and commit"
                return row
            row.update(
                status=result["status"], source="reference",
                evidence=result.get("evidence", []),
                notes=f"reference: {ref['record']} @ {ref['commit']}",
            )
            return row
        row.update(
            status=result["status"], source="result",
            evidence=result.get("evidence", []), notes=result.get("notes"),
        )
        return row
    if case.get("status") == "blocked":
        row["notes"] = "case is blocked by an open adjudication; see traces"
    elif auto_outcome:
        row["notes"] = auto_outcome.get("detail")
    return row


def gate_verdict(rows: list[dict]) -> str:
    gated = [r for r in rows if r.get("v010_gate") in ("run", "reference")]
    if any(r["status"] == "FAIL" for r in gated):
        return "FAIL"
    if any(r["status"] == "UNVERIFIED" for r in gated):
        return "INCOMPLETE"
    return "PASS"


def build_report(rows, identity, findings, gate=None) -> dict:
    statuses = [r["status"] for r in rows]
    return {
        "schema": "rt-control-release-test-report/v1",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "gate": gate,
        "release_identity": identity,
        "identity_findings": findings,
        "cases": rows,
        "summary": {
            "total": len(rows),
            "pass": statuses.count("PASS"),
            "fail": statuses.count("FAIL"),
            "unverified": statuses.count("UNVERIFIED"),
            "gate_verdict": gate_verdict(rows),
        },
    }


def render_summary(report: dict) -> str:
    s = report["summary"]
    lines = [
        "# RT-Control 发布测试报告摘要",
        "",
        f"- 发布身份 source commit: `{report['release_identity'].get('source_commit')}`",
        f"- 门禁: {report.get('gate') or '（未指定）'} → **{s['gate_verdict']}**",
        f"- PASS {s['pass']} / FAIL {s['fail']} / UNVERIFIED {s['unverified']}（共 {s['total']}）",
        "",
        "| 用例 | 状态 | 来源 | 证据 |",
        "| --- | --- | --- | --- |",
    ]
    for row in report["cases"]:
        evidence = "; ".join(row.get("evidence") or []) or "—"
        lines.append(f"| {row['id']} {row['title']} | {row['status']} | {row['source']} | {evidence} |")
    if report.get("identity_findings"):
        lines += ["", "## 发布身份异常", ""]
        lines += [f"- {f}" for f in report["identity_findings"]]
    return "\n".join(lines) + "\n"


def render_execution_cards(cases: list[dict]) -> str:
    lines = ["# T3/T4 人工执行卡", "",
             "以下用例需现场人工授权执行；runner 不代替输入任何硬件授权确认短语，",
             "确认短语以对应启动脚本的交互提示为准。结果回填 `results/<用例ID>.yaml`",
             "（字段：status/evidence/operator/date/notes，reference 用例另需 reference.record 与 reference.commit）。", ""]
    for case in cases:
        if case.get("risk") not in ("T3", "T4"):
            continue
        lines += [
            f"## {case['id']} {case.get('title')}",
            "",
            f"- 风险级: {case.get('risk')}；写动作: {case.get('writes')}",
            f"- 适用环境: {case.get('env')}{'（' + case['env_note'] + '）' if case.get('env_note') else ''}",
            f"- 前置条件: {case.get('preconditions')}",
            "- 入口:",
            "",
            "```bash",
            str(case.get("entry", "")).rstrip(),
            "```",
            "",
            "- 通过判据:",
        ]
        lines += [f"  - {p}" for p in case.get("pass", [])]
        lines += [
            f"- 证据要求: {case.get('evidence')}",
            f"- 回滚/清理: {case.get('cleanup')}",
            "- 本用例需现场人工授权执行。",
            "",
        ]
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------- baseline delta

def compute_delta(baseline: dict, candidate: dict, thresholds: dict | None = None) -> list[dict]:
    rows = []
    base_metrics = baseline.get("metrics", {}) or {}
    cand_metrics = candidate.get("metrics", {}) or {}
    limits = (thresholds or {}).get("metrics", {}) or {}
    zero_review = set((thresholds or {}).get("review_if_nonzero", []) or [])

    def verdict_for(metric: str, value: float) -> str:
        if metric in zero_review:
            return "REVIEW" if value != 0 else "OK"
        if metric in limits:
            return "FAIL" if value > limits[metric]["fail_above"] else "OK"
        return "REVIEW"

    for metric, base_value in base_metrics.items():
        if metric not in cand_metrics:
            rows.append({"metric": metric, "baseline": base_value, "candidate": None,
                         "delta": None, "delta_pct": None, "verdict": "MISSING"})
            continue
        cand_value = cand_metrics[metric]
        delta = cand_value - base_value
        pct = (delta / base_value * 100.0) if base_value else None
        rows.append({"metric": metric, "baseline": base_value, "candidate": cand_value,
                     "delta": delta, "delta_pct": pct,
                     "verdict": verdict_for(metric, cand_value)})
    for metric, cand_value in cand_metrics.items():
        if metric not in base_metrics:
            rows.append({"metric": metric, "baseline": None, "candidate": cand_value,
                         "delta": None, "delta_pct": None,
                         "verdict": verdict_for(metric, cand_value) if metric in zero_review or metric in limits else "NEW"})
    return rows


# ---------------------------------------------------------------- CLI

def _add_selection(parser):
    parser.add_argument("--gate", choices=["v010"], default=None)
    parser.add_argument("--tier", default=None, help="逗号分隔，如 T0,T1")
    parser.add_argument("--id", dest="ids", action="append", default=None)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument(
        "--cases-dir", type=Path, default=Path("domains/rt_control/testing/cases")
    )
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("validate")
    plan = sub.add_parser("plan")
    _add_selection(plan)
    plan.add_argument("--cards", type=Path, default=None)
    ident = sub.add_parser("identity")
    ident.add_argument("--image", default=None)
    run = sub.add_parser("run")
    _add_selection(run)
    run.add_argument("--image", default=None)
    run.add_argument("--results-dir", type=Path, default=None)
    run.add_argument("--evidence-dir", type=Path, default=Path("test-evidence"))
    run.add_argument("--execute", action="store_true")
    run.add_argument("--out", type=Path, default=Path("release-test-report.json"))
    run.add_argument("--summary", type=Path, default=None)
    delta = sub.add_parser("delta")
    delta.add_argument("--baseline", type=Path, required=True)
    delta.add_argument("--candidate", type=Path, required=True)
    delta.add_argument("--thresholds", type=Path, default=None)
    delta.add_argument("--out", type=Path, default=None)
    args = parser.parse_args(argv)

    cases, errors = load_cases(args.repo_root / args.cases_dir)
    if errors:
        for error in errors:
            print(f"CASE-ERROR: {error}", file=sys.stderr)
        if args.command == "validate":
            return 1
    if args.command == "validate":
        print(f"OK: {len(cases)} cases valid")
        return 0

    if args.command == "identity":
        identity, findings = collect_release_identity(args.repo_root, image=args.image)
        print(json.dumps({"release_identity": identity, "findings": findings},
                         indent=2, ensure_ascii=False))
        return 1 if findings else 0

    if args.command == "delta":
        rows = compute_delta(
            yaml.safe_load(args.baseline.read_text()),
            yaml.safe_load(args.candidate.read_text()),
            thresholds=yaml.safe_load(args.thresholds.read_text()) if args.thresholds else None,
        )
        payload = json.dumps({"delta": rows}, indent=2, ensure_ascii=False)
        (args.out.write_text(payload) if args.out else print(payload))
        return 1 if any(r["verdict"] == "FAIL" for r in rows) else 0

    tiers = args.tier.split(",") if args.tier else None
    selected = select_cases(cases, gate=args.gate, tiers=tiers, ids=args.ids)

    if args.command == "plan":
        for case in selected:
            print(f"{case['id']}\t{case['risk']}\t{case['status']}\t{case['v010_gate']}\t{case['title']}")
        cards = render_execution_cards(selected)
        if args.cards:
            args.cards.write_text(cards)
            print(f"cards -> {args.cards}")
        return 0

    # run
    identity, findings = collect_release_identity(args.repo_root, image=args.image)
    results, result_errors = load_results(args.results_dir)
    for error in result_errors:
        print(f"RESULT-ERROR: {error}", file=sys.stderr)
    rows = []
    for case in selected:
        auto_outcome = None
        if case.get("auto") and can_auto_execute(case) and case.get("status") != "blocked":
            auto_outcome = run_auto(case, args.evidence_dir, execute=args.execute)
        rows.append(evaluate_case(case, results.get(case["id"]), auto_outcome))
    report = build_report(rows, identity, findings, gate=args.gate)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    summary = render_summary(report)
    if args.summary:
        args.summary.write_text(summary)
    print(summary)
    print(f"report -> {args.out}")
    return 0 if report["summary"]["gate_verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
