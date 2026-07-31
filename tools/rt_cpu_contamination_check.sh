#!/usr/bin/env bash
set -euo pipefail

rt_cpu="14"
mode="warn"
samples=10
interval="0.1"

usage()
{
  cat <<'EOF'
Usage: tools/rt_cpu_contamination_check.sh [--cpu 14] [--mode warn|strict] [--samples 10] [--interval 0.1]

Checks realtime SCHED_FIFO/SCHED_RR threads before rt-control starts.  A thread
is a hard violation when its current PSR is the rt-control CPU, or when it has
tight affinity that includes the rt-control CPU.  Broad default masks that still
list CPU14 are LOG-ONLY unless rtprio >= 80, because isolcpus does not remove
CPU14 from default Cpus_allowed_list masks.
EOF
}

contains_cpu()
{
  local list="$1"
  local cpu="$2"
  local part
  local first
  local last
  local old_ifs="${IFS}"
  IFS=,
  for part in ${list}; do
    case "${part}" in
      *-*)
        first="${part%-*}"
        last="${part#*-}"
        if (( cpu >= first && cpu <= last )); then
          IFS="${old_ifs}"
          return 0
        fi
        ;;
      "${cpu}")
        IFS="${old_ifs}"
        return 0
        ;;
    esac
  done
  IFS="${old_ifs}"
  return 1
}

cpu_count()
{
  local list="$1"
  local total=0
  local part
  local first
  local last
  local old_ifs="${IFS}"
  IFS=,
  for part in ${list}; do
    case "${part}" in
      *-*)
        first="${part%-*}"
        last="${part#*-}"
        total=$((total + last - first + 1))
        ;;
      "")
        ;;
      *)
        total=$((total + 1))
        ;;
    esac
  done
  IFS="${old_ifs}"
  printf '%s\n' "${total}"
}

is_whitelisted_rt_thread()
{
  local comm="$1"
  case "${comm}" in
    EtherCAT-OP|migration/*|idle_inject/*|irq_work/*|rcuc/*)
      return 0
      ;;
  esac
  return 1
}

while (($# > 0)); do
  case "$1" in
    --cpu)
      rt_cpu="$2"
      shift 2
      ;;
    --mode)
      mode="$2"
      shift 2
      ;;
    --samples)
      samples="$2"
      shift 2
      ;;
    --interval)
      interval="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

case "${mode}" in
  warn|strict) ;;
  *) usage >&2; exit 2 ;;
esac
[[ "${samples}" =~ ^[0-9]+$ ]] && (( samples >= 1 )) || {
  usage >&2
  exit 2
}

violations=0
warnings=0

scan_once()
{
  while read -r pid tid psr cls rtprio comm args; do
    [[ "${cls}" == "FF" || "${cls}" == "RR" ]] || continue
    if is_whitelisted_rt_thread "${comm}" "${rtprio}"; then
      continue
    fi
    status="/proc/${tid}/status"
    allowed="$(sed -n 's/^Cpus_allowed_list:[[:space:]]*//p' "${status}" 2>/dev/null || true)"
    [[ -n "${allowed}" ]] || continue

    if [[ "${psr}" == "${rt_cpu}" ]]; then
      printf 'VIOLATION: realtime thread on rt CPU PSR=%s pid=%s tid=%s cls=%s rtprio=%s comm=%s args=%s\n' \
        "${psr}" "${pid}" "${tid}" "${cls}" "${rtprio}" "${comm}" "${args}" >&2
      violations=$((violations + 1))
      continue
    fi

    if contains_cpu "${allowed}" "${rt_cpu}" &&
       (( "$(cpu_count "${allowed}")" <= 4 )); then
      printf 'VIOLATION: realtime thread has tight affinity including CPU%s pid=%s tid=%s allowed=%s cls=%s rtprio=%s comm=%s args=%s\n' \
        "${rt_cpu}" "${pid}" "${tid}" "${allowed}" "${cls}" "${rtprio}" "${comm}" "${args}" >&2
      violations=$((violations + 1))
      continue
    fi

    if contains_cpu "${allowed}" "${rt_cpu}"; then
      case "${args}" in
        *ToDesk*|*todesk*|*sunlogin*|*Sunlogin*|*rviz2*|*fastlio*|*alfa_nav*)
          if [[ "${rtprio}" =~ ^[0-9]+$ ]] && (( rtprio >= 80 )); then
            printf 'WARN: rtprio >= 80 non-rt-control thread has broad CPU%s mask but PSR is %s; LOG-ONLY unless it runs on CPU%s pid=%s tid=%s allowed=%s cls=%s rtprio=%s comm=%s args=%s\n' \
              "${rt_cpu}" "${psr}" "${rt_cpu}" "${pid}" "${tid}" "${allowed}" "${cls}" "${rtprio}" "${comm}" "${args}" >&2
            warnings=$((warnings + 1))
          else
            printf 'LOG-ONLY: broad mask includes CPU%s but current PSR=%s pid=%s tid=%s allowed=%s cls=%s rtprio=%s comm=%s args=%s\n' \
              "${rt_cpu}" "${psr}" "${pid}" "${tid}" "${allowed}" "${cls}" "${rtprio}" "${comm}" "${args}" >&2
          fi
          ;;
      esac
    fi
  done < <(ps -eLo pid=,tid=,psr=,cls=,rtprio=,comm=,args=)
}

for ((sample = 1; sample <= samples; sample++)); do
  scan_once
  if (( sample < samples )); then
    sleep "${interval}"
  fi
done

if (( violations > 0 )); then
  [[ "${mode}" == "strict" ]] && exit 1
fi

printf 'PASS: no non-whitelisted realtime thread executed on CPU%s across %s PSR samples; warnings=%s\n' \
  "${rt_cpu}" "${samples}" "${warnings}"
