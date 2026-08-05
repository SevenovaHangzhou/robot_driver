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

Checks realtime SCHED_FIFO/SCHED_RR threads before rt-control starts. A
non-whitelisted thread is a hard violation when its current PSR is the
rt-control CPU, or when it has tight affinity that includes the rt-control CPU.
Broad default masks that still list CPU14 are summarized as one WARN line for
rtprio >= 80, because isolcpus does not remove CPU14 from default
Cpus_allowed_list masks.
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
    EtherCAT-OP|migration/*|idle_inject/*|irq_work/*|rcuc/*|rcub/*)
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

declare -A violation_records=()
declare -A warn_records=()

record_violation()
{
  local key="$1"
  shift
  if [[ -z "${violation_records[${key}]:-}" ]]; then
    violation_records["${key}"]="$*"
  fi
}

record_warning()
{
  local key="$1"
  shift
  if [[ -z "${warn_records[${key}]:-}" ]]; then
    warn_records["${key}"]="$*"
  fi
}

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
      record_violation "${tid}:psr" \
        "realtime thread on rt CPU PSR=${psr} pid=${pid} tid=${tid} cls=${cls} rtprio=${rtprio} comm=${comm} args=${args}"
      continue
    fi

    if contains_cpu "${allowed}" "${rt_cpu}" &&
       (( "$(cpu_count "${allowed}")" <= 4 )); then
      record_violation "${tid}:tight" \
        "realtime thread has tight affinity including CPU${rt_cpu} pid=${pid} tid=${tid} allowed=${allowed} cls=${cls} rtprio=${rtprio} comm=${comm} args=${args}"
      continue
    fi

    if contains_cpu "${allowed}" "${rt_cpu}" &&
       [[ "${rtprio}" =~ ^[0-9]+$ ]] && (( rtprio >= 80 )); then
      record_warning "${tid}:broad" \
        "pid=${pid} tid=${tid} current_psr=${psr} allowed=${allowed} cls=${cls} rtprio=${rtprio} comm=${comm} args=${args}"
    fi
  done < <(ps -eLo pid=,tid=,psr=,cls=,rtprio=,comm=,args=)
}

print_records()
{
  local label="$1"
  local limit="$2"
  shift 2
  local -n records_ref="$1"
  local count="${#records_ref[@]}"
  local extra=0
  local key
  local shown=0

  (( count > 0 )) || return
  printf '%s' "${label}" >&2
  for key in "${!records_ref[@]}"; do
    if (( shown >= limit )); then
      extra=$((extra + 1))
      continue
    fi
    printf '%s%s' "$([[ "${shown}" -eq 0 ]] && printf '' || printf '; ')" "${records_ref[${key}]}" >&2
    shown=$((shown + 1))
  done
  if (( extra > 0 )); then
    printf '; ... %s more' "${extra}" >&2
  fi
  printf '\n' >&2
}

for ((sample = 1; sample <= samples; sample++)); do
  scan_once
  if (( sample < samples )); then
    sleep "${interval}"
  fi
done

violations="${#violation_records[@]}"
warnings="${#warn_records[@]}"

if (( violations > 0 )); then
  print_records "VIOLATION: " 8 violation_records
  [[ "${mode}" == "strict" ]] && exit 1
fi

if (( warnings > 0 )); then
  print_records "WARN: rtprio >= 80 non-rt-control realtime thread(s) have broad CPU${rt_cpu} masks but were not observed running on CPU${rt_cpu}; threads: " 5 warn_records
fi

printf 'PASS: no non-whitelisted realtime thread executed on CPU%s across %s PSR samples; warnings=%s\n' \
  "${rt_cpu}" "${samples}" "${warnings}"
