# shellcheck shell=bash
# Sourced by verify-host.sh and can-install.sh.
#
# A legacy systemd unit is harmless while it cannot run: not active, not enabled at boot and
# not pulled in by any other unit. Its file may stay installed (user 2026-11-05).
# Prints the reason it could run, or nothing.
legacy_unit_problem() {
  local unit="$1"
  local state
  local dependents
  [[ -e "/etc/systemd/system/${unit}" ]] || return 0
  if systemctl is-active --quiet "${unit}"; then
    echo "${unit} is running"
    return 0
  fi
  state="$(systemctl is-enabled "${unit}" 2>/dev/null || true)"
  if [[ "${state}" != "disabled" && "${state}" != "masked" ]]; then
    echo "${unit} is ${state:-unknown} at boot"
    return 0
  fi
  dependents="$(systemctl list-dependencies --reverse --plain --no-legend "${unit}" 2>/dev/null |
    tail -n +2 | tr -d ' ' | paste -sd, -)"
  if [[ -n "${dependents}" ]]; then
    echo "${unit} is pulled in by ${dependents}"
  fi
}
