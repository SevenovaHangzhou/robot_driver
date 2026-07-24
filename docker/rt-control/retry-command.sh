#!/usr/bin/env bash
set -uo pipefail

if (( $# == 0 )); then
  printf 'usage: %s COMMAND [ARG ...]\n' "$0" >&2
  exit 64
fi

readonly max_retries=5
retry_count=0

while true; do
  if "$@"; then
    exit 0
  else
    command_status=$?
  fi

  if (( retry_count >= max_retries )); then
    printf 'command failed after %d retries (exit %d):' \
      "$max_retries" "$command_status" >&2
    printf ' %q' "$@" >&2
    printf '\n' >&2
    exit "$command_status"
  fi

  (( retry_count += 1 ))
  printf 'command failed (exit %d); retry %d/%d in %d s:' \
    "$command_status" "$retry_count" "$max_retries" "$retry_count" >&2
  printf ' %q' "$@" >&2
  printf '\n' >&2
  sleep "$retry_count"
done
