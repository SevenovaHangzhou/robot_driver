#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

: "${RT_CONTROL_CPUSET:?Set RT_CONTROL_CPUSET only after target-host CPU topology validation}"

export RT_CONTROL_IMAGE_TAG
RT_CONTROL_IMAGE_TAG="$(git -C "${repository_root}" rev-parse --verify HEAD)"

cd "${repository_root}"
exec docker compose \
  --project-directory . \
  --env-file versions.env \
  -f docker/compose.yaml \
  "$@"
