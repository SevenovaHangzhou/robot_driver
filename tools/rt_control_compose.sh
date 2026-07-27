#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
default_repository_root="$(cd -- "${script_dir}/.." && pwd)"
repository_root="${RT_CONTROL_PROJECT_ROOT:-${default_repository_root}}"

if [[ ! -d "${repository_root}" || ! -f "${repository_root}/docker/compose.yaml" ]]; then
  echo "RT_CONTROL_PROJECT_ROOT is not an rt-control release: ${repository_root}" >&2
  exit 2
fi
repository_root="$(cd -- "${repository_root}" && pwd)"

: "${RT_CONTROL_CPUSET:?Set RT_CONTROL_CPUSET only after target-host CPU topology validation}"

export RT_CONTROL_IMAGE_TAG
if git -C "${repository_root}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  RT_CONTROL_IMAGE_TAG="$(git -C "${repository_root}" rev-parse --verify HEAD)"
else
  : "${RT_CONTROL_IMAGE_TAG:?Set RT_CONTROL_IMAGE_TAG to the audited release SHA for a non-Git export}"
  if [[ ! "${RT_CONTROL_IMAGE_TAG}" =~ ^[0-9a-f]{40}$ ]]; then
    echo "RT_CONTROL_IMAGE_TAG must be a full 40-character lowercase Git SHA" >&2
    exit 2
  fi
fi

cd "${repository_root}"
exec docker compose \
  --project-directory . \
  --env-file versions.env \
  -f docker/compose.yaml \
  "$@"
