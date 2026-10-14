#!/usr/bin/env bash
# Manual packaging only. Never starts the production image or maps a device.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
if [[ $# != 1 || ! "$1" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$ ]]; then
  echo "Usage: tools/build_rt_control_image.sh APPROVED_TAG" >&2
  exit 2
fi
# shellcheck source=../versions.env
source "${repository_root}/versions.env"
base_image="$(awk '$1 == "FROM" {print $2; exit}' "${repository_root}/docker/rt-control/Dockerfile")"
# Use the same apt sources as the Dockerfile; do not pin the security patch.
jsonschema_version="$(docker run --rm --entrypoint /bin/bash "${base_image}" -euc '
  unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy ALL_PROXY all_proxy
  sed -i \
    -e "s|http://archive.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g" \
    -e "s|http://security.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g" \
    /etc/apt/sources.list
  apt-get update >&2
  apt-cache policy python3-jsonschema | awk '\''$1 == "Candidate:" {print $2}'\''
')"
if [[ ! "${jsonschema_version}" =~ ^3\.2\.[0-9][A-Za-z0-9.+:~-]*$ ]]; then
  echo "R10: expected a python3-jsonschema 3.2.x apt candidate, got: ${jsonschema_version}" >&2
  exit 1
fi
exec docker build --file "${repository_root}/docker/rt-control/Dockerfile" \
  --build-arg "IGH_VERSION=${IGH_VERSION}" \
  --build-arg "IGH_COMMIT=${IGH_COMMIT}" \
  --build-arg "PYTHON3_JSONSCHEMA_VERSION=${jsonschema_version}" \
  --tag "rt-control:$1" "${repository_root}"
