#!/usr/bin/env bash
set -euo pipefail

readonly docker_fingerprint="9DC858229FC7DD38854AE2D88D81803C0EBFCD88"
readonly docker_ce_version="5:29.6.2-1~ubuntu.22.04~jammy"
readonly containerd_version="2.2.6-1~ubuntu.22.04~jammy"
readonly buildx_version="0.35.0-1~ubuntu.22.04~jammy"
readonly compose_version="5.3.1-1~ubuntu.22.04~jammy"

if [[ ${EUID} -ne 0 ]]; then
  echo "run this script as root" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "jammy" ]] ||
   [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
  echo "this frozen package set is only approved for Ubuntu 22.04 amd64" >&2
  exit 1
fi

for conflicting_package in \
  docker.io docker-compose docker-compose-v2 podman-docker containerd runc; do
  if dpkg-query -W -f='${db:Status-Abbrev}' "${conflicting_package}" 2>/dev/null |
     grep -q '^ii '; then
    echo "remove or adjudicate conflicting package ${conflicting_package} first" >&2
    exit 1
  fi
done

declare -A expected_versions=(
  [docker-ce]="${docker_ce_version}"
  [docker-ce-cli]="${docker_ce_version}"
  [docker-ce-rootless-extras]="${docker_ce_version}"
  [containerd.io]="${containerd_version}"
  [docker-buildx-plugin]="${buildx_version}"
  [docker-compose-plugin]="${compose_version}"
)
for package in "${!expected_versions[@]}"; do
  installed_version="$(dpkg-query -W -f='${Version}' "${package}" 2>/dev/null || true)"
  if [[ -n "${installed_version}" && "${installed_version}" != "${expected_versions[${package}]}" ]]; then
    echo "${package} is ${installed_version}, expected ${expected_versions[${package}]}" >&2
    exit 1
  fi
done

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends --no-upgrade ca-certificates curl gnupg

key_tmp=""
cleanup() {
  if [[ -n "${key_tmp}" ]]; then
    rm -f -- "${key_tmp}"
  fi
}
trap cleanup EXIT
key_source="/etc/apt/keyrings/docker.asc"
if [[ ! -r "${key_source}" ]]; then
  key_tmp="$(mktemp)"
  curl --proto '=https' --tlsv1.2 --fail --show-error --silent --location \
    https://download.docker.com/linux/ubuntu/gpg > "${key_tmp}"
  key_source="${key_tmp}"
fi
actual_fingerprint="$(gpg --show-keys --with-colons "${key_source}" |
  sed -n 's/^fpr:::::::::\([^:]*\):/\1/p' | head -n 1)"
if [[ "${actual_fingerprint}" != "${docker_fingerprint}" ]]; then
  echo "Docker signing-key fingerprint mismatch: ${actual_fingerprint}" >&2
  exit 1
fi
install -d -m 0755 /etc/apt/keyrings
if [[ "${key_source}" != "/etc/apt/keyrings/docker.asc" ]]; then
  install -o root -g root -m 0644 "${key_source}" /etc/apt/keyrings/docker.asc
fi

repository_tmp="$(mktemp)"
printf '%s\n' \
  'deb [arch=amd64 signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu jammy stable' \
  > "${repository_tmp}"
install -o root -g root -m 0644 "${repository_tmp}" /etc/apt/sources.list.d/docker.list
rm -f -- "${repository_tmp}"

apt-get update
apt-get install -y --no-install-recommends \
  "containerd.io=${containerd_version}" \
  "docker-buildx-plugin=${buildx_version}" \
  "docker-ce=${docker_ce_version}" \
  "docker-ce-cli=${docker_ce_version}" \
  "docker-ce-rootless-extras=${docker_ce_version}" \
  "docker-compose-plugin=${compose_version}"
apt-mark hold \
  containerd.io docker-buildx-plugin docker-ce docker-ce-cli \
  docker-ce-rootless-extras docker-compose-plugin
systemctl enable --now containerd.service docker.service

test "$(dpkg-query -W -f='${Version}' docker-ce)" = "${docker_ce_version}"
test "$(dpkg-query -W -f='${Version}' containerd.io)" = "${containerd_version}"
docker version
docker compose version

echo "Docker installed and held; no user was added to the docker group"
