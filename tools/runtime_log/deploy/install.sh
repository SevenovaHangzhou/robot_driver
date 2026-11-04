#!/usr/bin/env bash
# Install or update the user service from this checkout (run on the target, as the robot user).
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
[[ -f "$here/runtime_log/__main__.py" && -f "$here/profiles/rt_control.yaml" ]] || { echo "incomplete Runtime-LOG installation: $here" >&2; exit 1; }
python3 -c 'import yaml' || { echo "python3-yaml missing" >&2; exit 1; }
command -v ethercat >/dev/null || echo "WARN: ethercat CLI not found; master polling will record errors only"
mkdir -p "$HOME/.config/systemd/user" "$HOME/runtime-log/runs"
python3 - "$here" "$HOME/.config/systemd/user/runtime-log.service" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1]).resolve()
if '\n' in str(root) or '\r' in str(root):
    raise ValueError('Runtime-LOG path must not contain a newline')
escaped = str(root).replace('%', '%%')
template = (root / 'systemd/runtime-log.service').read_text()
Path(sys.argv[2]).write_text(template.replace('@RUNTIME_LOG_ROOT@', escaped))
PY
[[ -f "$HOME/.config/runtime-log.env" ]] || printf 'RUNTIME_LOG_SERVE_ARGS=\n' > "$HOME/.config/runtime-log.env"
systemctl --user daemon-reload
if [[ "$(loginctl show-user "$USER" -p Linger --value)" != "yes" ]]; then
  echo "NOTE: linger is off; run once: sudo loginctl enable-linger $USER (otherwise the service stops at logout)"
fi
echo "installed; start with: systemctl --user enable --now runtime-log"
