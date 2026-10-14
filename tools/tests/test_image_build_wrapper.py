"""Exercise packaging arguments without invoking Docker or any device."""

import os
from pathlib import Path
import subprocess

import pytest


@pytest.mark.parametrize("candidate,accepted", [("3.2.0-0ubuntu2.1", True), ("4.0.0-1", False)])
def test_image_builder_requires_supported_candidate_and_passes_label_version(
    tmp_path, candidate, accepted
):
    docker = tmp_path / "docker"
    docker.write_text(
        "#!/bin/bash\n"
        'if [[ "$1" == run ]]; then printf "%s\\n" "$TEST_CANDIDATE"; exit; fi\n'
        'printf "%s\\n" "$@" > "$TEST_BUILD_ARGUMENTS"\n'
    )
    docker.chmod(0o755)
    arguments = tmp_path / "arguments"
    env = {
        **os.environ,
        "PATH": str(tmp_path) + os.pathsep + os.environ["PATH"],
        "TEST_CANDIDATE": candidate,
        "TEST_BUILD_ARGUMENTS": str(arguments),
    }
    root = Path(__file__).resolve().parents[2]
    result = subprocess.run(
        ["bash", str(root / "tools/build_rt_control_image.sh"), "test-only"],
        env=env,
        capture_output=True,
        text=True,
    )
    assert (result.returncode == 0) == accepted, result.stderr
    if accepted:
        args = arguments.read_text().splitlines()
        assert args[0] == "build"
        assert f"PYTHON3_JSONSCHEMA_VERSION={candidate}" in args
        assert "rt-control:test-only" in args
    else:
        assert not arguments.exists()
