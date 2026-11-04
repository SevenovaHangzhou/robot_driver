"""The compose wrapper sets a safe ROS domain before it calls docker."""

import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
COMPOSE_WRAPPER = ROOT / "tools/rt_control_compose.sh"


def _run_compose_wrapper(tmp_path: Path, domain_id: str | None) -> subprocess.CompletedProcess[str]:
    fake_docker = tmp_path / "docker"
    fake_docker.write_text(
        "#!/usr/bin/env bash\n"
        "printf 'domain=%s\\n' \"${RT_CONTROL_ROS_DOMAIN_ID-unset}\"\n"
        "printf 'args=%s\\n' \"$*\"\n",
        encoding="utf-8",
    )
    fake_docker.chmod(0o755)

    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{tmp_path}:{environment['PATH']}",
            "RT_CONTROL_CPUSET": "14",
            "RT_CONTROL_IMAGE_TAG": "domain-contract-test",
            "RT_CONTROL_PROJECT_ROOT": str(ROOT),
        }
    )
    if domain_id is None:
        environment.pop("RT_CONTROL_ROS_DOMAIN_ID", None)
    else:
        environment["RT_CONTROL_ROS_DOMAIN_ID"] = domain_id

    return subprocess.run(
        [str(COMPOSE_WRAPPER), "config"],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )


def test_compose_wrapper_defaults_ros_domain_to_zero(tmp_path: Path) -> None:
    result = _run_compose_wrapper(tmp_path, None)

    assert result.returncode == 0
    assert "domain=0" in result.stdout


def test_compose_wrapper_accepts_explicit_safe_ros_domain(tmp_path: Path) -> None:
    result = _run_compose_wrapper(tmp_path, "12")

    assert result.returncode == 0
    assert "domain=12" in result.stdout


def test_compose_wrapper_rejects_invalid_ros_domains_before_docker(tmp_path: Path) -> None:
    for invalid_domain in ("-1", "08", "233", "not-a-number"):
        result = _run_compose_wrapper(tmp_path, invalid_domain)

        assert result.returncode == 2
        assert "RT_CONTROL_ROS_DOMAIN_ID must be a decimal integer in 0..232" in result.stderr
        assert "args=" not in result.stdout
