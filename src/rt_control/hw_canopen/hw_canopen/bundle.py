"""Generate the deployment CANopen bundle from a verified composition artifact.

Inputs come only from the artifact (canopen_bus.yaml, canopen_sources.yaml); the output
directory is the bus option dcf_path. Runs dcfgen offline; never touches the bus.
"""

import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess

import yaml


def generate(artifact_dir, *, dcfgen=None):
    artifact = Path(artifact_dir)
    source_text = (artifact / "canopen_bus.yaml").read_text(encoding="utf-8")
    bus = yaml.safe_load(source_text)
    sources = yaml.safe_load((artifact / "canopen_sources.yaml").read_text(encoding="utf-8"))
    output = Path(bus["options"]["dcf_path"])
    if not output.is_absolute():
        raise ValueError("CANopen bundle directory must be absolute")
    nodes = [name for name in bus if name not in ("options", "master")]
    if set(nodes) != set(sources):
        raise ValueError("CANopen bus nodes and EDS sources differ")
    exe = dcfgen or shutil.which("dcfgen")
    if exe is None:
        raise RuntimeError("dcfgen is not installed in this environment")
    output.parent.mkdir(parents=True, exist_ok=True)
    # dcfgen writes dcf_path into master.dcf, so generate at the final path; keep the
    # previous bundle aside and restore it on any failure.
    previous = output.parent / f".{output.name}.previous"
    if previous.exists():
        shutil.rmtree(previous)
    if output.exists():
        output.rename(previous)
    output.mkdir()
    try:
        generation = yaml.safe_load(source_text)
        for name in nodes:
            target = output / f"{name}.eds"
            shutil.copyfile(sources[name], target)
            generation[name]["dcf"] = str(target)
        (output / "generation-bus.yml").write_text(yaml.safe_dump(generation, sort_keys=False))
        result = subprocess.run(
            [exe, "-r", "-v", "-d", str(output), str(output / "generation-bus.yml")],
            text=True,
            capture_output=True,
            timeout=60,
        )
        (output / "generation.log").write_text(result.stdout + "\n" + result.stderr)
        if result.returncode != 0:
            raise RuntimeError(f"dcfgen rejected the bus: {result.stderr.strip()[-400:]}")
        if not (output / "master.dcf").is_file():
            raise RuntimeError("dcfgen produced no master.dcf")
        if not (output / "master.bin").exists():
            # Without concise downloads dcfgen writes no bin; an empty concise list is 4 zeros.
            (output / "master.bin").write_bytes(b"\0\0\0\0")
        # The runtime bus is the artifact source itself: relative dcf names, final dcf_path.
        (output / "bus.yml").write_text(source_text, encoding="utf-8")
        (output / "source.sha256").write_text(
            hashlib.sha256(source_text.encode("utf-8")).hexdigest() + "\n"
        )
    except BaseException:
        shutil.rmtree(output, ignore_errors=True)
        if previous.exists():
            previous.rename(output)
        raise
    shutil.rmtree(previous, ignore_errors=True)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact_dir", help="absolute composition artifact directory")
    args = parser.parse_args(argv)
    print(f"CANOPEN_BUNDLE {generate(args.artifact_dir)}")
    return 0
