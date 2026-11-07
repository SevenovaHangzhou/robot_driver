"""Compose every selection of one source tree with that tree's own composer (in memory).

Usage: compose_worker.py <tree-root> <output.json>
Run in a separate process per tree so the composer and renderers are imported from the tree
being checked. Selections: every machine variant x unit x physical profile x allowed scope,
mock and real. Absolute paths of the tree and of the runtime directory are replaced by
placeholders so two trees compare by content.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

RUNTIME = Path("/arch-check/runtime")


def main(tree: Path, output: Path) -> int:
    rt = tree / "src/rt_control"
    shares = {path.name: path for path in rt.iterdir() if (path / "package.xml").is_file()}
    shares["robot_description"] = tree / "src/description/robot_description"
    sys.path[:0] = [str(path) for path in shares.values()]

    def share(name: str) -> str:
        return str(shares[name])

    # Renderers also look their share up directly; resolve every lookup inside this tree.
    import ament_index_python.packages as index

    index.get_package_share_directory = share
    from integ_composer.compose import compose
    from integ_composer.schema import manifest_load

    # Bases older than read_with_parts (#231) name the same function read_manifest.
    read_with_parts = getattr(manifest_load, "read_with_parts", None) or manifest_load.read_manifest

    results = {}
    data = shares["hwcfg_machines"]
    units = sorted(path.name for path in (data / "units").iterdir() if path.is_dir())
    for manifest_path in sorted((data / "machines").glob("*/manifest.yaml")):
        variant = manifest_path.parent.name
        manifest = read_with_parts(manifest_path)
        for unit in units:
            for profile, entry in sorted(manifest["profiles"].items()):
                for scope in entry["allowed_scopes"]:
                    for mock in (True, False):
                        key = f"{variant}/{unit}/{profile}/{scope}/{'mock' if mock else 'real'}"
                        try:
                            composed = compose(
                                robot_variant=variant,
                                machine_unit=unit,
                                physical_profile=profile,
                                control_scope=scope,
                                runtime_dir=RUNTIME,
                                use_mock_hardware=mock,
                                package_share=share,
                                startup_id="arch-check",
                            )
                        except Exception as exc:  # noqa: BLE001 - a failed selection is a result
                            results[key] = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
                            continue
                        results[key] = {
                            "ok": True,
                            "files": {
                                name: text.replace(str(tree), "<TREE>").replace(
                                    str(RUNTIME), "<RUNTIME>"
                                )
                                for name, text in sorted(composed.files.items())
                            },
                        }
    output.write_text(json.dumps(results, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1]).resolve(), Path(sys.argv[2])))
