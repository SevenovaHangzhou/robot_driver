#!/usr/bin/env python3
"""Compare the real offline executable with an explicitly supplied static_gravity oracle."""

import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview", type=Path, required=True)
    parser.add_argument("--reference-library", type=Path, required=True)
    parser.add_argument("--reference-parameters", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--base", required=True)
    parser.add_argument("--tips", nargs=2, required=True)
    args = parser.parse_args()
    metadata = json.loads(args.reference_parameters.read_text())
    library = ctypes.CDLL(str(args.reference_library.resolve()))
    library.arm_gravity_model_signature.restype = ctypes.c_char_p
    if library.arm_gravity_model_signature().decode() != metadata["parameter_signature"]:
        raise ValueError("reference library and parameter JSON signatures differ")
    pointer = ctypes.POINTER(ctypes.c_double)
    library.arm_gravity_compute.argtypes = [pointer] * 5
    library.arm_gravity_compute.restype = ctypes.c_bool
    rng = np.random.default_rng(20261102)
    angles = np.vstack([np.zeros(14), rng.uniform(-1.5, 1.5, size=(1000, 14))])
    payload = np.zeros(8)
    worst = 0.0
    count = 0
    for gravity in ([0.0, 0.0, -9.81], [1.3, -2.1, -8.7], [0.0, 0.0, 0.0]):
        gravity_array = np.array(gravity, dtype=np.float64)
        expected = np.zeros_like(angles)
        for index, q in enumerate(angles):
            if not library.arm_gravity_compute(
                q.ctypes.data_as(pointer),
                gravity_array.ctypes.data_as(pointer),
                payload.ctypes.data_as(pointer),
                None,
                expected[index].ctypes.data_as(pointer),
            ):
                raise ValueError("reference rejected a finite sample")
        for side, tip in enumerate(args.tips):
            arm = metadata["arms"][sorted(metadata["arms"])[side]]
            # The reference accepts world gravity; the controller accepts base gravity.
            rotation = np.asarray(arm["base_rotation_world"])
            gravity_in_base = rotation.T @ gravity_array
            group = angles[:, 7 * side : 7 * (side + 1)]
            result = subprocess.run(
                [str(args.preview), str(args.urdf), args.base, tip, *map(str, gravity_in_base)],
                input="\n".join(" ".join(map(str, row)) for row in group) + "\n",
                text=True,
                capture_output=True,
                check=True,
            )
            actual = np.array(
                [[float(x) for x in row.split()] for row in result.stdout.splitlines()]
            )
            reference = expected[:, 7 * side : 7 * (side + 1)]
            if actual.shape != reference.shape or not np.all(np.isfinite(actual)):
                raise ValueError(f"wrong preview output shape: {actual.shape}")
            error = float(np.max(np.abs(actual - reference)))
            worst = max(worst, error)
            if error > 1e-9:
                raise ValueError(f"gravity mismatch: {error} N*m")
            count += len(group)
    print(
        json.dumps(
            {
                "chain_samples": count,
                "max_absolute_error_nm": worst,
                "urdf_sha256": hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
                "reference_urdf_sha256": metadata["source_urdf_sha256"],
                "reference_signature": metadata["parameter_signature"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
