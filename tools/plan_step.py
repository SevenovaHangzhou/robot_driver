"""Export an OFFLINE trajectory for review/replay; never sends a robot command."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gripper_jog.server import ROOT, build_mock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', required=True, type=Path)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/mock.json')
    parser.add_argument('--direction', choices=['x+', 'x-', 'y+', 'y-', 'z+', 'z-'], required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    mock = build_mock(args.urdf, json.loads(args.config.read_text()))
    plan = mock.planner.step(mock.q, args.direction, .0005, require_collision=False)
    times = np.linspace(0, plan.duration, min(2000, max(2, int(np.ceil(plan.duration / .01)) + 1)))
    q, v, a, _ = plan.state(times)
    document = {'format': 'offline-cartesian-jog/1', 'hardware_ready': False,
                'model_hash': mock.planner.chain.model_hash, 'chain_identity': mock.planner.chain.identity,
                'base_frame': mock.planner.chain.base, 'tip_frame': mock.planner.chain.tip,
                'joint_names': mock.planner.chain.names, 'interpolation': 'offline analytic spline; not a controller contract',
                **plan.document(), 'points': [{'time_s': float(t), 'positions': qi.tolist(),
                'velocities': vi.tolist(), 'accelerations': ai.tolist()} for t, qi, vi, ai in zip(times, q, v, a)]}
    with args.output.open('x') as output:
        json.dump(document, output, ensure_ascii=False, allow_nan=False, indent=2)
    print(f'Wrote {len(times)} offline samples: {args.output}')


if __name__ == '__main__':
    main()
