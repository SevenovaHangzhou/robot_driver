"""Serve the real keyboard gateway; does not start/enable hardware or switch mode."""
import argparse
import json
from pathlib import Path

from aiohttp import web
import yaml

from .artifacts import inspect_artifact
from .gateway import GatewayServer
from .model_assets import ModelAssets
from .real_collision import CollisionPolicy
from .real_keyboard import RealKeyboard
from .latency import PROFILES
from .real_model import ArtifactMink
from .real_observe import ROOT, ReadOnlyJog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--primary', required=True, help='Same canonical console used by the joint page')
    parser.add_argument('--artifact-dir', required=True, type=Path)
    parser.add_argument('--model-share', required=True, type=Path)
    parser.add_argument('--domain-id', required=True, type=int)
    parser.add_argument('--collision-config', type=Path, default=ROOT / 'config/real_collision.json')
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', default=18131, type=int)
    parser.add_argument('--timing-profile', choices=PROFILES, default='baseline',
                        help='Explicit timing trial; baseline retains 120/100 ms leads')
    args = parser.parse_args()
    files = {name: (args.artifact_dir / name).read_text() for name in
             ('robot_description.xml', 'controllers.yaml', 'artifact.yaml')}
    inspect_artifact(files)
    config = yaml.safe_load(files['controllers.yaml'])
    rolling = [name for name, value in config['controller_manager']['ros__parameters'].items()
               if isinstance(value, dict) and value.get('type', '').endswith('/RollingTrajectoryController')]
    if len(rolling) != 1 or not 0 <= args.domain_id <= 232:
        parser.error('A unique Rolling controller and valid ROS Domain are required')
    parameters = config[rolling[0]]['ros__parameters']
    policy = json.loads(args.collision_config.read_text())
    collision = CollisionPolicy(**{k: v for k, v in policy.items() if k not in ('status', 'source')})
    model = ArtifactMink(files['robot_description.xml'], parameters,
                         model_share=args.model_share, collision_policy=collision)
    jog = ReadOnlyJog(files['robot_description.xml'], args.domain_id)
    assets = ModelAssets(args.artifact_dir / 'robot_description.xml', args.model_share)
    keyboard = RealKeyboard(model, jog, timing_profile=args.timing_profile)
    app = GatewayServer(args.primary, jog, assets, args.domain_id, keyboard=keyboard).build_app()
    print(f'真机键盘入口：http://{args.bind}:{args.port}/jog；共享控制权：{args.primary}', flush=True)
    web.run_app(app, host=args.bind, port=args.port, access_log=None, print=None)


if __name__ == '__main__':
    main()
