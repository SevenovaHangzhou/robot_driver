"""Prepare the current real Rolling experiment without starting hardware."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shlex
import subprocess

import numpy as np
import yaml

from .artifacts import inspect_artifact
from .current_rolling import trajectory_window
from .errors import JogError
from .model_assets import ModelAssets
from .real_model import ArtifactMink
from .real_collision import CollisionPolicy
from .real_observe import ROOT, SSH_SKILL, ReadOnlySSH, inspect_command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--workspace', type=Path, help='Explicit deployed workspace on the IPC')
    source.add_argument('--artifact-dir', type=Path, help='Previously collected local artifact directory')
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--model-share', type=Path, default=ROOT.parents[1] / 'sim/robot_description')
    parser.add_argument('--snapshot', type=Path, help='Replay only: snapshot must have the same model SHA')
    parser.add_argument('--collision-config', type=Path, default=ROOT / 'config/real_collision.json',
                        help='Explicit offline collision policy; geometry comes from the artifact URDF')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--arm', choices=('left', 'right'), default='left')
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error('ROS Domain must be 0..232')
    output = (args.output or ROOT / '.runtime' / datetime.now().strftime('real-experiment-%Y%m%d-%H%M%S')).resolve()
    try:
        info, files, sample = collect(args)
        artifact, hashes = inspect_artifact(files)
        config = yaml.safe_load(files['controllers.yaml'])
        controllers = config['controller_manager']['ros__parameters']
        rolling = [n for n, v in controllers.items() if isinstance(v, dict)
                   and v.get('type', '').endswith('/RollingTrajectoryController')]
        if len(rolling) != 1:
            raise JogError('rolling_controller', '需要唯一的 Rolling 控制器声明')
        parameters = config[rolling[0]]['ros__parameters']
        collision_config = json.loads(args.collision_config.read_text())
        policy = CollisionPolicy(**{k: v for k, v in collision_config.items() if k not in ('status', 'source')})
        model = ArtifactMink(files['robot_description.xml'], parameters,
                             model_share=args.model_share, collision_policy=policy)
        # Reference configurations validate conversion; they are never hardware
        # initialization values and are not saved as an executable trajectory.
        zero = {n: 0. for n in model.names}
        fk = model.verify_fk(zero)
        output.mkdir(parents=True, exist_ok=False)
        for name, text in files.items():
            (output / name).write_text(text)
        assets = ModelAssets(output / 'robot_description.xml', args.model_share)
        report = {'prepared_at': datetime.now(timezone.utc).isoformat(),
                  'phase': 'static_prepared', **info, 'domain_id': args.domain_id,
                  'selection': artifact['selection'], 'sha256': hashes,
                  'rolling_controller': rolling[0], 'joint_order': list(model.names),
                  'protocol': {'command': '/rt/arms/rolling/target_batch',
                               'state': '/rt/arms/rolling/state',
                               'fields': ['epoch', 'sequence', 'replace_from', 'points', 'final'],
                               'sender': 'robot_interfaces_rolling_client::RollingSender'},
                  'limits_source': 'controllers.yaml from the deployed artifact',
                  'model_conversion_fk': fk, 'direction_mapping': model.direction_maps,
                  'visual_mesh_count': len(assets.meshes),
                  'mink_model': {'purpose': 'kinematics_only', 'dof': model.model.nv,
                                 'gravity_compensation': 'off', 'collision': model.collision.info()},
                  'collision_config': collision_config,
                  'shadow': [], 'hardware_motion_ready': False, 'commands_published': 0,
                  'blockers': ['共享控制权尚未在当前主操作台部署和验证',
                               '碰撞余量仍为离线候选值，几何和现场方向、空间尚需核验',
                               '本工具尚未执行现场使能、模式切换与首段运动']}
        if sample is not None:
            (output / 'snapshot.json').write_text(json.dumps(sample, ensure_ascii=False, indent=2) + '\n')
            report['snapshot_captured_at'] = sample.get('captured_at')
            report['snapshot_live'] = bool(args.workspace and info['runtime_running'] and not args.snapshot)
            run_shadow(model, sample, args.arm, report, output)
        else:
            report['snapshot_live'] = False
            report['blockers'].insert(0, '当前控制栈停止，没有本轮新鲜实测 q；未生成实机轨迹')
        (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    except (JogError, OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        parser.error(str(exc))
    print(f'实验准备记录：{output}\nRolling 控制器：{rolling[0]}\n阶段：{report["phase"]}')
    print('仅完成静态/影子求解；未启动控制栈、使能、切模式或发布运动。')
    for blocker in report['blockers']:
        print('待完成：' + blocker)


def collect(args):
    sample = json.loads(args.snapshot.read_text()) if args.snapshot else None
    if args.artifact_dir:
        files = {n: (args.artifact_dir / n).read_text() for n in
                 ('robot_description.xml', 'controllers.yaml', 'artifact.yaml')}
        return {'workspace': None, 'runtime_running': False, 'controller_pid': None}, files, sample
    ssh = ReadOnlySSH('ar@192.168.100.40', SSH_SKILL / 'credentials/ar-192.168.100.40.password',
                      SSH_SKILL / 'scripts/run_ssh.py')
    argv = ['python3', '-', '--workspace', str(args.workspace), '--allow-stopped']
    info = ssh.read((ROOT / 'script/read_real_artifacts.py').read_text(), shlex.join(argv))
    files = info.pop('files')
    if info['runtime_running'] and sample is None:
        sample = ssh.read((ROOT / 'script/inspect_real_state.py').read_text(), inspect_command(info, args.domain_id))
    return info, files, sample


def run_shadow(model, sample, side, report, output):
    if sample.get('readonly') is not True or sample.get('domain_id') != report['domain_id']:
        report['blockers'].insert(0, '快照的 ROS Domain 或只读来源无效；未进行影子求解')
        return
    if sample.get('model_sha256') != model.chains[side].model_hash or sample.get('source_valid') is not True:
        report['blockers'].insert(0, '快照模型身份不匹配；拒绝混用旧模型关节值进行影子求解')
        return
    positions = sample['observed']['positions']
    report['snapshot_fk'] = model.verify_fk(positions)
    report['unobserved_auxiliary_joints'] = [n for n in model.joint_ids if n not in positions]
    for direction in model.direction_maps[side]:
        try:
            curve, metrics = model.shadow_path(positions, side, direction)
            report['shadow'].append(metrics)
            if metrics['kinematic_path_ok']:
                # A file of offline Window inputs, not a stream of ROS commands.
                starts = np.arange(0., max(float(curve.x[-1]) - .4, .001), 1/30).tolist()
                starts.append(max(0., float(curve.x[-1]) - .4))
                windows = [trajectory_window(curve, t, 0, .4) for t in starts]
                (output / ('shadow-' + direction + '.json')).write_text(
                    json.dumps({'replay_only': True, 'origin_ros_ns_is_placeholder': True,
                                'windows': windows}, indent=2) + '\n')
        except (JogError, ValueError) as exc:
            report['shadow'].append({'robot_direction': direction, 'kinematic_path_ok': False, 'error': str(exc)})


if __name__ == '__main__':
    main()
