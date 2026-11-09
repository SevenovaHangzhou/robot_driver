"""Read-only hardware preparation and feedback display over an SSH subscriber.

This entry point has no motion transport. It never enables, switches controllers,
opens sessions or publishes targets, including during HTTP stop and shutdown.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .errors import JogError
from .artifacts import inspect_artifact
from .model import Chain, robot_direction_map
from .model_assets import ModelAssets
from .rolling import JOINT_NAMES

ROOT = Path(__file__).resolve().parent.parent
SSH_SKILL = Path.home() / '.codex/skills/ipc-ssh'


class ReadOnlyJog:
    """Exact-name feedback binding; no conversion, offsets or simulated joints."""
    mode = 'real-observe'
    active = None

    def __init__(self, xml, domain_id, *, clock=time.monotonic):
        self.clock, self.domain_id = clock, domain_id
        self.lock = threading.RLock()
        self.chains = {side: Chain(xml, 'arm_carriage', side + '_tool0', margin=0)
                       for side in ('left', 'right')}
        names = [name for chain in self.chains.values() for name in chain.names]
        if len(names) != 14 or set(names) != set(JOINT_NAMES):
            raise JogError('real_axes', '真机模型双臂轴名与 Rolling 14 轴不一致')
        root = ET.fromstring(xml)
        self.model_joints = tuple(j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed')
        self.selected_arm = 'left'
        self.planner = SimpleNamespace(chain=self.chains['left'])
        self.direction_maps = {side: robot_direction_map(xml, 'base_link', c.base) for side, c in self.chains.items()}
        self.latest, self.received_at = None, -math.inf
        self.positions, self.velocities = {}, {}
        self.fault = '等待真机关节反馈'
        self.transport = None

    def ingest(self, sample):
        with self.lock:
            try:
                if not isinstance(sample, dict):
                    raise ValueError('反馈格式无效')
                if sample.get('readonly') is not True or sample.get('domain_id') != self.domain_id:
                    raise ValueError('ROS Domain 或只读来源不一致')
                if sample.get('source_valid') is not True or sample.get('model_sha256') != self.planner.chain.model_hash:
                    raise ValueError('真机运行栈或模型已改变，请重新准备并启动观测')
                positions = sample['observed']['positions']
                for chain in self.chains.values():
                    chain.validate([positions[name] for name in chain.names])
                if any(type(v) not in (int, float) or not math.isfinite(v) for v in positions.values()):
                    raise ValueError('关节反馈包含无效值')
                age = sample['receive_age_s']['positions']
                if not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
                    raise ValueError('反馈时间无效')
                self.positions = {name: v for name, v in positions.items() if name in self.model_joints}
                self.velocities = sample['observed'].get('velocities', {})
                self.latest, self.received_at, self.fault = deepcopy(sample), self.clock() - age, ''
                return True
            except (JogError, ValueError, TypeError, KeyError) as exc:
                self.fault = str(exc)
                return False

    def disconnected(self):
        with self.lock:
            self.fault = '真机只读连接已结束；显示最后有效姿态'

    def tick(self):
        pass

    def state(self, _session, _client):
        with self.lock:
            chain = self.planner.chain
            age = self.clock() - self.received_at
            fresh = not self.fault and age <= .75  # Display freshness, not a motor safety threshold.
            sample = self.latest or {}
            missing = [n for n in self.model_joints if n not in self.positions]
            controllers = sample.get('controllers', [])
            rolling = [c for c in controllers if 'RollingTrajectoryController' in c.get('type', '')]
            blockers = ['只读观测：运动接口未开放', '主操作台与末端点动的共享控制权尚未现场验证',
                        '当前 Rolling 发布协议与运动接入尚未现场验证',
                        'Mink 运动模型、TCP 标定、现场方向与硬件动态包络待核验']
            if missing:
                blockers.append('无反馈关节按模型零位显示：' + ', '.join(missing))
            if not fresh:
                blockers.insert(0, self.fault or '真机反馈已超时，显示最后有效姿态')
            state = {'mode': self.mode, 'readonly': True, 'hardware_connected': bool(fresh),
                     'direction_frame': 'base_link', 'direction_mapping': self.direction_maps[self.selected_arm],
                     'selected_arm': self.selected_arm, 'arms': list(self.chains),
                     'model_joint_names': list(self.positions), 'model_q_rad': list(self.positions.values()),
                     'unobserved_model_joints': missing, 'lease': 'none', 'armed': False,
                     'epoch': '', 'busy': False, 'stopping': False, 'feedback_fresh': bool(fresh),
                     'feedback_age_s': age if math.isfinite(age) else None,
                     'model_only': not bool(self.positions),
                     'frame': chain.base, 'tip': chain.tip, 'model_hash': chain.model_hash,
                     'model_identity': chain.identity, 'tcp_xyz_m': chain.tcp[:3, 3].tolist(),
                     'joint_names': list(chain.names), 'tcp_mm': None, 'rpy_deg': None,
                     'can_arm': False, 'last_plan': None, 'blockers': blockers,
                     'result': '真机只读观测；末端运动未开放' if fresh else blockers[0],
                     'rolling_state': sample.get('observed', {}).get('rolling'),
                     'controllers': controllers, 'captured_at': sample.get('captured_at'),
                     'controllers_age_s': sample.get('controllers_age_s'),
                     'control_authority': {'required': True, 'verified': False,
                                           'primary_console': 'http://192.168.100.40:18121',
                                           'resource': 'dual_arms', 'status': 'integration_pending'},
                     'publication': {'mode': 'observe', 'backend': 'real_readonly',
                                     'feedback_source': 'hardware_joint_states',
                                     'rolling_detected': bool(rolling), 'motion_enabled': False}}
            if self.positions:
                q = [self.positions[n] for n in chain.names]
                pose, jac = chain.fk_jacobian(q)
                state.update(q_rad=q, tcp_mm=(pose[:3, 3] * 1000).tolist(),
                             tcp_orientation_xyzw=Rotation.from_matrix(pose[:3, :3]).as_quat().tolist(),
                             rotation_frame=chain.rotation_frame,
                             rotation_orientation_xyzw=Rotation.from_matrix(
                                 pose[:3, :3] @ chain.end_to_tcp[:3, :3].T).as_quat().tolist(),
                             rpy_deg=Rotation.from_matrix(pose[:3, :3]).as_euler('xyz', degrees=True).tolist())
                dq = [self.velocities.get(n) for n in chain.names]
                if fresh and all(isinstance(v, (int, float)) and math.isfinite(v) for v in dq):
                    state['tcp_actual_speed_mm_s'] = float(np.linalg.norm(jac[:3] @ dq) * 1000)
                    state['tcp_actual_angular_speed_deg_s'] = float(np.rad2deg(np.linalg.norm(jac[3:] @ dq)))
            return state

    def dispatch(self, action, _session, body):
        with self.lock:
            if action == 'select_arm':
                side = body.get('arm')
                if side not in self.chains:
                    raise JogError('bad_arm', '所选机械臂未配置')
                self.selected_arm, self.planner = side, SimpleNamespace(chain=self.chains[side])
                return {'selected_arm': side}
            if action == 'stop':
                # Never stop or finalize a session owned by the other chat.
                return {'readonly': True, 'stopping': False}
            raise JogError('real_readonly', '当前为真机只读观测，不能获取控制权或发送运动')

    def close(self):
        if self.transport:
            self.transport.close()


class ReadOnlySSH:
    def __init__(self, target, password_file, runner):
        self.target, self.password_file, self.runner = target, Path(password_file), Path(runner)
        self.env = os.environ.copy()
        bundled = ROOT / '.runtime/ssh-client/usr/bin'
        if bundled.is_dir():
            self.env['PATH'] = str(bundled) + os.pathsep + self.env.get('PATH', '')

    def command(self, command):
        return [sys.executable, str(self.runner), '--target', self.target,
                '--password-file', str(self.password_file), '--', command]

    def read(self, script, command, *, timeout=30):
        result = subprocess.run(self.command(command), input=script, text=True, capture_output=True,
                                env=self.env, timeout=timeout, check=True)
        return json.loads(result.stdout)

    def stream(self, script, command, jog, log_path):
        self.log = log_path.open('w')
        self.process = subprocess.Popen(self.command(command), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.log, text=True, env=self.env, start_new_session=True)
        self.process.stdin.write(script)
        self.process.stdin.close()

        def consume():
            try:
                for line in self.process.stdout:
                    try:
                        jog.ingest(json.loads(line))
                    except (json.JSONDecodeError, TypeError):
                        jog.disconnected()
            finally:
                jog.disconnected()

        self.reader = threading.Thread(target=consume, name='readonly-feedback', daemon=True)
        self.reader.start()

    def close(self):
        if not hasattr(self, 'process'):
            return
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=2)
        if hasattr(self, 'reader'):
            self.reader.join(timeout=2)
        self.process.stdout.close()
        self.log.close()


class LocalReadOnly(ReadOnlySSH):
    """Run the same read-only collector on the IPC, without an SSH round trip."""
    def __init__(self):
        self.target = 'local-runtime'
        self.env = os.environ.copy()

    def command(self, command):
        return ['bash', '-c', command]


def inspect_command(info, domain_id, *, stream=False):
    workspace = Path(info['workspace'])
    argv = ['python3', '-u', '-', '--domain-id', str(domain_id),
            '--urdf', str(Path(info['artifact_dir']) / 'robot_description.xml'),
            '--controller-pid', str(info['controller_pid'])]
    if stream:
        argv.append('--stream')
    command = ('source /opt/ros/humble/setup.bash\nsource ' + shlex.quote(str(workspace / 'install/setup.bash'))
               + '\nexport ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp\nexec ' + shlex.join(argv))
    return 'bash -c ' + shlex.quote(command)


def prepare(ssh, destination, domain_id, model_share, workspace=None):
    argv = ['python3', '-', '--scope', 'bench_dual_arm_head']
    if workspace:
        argv.extend(['--workspace', str(workspace)])
    info = ssh.read((ROOT / 'script/read_real_artifacts.py').read_text(), shlex.join(argv))
    _, hashes = inspect_artifact(info['files'])
    destination.mkdir(parents=True, exist_ok=False)
    for name, content in info.pop('files').items():
        (destination / name).write_text(content)
    assets = ModelAssets(destination / 'robot_description.xml', model_share)
    jog = ReadOnlyJog(assets.xml, domain_id)
    probe_source = (ROOT / 'script/inspect_real_state.py').read_text()
    sample = ssh.read(probe_source, inspect_command(info, domain_id))
    (destination / 'initial-state.json').write_text(json.dumps(sample, ensure_ascii=False, indent=2))
    jog.ingest(sample)
    report = {**info, 'target': ssh.target, 'domain_id': domain_id, 'readonly': True,
              'prepared_at': datetime.now(timezone.utc).isoformat(), 'sha256': hashes,
              'local_mesh_count': len(assets.meshes), 'model_share': str(model_share),
              'feedback_binding': {side: list(c.names) for side, c in jog.chains.items()},
              'direction_mapping': jog.direction_maps,
              'state': jog.state('', ''), 'rolling_acceptance': 'pending', 'motion_enabled': False}
    (destination / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return jog, assets, report, probe_source


def prepare_local(source, destination, domain_id, model_share):
    """Show a collected model while the runtime is stopped; no live feedback."""
    files = {name: (source / name).read_text() for name in
             ('robot_description.xml', 'controllers.yaml', 'artifact.yaml')}
    _, hashes = inspect_artifact(files)
    destination.mkdir(parents=True, exist_ok=False)
    for name, content in files.items():
        (destination / name).write_text(content)
    assets = ModelAssets(destination / 'robot_description.xml', model_share)
    jog = ReadOnlyJog(assets.xml, domain_id)
    jog.fault = '等待真机反馈；当前仅加载已采集模型，零位预览不代表实际姿态'
    report = {'workspace': None, 'artifact_source': str(source.resolve()), 'readonly': True,
              'runtime_running': False, 'motion_enabled': False, 'sha256': hashes,
              'state': jog.state('', '')}
    (destination / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return jog, assets, report, None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--target', default='ar@192.168.100.40')
    parser.add_argument('--local-runtime', action='store_true', help='Observe ROS on this IPC; no SSH')
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--workspace', type=Path, help='Optional disambiguation; must be running')
    source.add_argument('--artifact-dir', type=Path, help='Collected model only; no live feedback or ROS connection')
    parser.add_argument('--password-file', type=Path, default=SSH_SKILL / 'credentials/ar-192.168.100.40.password')
    parser.add_argument('--ssh-runner', type=Path, default=SSH_SKILL / 'scripts/run_ssh.py')
    parser.add_argument('--model-share', type=Path, default=ROOT.parents[1] / 'sim/robot_description')
    parser.add_argument('--output', type=Path, help='New directory for the read-only evidence bundle')
    parser.add_argument('--prepare-only', action='store_true', help='Collect and check without starting the web observer')
    parser.add_argument('--gateway', help='Primary console HTTP origin; share login and arm authority')
    parser.add_argument('--port', type=int, default=18131)
    parser.add_argument('--bind', default='127.0.0.1', help='Bind the observer to this host address')
    parser.add_argument('--token-file', type=Path, help='Protected access token; required for a LAN observer')
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232 or not 0 <= args.port <= 65535:
        parser.error('Invalid ROS domain or local HTTP port')
    if args.bind != '127.0.0.1' and not args.gateway and args.token_file is None:
        parser.error('A LAN observer requires --token-file')
    token = 'mock'
    if args.token_file:
        if args.token_file.stat().st_mode & 0o077:
            parser.error('Access token file must not be readable by group or others')
        token = args.token_file.read_text().strip()
        if not token or token == 'mock':
            parser.error('A LAN observer needs a non-preview access token')
    if args.target != 'ar@192.168.100.40' and args.password_file == SSH_SKILL / 'credentials/ar-192.168.100.40.password':
        parser.error('A different target requires its own --password-file')
    destination = (args.output or ROOT / '.runtime' / datetime.now().strftime('real-observe-%Y%m%d-%H%M%S')).resolve()
    ssh = (None if args.artifact_dir else LocalReadOnly() if args.local_runtime else
           ReadOnlySSH(args.target, args.password_file, args.ssh_runner))
    try:
        if args.artifact_dir:
            jog, assets, info, probe = prepare_local(args.artifact_dir, destination, args.domain_id, args.model_share.resolve())
        else:
            jog, assets, info, probe = prepare(ssh, destination, args.domain_id, args.model_share.resolve(), args.workspace)
    except (OSError, ValueError, KeyError, JogError, subprocess.SubprocessError) as exc:
        parser.error(str(exc))
    print(f"只读检查记录：{destination}\n真机工作区：{info['workspace']}\n模型 SHA-256：{jog.planner.chain.model_hash}", flush=True)
    print('反馈：' + jog.state('', '')['result'], flush=True)
    if args.prepare_only:
        if not jog.state('', '')['feedback_fresh']:
            raise SystemExit(1)
        return
    jog.transport = ssh
    server = None
    try:
        if args.gateway:
            from aiohttp import web
            from .gateway import GatewayServer
            app = GatewayServer(args.gateway, jog, assets, args.domain_id).build_app()
        else:
            from .server import JogHTTPServer
            server = JogHTTPServer((args.bind, args.port), jog, model_assets=assets, login_token=token)
        if ssh:
            ssh.stream(probe, inspect_command(info, args.domain_id, stream=True), jog, destination / 'observer.log')
        print(f'真机观测：http://{args.bind}:{args.port}/jog （末端运动关闭）', flush=True)
        if args.gateway:
            print(f'认证与双臂控制权来自：{args.gateway}', flush=True)
            web.run_app(app, host=args.bind, port=args.port, access_log=None, print=None)
        else:
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if server is not None:
            server.server_close()
        jog.close()


if __name__ == '__main__':
    main()
