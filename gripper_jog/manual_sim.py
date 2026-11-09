"""Persistent local web -> ROS rolling -> Cia402MockSystem -> MuJoCo experiment.

This launcher owns its Mock stack. It never attaches to an existing controller.
ROS imports are delayed so --help and the lifecycle unit tests need no ROS.
"""
import argparse
from collections import deque
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET

from .errors import JogError
from .rolling import JOINT_NAMES, PREFIX, RollingMotion
from .rolling_ros import RosRollingTransport
from .server import ROOT, MockServer, build_mock
from .model_assets import ModelAssets

CONTROL_DOMAIN, FEEDBACK_DOMAIN = 191, 192
MOCK_PLUGIN = 'robot_hw_ethercat/Cia402MockSystem'


def verify_mock_artifact(path):
    robot = ET.parse(path / 'robot_description.xml').getroot()
    plugins = [p.text for p in robot.findall('ros2_control/hardware/plugin')]
    names = [p.attrib['name'] for p in robot.findall('ros2_control/joint')]
    if not plugins or set(plugins) != {MOCK_PLUGIN} or sorted(names) != sorted(JOINT_NAMES):
        raise RuntimeError('生成物不是完整双臂 Cia402MockSystem，拒绝使能')


def child_environment(environment, client_site):
    result = dict(environment, ROS_LOCALHOST_ONLY='1')
    # MuJoCo must use its own venv's NumPy/CFFI, not the web client's packages.
    result['PYTHONPATH'] = os.pathsep.join(
        p for p in result.get('PYTHONPATH', '').split(os.pathsep)
        if p and p not in (str(client_site), '/usr/lib/python3/dist-packages'))
    return result


@contextmanager
def instance_lock(directory):
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / 'manual-sim.lock').open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('已有手动联调实例运行，请使用其网页或先关闭该实例') from None
        yield


class RosExecution:
    def __init__(self, transport):
        self.transport, self.motion = transport, None

    def start(self, plan, positions):
        if set(positions) != set(JOINT_NAMES):
            raise JogError('rolling_axes', '缺少完整双臂位置')
        if self.motion is not None and not self.motion.complete:
            raise JogError('rolling_busy', '上一会话尚未结束')
        desired = self.transport.state()['desired_positions']
        if len(desired) != len(JOINT_NAMES) or any(positions[name] != q for name, q in zip(JOINT_NAMES, desired)):
            raise JogError('rolling_anchor', '网页起点与控制器命令不一致，请重新启动联调')
        self.motion = RollingMotion(plan, self.transport)
        return self.motion

    def publication(self):
        state = self.motion.publication() if self.motion else {'mode': 'rolling', 'sequence': 0}
        return {**state, 'backend': 'mujoco_ros_mock', 'feedback_source': 'controller_command'}

    def foreign_session(self, snapshot):
        if not snapshot['has_session']:
            return False
        identity = self.motion.identity if self.motion is not None else {}
        # FINALIZE can be acknowledged before the matching idle state arrives.
        # The last completed session is still ours during that DDS interval.
        return any(snapshot[key] != identity.get(key)
                   for key in ('controller_boot_id', 'client_instance_id'))


class ManualSimulation:
    def __init__(self, args, output):
        self.args, self.output = args, output
        self.stop = threading.Event()
        self.process = self.server = self.transport = self.action = None
        self.contexts, self.nodes, self.executors, self.threads = [], [], [], []
        self.mock, self.physical, self.motors = deque(maxlen=1), deque(maxlen=1), deque(maxlen=1)
        self.artifact, self.mock_verified = None, False
        self.log = None
        self.errors = []
        self.stage, self.reason = 'starting', ''
        self.ready, self.shutdown_code = False, None

    def wait(self, predicate, timeout=15., *, cleanup=False):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            if not cleanup:
                if self.stop.is_set():
                    raise InterruptedError('收到退出请求')
                if self.process is not None and self.process.poll() is not None:
                    raise RuntimeError(f'联合入口提前退出 ({self.process.returncode})，请检查 {self.output / "stack.log"}')
            try:
                result = predicate()
                if result:
                    return result
            except (JogError, LookupError) as error:
                last = error
            time.sleep(.02)
        raise TimeoutError(f'{self.stage}: 等待超时 ({last})')

    def service(self, client, request, timeout=20., *, cleanup=False):
        self.wait(client.service_is_ready, timeout, cleanup=cleanup)
        future = client.call_async(request)
        self.wait(future.done, timeout, cleanup=cleanup)
        return future.result()

    def start(self):
        from ament_index_python.packages import get_package_share_directory
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from control_msgs.action import FollowJointTrajectory
        from control_msgs.msg import DynamicJointState
        from trajectory_msgs.msg import JointTrajectoryPoint
        from controller_manager_msgs.srv import ListControllers
        from rt_control_interfaces.srv import RtEnable
        from robot_rt_control_interfaces.srv import SetJointControlMode
        from sensor_msgs.msg import JointState
        from robot_interfaces_qos import rolling_command, rolling_state

        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        self.enable_type = RtEnable
        urdf = self.args.workspace / 'sim/robot_description/urdf/robot_alfav3_mobile.urdf'
        installed = Path(get_package_share_directory('robot_description')) / 'urdf/robot_alfav3_mobile.urdf'
        if urdf.read_bytes() != installed.read_bytes():
            raise RuntimeError('模型源码与 driver 安装副本不一致，请先更新并重建模型')
        self.model_hash = hashlib.sha256(urdf.read_bytes()).hexdigest()
        config = json.loads((ROOT / 'config/mock.json').read_text())
        if config.get('publication_mode') != 'rolling':
            raise RuntimeError('手动联调要求 config/mock.json 使用 rolling 发布')
        self.jog = build_mock(urdf, config)
        # Reserve the HTTP port before any Mock enable or trajectory is sent.
        self.server = MockServer(('127.0.0.1', self.args.port), self.jog,
                                 model_assets=ModelAssets(urdf))
        for domain in (CONTROL_DOMAIN, FEEDBACK_DOMAIN):
            context = Context()
            rclpy.init(args=[], context=context, domain_id=domain,
                       signal_handler_options=SignalHandlerOptions.NO)
            self.contexts.append(context)
            node = rclpy.create_node(f'jog_manual_sim_{domain}', context=context)
            self.nodes.append(node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self.executors.append(executor)
            thread = threading.Thread(target=executor.spin, daemon=True)
            self.threads.append(thread)
            thread.start()
        node, feedback_node = self.nodes
        # Discover before launch; never issue enable to an existing DDS stack.
        for _ in range(40):
            if self.stop.wait(.05):
                raise InterruptedError('收到退出请求')
            services = {name for name, _ in node.get_service_names_and_types()}
            if services & {'/rt/enable', '/controller_manager/list_controllers', '/rt/joint_control/set_mode'} \
                    or node.count_publishers(PREFIX + '/state') or feedback_node.count_publishers('/joint_states'):
                raise RuntimeError('DDS 191/192 已有控制或反馈节点，请先关闭旧实验')
        self.transport = RosRollingTransport(node, command_qos=rolling_command(), state_qos=rolling_state(),
                                             service_timeout=.5, state_timeout=.5)
        node.create_subscription(JointState, '/joint_states', self.mock.append, 5)
        feedback_node.create_subscription(JointState, '/joint_states',
                                          lambda m: self.physical.append((time.monotonic(), m)), 5)
        feedback_node.create_subscription(DynamicJointState, '/rt_internal_state_broadcaster/dynamic_joint_states',
                                          lambda m: self.motors.append((time.monotonic(), m)), 5)
        enable = node.create_client(RtEnable, '/rt/enable')
        self.disable = node.create_client(RtEnable, '/rt/disable')
        mode = node.create_client(SetJointControlMode, '/rt/joint_control/set_mode')
        controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.action = ActionClient(node, FollowJointTrajectory, '/whole_body_jtc/follow_joint_trajectory')
        sim = self.args.workspace / 'sim/robot_simulation'
        command = ['bash', str(sim / 'control_synchronizer/scripts/start_dual_arm_simulation.sh'),
                   '--setup', str(self.args.setup), '--source-domain-id', str(CONTROL_DOMAIN),
                   '--domain-id', str(FEEDBACK_DOMAIN)]
        if self.args.headless:
            command.append('--headless')
        self.log = (self.output / 'stack.log').open('w')
        client_site = ROOT / '.venv/lib/python3.10/site-packages'
        self.process = subprocess.Popen(command, cwd=sim,
                                        env=child_environment(os.environ, client_site),
                                        stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        print('正在启动双臂 Mock 与 MuJoCo…', flush=True)
        self.wait(lambda: self.mock and self.physical and self.motors and self.action.server_is_ready(), 40)
        text = (self.output / 'stack.log').read_text()
        self.artifact = Path(next(line.split('Artifact: ', 1)[1] for line in text.splitlines()
                                  if line.startswith('Artifact: ')))
        verify_mock_artifact(self.artifact)
        self.mock_verified = True
        self.wait(controllers.service_is_ready)
        self.wait(lambda: any(c.name == 'enable_manager' and c.state == 'active'
                              for c in self.service(controllers, ListControllers.Request()).controller))
        self.stage = 'preparing'
        print('Mock 已核验，正在使能并移动到初始姿态…', flush=True)
        response = self.service(enable, RtEnable.Request(), 30)
        if not response.ok:
            raise RuntimeError(f'Mock 使能失败: {response}')
        current = dict(zip(self.mock[-1].name, self.mock[-1].position))
        target_by_name = {name: float(q) for planner, positions in self.jog.arms.values()
                          for name, q in zip(planner.chain.names, positions)}
        target = [target_by_name[name] for name in JOINT_NAMES]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        for seconds, values in ((0, [current[name] for name in JOINT_NAMES]), (4, target)):
            point = JointTrajectoryPoint()
            point.time_from_start.sec = seconds
            point.positions, point.velocities = values, [0.] * 14
            goal.trajectory.points.append(point)
        future = self.action.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('初始姿态 FJT 被拒绝')
        future = handle.get_result_async()
        self.wait(future.done, 10)
        if future.result().result.error_code != 0:
            raise RuntimeError(f'初始姿态 FJT 失败: {future.result()}')
        self.wait(lambda: all(dict(zip(self.mock[-1].name, self.mock[-1].position))[n] == q
                              for n, q in zip(JOINT_NAMES, target)))
        request = SetJointControlMode.Request()
        request.protocol_major = 1
        request.client_instance_id.uuid = list(uuid.uuid4().bytes)
        request.request_id.uuid = list(uuid.uuid4().bytes)
        request.expected_mode.value = request.expected_mode.FJT_READY
        request.target_mode.value = request.target_mode.ROLLING_READY
        response = self.service(mode, request)
        if not response.accepted or response.mode.value != request.target_mode.value:
            raise RuntimeError(f'Rolling 模式切换失败: {response}')
        state = self.wait(self.transport.state)
        RollingMotion._protocol(state)
        if state['has_session']:
            raise RuntimeError('Rolling 已有其他会话')
        self.boot_id = state['controller_boot_id']
        self.wait(lambda: self.transport.publisher.get_subscription_count() == 1)
        self.execution = self.jog.execution = RosExecution(self.transport)
        self.http_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.http_thread.start()
        self.ready, self.stage = True, 'ready'
        self.write_status()
        print(f'就绪：http://127.0.0.1:{self.args.port}/jog  令牌：mock\n'
              '获取控制权 → 选择机械臂 → 开启键盘接管。关闭 MuJoCo 窗口或 Ctrl+C 退出。', flush=True)

    def monitor(self):
        last_write = 0.
        while not self.stop.wait(.05):
            if self.process.poll() is not None:
                self.reason = '联合入口退出（包括关闭 MuJoCo 窗口）'
                if self.process.returncode != 0:
                    raise RuntimeError(f'联合入口异常退出: {self.process.returncode}')
                return
            now = time.monotonic()
            fresh = bool(self.physical and self.motors and
                         now - self.physical[-1][0] < .5 and now - self.motors[-1][0] < .5)
            with self.jog.lock:
                try:
                    state = self.transport.state()
                    if state['controller_boot_id'] != self.boot_id:
                        raise RuntimeError('控制器重新启动，结束本次联调')
                    self.jog.external_busy = self.execution.foreign_session(state)
                except JogError:
                    fresh = False
                self.jog.feedback_fresh = fresh
            if now - last_write >= 1.:
                self.write_status()
                last_write = now

    def write_status(self):
        data = {'stage': self.stage, 'ready': self.ready, 'reason': self.reason,
                'url': f'http://127.0.0.1:{self.args.port}/jog',
                'control_domain': CONTROL_DOMAIN, 'feedback_domain': FEEDBACK_DOMAIN,
                'hardware_commands': False, 'model_hash': getattr(self, 'model_hash', None),
                'artifact': str(self.artifact), 'shutdown_code': self.shutdown_code, 'errors': self.errors}
        if self.physical:
            stamp, message = self.physical[-1]
            data['physical_age_s'] = time.monotonic() - stamp
            data['physical_positions'] = dict(zip(message.name, message.position))
        if self.motors:
            _, message = self.motors[-1]
            data['motor_interfaces'] = {name: dict(zip(values.interface_names, values.values))
                                        for name, values in zip(message.joint_names, message.interface_values)}
        if self.transport is not None:
            try:
                data['rolling'] = self.transport.state()
            except JogError:
                pass
        temporary = self.output / 'status.tmp'
        temporary.write_text(json.dumps(data, indent=2))
        temporary.replace(self.output / 'status.json')

    def close(self):
        self.stage, self.ready = 'stopping', False
        print('正在停止网页、失能并关闭仿真，请等待…', flush=True)
        # Each cleanup step must run even if the preceding step fails.
        def attempt(label, operation):
            try:
                return operation()
            except Exception as error:
                self.errors.append(f'{label}: {error}')
                return None
        if self.server is not None:
            if hasattr(self, 'http_thread'):
                attempt('HTTP shutdown', self.server.shutdown)
                self.http_thread.join(timeout=3)
            attempt('rolling stop/finalize', self.server.server_close)
        if self.mock_verified and self.process is not None and self.process.poll() is None:
            def disable():
                result = self.service(self.disable, self.enable_type.Request(), 25, cleanup=True)
                if not result.ok:
                    raise RuntimeError(str(result))
            attempt('Mock disable', disable)
        if self.process is not None:
            def stop_stack():
                if self.process.poll() is None:
                    os.killpg(self.process.pid, signal.SIGINT)
                try:
                    self.shutdown_code = self.process.wait(timeout=50)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    try:
                        self.shutdown_code = self.process.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        # Include ROS descendants that detached with setsid.
                        import sys
                        sys.path.insert(0, str(self.args.workspace / 'sim/robot_simulation'))
                        from control_synchronizer.dual_arm_launch import live_groups, send_groups
                        groups = {self.process.pid}
                        send_groups(live_groups([self.process], groups), signal.SIGKILL)
                        self.shutdown_code = self.process.wait(timeout=5)
                    raise RuntimeError('联合入口超出有序停机时限')
            attempt('stack shutdown', stop_stack)
        if self.transport is not None:
            attempt('transport destroy', self.transport.destroy)
        if self.action is not None:
            attempt('action destroy', self.action.destroy)
        for executor in self.executors:
            attempt('executor shutdown', lambda: executor.shutdown(timeout_sec=2))
        for thread in self.threads:
            thread.join(timeout=2)
        for node in self.nodes:
            attempt('node destroy', node.destroy_node)
        for context in self.contexts:
            attempt('context shutdown', context.shutdown)
        if self.log is not None:
            self.log.close()
            text = (self.output / 'stack.log').read_text()
            if self.mock_verified and ('UNCLEAN_SHUTDOWN' in text or
                                      'rt_control shutdown EtherCAT hardware state: inactive' not in text):
                self.errors.append('未确认 driver 硬件 inactive 的有序退出')
        self.stage = 'failed' if self.errors else 'stopped'
        self.write_status()
        print(f'{"退出存在错误" if self.errors else "已停止"}。日志：{self.output}', flush=True)


def parser():
    result = argparse.ArgumentParser(description='启动常驻网页、C++ 双臂 Mock 和 MuJoCo 手动联调')
    workspace = ROOT.parents[1]
    result.add_argument('--workspace', type=Path, default=workspace)
    result.add_argument('--setup', type=Path, default=workspace / 'sim/robot_simulation/control_synchronizer/.runtime/driver/install/setup.bash')
    result.add_argument('--port', type=int, default=18132)
    result.add_argument('--headless', action='store_true', help='关闭 MuJoCo 窗口，网页仍可手动控制')
    return result


def main(argv=None):
    arguments = parser()
    args = arguments.parse_args(argv)
    args.workspace, args.setup = args.workspace.resolve(), args.setup.resolve()
    if not 1024 <= args.port <= 65535:
        arguments.error('端口必须在 1024..65535 范围')
    if not args.headless and not os.environ.get('DISPLAY'):
        arguments.error('未找到桌面 DISPLAY；请在桌面终端启动，或使用 --headless')
    if not args.setup.is_file():
        arguments.error(f'未找到 driver setup: {args.setup}')
    try:
        with instance_lock(ROOT / '.runtime'):
            output_root = ROOT / '.runtime/manual-sim'
            output_root.mkdir(parents=True, exist_ok=True)
            output = Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%d-%H%M%S-'), dir=output_root))
            runtime = ManualSimulation(args, output)
            old_handlers = {}
            def stop(signum, _frame):
                runtime.reason = f'收到 {signal.Signals(signum).name}'
                runtime.stop.set()
            for signum in (signal.SIGINT, signal.SIGTERM):
                old_handlers[signum] = signal.signal(signum, stop)
            print(f'日志目录：{output}', flush=True)
            try:
                runtime.start()
                runtime.monitor()
            except InterruptedError:
                pass
            except Exception as error:
                runtime.errors.append(str(error))
                print(f'启动/运行失败：{error}', flush=True)
            finally:
                runtime.close()
                for signum, handler in old_handlers.items():
                    signal.signal(signum, handler)
            return 1 if runtime.errors else 0
    except (RuntimeError, OSError) as error:
        print(f'无法启动手动联调：{error}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
