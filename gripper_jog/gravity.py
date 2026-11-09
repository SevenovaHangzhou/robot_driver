"""Model-matched static gravity and a single MuJoCo servo implementation.

The external static_gravity algorithm is built into a private, content-addressed
cache using this simulation's URDF. Its original parameters are never rewritten.
No ROS effort/current conversion is performed here.
"""
import fcntl
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import mujoco
import numpy as np

from .errors import JogError


SOURCE_FILES = ('__init__.py', 'urdf_parameters.py', 'export_parameters.py',
                'arm_gravity.cpp', 'arm_gravity.hpp', 'arm_gravity_c.h', 'CMakeLists.txt')
CACHE = Path(__file__).resolve().parents[1] / '.runtime/static-gravity'


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def load_static_gravity(source, urdf, names, *, cache=CACHE):
    """Compile once per algorithm + URDF, then validate and load the C ABI."""
    source, urdf, cache = Path(source).resolve(), Path(urdf).resolve(), Path(cache).resolve()
    if (source / 'static_gravity').is_dir():
        source /= 'static_gravity'
    try:
        files = {name: (source / name).read_bytes() for name in SOURCE_FILES}
    except OSError as exc:
        raise JogError('gravity_missing', f'静态重力源码不完整，请用 --gravity-source 指定 static_gravity 目录：{source}') from exc
    identity = {'source_sha256': {name: _sha(data) for name, data in files.items()},
                'urdf_sha256': _sha(urdf.read_bytes())}
    key = _sha(json.dumps(identity, sort_keys=True).encode())
    destination = cache / key
    cache.mkdir(parents=True, exist_ok=True)
    with (cache / 'build.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not destination.exists():
            with tempfile.TemporaryDirectory(prefix='build-', dir=cache) as temporary:
                stage = Path(temporary)
                package = stage / 'static_gravity'
                package.mkdir()
                for name, data in files.items():
                    (package / name).write_bytes(data)
                commands = ([sys.executable, '-B', str(package / 'export_parameters.py'), '--urdf', str(urdf)],
                            ['cmake', '-S', str(package), '-B', str(package / 'build'),
                             '-DCMAKE_BUILD_TYPE=Release', '-DARM_GRAVITY_BUILD_EXAMPLES=OFF'],
                            ['cmake', '--build', str(package / 'build'), '-j2'])
                try:
                    for command in commands:
                        subprocess.run(command, check=True, capture_output=True, text=True, timeout=60)
                except (OSError, subprocess.SubprocessError) as exc:
                    details = (getattr(exc, 'stderr', '') or str(exc))[-2000:]
                    raise JogError('gravity_build', f'静态重力库构建失败，请检查 CMake/C++ 编译器：{details}') from exc
                manifest = {**identity, 'library_sha256': _sha((package / 'build/libarm_static_gravity.so').read_bytes())}
                (stage / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
                # Rename only a complete package; readers cannot see a partial build.
                stage.rename(destination)
        package = destination / 'static_gravity'
        try:
            manifest = json.loads((destination / 'manifest.json').read_text())
            if (any(manifest.get(k) != v for k, v in identity.items())
                    or _sha((package / 'build/libarm_static_gravity.so').read_bytes()) != manifest['library_sha256']
                    or any((package / name).read_bytes() != data for name, data in files.items())):
                raise ValueError('Cache identity differs')
            module_name = '_jog_static_gravity_' + _sha(str(package).encode())
            if module_name not in sys.modules:
                spec = importlib.util.spec_from_file_location(module_name, package / '__init__.py',
                                                              submodule_search_locations=[str(package)])
                module = importlib.util.module_from_spec(spec)
                sys.modules[module_name] = module
                try:
                    spec.loader.exec_module(module)
                except Exception:
                    sys.modules.pop(module_name, None)
                    raise
            calculator = sys.modules[module_name].Gravity(urdf=urdf, names=names)
            if calculator.parameters['source_urdf_sha256'] != identity['urdf_sha256']:
                raise ValueError('URDF changed during gravity build')
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            raise JogError('gravity_model', f'静态重力缓存或模型校验失败：{exc}；检查后删除缓存重建：{destination}') from exc
    return calculator


class GravityBackend:
    """Delegate model state; own all actuator commands, including preview data."""

    def __init__(self, backend, mode, calculator=None):
        if mode not in ('static', 'mujoco', 'off') or (mode == 'static' and calculator is None):
            raise ValueError('Invalid gravity mode/calculator')
        self.original = backend
        self.mode, self.calculator = mode, calculator
        self.config = {**backend.config, 'gravity_feedforward': mode != 'off'}
        self.last_torques = np.zeros(14)
        self.last_compute_s = 0.
        self.gravity_fault = ''
        self.evaluations = 0
        self.validation_error_nm = None
        if calculator is not None:
            self.base_names = [calculator.parameters['topology']['arms'][side]['base_link']
                               for side in ('left', 'right')]
            self._validate_model()

    def __getattr__(self, name):
        return getattr(self.original, name)

    def _torques(self, data):
        if self.mode == 'off':
            return np.zeros(14)
        if self.mode == 'mujoco':
            return data.qfrc_bias[self.vadr].copy()
        # Actual measured angles and actual parent-to-world rotations. The model
        # already contains the J1 installation transform and both fixed tools.
        bases = np.array([data.body(name).xmat.reshape(3, 3) for name in self.base_names])
        return self.calculator.torques(data.qpos[self.qadr], gravity=self.model.opt.gravity, bases=bases)

    def _validate_model(self):
        data = mujoco.MjData(self.model)
        maximum = 0.
        rng = np.random.default_rng(2011)
        # A separate mjData and zero velocities isolate g(q), leaving the live
        # pose, mocap and simulation clock untouched. Catch stale scene inertials.
        poses = [self.q, np.zeros(14), *rng.uniform(self.lower, self.upper, size=(6, 14))]
        for q in poses:
            data.qpos[:] = self.data.qpos
            data.qpos[self.qadr] = q
            data.qvel[:] = 0
            mujoco.mj_forward(self.model, data)
            difference = float(np.max(np.abs(self._torques(data) - data.qfrc_bias[self.vadr])))
            maximum = max(maximum, difference)
            if not np.isfinite(difference) or difference > 1e-7:
                raise JogError('gravity_model', f'静态重力与当前 MuJoCo 模型不一致（最大差 {difference:g} N·m），请同步 URDF 和场景')
        self.validation_error_nm = maximum

    def command(self, data, q, v, *, record=False):
        if self.gravity_fault:
            raise RuntimeError(self.gravity_fault)
        q, v = np.asarray(q, dtype=float), np.asarray(v, dtype=float)
        if q.shape != (14,) or v.shape != (14,) or not np.isfinite([q, v]).all():
            raise ValueError('Invalid desired state')
        if np.any(q < self.lower - 1e-8) or np.any(q > self.upper + 1e-8):
            raise ValueError('Desired position outside configured simulation limits')
        started = time.perf_counter()
        try:
            torque = np.asarray(self._torques(data), dtype=float)
            if torque.shape != (14,) or not np.isfinite(torque).all():
                raise ValueError('Invalid gravity torque')
            control = q + (self.kv * v + torque) / self.kp
            if not np.isfinite(control).all():
                raise ValueError('Invalid actuator command')
        except Exception as exc:
            self.gravity_fault = f'静态重力计算失败，需重启仿真：{exc}'
            self.last_torques[:] = 0
            # Remove old feedforward before handing failure to the jog stop path.
            self.data.ctrl[self.aids] = self.q
            data.ctrl[self.aids] = data.qpos[self.qadr]
            raise RuntimeError(self.gravity_fault) from exc
        # MuJoCo position actuator: kp*(ctrl-q_actual)-kv*qvel_actual.
        # This is the only gravity term; never add qfrc_bias again in static mode.
        data.ctrl[self.aids] = control
        if record:
            self.last_torques = torque.copy()
            self.last_compute_s = time.perf_counter() - started
            self.evaluations += 1

    def step(self, q, v=None):
        self.command(self.data, q, np.zeros(14) if v is None else v, record=True)
        mujoco.mj_step(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError('Non-finite MuJoCo state')

    def gravity_state(self):
        parameters = self.calculator.parameters if self.calculator else {}
        return {'mode': self.mode, 'enabled': self.mode != 'off', 'backend': 'mujoco_actuator',
                'source': 'static_gravity' if self.mode == 'static' else self.mode,
                'joint_names': list(self.names), 'torques_nm': self.last_torques.tolist(),
                'period_s': self.dt, 'evaluations': self.evaluations,
                'last_compute_s': self.last_compute_s, 'fault': self.gravity_fault,
                'parameter_signature': parameters.get('parameter_signature'),
                'source_urdf_sha256': parameters.get('source_urdf_sha256'),
                'validation_max_error_nm': self.validation_error_nm,
                'hardware_connected': False}


class GravityFollowerMixin:
    """Use the same actuator equation for direct Mink collision previews."""

    def preview_is_clear(self, target, velocity):
        b, data = self.backend, self.preview_data
        mujoco.mj_copyData(data, b.model, b.data)
        for step in range(self.preview_steps):
            b.command(data, target, velocity if step < self.substeps else np.zeros(14))
            mujoco.mj_step(b.model, data)
            mujoco.mj_fwdPosition(b.model, data)
            if (not np.isfinite(data.qpos).all()
                    or self.collision_clearance(data, self.guard_distance)['distance_m'] < self.guard_distance):
                return False
        return True
