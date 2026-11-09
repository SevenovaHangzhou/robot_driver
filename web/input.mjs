// Pure input state machine; DOM/network handling lives in jog.mjs.
export const KEY_DIRECTIONS = Object.freeze({
  ArrowUp: 'x+', ArrowDown: 'x-', ArrowLeft: 'y+', ArrowRight: 'y-',
  Equal: 'z+', Minus: 'z-', NumpadAdd: 'z+', NumpadSubtract: 'z-',
  KeyQ: 'yaw+', KeyZ: 'yaw-', KeyW: 'pitch+', KeyX: 'pitch-', KeyE: 'roll+', KeyC: 'roll-',
});

export const isRotation = direction => /^(yaw|pitch|roll)[+-]$/.test(direction);
export function motionCommand(direction, speedMmS, speedDegS) {
  return isRotation(direction) ? {robot_rotation: direction, rotation_frame: 'j7', speed_deg_s: speedDegS}
    : {robot_direction: direction, speed_mm_s: speedMmS};
}

// Moving J7-frame incremental quaternion log; unlike Euler subtraction this crosses
// ±180° and pitch ±90° without inventing an angular jump.
function rotationIncrement(previous, current, axis) {
  const [x, y, z, w] = current, [a, b, c, d] = previous;
  let vector = [d*x-a*w-b*z+c*y, d*y+a*z-b*w-c*x, d*z-a*y+b*x-c*w];
  let scalar = w*d+x*a+y*b+z*c;
  if (scalar < 0) { scalar = -scalar; vector = vector.map(v => -v); }
  const norm = Math.hypot(...vector);
  return norm < 1e-12 ? 0 : 2 * Math.atan2(norm, scalar) * vector[axis] / norm * 180 / Math.PI;
}

// Display movement from controller-reported TCP coordinates, never wall time.
export class MotionFeedback {
  begin(state, direction, distanceMm, durationSeconds, speedMmS = null, holdId = null, rotationAxis = null) {
    const previous = this.segment;
    const continuing = holdId !== null && previous?.holdId === holdId &&
      previous.identity === state.model_identity && previous.direction === direction;
    this.segment = {identity: state.model_identity, start: [...state.tcp_mm],
      axis: 'xyz'.indexOf(direction[0]), sign: direction[1] === '+' ? 1 : -1,
      distanceMm, durationSeconds, speedMmS, holdId, direction,
      holdStart: continuing ? previous.holdStart : [...state.tcp_mm]};
    if (isRotation(direction)) {
      Object.assign(this.segment, {angular: true, axis: 'xyz'.indexOf(rotationAxis?.[0]),
        sign: rotationAxis?.[1] === '+' ? 1 : -1, angle: continuing ? previous.angle : 0,
        previousOrientation: continuing ? previous.previousOrientation
          : state.rotation_orientation_xyzw ? [...state.rotation_orientation_xyzw] : null});
    }
  }
  update(state) {
    const segment = this.segment;
    if (!segment || state.model_identity !== segment.identity || !state.tcp_mm) return null;
    if (segment.angular && state.rotation_orientation_xyzw && segment.previousOrientation) {
      segment.angle += rotationIncrement(segment.previousOrientation, state.rotation_orientation_xyzw, segment.axis) * segment.sign;
      segment.previousOrientation = [...state.rotation_orientation_xyzw];
    }
    const travelled = segment.angular ? Math.max(0, segment.angle)
      : Math.max(0, (state.tcp_mm[segment.axis] - segment.start[segment.axis]) * segment.sign);
    const continuous = segment.distanceMm === null;
    return {travelled, remaining: continuous ? null : Math.max(0, segment.distanceMm - travelled),
      target: segment.distanceMm, percent: continuous ? null : Math.min(100, travelled / segment.distanceMm * 100),
      duration: segment.durationSeconds,
      angular: Boolean(segment.angular),
      holdTravelled: segment.angular ? travelled
        : Math.max(0, (state.tcp_mm[segment.axis] - segment.holdStart[segment.axis]) * segment.sign),
      requestedSpeed: segment.speedMmS,
      commandSpeed: segment.angular ? state.tcp_actual_angular_speed_deg_s ?? state.tcp_command_angular_speed_deg_s ?? null
        : Number.isFinite(state.tcp_actual_speed_mm_s) ? state.tcp_actual_speed_mm_s
        : Number.isFinite(state.tcp_command_speed_mm_s) ? state.tcp_command_speed_mm_s : null,
      label: state.stopping ? '正在减速' : state.busy ? (continuous ? segment.angular ? '连续旋转' : '连续移动' : travelled < .01 ? '轨迹已接受 · 起步位移很小' : '正在运动')
        : continuous ? '已停止' : '本段已结束'};
  }
}

export class JogInput {
  constructor({step, stop, release = stop}) {
    this.step = step; this.stop = stop; this.release = release;
    this.armed = false; this.busy = false; this.held = new Set();
  }
  arm() { this.held.clear(); this.armed = true; }
  cancel(reason) {
    const active = this.armed || this.held.size > 0;
    this.armed = false; this.held.clear();
    if (active) this.stop(reason);
  }
  down(code, {repeat = false, editable = false, modified = false} = {}) {
    if (code === 'Escape') { this.cancel('键盘停止'); return true; }
    if (!(code in KEY_DIRECTIONS) || editable || modified || !this.armed) return false;
    if (repeat || this.held.has(code)) return true;
    this.held.add(code);
    if (this.held.size > 1) { this.cancel('多方向按键冲突'); return true; }
    if (!this.busy) { this.busy = true; this.step(KEY_DIRECTIONS[code]); }
    return true;
  }
  resume() {
    if (!this.armed || this.busy || this.held.size !== 1) return false;
    this.busy = true;
    this.step(KEY_DIRECTIONS[this.held.values().next().value]);
    return true;
  }
  up(code) {
    if (!this.held.has(code)) return false;
    this.held.delete(code);
    // Every release invalidates the server epoch, including requests still planning.
    this.release('已松开方向键');
    return true;
  }
}
