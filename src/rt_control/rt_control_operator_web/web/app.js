// ELECTRI-109 operator console client.
//
// The browser only sends the raw stick (unit disc, base_link x forward / y left)
// and a yaw direction. The server shapes the stick, applies the gear, the
// S-curve acceleration profile and all stop behaviour, so a released stick is
// ramped down server-side rather than cut to zero. Entering the chassis page
// takes the control lease; leaving it (or backgrounding the page) releases it.

import { ParkingBeeper } from "./beeper.js";

const TOKEN_KEY = "rt_operator_web_token";
const MUTE_KEY = "rt_operator_web_muted";
const NS = "http://www.w3.org/2000/svg";

const GEAR_LABELS = { leisure: "休闲档", sport: "运动档" };
const STOP_REASONS = {
  idle: "空闲",
  lease_acquired: "已获取控制权",
  lease_released: "已释放控制权",
  client_disconnected: "终端断开",
  soft_stop: "刹车",
  stick_released: "松开摇杆",
  command_timeout: "指令超时",
  mixed_input_rejected: "平移与旋转同时输入被拒绝",
  invalid_input: "输入无效",
  server_shutdown: "服务关闭",
  process_shutdown: "进程退出",
};
const CONTROLLER_STATES = {
  running: ["运行中", "ok"],
  alignment_gated: ["转舵对准中", "warn"],
  command_timeout: ["待命", "ok"],
  inactive: ["未激活", "warn"],
  feedback_fault: ["反馈故障", "bad"],
  invalid_command: ["指令无效", "bad"],
  steering_limit: ["转向限位", "bad"],
  wheel_slip: ["车轮打滑", "warn"],
};
const CORNER_POS = {
  front_left: [1, 1], front_right: [1, -1], rear_left: [-1, 1], rear_right: [-1, -1],
};
const FACING_VEC = { front: [1, 0], rear: [-1, 0], left: [0, 1], right: [0, -1] };
const STALE_S = 1.0;
const ACQUIRE_RETRY_MS = 1000;

const $ = (id) => document.getElementById(id);
const state = {
  ws: null,
  authed: false,
  reconnectDelay: 500,
  gears: {},
  gear: null,
  stickCfg: { deadzone: 0.1, expo: 2, snap_deg: 8 },
  minSend: 33,
  heartbeat: 50,
  ultrasonic: { channels: [], layout_confirmed: false, bands_m: [0.3, 0.6, 1.0, 1.5, 2.5] },
  lease: "none",
  leasePaused: false, // set after soft stop / timeout until the operator touches a control
  lastAcquire: 0,
  moving: false,
  stick: { pointer: null, x: 0, y: 0 },
  yaw: { pointer: null, value: 0 },
  lastSent: 0,
  sendTimer: null,
  heartbeatTimer: null,
  geometryKey: "",
  model: null,
  modelRequested: null,
  muted: localStorage.getItem(MUTE_KEY) === "1",
};

// ---------------------------------------------------------------- helpers
function send(message) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN && state.authed) {
    state.ws.send(JSON.stringify(message));
    return true;
  }
  return false;
}

let toastTimer = null;
function toast(text, warn = false) {
  const el = $("toast");
  el.textContent = text;
  el.classList.toggle("warn", warn);
  el.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove("show"), 2200);
}

const fmt = (v, d = 2) => (typeof v === "number" && Number.isFinite(v) ? v.toFixed(d) : "--");
const chassisVisible = () =>
  document.visibilityState === "visible" && $("view-chassis").classList.contains("active");

// ---------------------------------------------------------------- automatic lease
function wantLease() {
  return state.authed && chassisVisible() && !state.leasePaused;
}

function syncLease() {
  if (!state.authed) return;
  if (wantLease()) {
    if (state.lease === "none" && performance.now() - state.lastAcquire > ACQUIRE_RETRY_MS) {
      state.lastAcquire = performance.now();
      send({ type: "acquire" });
    }
  } else if (state.lease === "you" && !state.leasePaused) {
    resetInputs();
    send({ type: "release" });
  }
}

// A touch on a control resumes control after a soft stop or a timeout.
function ensureLeaseForInput() {
  beeper.unlock();
  if (state.lease === "you") return true;
  if (state.lease === "other") { toast("其他终端正在控制", true); return false; }
  state.leasePaused = false;
  state.lastAcquire = performance.now();
  send({ type: "acquire" }); // ordered before the drive message on the same socket
  state.lease = "pending";
  return true;
}

// ---------------------------------------------------------------- drive input
function sendDrive() {
  if (state.lease !== "you" && state.lease !== "pending") return;
  clearTimeout(state.sendTimer);
  state.sendTimer = null;
  state.lastSent = performance.now();
  send({ type: "drive", x: state.stick.x, y: state.stick.y, yaw: state.yaw.value });
}

// Change-driven send, throttled to one message per min_send_period.
function queueDrive() {
  const wait = state.minSend - (performance.now() - state.lastSent);
  if (wait <= 0) sendDrive();
  else if (state.sendTimer === null) state.sendTimer = setTimeout(sendDrive, wait);
}

function startHeartbeat() {
  stopHeartbeat();
  state.heartbeatTimer = setInterval(() => {
    if (performance.now() - state.lastSent >= state.heartbeat - 5) sendDrive();
  }, state.heartbeat);
}
function stopHeartbeat() {
  clearInterval(state.heartbeatTimer);
  state.heartbeatTimer = null;
}

function resetInputs(sendNow = true) {
  const had = state.stick.x !== 0 || state.stick.y !== 0 || state.yaw.value !== 0;
  state.stick = { pointer: null, x: 0, y: 0 };
  state.yaw = { pointer: null, value: 0 };
  document.querySelectorAll(".yaw-btn").forEach((b) => b.classList.remove("pressed"));
  refreshInputLocks();
  if (had && sendNow) sendDrive();
}

// ---------------------------------------------------------------- stick (game-style)
// The drag starts on the stick; afterwards the finger is tracked anywhere on
// the screen. Beyond the rim the knob stays on the rim in the finger's
// direction at full deflection, like a game virtual stick.
const stickEl = $("stick");
const knobEl = $("stick-knob");
const ghostEl = $("stick-ghost");

function stickRadius() {
  return stickEl.clientWidth * 0.34; // knob travel radius
}

function placeKnob(dx, dy, follow) {
  knobEl.classList.toggle("follow", follow);
  knobEl.style.transform = `translate(${dx.toFixed(1)}px, ${dy.toFixed(1)}px)`;
}

function updateStick(event) {
  const rect = stickEl.getBoundingClientRect();
  const r = stickRadius();
  let dx = event.clientX - (rect.left + rect.width / 2);
  let dy = event.clientY - (rect.top + rect.height / 2);
  const len = Math.hypot(dx, dy);
  if (len > r) { dx *= r / len; dy *= r / len; }
  placeKnob(dx, dy, false);
  // Screen up = +X (forward), screen left = +Y (left).
  // Truncate toward zero so rounding can never push the vector outside the unit disc.
  state.stick.x = Math.trunc((-dy / r) * 1000) / 1000;
  state.stick.y = Math.trunc((-dx / r) * 1000) / 1000;
  queueDrive();
}

function endStick() {
  if (state.stick.pointer === null) return;
  state.stick = { pointer: null, x: 0, y: 0 };
  refreshInputLocks();
  sendDrive(); // target zero immediately; the server ramps the chassis down
}

stickEl.addEventListener("pointerdown", (event) => {
  event.preventDefault();
  if (state.stick.pointer !== null) return;
  if (state.yaw.value !== 0) { toast("旋转中不能平移", true); return; }
  if (!ensureLeaseForInput()) return;
  state.stick.pointer = event.pointerId;
  try { stickEl.setPointerCapture(event.pointerId); } catch (_) { /* window listeners still track */ }
  refreshInputLocks();
  updateStick(event);
});
window.addEventListener("pointermove", (event) => {
  if (event.pointerId === state.stick.pointer) updateStick(event);
}, { passive: true });
for (const type of ["pointerup", "pointercancel"]) {
  window.addEventListener(type, (event) => {
    if (event.pointerId === state.stick.pointer) endStick();
  });
}

// ---------------------------------------------------------------- yaw buttons
document.querySelectorAll(".yaw-btn").forEach((btn) => {
  const value = Number(btn.dataset.yaw);
  btn.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    if (state.yaw.pointer !== null) return;
    if (state.stick.pointer !== null) { toast("平移中不能旋转", true); return; }
    if (!ensureLeaseForInput()) return;
    try { btn.setPointerCapture(event.pointerId); } catch (_) { /* window listeners still track */ }
    state.yaw = { pointer: event.pointerId, value };
    btn.classList.add("pressed");
    refreshInputLocks();
    sendDrive();
  });
  btn.addEventListener("contextmenu", (event) => event.preventDefault());
});
for (const type of ["pointerup", "pointercancel"]) {
  window.addEventListener(type, (event) => {
    if (event.pointerId !== state.yaw.pointer) return;
    state.yaw = { pointer: null, value: 0 };
    document.querySelectorAll(".yaw-btn").forEach((b) => b.classList.remove("pressed"));
    refreshInputLocks();
    sendDrive();
  });
}

function refreshInputLocks() {
  const blocked = state.lease === "other" || !state.authed;
  stickEl.classList.toggle("disabled", blocked || state.yaw.value !== 0);
  document.querySelectorAll(".yaw-btn").forEach((b) => {
    b.disabled = blocked || state.stick.pointer !== null;
  });
}

// Visibility and tab changes drive the automatic lease.
document.addEventListener("visibilitychange", () => {
  if (document.visibilityState !== "visible") resetInputs();
  syncLease();
});
window.addEventListener("pagehide", () => {
  resetInputs();
  if (state.lease === "you") send({ type: "release" });
});
window.addEventListener("blur", () => resetInputs());
document.addEventListener("contextmenu", (event) => event.preventDefault());

// ---------------------------------------------------------------- executed command
// The ghost ring shows what the chassis is executing while the stick is held.
// After release the knob itself follows that command back to the centre, so
// its return speed is exactly the chassis deceleration.
function commandToStick(command) {
  const gear = state.gears[state.gear];
  if (!gear) return null;
  const ratio = Math.min(Math.hypot(command.vx, command.vy) / gear.linear, 1);
  if (ratio < 1e-3) return { dx: 0, dy: 0, ratio: 0 };
  const { deadzone, expo } = state.stickCfg;
  const r = stickRadius() * (deadzone + (1 - deadzone) * Math.pow(ratio, 1 / expo));
  const angle = Math.atan2(command.vy, command.vx);
  return { dx: -Math.sin(angle) * r, dy: -Math.cos(angle) * r, ratio };
}

function renderCommand(command) {
  const pos = commandToStick(command) || { dx: 0, dy: 0, ratio: 0 };
  if (state.stick.pointer === null) {
    placeKnob(pos.dx, pos.dy, true);
    ghostEl.classList.remove("show");
  } else if (pos.ratio > 0) {
    ghostEl.style.transform = `translate(${pos.dx.toFixed(1)}px, ${pos.dy.toFixed(1)}px)`;
    ghostEl.classList.add("show");
  } else {
    ghostEl.classList.remove("show");
  }
  const gear = state.gears[state.gear];
  const yawRatio = gear && gear.angular > 0 ? Math.min(Math.abs(command.wz) / gear.angular, 1) : 0;
  document.querySelectorAll(".yaw-btn").forEach((b) => {
    const active = Math.sign(command.wz) === Number(b.dataset.yaw) && yawRatio > 1e-3;
    b.querySelector(".yaw-fill").style.opacity = active ? (0.15 + 0.45 * yawRatio).toFixed(2) : "0";
    b.classList.toggle("active-cmd", active);
  });
}

// ---------------------------------------------------------------- radar / top view
// The top view is drawn from the shared Robot Model: the server projects the
// V3 URDF chassis meshes into base_link silhouettes (plate with its openings,
// brackets, suspension carriage, steering turntables and tyres). Without a
// model it falls back to a schematic built from the controller geometry.
const radarSvg = $("radar");
let radarSensors = [];
let radarWheels = [];
let radarLabels = {};

function el(tag, attrs = {}, parent = null) {
  const node = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (parent) parent.appendChild(node);
  return node;
}

function sectorPath(r0, r1, half) {
  // Annular sector centred on +x in local coordinates.
  const p = (r, a) => `${(r * Math.cos(a)).toFixed(2)} ${(r * Math.sin(a)).toFixed(2)}`;
  return `M ${p(r0, -half)} L ${p(r1, -half)} A ${r1} ${r1} 0 0 1 ${p(r1, half)} ` +
    `L ${p(r0, half)} A ${r0} ${r0} 0 0 0 ${p(r0, -half)} Z`;
}

function arcPath(r, half) {
  const p = (a) => `${(r * Math.cos(a)).toFixed(2)} ${(r * Math.sin(a)).toFixed(2)}`;
  return `M ${p(-half)} A ${r} ${r} 0 0 1 ${p(half)}`;
}

function loopsBounds(loops, dx = 0, dy = 0) {
  const b = [Infinity, Infinity, -Infinity, -Infinity];
  for (const loop of loops) for (const [x, y] of loop) {
    b[0] = Math.min(b[0], x + dx); b[1] = Math.min(b[1], y + dy);
    b[2] = Math.max(b[2], x + dx); b[3] = Math.max(b[3], y + dy);
  }
  return b;
}

// Schematic stand-in built from the controller geometry (no Robot Model yet).
function schematicModel(geometry) {
  const mx = geometry ? geometry.module_x : [0.5, 0.5, -0.5, -0.5];
  const my = geometry ? geometry.module_y : [0.4, -0.4, 0.4, -0.4];
  const wr = geometry ? geometry.wheel_radius : [0.1, 0.1, 0.1, 0.1];
  const rect = (x0, y0, x1, y1) => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
  const x0 = Math.min(...mx) - 0.2, x1 = Math.max(...mx) + 0.2;
  const y0 = Math.min(...my) - 0.15, y1 = Math.max(...my) + 0.15;
  return {
    schematic: true,
    parts: [{ link: "schematic", hidden: false, loops: [rect(x0, y0, x1, y1)] }],
    modules: mx.map((x, i) => ({
      x, y: my[i],
      parts: [{ role: "wheel", loops: [rect(-wr[i], -0.03, wr[i], 0.03)] }],
    })),
    bounds: [x0, y0, x1, y1],
  };
}

function buildRadar(model) {
  radarSvg.textContent = "";
  radarSensors = [];
  radarWheels = [];
  radarLabels = {};
  const [bx0, by0, bx1, by1] = model.bounds;
  const cx = (bx0 + bx1) / 2;
  const cy = (by0 + by1) / 2;
  const s = Math.min(118 / ((bx1 - bx0) / 2), 96 / ((by1 - by0) / 2));
  // Robot (x forward, y left) -> SVG (x right, y down), front up, centred on the chassis.
  const P = (x, y) => [-(y - cy) * s, -(x - cx) * s];
  const R = (x, y) => [-y * s, -x * s]; // relative vectors
  const pathOf = (loops, map) => loops.map((loop) =>
    "M " + loop.map(([x, y]) => map(x, y).map((v) => v.toFixed(1)).join(" ")).join(" L ") + " Z").join(" ");

  // Faint 0.25 m grid around the chassis.
  const grid = el("g", { class: "grid" }, radarSvg);
  const step = 0.25;
  for (let x = Math.floor((bx0 - 0.5) / step) * step; x <= bx1 + 0.5; x += step) {
    const [, y] = P(x, 0);
    el("line", { x1: -180, x2: 180, y1: y.toFixed(1), y2: y.toFixed(1) }, grid);
  }
  for (let y = Math.floor((by0 - 0.5) / step) * step; y <= by1 + 0.5; y += step) {
    const [x] = P(0, y);
    el("line", { y1: -200, y2: 200, x1: x.toFixed(1), x2: x.toFixed(1) }, grid);
  }

  const radarLayer = el("g", {}, radarSvg);

  // Body parts: the largest visible part is the plate.
  const visible = model.parts.filter((p) => !p.hidden);
  const area = (p) => { const b = loopsBounds(p.loops); return (b[2] - b[0]) * (b[3] - b[1]); };
  const plate = visible.reduce((a, b) => (area(b) > area(a) ? b : a), visible[0]);
  for (const part of [plate, ...visible.filter((p) => p !== plate)]) {
    el("path", { class: part === plate ? "body-plate" : "body-part", d: pathOf(part.loops, P), "fill-rule": "evenodd" }, radarSvg);
  }
  for (const part of model.parts.filter((p) => p.hidden)) {
    el("path", { class: "body-hidden", d: pathOf(part.loops, P), "fill-rule": "evenodd" }, radarSvg);
  }

  // Steering modules rotate about their true steering axis.
  model.modules.forEach((module) => {
    const [sx, sy] = P(module.x, module.y);
    const g = el("g", { transform: `translate(${sx.toFixed(1)} ${sy.toFixed(1)})` }, radarSvg);
    const rot = el("g", {}, g);
    let wheelEl = null;
    let wheelCenter = [0, 0];
    let wheelLen = 20;
    for (const part of module.parts.filter((p) => p.role !== "wheel")) {
      el("path", { class: "caster", d: pathOf(part.loops, R), "fill-rule": "evenodd" }, rot);
    }
    for (const part of module.parts.filter((p) => p.role === "wheel")) {
      wheelEl = el("path", { class: "wheel", d: pathOf(part.loops, R), "fill-rule": "evenodd" }, rot);
      const b = loopsBounds(part.loops);
      wheelCenter = R((b[0] + b[2]) / 2, (b[1] + b[3]) / 2);
      wheelLen = (b[2] - b[0]) * s;
    }
    const dir = el("path", { class: "wheel-dir", d: "" }, rot);
    el("circle", { class: "axis-dot", r: 1.6 }, g);
    radarWheels.push({ rot, dir, wheelEl, wheelCenter, wheelLen });
  });

  // base_link: the rotation centre of chassis commands.
  const [ox, oy] = P(0, 0);
  const mark = el("g", { class: "base-mark", transform: `translate(${ox.toFixed(1)} ${oy.toFixed(1)})` }, radarSvg);
  el("circle", { r: 4 }, mark);
  el("line", { x1: -8, x2: 8, y1: 0, y2: 0 }, mark);
  el("line", { x1: 0, x2: 0, y1: -8, y2: 8 }, mark);
  el("text", { class: "small", x: -7, y: 14, "text-anchor": "end" }, mark).textContent = "base_link";

  const pb = loopsBounds(plate.loops);
  // Heading chevron just outside the front edge, between the front sensors.
  const [fx, fy] = P(pb[2], cy);
  el("path", { class: "front-mark", d: `M ${fx - 9} ${fy - 6} L ${fx} ${fy - 15} L ${fx + 9} ${fy - 6} Z` }, radarSvg);
  el("text", { class: "small", x: fx, y: fy - 20, "text-anchor": "middle" }, radarSvg).textContent = "车头 +X";

  // Dimensions and scale bar.
  const [, dimY] = P(pb[0], cy);
  el("text", { class: "dim", x: 0, y: (dimY + 14).toFixed(1), "text-anchor": "middle" }, radarSvg).textContent =
    model.schematic ? "示意（未加载模型）" : `${(pb[2] - pb[0]).toFixed(2)} × ${(pb[3] - pb[1]).toFixed(2)} m`;
  const bar = el("g", { class: "scale-bar", transform: "translate(-170 188)" }, radarSvg);
  el("line", { x1: 0, x2: (0.5 * s).toFixed(1), y1: 0, y2: 0 }, bar);
  el("line", { x1: 0, x2: 0, y1: -4, y2: 4 }, bar);
  el("line", { x1: (0.5 * s).toFixed(1), x2: (0.5 * s).toFixed(1), y1: -4, y2: 4 }, bar);
  el("text", { class: "small", x: (0.25 * s).toFixed(1), y: -6, "text-anchor": "middle" }, bar).textContent = "0.5 m";

  // Ultrasonic sectors on the plate's rounded corners (schematic mapping).
  const bands = state.ultrasonic.bands_m;
  const half = (40 / 2) * (Math.PI / 180);
  const r0 = 8;
  const bandStep = 10;
  const inset = 0.12;
  state.ultrasonic.channels.forEach((ch, index) => {
    const [cxs, cys] = CORNER_POS[ch.corner];
    const [fxv, fyv] = FACING_VEC[ch.facing];
    const edgeX = cxs > 0 ? pb[2] : pb[0];
    const edgeY = cys > 0 ? pb[3] : pb[1];
    const sx = fxv !== 0 ? edgeX : edgeX - cxs * inset;
    const sy = fyv !== 0 ? edgeY : edgeY - cys * inset;
    const [px, py] = P(sx, sy);
    const [vx, vy] = R(fxv, fyv);
    const angle = (Math.atan2(vy, vx) * 180) / Math.PI;
    const g = el("g", { class: "sensor stale", transform: `translate(${px.toFixed(1)} ${py.toFixed(1)}) rotate(${angle.toFixed(1)})` }, radarLayer);
    const bandEls = bands.map((_, b) => el("path", { class: "band", d: sectorPath(r0 + b * bandStep + 1, r0 + (b + 1) * bandStep - 1, half) }, g));
    const wave = el("path", { class: "wave", d: arcPath(r0 + bands.length * bandStep, half) }, g);
    el("animateTransform", { attributeName: "transform", type: "scale", from: "0.15", to: "1", dur: "1.6s", begin: `${(index * 0.2).toFixed(1)}s`, repeatCount: "indefinite" }, wave);
    el("animate", { attributeName: "opacity", values: "0;0.8;0", dur: "1.6s", begin: `${(index * 0.2).toFixed(1)}s`, repeatCount: "indefinite" }, wave);
    el("circle", { class: "sensor-dot", r: 2.6 }, g);
    radarSensors.push({ g, bandEls, corner: ch.corner });
  });

  // One nearest-distance label per corner, diagonally outside the sectors.
  for (const corner of Object.keys(CORNER_POS)) {
    const [cxs, cys] = CORNER_POS[corner];
    const [lx, ly] = P(cxs > 0 ? pb[2] : pb[0], cys > 0 ? pb[3] : pb[1]);
    const off = r0 + bands.length * bandStep + 10;
    radarLabels[corner] = el("text", {
      x: (lx + Math.sign(lx) * off * 0.62).toFixed(1),
      y: (ly + Math.sign(ly) * off * 0.62 + 5).toFixed(1),
      "text-anchor": "middle", class: "dist-none",
    }, radarSvg);
    radarLabels[corner].textContent = "--";
  }
}

function bandIndex(distance) {
  const bands = state.ultrasonic.bands_m;
  for (let i = 0; i < bands.length; i += 1) if (distance < bands[i]) return i + 1;
  return 0; // beyond the last band: nothing lit
}

function renderRadar(robot) {
  const model = state.model || schematicModel(robot.geometry);
  const key = (state.model ? state.model.version : JSON.stringify(robot.geometry || null)) +
    state.ultrasonic.channels.length;
  if (key !== state.geometryKey) { state.geometryKey = key; buildRadar(model); }

  const wheels = robot.wheels ? robot.wheels.modules : [];
  radarWheels.forEach((w, i) => {
    const m = wheels[i];
    const angle = m && m.angle !== null ? m.angle : 0;
    const speed = m && m.speed !== null ? m.speed : 0;
    // Steering angle is CCW-positive about +Z; SVG rotation is clockwise.
    w.rot.setAttribute("transform", `rotate(${((-angle * 180) / Math.PI).toFixed(2)})`);
    const moving = Math.abs(speed) > 0.05;
    if (w.wheelEl) w.wheelEl.classList.toggle("active", moving);
    if (!moving) { w.dir.setAttribute("d", ""); return; }
    // Chevron ahead of the tyre in its rolling direction (+x of the module).
    const sign = Math.sign(speed);
    const [wx, wy] = w.wheelCenter;
    const tip = wy - sign * (w.wheelLen / 2 + 5 + Math.min(Math.abs(speed) * 2, 10));
    const back = tip + sign * 6;
    w.dir.setAttribute("d", `M ${(wx - 5).toFixed(1)} ${back.toFixed(1)} L ${wx.toFixed(1)} ${tip.toFixed(1)} L ${(wx + 5).toFixed(1)} ${back.toFixed(1)}`);
  });

  const nearest = {};
  let overall = Infinity;
  state.ultrasonic.channels.forEach((ch, i) => {
    const sensor = radarSensors[i];
    if (!sensor) return;
    const reading = robot.ultrasonic ? robot.ultrasonic[i] : null;
    const fresh = reading && reading.range !== null && reading.age_s !== null && reading.age_s <= STALE_S;
    sensor.g.classList.toggle("stale", !fresh);
    const hit = fresh ? bandIndex(reading.range) : 0;
    sensor.bandEls.forEach((b, idx) => {
      b.setAttribute("class", idx + 1 === hit ? `band hit-${hit}` : "band");
    });
    if (fresh) {
      nearest[ch.corner] = Math.min(nearest[ch.corner] ?? Infinity, reading.range);
      overall = Math.min(overall, reading.range);
    }
  });
  for (const [corner, label] of Object.entries(radarLabels)) {
    const d = nearest[corner];
    if (d === undefined || !Number.isFinite(d)) {
      label.textContent = "--";
      label.setAttribute("class", "dist-none");
    } else {
      const band = bandIndex(d);
      label.textContent = `${d.toFixed(2)} m`;
      label.setAttribute("class", band === 1 ? "dist-1" : band === 2 ? "dist-2" : "");
    }
  }
  beeper.update(Number.isFinite(overall) ? overall : null);

  const badges = [];
  if (!state.ultrasonic.layout_confirmed) badges.push("通道映射待确认");
  if (!state.model) badges.push(robot.model && robot.model.error ? "模型未加载" : "模型加载中");
  if (robot.model && robot.model.controller_mismatch) badges.push("控制器几何与模型不一致");
  if (!robot.geometry) badges.push("几何未读取");
  $("radar-badges").innerHTML = badges.map((b) => `<span class="badge">${b}</span>`).join("");
}

// ---------------------------------------------------------------- parking beeper
// Audible only while this terminal holds the lease and sound is not muted.
const beeper = {
  ctx: null,
  tone: null,
  engine: null,
  unlock() {
    if (!this.ctx) {
      const Ctx = window.AudioContext || window.webkitAudioContext;
      if (Ctx) this.ctx = new Ctx();
    }
    if (this.ctx && this.ctx.state === "suspended") this.ctx.resume();
  },
  enabled() {
    return !state.muted && state.lease === "you" && this.ctx && this.ctx.state === "running";
  },
  audio() {
    const self = this;
    return {
      now: () => (self.ctx ? self.ctx.currentTime : performance.now() / 1000),
      beep(freq, duration) {
        const ctx = self.ctx;
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        const t = ctx.currentTime;
        osc.frequency.value = freq;
        gain.gain.setValueAtTime(0.0001, t);
        gain.gain.exponentialRampToValueAtTime(0.25, t + 0.008);
        gain.gain.exponentialRampToValueAtTime(0.0001, t + duration);
        osc.connect(gain).connect(ctx.destination);
        osc.start(t);
        osc.stop(t + duration + 0.02);
      },
      toneOn(freq) {
        const ctx = self.ctx;
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.frequency.value = freq;
        gain.gain.value = 0.16;
        osc.connect(gain).connect(ctx.destination);
        osc.start();
        self.tone = { osc, gain };
      },
      toneOff() {
        if (!self.tone) return;
        self.tone.gain.gain.setTargetAtTime(0.0001, self.ctx.currentTime, 0.02);
        self.tone.osc.stop(self.ctx.currentTime + 0.1);
        self.tone = null;
      },
    };
  },
  // Short rising (sport) or falling (leisure) chirp when the gear changes.
  chirp(up) {
    if (!this.enabled()) return;
    const ctx = this.ctx;
    const t = ctx.currentTime;
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();
    osc.type = "sawtooth";
    osc.frequency.setValueAtTime(up ? 220 : 660, t);
    osc.frequency.exponentialRampToValueAtTime(up ? 880 : 240, t + 0.28);
    const filter = ctx.createBiquadFilter();
    filter.type = "lowpass";
    filter.frequency.value = 1800;
    gain.gain.setValueAtTime(0.0001, t);
    gain.gain.exponentialRampToValueAtTime(0.12, t + 0.03);
    gain.gain.exponentialRampToValueAtTime(0.0001, t + 0.32);
    osc.connect(filter).connect(gain).connect(ctx.destination);
    osc.start(t);
    osc.stop(t + 0.35);
  },
  // distance: nearest fresh ultrasonic distance (m) or null.
  update(distance) {
    if (!this.engine) {
      if (!this.ctx) return;
      this.engine = new ParkingBeeper(this.audio(), {
        set: (fn, ms) => setTimeout(fn, ms),
        clear: (id) => clearTimeout(id),
      });
    }
    const bands = state.ultrasonic.bands_m;
    this.engine.update(this.enabled() ? distance : null, bands[bands.length - 1]);
  },
};

function renderMute() {
  const btn = $("mute");
  btn.classList.toggle("muted", state.muted);
  $("mute-icon").setAttribute("d", state.muted
    ? "M4 9h4l5-4v14l-5-4H4z M16 9l6 6 M22 9l-6 6"
    : "M4 9h4l5-4v14l-5-4H4z M16 8.5a5 5 0 0 1 0 7 M18.5 6a8.5 8.5 0 0 1 0 12");
  btn.setAttribute("aria-label", state.muted ? "提示音已关闭" : "提示音已开启");
}


// ---------------------------------------------------------------- status rendering
function renderGears() {
  const box = $("gear-op");
  const names = Object.keys(state.gears);
  if (box.dataset.names === names.join(",")) return;
  box.dataset.names = names.join(",");
  box.textContent = "";
  for (const name of names) {
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "gear-btn";
    btn.dataset.gear = name;
    btn.setAttribute("role", "radio");
    btn.textContent = GEAR_LABELS[name] || name;
    btn.addEventListener("click", () => {
      if (name === state.gear) return;
      if (state.moving) { toast("停车后才能换档", true); return; }
      if (!ensureLeaseForInput()) return;
      send({ type: "gear", gear: name });
    });
    box.appendChild(btn);
  }
}

function renderBattery(battery) {
  const el = $("battery");
  let pct = null;
  if (battery && battery.present && typeof battery.percentage === "number") pct = battery.percentage;
  $("battery-pct").textContent = pct === null ? "--" : `${Math.round(pct * 100)}%`;
  $("battery-level").style.width = pct === null ? "0" : `${Math.max(0, Math.min(1, pct)) * 100}%`;
  el.classList.toggle("unknown", pct === null);
  el.classList.toggle("low", pct !== null && pct < 0.2 && pct >= 0.1);
  el.classList.toggle("critical", pct !== null && pct < 0.1);
  el.title = battery ? `${fmt(battery.voltage, 1)} V · ${fmt(battery.age_s, 0)} s 前` : "无 /battery_state";
  return pct;
}

function renderStatus(msg) {
  const prevLease = state.lease;
  const prevGear = state.gear;
  state.lease = msg.lease;
  state.moving = msg.moving;
  state.gear = msg.gear;
  document.body.classList.toggle("sport", msg.gear === "sport");
  if (prevGear && prevGear !== msg.gear) {
    const card = document.querySelector(".drive-card");
    card.classList.remove("gear-flash");
    void card.offsetWidth; // restart the animation
    card.classList.add("gear-flash");
    beeper.chirp(msg.gear === "sport");
  }
  if (prevLease === "you" && msg.lease !== "you") {
    resetInputs(false);
    stopHeartbeat();
    if (msg.stop_reason === "command_timeout" || msg.stop_reason === "soft_stop") {
      state.leasePaused = true;
      toast(msg.stop_reason === "soft_stop" ? "已刹车，触摸摇杆恢复控制" : "指令超时已停车，触摸摇杆恢复控制", true);
    }
  }
  if (prevLease !== "you" && msg.lease === "you") startHeartbeat();
  syncLease();

  const leaseText = { you: "本机控制中", other: "其他终端控制", none: state.leasePaused ? "已暂停" : "获取中" }[msg.lease];
  const chip = $("lease-chip");
  chip.className = `sb-item ${msg.lease === "you" ? "sb-accent" : msg.lease === "other" ? "sb-warn" : ""}`;
  chip.querySelector("b").textContent = leaseText;
  refreshInputLocks();

  renderGears();
  const gearBox = $("gear-op");
  gearBox.classList.toggle("locked", msg.moving);
  gearBox.querySelectorAll(".gear-btn").forEach((b) => {
    const active = b.dataset.gear === msg.gear;
    b.classList.toggle("active", active);
    b.setAttribute("aria-checked", String(active));
  });

  const c = msg.command;
  $("rd-vx").textContent = fmt(c.vx);
  $("rd-vy").textContent = fmt(c.vy);
  $("rd-wz").textContent = fmt(c.wz);
  renderCommand(c);

  const robot = msg.robot || {};
  const version = robot.model ? robot.model.version : null;
  if (version && (!state.model || state.model.version !== version) && state.modelRequested !== version) {
    state.modelRequested = version;
    send({ type: "get_model" });
  }
  const ctrl = robot.controller;
  let ctrlLabel = "控制器未连接";
  let ctrlClass = "bad";
  let ctrlDetail = "没有订阅者";
  if (robot.controller_connected) {
    if (ctrl && ctrl.age_s !== null && ctrl.age_s <= 2) {
      const [label, cls] = CONTROLLER_STATES[ctrl.state] || [ctrl.state, ctrl.level >= 2 ? "bad" : "warn"];
      ctrlLabel = label; ctrlClass = cls; ctrlDetail = ctrl.state;
    } else {
      ctrlLabel = "控制器已连接"; ctrlClass = "warn"; ctrlDetail = "无诊断数据";
    }
  }
  $("ctrl-state").textContent = ctrlLabel;
  $("ctrl-detail").textContent = ctrlDetail;
  $("ctrl-dot").className = `dot ${ctrlClass}`;

  const pct = renderBattery(robot.battery);
  renderRadar(robot);

  $("ov-controller").textContent = robot.controller_connected ? "已连接" : "无订阅者";
  $("ov-state").textContent = ctrlLabel;
  $("ov-lease").textContent = leaseText;
  $("ov-gear").textContent = GEAR_LABELS[msg.gear] || msg.gear;
  const t = msg.target;
  $("ov-target").textContent = `vx ${fmt(t.vx)} · vy ${fmt(t.vy)} m/s · wz ${fmt(t.wz)} rad/s`;
  $("ov-command").textContent = `vx ${fmt(c.vx)} · vy ${fmt(c.vy)} m/s · wz ${fmt(c.wz)} rad/s`;
  $("ov-stop").textContent = STOP_REASONS[msg.stop_reason] || msg.stop_reason || "--";
  $("ov-battery").textContent = pct === null ? "--" : `${Math.round(pct * 100)}%，${fmt(robot.battery.voltage, 1)} V`;
  $("ov-ultra").textContent = state.ultrasonic.layout_confirmed ? "已确认" : "示意布局，待确认";
}

function setConnection(text, cls) {
  const chip = $("conn");
  chip.className = `sb-item ${cls}`;
  chip.querySelector("b").textContent = text;
  $("ov-conn").textContent = text;
}

// ---------------------------------------------------------------- connection
function connect() {
  const token = sessionStorage.getItem(TOKEN_KEY);
  if (!token) { $("login").classList.remove("hidden"); return; }
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${scheme}://${location.host}/ws`);
  state.ws = ws;
  state.authed = false;
  setConnection("连接中", "sb-warn");

  ws.addEventListener("open", () => ws.send(JSON.stringify({ type: "auth", token })));
  ws.addEventListener("message", (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "auth_ok") {
      state.authed = true;
      state.reconnectDelay = 500;
      state.gears = msg.gears;
      state.stickCfg = msg.stick;
      state.minSend = msg.min_send_period_ms;
      state.heartbeat = msg.heartbeat_period_ms;
      state.ultrasonic = msg.ultrasonic;
      state.geometryKey = "";
      state.modelRequested = null;
      state.lastAcquire = 0;
      $("login").classList.add("hidden");
      setConnection("已连接", "sb-ok");
      syncLease();
    } else if (msg.type === "status") {
      renderStatus(msg);
    } else if (msg.type === "model") {
      if (msg.version) { state.model = msg; state.geometryKey = ""; }
    } else if (msg.type === "error") {
      if (msg.code === "auth_failed") {
        sessionStorage.removeItem(TOKEN_KEY);
        $("login-error").textContent = msg.message;
        $("login").classList.remove("hidden");
      } else if (msg.code === "lease_busy") {
        // Another terminal holds the lease; keep watching.
      } else {
        toast(msg.message, true);
        if (msg.code === "mixed_input_rejected" || msg.code === "invalid_input") resetInputs(false);
      }
    }
  });
  ws.addEventListener("close", (event) => {
    resetInputs(false);
    stopHeartbeat();
    state.authed = false;
    state.lease = "none";
    placeKnob(0, 0, true);
    refreshInputLocks();
    beeper.update(null);
    setConnection("未连接", "sb-bad");
    if (event.code === 4401) return;
    setTimeout(connect, state.reconnectDelay);
    state.reconnectDelay = Math.min(state.reconnectDelay * 2, 5000);
  });
}

// ---------------------------------------------------------------- controls
function bindControls() {
  $("login-form").addEventListener("submit", (event) => {
    event.preventDefault();
    sessionStorage.setItem(TOKEN_KEY, $("token").value.trim());
    $("token").value = "";
    $("login-error").textContent = "";
    beeper.unlock();
    if (state.ws && state.ws.readyState <= WebSocket.OPEN) state.ws.close();
    else connect();
  });
  $("soft-stop").addEventListener("click", () => {
    resetInputs(false);
    state.leasePaused = true;
    send({ type: "soft_stop" });
  });
  $("mute").addEventListener("click", () => {
    state.muted = !state.muted;
    localStorage.setItem(MUTE_KEY, state.muted ? "1" : "0");
    beeper.unlock();
    renderMute();
  });
  document.querySelectorAll(".seg-item").forEach((tab) => {
    tab.addEventListener("click", () => {
      resetInputs();
      document.querySelectorAll(".seg-item").forEach((t) => t.classList.toggle("active", t === tab));
      document.querySelectorAll(".view").forEach((v) => {
        v.classList.toggle("active", v.id === `view-${tab.dataset.view}`);
      });
      if (tab.dataset.view === "chassis") state.leasePaused = false;
      syncLease();
    });
  });
}

bindControls();
renderMute();
refreshInputLocks();
buildRadar(schematicModel(null));
connect();
