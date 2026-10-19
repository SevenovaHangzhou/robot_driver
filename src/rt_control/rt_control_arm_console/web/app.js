// Arm console page. The server checks every command; this page only proposes targets.
import { ArmViewer } from "./viewer.js";

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const RAD = Math.PI / 180;
const CHANGED_DEG = 0.05;

const state = {
  info: null,
  status: null,
  gear: null,
  target: {}, // rad, per joint
  edited: false, // true once the operator moved a slider since the last sync
  pending: null, // {kind: "target"}
  poses: [],
  sequence: [], // pose names, in execution order (repeats allowed)
};
let viewer = null;
let events = null;

async function post(path, body = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const document = await response.json().catch(() => ({}));
  if (!response.ok || document.ok === false) {
    throw new Error(document.message || document.error || `HTTP ${response.status}`);
  }
  return document;
}

function toast(message, bad = false) {
  const element = $("toast");
  element.textContent = message;
  element.className = bad ? "toast bad" : "toast";
  element.hidden = false;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => { element.hidden = true; }, bad ? 6000 : 2500);
}

async function guarded(action) {
  try {
    return await action();
  } catch (error) {
    toast(error.message, true);
    return null;
  }
}

// ------------------------------------------------------------------ login
$("login-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  $("login-error").textContent = "";
  try {
    await post("/api/login", { token: $("token").value });
    $("login").hidden = true;
    start();
  } catch (error) {
    $("login-error").textContent = error.message;
  }
});

async function start() {
  viewer = new ArmViewer($("viewer"));
  await loadInfo();
  events = new EventSource("/api/events");
  events.onmessage = (message) => onStatus(JSON.parse(message.data));
  events.onerror = () => setChip("conn-chip", "连接中断", "bad");
}

async function loadInfo() {
  const response = await fetch("/api/info");
  if (response.status === 401) { location.reload(); return; }
  const info = await response.json();
  if (!info.urdf || !info.joints.length) {
    setTimeout(loadInfo, 2000);
    return;
  }
  state.info = info;
  state.gear = state.gear || info.gears[0].name;
  buildGears();
  buildJoints();
  renderPoses(info.poses);
  renderSequence();
  $("model-msg").textContent = "正在加载整车模型…";
  await viewer.load(info.urdf, info.arm_links || []);
  $("model-msg").hidden = true;
}

// ------------------------------------------------------------------ controls
function buildGears() {
  const gears = state.info.gears;
  const container = $("gears");
  if (gears.length === 2) {
    const toggle = document.createElement("button");
    toggle.type = "button";
    toggle.id = "gear-switch";
    toggle.className = "gear-switch";
    toggle.setAttribute("role", "switch");
    toggle.innerHTML = '<span class="knob"></span>';
    toggle.addEventListener("click", () => {
      state.gear = state.gear === gears[0].name ? gears[1].name : gears[0].name;
      markGear();
    });
    const label = document.createElement("span");
    label.id = "gear-label";
    label.className = "gear-label";
    container.replaceChildren(toggle, label);
  } else {
    const nav = document.createElement("nav");
    nav.className = "seg";
    nav.append(...gears.map((gear) => {
      const button = document.createElement("button");
      button.type = "button";
      button.textContent = gear.label;
      button.dataset.gear = gear.name;
      button.addEventListener("click", () => { state.gear = gear.name; markGear(); });
      return button;
    }));
    container.replaceChildren(nav);
  }
  markGear();
}

function markGear() {
  const gears = state.info.gears;
  const gear = gears.find((item) => item.name === state.gear) || gears[0];
  const toggle = $("gear-switch");
  if (toggle) {
    const second = gear.name === gears[1].name;
    toggle.setAttribute("aria-checked", String(second));
    toggle.classList.toggle("on", second);
    toggle.title = `${gears[0].label} / ${gears[1].label}`;
    $("gear-label").textContent = gear.label;
    $("gear-label").classList.toggle("sport", second);
    return;
  }
  for (const button of $("gears").querySelectorAll("button")) {
    button.classList.toggle("active", button.dataset.gear === state.gear);
  }
}

function buildJoints() {
  const container = $("joints");
  container.replaceChildren(...state.info.joints.map((joint) => {
    const row = document.createElement("div");
    row.className = "joint";
    row.dataset.joint = joint.name;
    const label = document.createElement("label");
    label.textContent = joint.name.replace(/^left_joint/, "J");
    label.title = joint.name;
    const range = document.createElement("input");
    range.type = "range";
    range.min = (joint.lower * DEG).toFixed(1);
    range.max = (joint.upper * DEG).toFixed(1);
    range.step = "0.1";
    const number = document.createElement("input");
    number.type = "number";
    number.step = "0.1";
    number.min = range.min;
    number.max = range.max;
    const meta = document.createElement("div");
    meta.className = "meta";
    meta.innerHTML = `<span class="measured">实测 --</span><span>${range.min}° ~ ${range.max}°</span>`;
    const set = (degrees) => {
      const clamped = Math.min(Math.max(degrees, joint.lower * DEG), joint.upper * DEG);
      state.target[joint.name] = clamped * RAD;
      state.edited = true;
      renderTargets();
    };
    range.addEventListener("input", () => set(Number(range.value)));
    number.addEventListener("change", () => {
      if (Number.isFinite(Number(number.value))) set(Number(number.value));
    });
    row.append(label, range, number, meta);
    return row;
  }));
}

function measured() {
  return state.status ? state.status.positions : {};
}

function syncTargets() {
  const positions = measured();
  for (const joint of state.info.joints) {
    const value = positions[joint.name];
    if (Number.isFinite(value)) {
      state.target[joint.name] = Math.min(Math.max(value, joint.lower), joint.upper);
    }
  }
  state.edited = false;
  renderTargets();
}

function targetComplete() {
  return state.info.joints.every((joint) => Number.isFinite(state.target[joint.name]));
}

function renderTargets() {
  const positions = measured();
  const moving = state.status && state.status.moving;
  for (const row of $("joints").children) {
    const name = row.dataset.joint;
    const [, range, number] = row.children;
    const value = (moving ? moving.target[name] : state.target[name]) * DEG;
    if (Number.isFinite(value)) {
      if (document.activeElement !== number) number.value = value.toFixed(1);
      range.value = value.toFixed(1);
    }
    const current = positions[name] * DEG;
    row.querySelector(".measured").textContent =
      Number.isFinite(current) ? `实测 ${current.toFixed(2)}°` : "实测 --";
    row.classList.toggle("changed", Math.abs(value - current) > CHANGED_DEG);
  }
  const ghostTarget = moving ? moving.target : state.target;
  const differs = state.info.joints.some(
    (joint) => Math.abs((ghostTarget[joint.name] - positions[joint.name]) * DEG) > CHANGED_DEG);
  if (viewer) viewer.setGhost(ghostTarget, Boolean(moving) || differs);
}

$("sync-btn").addEventListener("click", syncTargets);

// ------------------------------------------------------------------ motion
async function openConfirm(kind) {
  if (!targetComplete()) return;
  const preview = await guarded(() => post("/api/preview", { target: state.target, gear: state.gear }));
  if (!preview) return;
  const gear = state.info.gears.find((item) => item.name === state.gear);
  state.pending = { kind };
  $("confirm-title").textContent = "发送到目标";
  $("confirm-body").textContent =
    `${gear.label}档 · 预计 ${preview.duration_s.toFixed(1)} s · 最大关节变化 ` +
    `${preview.max_delta_deg.toFixed(1)}°（S 型曲线，各轴同时到达）`;
  const positions = measured();
  const rows = state.info.joints.map((joint) => {
    const from = positions[joint.name] * DEG;
    const to = state.target[joint.name] * DEG;
    return `<tr><td>${joint.name}</td><td>${from.toFixed(1)}°</td><td>→</td>` +
      `<td>${to.toFixed(1)}°</td><td>${(to - from >= 0 ? "+" : "") + (to - from).toFixed(1)}°</td></tr>`;
  });
  $("confirm-table").innerHTML = rows.join("");
  $("confirm").showModal();
}

$("confirm").addEventListener("close", async () => {
  const pending = state.pending;
  state.pending = null;
  if ($("confirm").returnValue !== "ok" || !pending) return;
  await guarded(() => post("/api/execute", { target: state.target, gear: state.gear }));
});

$("send-btn").addEventListener("click", () => openConfirm("target"));
$("stop-btn").addEventListener("click", () => guarded(() => post("/api/stop")));

// ------------------------------------------------------------------ poses
$("pose-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const name = $("pose-name").value.trim();
  try {
    await post("/api/save_pose", { name });
  } catch (error) {
    if (!error.message.includes("已存在") || !confirm(`覆盖已有位姿「${name}」？`)) {
      if (!error.message.includes("已存在")) toast(error.message, true);
      return;
    }
    if (!(await guarded(() => post("/api/save_pose", { name, overwrite: true })))) return;
  }
  $("pose-name").value = "";
  toast(`已保存位姿「${name}」`);
});

function renderPoses(poses) {
  state.poses = poses;
  $("poses").replaceChildren(...poses.map((pose) => {
    const item = document.createElement("li");
    const label = document.createElement("span");
    label.textContent = pose.name;
    const saved = document.createElement("small");
    saved.textContent = (pose.saved_at || "").slice(5, 16).replace("T", " ");
    const go = document.createElement("button");
    go.type = "button";
    go.className = "small primary motion";
    go.textContent = "到达";
    go.addEventListener("click", async () => {
      // Saved poses go straight to execution; the server keeps every motion check.
      Object.assign(state.target, pose.joints);
      state.edited = true;
      renderTargets();
      await guarded(() => post("/api/goto_pose", { name: pose.name, gear: state.gear }));
    });
    const add = document.createElement("button");
    add.type = "button";
    add.className = "small add";
    add.textContent = "+ 序列";
    add.addEventListener("click", () => {
      state.sequence.push(pose.name);
      renderSequence();
    });
    const remove = document.createElement("button");
    remove.type = "button";
    remove.className = "small";
    remove.textContent = "删除";
    remove.addEventListener("click", () => {
      if (confirm(`删除位姿「${pose.name}」？`)) {
        guarded(() => post("/api/delete_pose", { name: pose.name }));
      }
    });
    item.append(label, saved, go, add, remove);
    return item;
  }));
  // Drop deleted poses from the sequence being edited.
  const names = new Set(poses.map((pose) => pose.name));
  const kept = state.sequence.filter((name) => names.has(name));
  if (kept.length !== state.sequence.length) {
    state.sequence = kept;
    renderSequence();
  }
  updateButtons();
}

function renderSequence() {
  const running = state.status && state.status.sequence;
  const names = running ? running.names : state.sequence;
  $("sequence").replaceChildren(...names.map((name, index) => {
    const item = document.createElement("li");
    const label = document.createElement("span");
    label.textContent = name;
    item.append(label);
    if (running) {
      item.classList.toggle("done", index < running.index);
      item.classList.toggle("current", index === running.index);
    } else {
      const remove = document.createElement("button");
      remove.type = "button";
      remove.className = "small remove";
      remove.textContent = "×";
      remove.title = "移出序列";
      remove.addEventListener("click", () => {
        state.sequence.splice(index, 1);
        renderSequence();
      });
      item.append(remove);
    }
    return item;
  }));
  $("sequence-empty").hidden = names.length > 0;
  $("sequence-count").textContent = running
    ? `执行中 ${Math.min(running.index + 1, running.total)} / ${running.total}`
    : names.length ? `${names.length} 个位姿` : "";
  updateButtons();
}

$("sequence-run").addEventListener("click", async () => {
  if (!state.sequence.length) return;
  await guarded(() => post("/api/run_sequence", { names: state.sequence, gear: state.gear }));
});
$("sequence-clear").addEventListener("click", () => {
  state.sequence = [];
  renderSequence();
});

// ------------------------------------------------------------------ lease / enable
$("lease-btn").addEventListener("click", () => {
  const mine = state.status && state.status.lease === "you";
  guarded(() => post(mine ? "/api/release" : "/api/acquire"));
});

$("enable-btn").addEventListener("click", async () => {
  if (!confirm("确认使能左臂？使能后关节保持当前位置。")) return;
  const result = await guarded(() => post("/api/enable", { enabled: true }));
  if (result) toast("使能完成");
});
$("disable-btn").addEventListener("click", async () => {
  const result = await guarded(() => post("/api/enable", { enabled: false }));
  if (result) toast("已失能");
});
$("reset-btn").addEventListener("click", async () => {
  if (!confirm("确认故障复位？\n只清除驱动故障并回到“禁止合闸”，不会使能，也不会运动。\n请先确认故障原因（急停、掉线、碰撞等）已经排除。")) return;
  const result = await guarded(() => post("/api/reset_fault"));
  if (result) toast("故障已复位，可以重新使能");
});

// ------------------------------------------------------------------ status
function setChip(id, text, kind = "") {
  const chip = $(id);
  chip.textContent = text;
  chip.className = `chip ${kind}`.trim() + (chip.classList.contains("button") ? " button" : "");
}

function onStatus(status) {
  const wasMoving = Boolean(state.status && state.status.moving);
  const sequenceChanged =
    JSON.stringify((state.status && state.status.sequence) || null) !==
    JSON.stringify(status.sequence || null);
  state.status = status;
  if (sequenceChanged) renderSequence();
  if (status.poses) renderPoses(status.poses);
  if (!state.info) return;
  // After a move (or stop) the sliders follow the measured pose again.
  if ((!state.edited || wasMoving) && !status.moving) syncTargets();
  setChip("conn-chip", status.feedback_fresh ? "反馈正常" : "反馈超时", status.feedback_fresh ? "ok" : "bad");
  const enable = status.enable;
  const enableText = { ENABLED: "已使能", IDLE: "未使能", DISCONNECTED: "使能管理未连接" }[enable.state]
    || (enable.state === "FAILED" ? `故障（${enable.stage}）` : `${enable.state} ${enable.stage}`);
  setChip("enable-chip", enable.busy ? "使能切换中…" : enableText,
    enable.state === "ENABLED" ? "ok" : enable.state === "IDLE" ? "" : "warn");
  setChip("ctrl-chip", `${status.controller.name || "控制器"} ${status.controller.active ? "激活" : "未激活"}`,
    status.controller.active ? "ok" : "");
  $("mock-chip").hidden = status.mock !== true;
  const lease = { you: "释放控制权", other: "他人控制中", none: "获取控制权" }[status.lease];
  $("lease-btn").textContent = lease;
  $("lease-btn").className = `lease ${status.lease === "you" ? "mine" : status.lease === "other" ? "other" : ""}`;
  if (viewer) viewer.setMeasured(status.positions);
  renderTargets();
  renderProgress(status.moving);
  $("ready-msg").textContent = status.motion_ready ? "" : status.not_ready_reason;
  $("result-msg").textContent = status.last_result;
  updateButtons();
}

function renderProgress(moving) {
  const element = $("progress");
  element.hidden = !moving;
  if (!moving) return;
  const fraction = moving.duration_s > 0 ? Math.min(moving.elapsed_s / moving.duration_s, 1) : 1;
  element.querySelector("i").style.width = `${(fraction * 100).toFixed(1)}%`;
  const what = moving.kind === "stop" ? "正在停止" : moving.label ? `前往「${moving.label}」` : "运动中";
  element.querySelector("span").textContent =
    `${what} ${moving.elapsed_s.toFixed(1)} / ${moving.duration_s.toFixed(1)} s`;
}

function updateButtons() {
  const status = state.status;
  const mine = status && status.lease === "you";
  const moving = Boolean(status && status.moving);
  const busy = status && status.enable.busy;
  const ready = status && status.motion_ready && mine && !moving;
  $("send-btn").disabled = !ready || !state.info;
  const sequencing = Boolean(status && status.sequence);
  $("stop-btn").disabled = !moving && !sequencing;
  $("sync-btn").disabled = moving;
  $("enable-btn").disabled = !mine || moving || busy || (status && status.enable.state === "ENABLED");
  $("disable-btn").disabled = !mine || moving || busy || (status && status.enable.state !== "ENABLED");
  // Only offered when enable_manager reports FAILED; enable_manager still decides.
  $("reset-btn").disabled = !mine || moving || busy || !(status && status.enable.state === "FAILED");
  $("reset-btn").hidden = !(status && status.enable.state === "FAILED");
  $("pose-form").querySelector("button").disabled = !mine || moving || !(status && status.feedback_fresh);
  for (const row of $("joints").children) {
    for (const input of row.querySelectorAll("input")) input.disabled = moving;
  }
  for (const button of $("poses").querySelectorAll("button.motion")) button.disabled = !ready || sequencing;
  for (const button of $("poses").querySelectorAll("button.add")) button.disabled = sequencing;
  for (const button of $("poses").querySelectorAll("button:not(.motion):not(.add)")) button.disabled = !mine;
  $("sequence-run").disabled = !ready || sequencing || !state.sequence.length;
  $("sequence-clear").disabled = sequencing || !state.sequence.length;
}
