// Status page view. Renders one snapshot from /ws/status; no data logic lives here.
import { Stage } from "./stage.js";
import {
  PX, MAXM, WARN, DANGER, CURRENT_MAX_A, SOC_WARN, SOC_DANGER, CORE_HOT, RT_TICK, MATRIX_ROWS, RECOVERED_MAX,
  CAMERA_WIDTH, CAMERA_STATUS_PERIOD_MS,
} from "./config.js";

const $ = (id) => document.getElementById(id);
const fix = (v, d) => (v == null || !Number.isFinite(v) ? "—" : v.toFixed(d));
const esc = (t) => String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const clock = (s) => {
  const h = Math.floor(s / 3600), m = Math.floor(s / 60) % 60, sec = Math.round(s % 60);
  const p = (n) => String(n).padStart(2, "0");
  return h ? `${p(h)}:${p(m)}:${p(sec)}` : `${p(m)}:${p(sec)}`;
};

const WORD = { ok: "运行中", warn: "有警告", fault: "故障", offline: "无数据" };
const LEVEL = { fault: "故障", warn: "警告", stale: "无数据", recovered: "已恢复" };
let latest = null;

// ------------------------------------------------------------------ header
function renderHeader(s) {
  document.body.dataset.state = s.state;
  $("status-word").textContent = WORD[s.state] || s.state;
  const label = $("robot-label");
  label.textContent = s.robot_label || "";
  label.hidden = !s.robot_label;
  const counts = ["fault", "warn", "stale"]
    .map((level) => [level, s.alerts.filter((a) => a.level === level).length])
    .filter(([, n]) => n)
    .map(([level, n]) => `${n} ${LEVEL[level]}`)
    .join(" · ");
  $("status-note").textContent = s.state === "offline" ? "与机器人断开" : counts || "无告警";
  $("freshness").textContent = s.ages.joints_s == null ? "—" : `${s.ages.joints_s.toFixed(1)} 秒前`;
  $("offline-banner").hidden = s.state !== "offline";
  $("offline-age").textContent = s.ages.joints_s == null ? "" : Math.round(s.ages.joints_s);
}
setInterval(() => ($("clock").textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false })), 250);

// ------------------------------------------------------------------ pose
const stage = new Stage($("stage-view"));
let modelState = "idle";
function loadModel(s) {
  if (!s.model || modelState !== "idle") return;
  modelState = "loading";
  stage
    .load("/api/status/model", "/status-mesh")
    .then(() => { modelState = "ready"; $("stage-loading").hidden = true; })
    .catch(() => { modelState = "failed"; $("stage-loading").textContent = "整机模型不可用"; });
}
function renderPose(s) {
  stage.set(s.joints);
  $("updown").textContent = `${fix(s.pose.updown_m, 3)} m`;
  $("pitch").textContent = `${fix(s.pose.head_pitch_deg, 1)}°`;
  $("yaw").textContent = `${fix(s.pose.head_yaw_deg, 1)}°`;
}

// ------------------------------------------------------------------ batteries
const packs = [0, 1].map((i) => {
  const root = $(`pack-${i}`);
  const cells = root.querySelector(".cells");
  cells.innerHTML = "<i></i>".repeat(10);
  return { root, cells: [...cells.children], history: [] };
});
function renderBattery(index, b) {
  const p = packs[index];
  const root = p.root;
  if (b.present && b.current_a != null) {
    p.history.push(b.current_a);
    if (p.history.length > 60) p.history.shift();
  }
  const soc = b.present ? b.soc_pct : null;
  root.classList.toggle("absent", !b.present);
  root.classList.toggle("temp-warn", b.present && b.temperature_c != null && SOC_WARN != null && false);
  root.classList.toggle("soc-danger", soc != null && SOC_DANGER != null && soc < SOC_DANGER);
  root.classList.toggle("soc-warn", soc != null && SOC_WARN != null && soc < SOC_WARN && !(SOC_DANGER != null && soc < SOC_DANGER));
  root.querySelector(".pack-name").textContent = b.name;
  root.querySelector(".pack-state").textContent = b.present ? "供电中" : "无数据";
  root.querySelector(".soc-n").textContent = soc == null ? "—" : Math.round(soc);
  const lit = soc == null ? 0 : Math.round(soc / 10);
  p.cells.forEach((cell, i) => (cell.className = i < lit ? (i === lit - 1 ? "on tip" : "on") : ""));
  const h = p.history.length > 1 ? p.history : [0, 0];
  const pts = h.map((v, i) => [(i * 116) / (h.length - 1), 28 - (Math.min(Math.max(v, 0), CURRENT_MAX_A) / CURRENT_MAX_A) * 26]);
  root.querySelector("polyline").setAttribute("points", pts.map(([x, y]) => `${x.toFixed(1)},${y.toFixed(1)}`).join(" "));
  const dot = root.querySelector("circle");
  dot.setAttribute("cx", pts.at(-1)[0]);
  dot.setAttribute("cy", pts.at(-1)[1]);
  root.querySelector(".cur").textContent = `${fix(b.current_a, 1)} A`;
  root.querySelector(".volt").textContent = `${fix(b.voltage_v, 1)} V`;
  root.querySelector(".temp").textContent = `${fix(b.temperature_c, 0)} °C`;
}
// ------------------------------------------------------------------ chassis plan
// Screen: up = model +X (forward), left = model +Y, matching REP-103. Positions come from the
// model the stack loaded; the placeholder layout is only used when it has no sonar frames.
const CENTRE = { x: 230, y: 215 };
const BODY_HALF_UNITS = 75;
const PLACEHOLDER_SENSORS = [
  { x: 195, y: 115, rot: 0, ch: 1 }, { x: 265, y: 115, rot: 0, ch: 2 },
  { x: 305, y: 160, rot: 90, ch: 3 }, { x: 305, y: 270, rot: 90, ch: 4 },
  { x: 265, y: 315, rot: 180, ch: 6 }, { x: 195, y: 315, rot: 180, ch: 5 },
  { x: 155, y: 270, rot: 270, ch: 7 }, { x: 155, y: 160, rot: 270, ch: 8 },
];
const PLACEHOLDER_WHEELS = [{ x: 185, y: 150 }, { x: 275, y: 150 }, { x: 185, y: 280 }, { x: 275, y: 280 }];
const ST = {
  safe: { f: "rgba(255,255,255,.05)", a: "rgba(255,255,255,.55)", w: 1.5, t: "rgba(255,255,255,.64)" },
  warn: { f: "rgba(255,166,61,.16)", a: "#ffa63d", w: 2, t: "#ffa63d" },
  danger: { f: "rgba(255,90,121,.24)", a: "#ff5a79", w: 2.5, t: "#ff5a79" },
  none: { f: "none", a: "rgba(255,255,255,.3)", w: 1, t: "rgba(255,255,255,.5)" },
};
const level = (d) => (d == null ? "none" : d < DANGER ? "danger" : d <= WARN ? "warn" : "safe");
// Places the model's channel numbers at the position they are declared in; wording only.
const SENSOR_WHERE = { 1: "左前", 2: "左前", 3: "右前", 4: "右前", 5: "右后", 6: "右后", 7: "左后", 8: "左后" };

// Model metres -> SVG units, so the body keeps the size the reference layout was drawn for.
function plan(geo) {
  const sonar = (geo && geo.sonar) || [];
  const body = (geo && geo.body) || null;
  if (!sonar.length || !body) {
    return { placeholder: true, scale: 0, body, sensors: PLACEHOLDER_SENSORS, wheels: PLACEHOLDER_WHEELS };
  }
  const scale = BODY_HALF_UNITS / Math.max(body.half_length_m, body.half_width_m, 1e-6);
  const toScreen = (x, y) => ({ x: CENTRE.x - y * scale, y: CENTRE.y - x * scale });
  return {
    placeholder: false,
    scale,
    body,
    sensors: sonar.map((item) => {
      const point = toScreen(item.x_m, item.y_m);
      return { x: point.x, y: point.y, rot: -item.yaw_deg, ch: item.channel };
    }),
    wheels: ((geo && geo.wheels) || []).map((item) => toScreen(item.x_m, item.y_m)),
  };
}

function drawChassisFrame(p) {
  const cx = CENTRE.x, cy = CENTRE.y;
  const halfX = p.placeholder ? 75 : p.body.half_width_m * p.scale;
  const halfY = p.placeholder ? 100 : p.body.half_length_m * p.scale;
  const rings = [0.5, 1.0].map((range) => ({
    range,
    x: cx - halfX - range * PX,
    y: cy - halfY - range * PX,
    w: 2 * (halfX + range * PX),
    h: 2 * (halfY + range * PX),
  }));
  const ring = rings
    .map((r) => `<rect x="${r.x.toFixed(1)}" y="${r.y.toFixed(1)}" width="${r.w.toFixed(1)}" height="${r.h.toFixed(1)}" rx="${(r.w * 0.28).toFixed(1)}" fill="none" stroke="rgba(255,255,255,.14)" stroke-dasharray="2 5"/>`)
    .join("");
  const ringLabels = `<text x="${(cx + rings[1].w / 2 + 3).toFixed(1)}" y="${cy - 6}" font-size="11" fill="rgba(255,255,255,.5)">0.5</text>`
    + `<text x="${(cx + rings[1].w / 2 + 3).toFixed(1)}" y="${cy + 12}" font-size="11" fill="rgba(255,255,255,.5)">1.0 m</text>`;
  return `<g id="plan-frame">${ring}${ringLabels}
    <rect x="${(cx - halfX).toFixed(1)}" y="${(cy - halfY).toFixed(1)}" width="${(2 * halfX).toFixed(1)}" height="${(2 * halfY).toFixed(1)}" rx="${Math.min(halfX, halfY) * 0.27}" fill="rgba(255,255,255,.06)" stroke="rgba(255,255,255,.35)" stroke-width="1.2"/>
    <path d="M${cx - 8} ${cy - halfY + 13}L${cx} ${cy - halfY + 6}L${cx + 8} ${cy - halfY + 13}" fill="none" stroke="rgba(255,255,255,.6)" stroke-width="1.2" stroke-linecap="round" stroke-linejoin="round"/>
  </g>`;
}

function renderChassis(c, geo) {
  const p = plan(geo);
  const frame = document.getElementById("plan-frame");
  const html = drawChassisFrame(p);
  if (!frame || frame.outerHTML.slice(0, 200) !== html.slice(0, 200)) {
    const existing = document.getElementById("plan-frame");
    if (existing) existing.outerHTML = html;
  }
  let s = "";
  p.sensors.forEach((q, i) => {
    const d = c.sonar[i];
    const lv = level(d), st = ST[lv];
    const L = (d == null ? 1 : Math.min(d, MAXM)) * PX, hx = (L * 0.3746).toFixed(1), hy = (L * 0.9272).toFixed(1);
    s += `<g transform="translate(${q.x.toFixed(1)} ${q.y.toFixed(1)}) rotate(${q.rot.toFixed(1)})"><path d="M0 0L-${hx} -${hy}A${L} ${L} 0 0 1 ${hx} -${hy}Z" fill="${st.f}"${lv === "none" ? ' stroke="rgba(255,255,255,.3)" stroke-dasharray="3 4"' : ""}/>`;
    if (lv !== "none") s += `<path d="M-${hx} -${hy}A${L} ${L} 0 0 1 ${hx} -${hy}" fill="none" stroke="${st.a}" stroke-width="${st.w}" stroke-linecap="round"${lv === "danger" ? ' style="filter:drop-shadow(0 0 5px #ff5a79)"' : ""}/>`;
    s += '<circle r="2" fill="rgba(255,255,255,.45)"/></g>';
    // The fan points along the beam: after rotate(rot) the up vector maps to (sin, -cos).
    const radians = (q.rot * Math.PI) / 180;
    const reach = L + 14;
    const tx = q.x + Math.sin(radians) * reach;
    const ty = q.y - Math.cos(radians) * reach;
    const sideways = Math.abs(Math.sin(radians)) > 0.5;
    const anchor = sideways ? (Math.sin(radians) > 0 ? "start" : "end") : "middle";
    s += `<text x="${tx.toFixed(1)}" y="${(ty + 4).toFixed(1)}" text-anchor="${anchor}" font-size="13" fill="${st.t}">${d == null ? "—" : d.toFixed(2)}</text>`;
  });
  $("sensors").innerHTML = s;
  const wheels = p.wheels.length === 4 ? p.wheels : PLACEHOLDER_WHEELS;
  $("wheels").innerHTML = wheels.map((q, i) => {
    const deg = c.steer_deg[i];
    const lx = q.x < CENTRE.x ? q.x - 26 : q.x + 26;
    return `<g transform="translate(${q.x.toFixed(1)} ${q.y.toFixed(1)}) rotate(${deg == null ? 0 : -deg})"><rect x="-6.5" y="-18" width="13" height="36" rx="4.5" fill="rgba(0,0,0,.35)" stroke="rgba(255,255,255,.7)" stroke-width="1.2"/><line x1="0" y1="-12" x2="0" y2="-26" stroke="#a99bff" stroke-width="1.6" stroke-linecap="round"/></g><text x="${lx.toFixed(1)}" y="${(q.y + 4).toFixed(1)}" text-anchor="middle" font-size="12" fill="rgba(255,255,255,.5)">${fix(deg, 0)}°</text>`;
  }).join("");
  const known = c.vx != null && c.vy != null;
  const v = known ? Math.hypot(c.vx, c.vy) : null;
  const heading = known ? (-Math.atan2(c.vy, c.vx) * 180) / Math.PI : 0;
  $("vel").innerHTML = `<g transform="translate(${CENTRE.x} ${CENTRE.y}) rotate(${heading.toFixed(1)})">${v ? '<line x1="0" y1="0" x2="0" y2="-30" stroke="#fff" stroke-width="1.5" stroke-linecap="round"/><path d="M-4.5 -28L0 -38L4.5 -28Z" fill="#fff"/>' : ""}<circle r="2.5" fill="#fff"/></g>`;
  $("speed").textContent = fix(v, 2);
  $("twist").textContent = `vx ${fix(c.vx, 2)} · vy ${fix(c.vy, 2)} · ω ${fix(c.wz, 2)}`;
  let near = -1;
  c.sonar.forEach((d, i) => { if (d != null && (near < 0 || d < c.sonar[near])) near = i; });
  const nd = near < 0 ? null : c.sonar[near];
  $("nearest").textContent = fix(nd, 2);
  $("nearest-big").className = `big stat ${nd == null ? "" : level(nd) === "safe" ? "" : level(nd)}`;
  $("nearest-where").textContent = near < 0
    ? "无数据"
    : p.placeholder
      ? `超声波 ${near + 1} 号`
      : `${SENSOR_WHERE[p.sensors[near].ch] || "超声波"} · 超声波 ${p.sensors[near].ch} 号`;
}

// ------------------------------------------------------------------ cpu
function renderCpu(c, log) {
  $("cpu-avg").textContent = fix(c.usage_pct, 0);
  $("cpu-count").textContent = `${c.count} 核平均`;
  const isolated = new Set(c.isolated);
  const rt = [...isolated].sort((a, b) => a - b);
  $("rt-range").textContent = rt.length ? `${rt[0]}–${rt.at(-1)}` : "未隔离";
  $("rt").innerHTML = rt
    .map((id) => {
      const core = c.cores.find((row) => row.id === id);
      const pct = core && core.total ? (100 * core.busy) / core.total : null;
      return `<div class="rt"><span>${id}</span><div class="track"><i style="width:${pct == null ? 0 : Math.min(pct, 100).toFixed(0)}%"></i><u style="left:${RT_TICK}%"></u></div><span>${fix(pct, 0)}%</span></div>`;
    })
    .join("");
  const general = c.cores.filter((row) => !isolated.has(row.id));
  const pcts = general.map((row) => (row.total ? (100 * row.busy) / row.total : 0));
  $("gp-note").textContent = general.length
    ? `${general[0].id}–${general.at(-1).id} · 最高 ${fix(Math.max(...pcts), 0)}%`
    : "—";
  let m = "";
  for (let row = MATRIX_ROWS - 1; row >= 0; row--) {
    pcts.forEach((v) => {
      const lit = Math.max(1, Math.round((v / 100) * MATRIX_ROWS));
      m += `<i class="${row < lit ? (v >= CORE_HOT ? "hot" : "on") : ""}"></i>`;
    });
  }
  $("matrix").innerHTML = m;
  // Cycle statistics come from the log station: cumulative since this run started.
  const cycle = log && log.cycle;
  const stale = !cycle || cycle.stale;
  $("m-period").textContent = cycle && cycle.nominal_period_us != null ? fix(cycle.nominal_period_us / 1000, 2) : "—";
  $("m-jitter").textContent = cycle && cycle.max_jitter_us != null ? fix(cycle.max_jitter_us, 0) : "—";
  $("m-overrun").textContent = cycle && cycle.overruns != null ? fix(cycle.overruns, 0) : "—";
  $("m-overrun-b").className = stale || cycle.overruns == null ? "" : cycle.overruns === 0 ? "ok" : "bad";
  $("m-note").textContent = !log || !log.available
    ? (log && log.error === "pending" ? "日志站连接中" : log && log.error === "not_configured" ? "日志站未配置" : "日志站不可达")
    : stale ? `日志站数据 ${fix(cycle && cycle.age_s, 0)} 秒未更新` : `日志站 · 本次运行累计 · ${fix(cycle.age_s, 0)} 秒前更新`;
  $("m-temp").textContent = fix(c.temperature_c, 0);
}

// ------------------------------------------------------------------ alerts
const ORDER = { fault: 0, warn: 1, stale: 2 };
const COLOR = { fault: "#ff5a79", warn: "#ffa63d", stale: "rgba(255,255,255,.5)" };
let alertKey = "";
function renderAlerts(s) {
  const recovered = s.alerts.filter((a) => a.level === "recovered").slice(0, RECOVERED_MAX);
  const shown = s.alerts
    .filter((a) => a.level !== "recovered")
    .sort((a, b) => ORDER[a.level] - ORDER[b.level])
    .concat(recovered);
  let html;
  if (s.state === "offline") {
    html = '<div class="glass al-summary blind"><span class="dot"></span><span>无法判断</span><span>与机器人断开，恢复连接后重新判断</span></div>';
  } else if (!shown.length) {
    html = '<div class="glass al-summary"><span class="dot"></span><span>全部正常</span><span>诊断与就绪状态都没有问题</span></div>';
  } else {
    html = shown
      .slice(0, 5)
      .map((a) => {
        const c = COLOR[a.level];
        return `<article class="glass al ${a.level}"><div class="code"><span>${esc(a.code)}</span><span class="dot" style="${a.level === "stale" ? "" : `background:${c}`}"></span></div>`
          + `<div class="ttl">${esc(a.title)}</div>`
          + `<div class="pills"><span class="pill"><span class="dot" style="background:${c}"></span>${LEVEL[a.level]}</span>${a.part ? `<span class="pill">${esc(a.part)}</span>` : ""}</div>`
          + `<div class="foot"><span>${a.evidence && a.evidence.at ? new Date(a.evidence.at * 1000).toLocaleTimeString("zh-CN", { hour12: false }) : esc(a.code)}</span>`
          + (a.evidence && a.evidence.url
            ? `<a href="${esc(a.evidence.url)}" target="_blank" rel="noopener">${a.evidence.kind === "waveform" ? "波形" : "运行记录"}</a>`
            : `<button type="button" disabled>无记录</button>`)
          + `</div></article>`;
      })
      .join("");
  }
  if (html !== alertKey) {
    alertKey = html;
    $("alerts").innerHTML = html;
  }
}

// ------------------------------------------------------------------ raw drawer
function renderRaw(s) {
  $("raw-body").innerHTML = `<details open><summary><i>状态</i>快照</summary><table>`
    + `<tr><td>state</td><td>${esc(s.state)}</td></tr>`
    + `<tr><td>joints_age_s</td><td>${esc(s.ages.joints_s)}</td></tr>`
    + `<tr><td>readiness.ready</td><td>${esc(s.readiness ? s.readiness.ready : "—")}</td></tr>`
    + `<tr><td>readiness.status</td><td>${esc(s.readiness ? s.readiness.status : "—")}</td></tr>`
    + `<tr><td>readiness.blockers</td><td>${esc((s.readiness?.blockers || []).join(", ") || "—")}</td></tr>`
    + `<tr><td>log.run</td><td>${esc(s.log?.run?.id || "—")}</td></tr>`
    + `<tr><td>log.cycle</td><td>${esc(s.log?.cycle ? JSON.stringify(s.log.cycle) : "—")}</td></tr>`
    + `<tr><td>log.error</td><td>${esc(s.log?.error || "—")}</td></tr>`
    + `</table></details>`
    + s.alerts.map((a) => `<details><summary><i>${esc(a.level)}</i>${esc(a.code)}</summary><table>`
      + `<tr><td>title</td><td>${esc(a.title)}</td></tr>`
      + Object.entries(a.values || {}).map(([k, v]) => `<tr><td>${esc(k)}</td><td>${esc(v)}</td></tr>`).join("")
      + `</table></details>`).join("");
}

// ------------------------------------------------------------------ cameras (NX preview service)
let cameraFps = 5;
const cams = [...document.querySelectorAll("[data-camera]")].map((root) => ({
  root, name: root.dataset.camera, img: root.querySelector("img"), text: root.querySelector(".ph-text"), src: "",
}));
function setCameraSource(cam, base, live) {
  const src = live ? `${base}/stream/${cam.name}.mjpg?fps=${cameraFps}&width=${CAMERA_WIDTH}` : "";
  if (src === cam.src) return;
  cam.src = src;
  if (src) cam.img.src = src; else cam.img.removeAttribute("src");
}
function showCamera(cam, live, message) {
  cam.root.classList.toggle("live", live);
  cam.img.hidden = !live;
  cam.text.hidden = live;
  cam.text.textContent = message;
}
cams.forEach((cam) => {
  cam.img.onerror = () => { cam.src = ""; showCamera(cam, false, "预览流中断，正在重连"); };
  cam.img.onload = () => showCamera(cam, true, "");
});
async function pollCameras() {
  const base = latest?.camera_base || "";
  if (!base) {
    cams.forEach((cam) => { setCameraSource(cam, base, false); showCamera(cam, false, "未配置相机预览服务"); });
    return;
  }
  try {
    const status = (await (await fetch(`${base}/status.json`, { cache: "no-store" })).json()).cameras;
    cams.forEach((cam) => {
      const s = status[cam.name];
      const fresh = s && s.publishers > 0 && (!s.subscribed || (s.frame_age_s != null && s.frame_age_s < 2));
      setCameraSource(cam, base, Boolean(fresh));
      if (!fresh) showCamera(cam, false, !s || !s.publishers ? "相机未启动" : "相机暂无画面");
      cam.root.querySelector(".fps-now").textContent = s && s.source_fps ? `${cameraFps} fps · 源 ${s.source_fps} fps` : `${cameraFps} fps`;
    });
  } catch {
    cams.forEach((cam) => { setCameraSource(cam, base, false); showCamera(cam, false, "无法连接预览服务"); });
  }
}
setInterval(pollCameras, CAMERA_STATUS_PERIOD_MS);
document.querySelectorAll("[data-fps]").forEach((b) => (b.onclick = () => {
  cameraFps = Number(b.dataset.fps);
  document.querySelectorAll("[data-fps]").forEach((x) => x.classList.toggle("on", x === b));
  cams.forEach((cam) => { const live = Boolean(cam.src); cam.src = ""; setCameraSource(cam, latest?.camera_base || "", live); });
  pollCameras();
}));

$("raw-open").onclick = (e) => { e.preventDefault(); if (latest) renderRaw(latest); $("raw").hidden = false; };
$("raw-close").onclick = () => ($("raw").hidden = true);
addEventListener("keydown", (e) => { if (e.key === "Escape") $("raw").hidden = true; });

// ------------------------------------------------------------------ snapshot stream
function render(s) {
  latest = s;
  renderHeader(s);
  renderPose(s);
  s.batteries.forEach((b, i) => { b.name = ["左电池", "右电池"][i]; renderBattery(i, b); });
  renderChassis(s.chassis, s.model_geometry);
  renderCpu(s.cpu, s.log);
  renderAlerts(s);
  loadModel(s);
}
function showLogin(error) {
  $("login-error").textContent = error || "";
  $("login").hidden = false;
  $("login-token").focus();
}
$("login-form").onsubmit = async (event) => {
  event.preventDefault();
  const token = $("login-token").value;
  const response = await fetch("/api/login", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ token }),
  });
  if (!response.ok) {
    showLogin("令牌无效");
    return;
  }
  $("login").hidden = true;
  $("login-token").value = "";
  connect();
};

let closedForAuth = false;
function connect() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws/status`);
  ws.onopen = () => { closedForAuth = false; $("login").hidden = true; };
  ws.onmessage = (event) => {
    try { render(JSON.parse(event.data)); } catch { /* keep the last good snapshot */ }
  };
  ws.onclose = (event) => {
    if (event.code === 4401 || event.code === 1008) { closedForAuth = true; showLogin(""); return; }
    if (!closedForAuth) setTimeout(connect, 2000);
  };
}
connect();
pollCameras();
