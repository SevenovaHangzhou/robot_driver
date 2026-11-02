// Head page: the server checks every command; this page only proposes absolute targets.
const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
let socket = null;
let nextRequest = 1;
const pending = new Map();
let status = null;
let builtFor = "";

function request(type, body = {}) {
  return new Promise((resolve, reject) => {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      reject(new Error("网页未连接"));
      return;
    }
    const id = nextRequest++;
    pending.set(id, { resolve, reject });
    socket.send(JSON.stringify({ type, id, ...body }));
  });
}

async function run(type, body) {
  try {
    const reply = await request(type, body);
    $("result").textContent = "完成";
    return reply;
  } catch (error) {
    $("result").textContent = error.message;
    return null;
  }
}

function buildJoints(joints) {
  const key = joints.map((joint) => `${joint.name}:${joint.lower}:${joint.upper}`).join(",");
  if (key === builtFor) return;
  builtFor = key;
  const root = $("joints");
  root.replaceChildren();
  for (const joint of joints) {
    const row = document.createElement("div");
    row.className = "joint";
    const name = document.createElement("span");
    name.textContent = joint.name;
    const actual = document.createElement("span");
    actual.id = `actual-${joint.name}`;
    const slider = document.createElement("input");
    slider.type = "range";
    slider.step = "0.1";
    slider.min = String(joint.lower * DEG);
    slider.max = String(joint.upper * DEG);
    slider.value = String((joint.position ?? 0) * DEG);
    slider.id = `target-${joint.name}`;
    const number = document.createElement("input");
    number.type = "number";
    number.step = "0.1";
    number.min = slider.min;
    number.max = slider.max;
    number.value = Number(slider.value).toFixed(1);
    slider.addEventListener("input", () => { number.value = Number(slider.value).toFixed(1); });
    number.addEventListener("input", () => { slider.value = number.value; });
    row.append(name, actual, slider, number);
    root.append(row);
  }
}

function render(data) {
  status = data;
  const mine = data.lease === "you";
  $("lease").textContent = mine ? "持有控制权" : data.lease === "other" ? "其他终端控制中" : "无控制权";
  $("lease").className = mine ? "chip ok" : "chip";
  $("lease-op").textContent = mine ? "释放控制权" : "获取控制权";
  $("lease-op").disabled = data.lease === "other";
  const enable = data.enable;
  const parts = [
    data.capability ? `作用域 ${data.capability.scope}${data.capability.mock ? "（Mock）" : ""}` : "运行栈未声明头部位置能力",
    `使能 ${enable.state}`,
    enable.motion_allowed ? "允许运动" : "未允许运动",
    enable.device_fault ? "设备故障锁存" : "",
    data.feedback_fresh ? "反馈正常" : "反馈超时",
  ];
  $("state").textContent = parts.filter(Boolean).join(" · ");
  $("enable-op").disabled = !mine || data.busy;
  $("disable-op").disabled = !mine || data.busy;
  $("reset-op").disabled = !mine || data.busy;
  const ready = mine && !data.busy && enable.state === "ENABLED" && enable.motion_allowed &&
    !enable.device_fault && data.feedback_fresh;
  $("move-op").disabled = !ready;
  $("hold-op").disabled = !data.capability || !data.feedback_fresh;
  if (data.joints.length) buildJoints(data.joints);
  else $("joints").firstElementChild.textContent = data.model_error || "等待头部控制器与机器人模型";
  for (const joint of data.joints) {
    const cell = $(`actual-${joint.name}`);
    if (cell) cell.textContent = joint.position == null ? "--" : `${(joint.position * DEG).toFixed(2)}°`;
  }
}

function connect() {
  socket = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws/head`);
  socket.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.type === "status") {
      $("login").classList.add("hidden");
      $("conn").textContent = "已连接";
      $("conn").className = "chip ok";
      render(data);
    } else if (data.type === "result") {
      const waiting = pending.get(data.id);
      if (!waiting) return;
      pending.delete(data.id);
      if (data.ok) waiting.resolve(data);
      else waiting.reject(new Error(data.message || data.error || "操作失败"));
    }
  };
  socket.onclose = (event) => {
    for (const waiting of pending.values()) waiting.reject(new Error("网页连接已断开"));
    pending.clear();
    $("conn").textContent = "未连接";
    $("conn").className = "chip bad";
    // 4401: no valid session cookie; the login form exchanges the token for one.
    if (event.code === 4401) $("login").classList.remove("hidden");
    else setTimeout(connect, 1000);
  };
}

$("lease-op").addEventListener("click", () => run(status?.lease === "you" ? "release" : "acquire"));
$("enable-op").addEventListener("click", () => run("enable", { enabled: true }));
$("disable-op").addEventListener("click", () => run("enable", { enabled: false }));
$("reset-op").addEventListener("click", () => {
  if (confirm("确认对头部执行故障复位？复位前应先排除故障原因。")) run("reset_fault");
});
$("move-op").addEventListener("click", () => {
  const target = {};
  for (const joint of status.joints) target[joint.name] = Number($(`target-${joint.name}`).value) / DEG;
  run("move", { target });
});
$("hold-op").addEventListener("click", () => run("hold"));
$("login-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const token = $("token").value.trim();
  $("token").value = "";
  $("login-error").textContent = "";
  const response = await fetch("/api/login", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ token }),
  });
  if (response.ok) connect();
  else $("login-error").textContent = (await response.json()).message || "登录失败";
});

connect();
