// Head controls embedded in the arm page; login belongs to the containing page.
const DEG = 180 / Math.PI;

// onPositions receives measured head joint positions (rad) for the shared 3D model.
export function headControls(root, loginRequired, onPositions, scopeLabel = () => "当前控制栈") {
  const $ = (name) => root.querySelector(`[data-head="${name}"]`);
  let socket = null;
  let nextRequest = 1;
  const pending = new Map();
  const targets = new Map();
  let status = null;
  let builtFor = "";

  function request(type, body = {}) {
    return new Promise((resolve, reject) => {
      if (!socket || socket.readyState !== WebSocket.OPEN) {
        reject(new Error("头部连接已断开"));
        return;
      }
      const id = nextRequest++;
      pending.set(id, { resolve, reject });
      socket.send(JSON.stringify({ type, id, ...body }));
    });
  }

  async function run(type, body) {
    try {
      await request(type, body);
      $("result").textContent = "完成";
    } catch (error) {
      $("result").textContent = error.message;
    }
  }

  function buildJoints(joints) {
    const key = joints.map((joint) => `${joint.name}:${joint.lower}:${joint.upper}`).join(",");
    if (key === builtFor) return;
    builtFor = key;
    targets.clear();
    const container = $("joints");
    container.replaceChildren();
    for (const joint of joints) {
      // Same row layout as the arm joint sliders (arm/style.css .joint).
      const row = document.createElement("div");
      row.className = "joint";
      const label = document.createElement("label");
      label.textContent = joint.name;
      const slider = document.createElement("input");
      slider.type = "range";
      slider.step = "0.1";
      slider.min = (joint.lower * DEG).toFixed(1);
      slider.max = (joint.upper * DEG).toFixed(1);
      slider.value = String((joint.position ?? 0) * DEG);
      slider.setAttribute("aria-label", `${joint.name} 目标角度滑块`);
      const number = document.createElement("input");
      number.type = "number";
      number.step = "0.1";
      number.min = slider.min;
      number.max = slider.max;
      number.value = Number(slider.value).toFixed(1);
      number.setAttribute("aria-label", `${joint.name} 目标角度（度）`);
      const meta = document.createElement("div");
      meta.className = "meta";
      const actual = document.createElement("span");
      actual.textContent = "实测 --";
      const span = document.createElement("span");
      span.textContent = `${slider.min}° ~ ${slider.max}°`;
      meta.append(actual, span);
      slider.addEventListener("input", () => { number.value = Number(slider.value).toFixed(1); });
      number.addEventListener("input", () => { slider.value = number.value; });
      targets.set(joint.name, { slider, number, actual });
      row.append(label, slider, number, meta);
      container.append(row);
    }
  }

  function render(data) {
    status = data;
    const mine = data.lease === "you";
    $("lease").textContent = mine ? "持有头部控制权" : data.lease === "other" ? "其他终端控制中" : "无控制权";
    $("lease").className = mine ? "chip ok" : "chip";
    $("lease-op").textContent = mine ? "释放头部控制权" : "获取头部控制权";
    $("lease-op").disabled = !data.capability || data.lease === "other";
    const enable = data.enable;
    $("state").textContent = data.capability ? [
      `作用域 ${data.capability.scope}${data.capability.mock ? "（Mock）" : ""}`,
      `使能 ${enable.state}`,
      enable.motion_allowed ? "允许运动" : "未允许运动",
      enable.device_fault ? "设备故障锁存" : "",
      data.feedback_fresh ? "反馈正常" : "反馈超时",
    ].filter(Boolean).join(" · ") : "当前运行栈未提供头部控制";
    for (const action of ["enable-op", "disable-op", "reset-op"]) {
      $(action).disabled = !data.capability || !mine || data.busy;
    }
    const ready = !!data.capability && mine && !data.busy && enable.state === "ENABLED" &&
      enable.motion_allowed && !enable.device_fault && data.feedback_fresh && data.joints.length > 0;
    $("move-op").disabled = !ready;
    $("hold-op").disabled = !data.capability || !data.feedback_fresh;
    if (data.joints.length) buildJoints(data.joints);
    else {
      builtFor = "";
      targets.clear();
      $("joints").textContent = data.model_error || "等待头部控制器与机器人模型";
    }
    onPositions(Object.fromEntries(data.joints.map((joint) => [joint.name, joint.position])));
    for (const joint of data.joints) {
      const controls = targets.get(joint.name);
      controls.actual.textContent = joint.position == null ? "实测 --" : `实测 ${(joint.position * DEG).toFixed(2)}°`;
    }
  }

  function connect() {
    if (socket && socket.readyState < WebSocket.CLOSING) return;
    socket = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws/head`);
    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === "status") {
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
      for (const waiting of pending.values()) waiting.reject(new Error("头部连接已断开"));
      pending.clear();
      status = null;
      $("conn").textContent = "未连接";
      $("conn").className = "chip bad";
      $("state").textContent = "头部连接已断开，等待重新连接";
      $("lease").textContent = "控制权待确认";
      for (const button of root.querySelectorAll("button")) button.disabled = true;
      if (event.code === 4401) loginRequired();
      else setTimeout(connect, 1000);
    };
  }

  $("lease-op").addEventListener("click", () => run(status?.lease === "you" ? "release" : "acquire"));
  $("enable-op").addEventListener("click", () => {
    if (confirm(`确认使能${scopeLabel()}？\n这是当前控制栈的整体使能操作；使能后关节保持当前位置。`)) {
      run("enable", { enabled: true });
    }
  });
  $("disable-op").addEventListener("click", () => run("enable", { enabled: false }));
  $("reset-op").addEventListener("click", () => {
    if (confirm(`确认对${scopeLabel()}执行整体故障复位？复位前应先排除故障原因。`)) run("reset_fault");
  });
  $("move-op").addEventListener("click", () => {
    if (!status) return;
    const target = {};
    for (const joint of status.joints) target[joint.name] = Number(targets.get(joint.name).slider.value) / DEG;
    run("move", { target });
  });
  $("hold-op").addEventListener("click", () => run("hold"));
  return { connect, getScope: () => status?.capability?.scope };
}
