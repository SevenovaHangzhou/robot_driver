// Read-only diagnostics page: self-check reports, readiness and raw /diagnostics rows.
const $ = (id) => document.getElementById(id);
const SEVERITY = { FAULT: "bad", WARN: "warn", OK: "ok", ERROR: "bad", STALE: "warn" };
const STAGE = { before_bus: "总线启动前（阶段 A）", after_activation: "硬件激活后（阶段 B）" };

function cell(row, text, cls) {
  const td = document.createElement("td");
  td.textContent = text;
  if (cls) td.className = cls;
  row.append(td);
  return td;
}

function fill(body, rows, render) {
  body.replaceChildren();
  for (const item of rows) render(body.insertRow(), item);
}

function renderReadiness(readiness) {
  const summary = $("readiness-summary");
  const body = $("readiness-errors").tBodies[0];
  if (!readiness) {
    summary.textContent = "诊断节点未运行或尚未发布";
    summary.className = "summary warn";
    $("readiness-blockers").textContent = "";
    body.replaceChildren();
    return;
  }
  summary.textContent = `${readiness.readiness_name}：${readiness.ready ? "就绪" : "未就绪"}（${readiness.status}，${readiness.operational_state}，${readiness.age_s} 秒前）`;
  summary.className = `summary ${readiness.ready ? (readiness.errors.length ? "warn" : "ok") : "bad"}`;
  $("readiness-blockers").textContent = readiness.blockers.length ? `阻断：${readiness.blockers.join("、")}` : "";
  fill(body, readiness.errors, (row, error) => {
    cell(row, error.source);
    cell(row, `${error.code} ${error.name}`);
    cell(row, error.severity, SEVERITY[error.severity]);
    cell(row, error.detail);
  });
}

function renderSelfChecks(reports) {
  const root = $("self-checks");
  root.replaceChildren();
  if (!reports.length) {
    const hint = document.createElement("p");
    hint.className = "hint";
    hint.textContent = "没有自检报告";
    root.append(hint);
    return;
  }
  for (const report of reports) {
    const title = document.createElement("h3");
    if (report.error) {
      title.textContent = `${report.control_scope}：${report.error}`;
      title.className = "bad";
      root.append(title);
      continue;
    }
    title.textContent = `${report.control_scope} · ${STAGE[report.stage] || report.stage} · ${report.checked_at} · ${report.passed ? "通过" : "未通过"}`;
    title.className = report.passed ? "ok" : "bad";
    root.append(title);
    if (!report.entries.length) continue;
    const table = document.createElement("table");
    const head = table.createTHead().insertRow();
    for (const text of ["位置", "错误码", "级别", "阻断", "说明"]) cell(head, text);
    const body = table.createTBody();
    fill(body, report.entries, (row, entry) => {
      cell(row, entry.source);
      cell(row, entry.dree);
      cell(row, entry.severity, SEVERITY[entry.severity]);
      cell(row, entry.blocking ? "是" : "否");
      cell(row, entry.detail);
    });
    root.append(table);
  }
}

function renderDiagnostics(rows) {
  fill($("diagnostics").tBodies[0], rows, (row, status) => {
    cell(row, status.name);
    cell(row, status.level, SEVERITY[status.level]);
    cell(row, status.message);
    cell(row, `${status.age_s} 秒`);
    const values = cell(row, "");
    const details = document.createElement("details");
    const summary = document.createElement("summary");
    summary.textContent = `${Object.keys(status.values).length} 项`;
    const list = document.createElement("pre");
    list.textContent = Object.entries(status.values).map(([key, value]) => `${key}: ${value}`).join("\n");
    details.append(summary, list);
    values.append(details);
  });
}

function connect() {
  const socket = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws/diag`);
  socket.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.type !== "status") return;
    $("login").classList.add("hidden");
    $("conn").textContent = "已连接";
    $("conn").className = "chip ok";
    renderReadiness(data.readiness);
    renderSelfChecks(data.self_checks);
    renderDiagnostics(data.diagnostics);
  };
  socket.onclose = (event) => {
    $("conn").textContent = "未连接";
    $("conn").className = "chip bad";
    // 4401: no valid session cookie; the login form exchanges the token for one.
    if (event.code === 4401) $("login").classList.remove("hidden");
    else setTimeout(connect, 2000);
  };
}

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
