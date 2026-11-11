const reasons = {
  present: "已加载", absent: "未检测到，已跳过", excluded: "按配置跳过", out_of_scope: "不在本次范围",
};
const communication = {
  not_loaded: "—", waiting: "等待运行反馈", reported_invalid: "运行反馈异常", reported_valid: "运行反馈有效",
};

export function moduleText(row) {
  return {
    policy: row.policy === "required" ? "必需" : "可选",
    selection: reasons[row.reason] || "选择状态未知",
    feedback: communication[row.communication] || "等待运行反馈",
    age: Number.isFinite(row.age_ms) ? `${row.age_ms.toFixed(2)} ms` : "",
  };
}

export function renderModules(container, rows) {
  container.replaceChildren();
  if (!rows.length) {
    const note = document.createElement("p");
    note.className = "hint";
    note.textContent = "当前启动配置没有模块选择记录";
    container.append(note);
    return;
  }
  const table = document.createElement("table");
  table.className = "module-table";
  const header = document.createElement("tr");
  for (const title of ["模块", "要求", "本次启动", "实时反馈"]) {
    const cell = document.createElement("th");
    cell.scope = "col";
    cell.textContent = title;
    header.append(cell);
  }
  const head = document.createElement("thead");
  head.append(header);
  table.append(head);
  const body = document.createElement("tbody");
  for (const row of rows) {
    const text = moduleText(row);
    const line = document.createElement("tr");
    for (const value of [row.label, text.policy, text.selection, [text.feedback, text.age].filter(Boolean).join(" · ")]) {
      const cell = document.createElement("td");
      cell.textContent = value;
      line.append(cell);
    }
    body.append(line);
  }
  table.append(body);
  container.append(table);
}
