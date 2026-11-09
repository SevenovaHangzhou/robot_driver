const stages = {source: '初始化求解', planning: '轨迹求解', feedback: '状态检查', window_rpc: '发布窗口', waiting: '等待下一周期', idle: '空闲'};
const sources = {planner: '规划与控制链', browser_or_disconnect: '按键、页面或连接'};
const labels = {key_to_publish: '按键至发出窗口', key_to_ack: '按键至控制器确认', begin_rpc: '启动请求', source: '模型初始化', window_plan: '窗口求解与检查', window_rpc: '发布请求', ack: '发布至确认', loop_interval: '规划周期'};

export function latencyRows(diagnostics) {
  return Object.entries(labels).filter(([key]) => diagnostics?.latency_ms?.[key])
    .map(([key, label]) => ({label, ...diagnostics.latency_ms[key]}));
}

export function failureText(diagnostics) {
  const exit = diagnostics?.last_exit;
  if (!exit) return '暂无退出记录';
  return `${sources[exit.source] || exit.source} / ${stages[exit.stage] || exit.stage || '—'}${exit.code ? ` / ${exit.code}` : ''}：${exit.message}`;
}

export function renderDiagnostics(document, state) {
  const d = state?.diagnostics;
  document.getElementById('diagnostics').hidden = !d;
  if (!d) return;
  const rows = latencyRows(d).map(row => {
    const tr = document.createElement('tr');
    for (const value of [row.label, row.retained, row.p50, row.p95, row.p99, row.max]) {
      const td = document.createElement('td'); td.textContent = String(value); tr.append(td);
    }
    return tr;
  });
  document.getElementById('latency-rows').replaceChildren(...rows);
  document.getElementById('last-exit').textContent = failureText(d);
  const t = d.timing;
  document.getElementById('timing-note').textContent = !t ? '' :
    `首窗提前 ${t.initial_lead_ms} ms，续窗提前 ${t.replacement_lead_ms} ms。` +
    (t.profile === 'trial' ? '当前为提前量试验档，继续观测耗时峰值。' :
      !t.sample_ready ? '有效样本不足 30 组，继续采集。' :
      t.trial_fits_observed_budget ? '已有耗时记录支持下一档提前量试验，需停止后由启动配置选择。' : '已观测的耗时峰值需要保留当前提前量。');
}
