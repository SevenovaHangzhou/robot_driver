import {renderDiagnostics} from './diagnostics.mjs';
import {JogInput, MotionFeedback, KEY_DIRECTIONS, isRotation, motionCommand} from './input.mjs';
import {AuthoritySocket} from './gateway.mjs';
const $ = id => document.getElementById(id);
const client = crypto.randomUUID?.() || Array.from(crypto.getRandomValues(new Uint8Array(16)),
  value => value.toString(16).padStart(2, '0')).join('');
let state = null, epoch = '', sequence = 0, heartbeatSequence = 0, connected = false, pollBusy = false;
let heartbeatBusy = false, stopping = false, localBusy = false, lastState = 0;
let armGeneration = 0, notice = '';
const pointerCodes = new Map();
let modelView = null, modelTask = null, modelReady = false;
let switchBusy = false;
const motionFeedback = new MotionFeedback();
let feedbackRevision = 0;
let holdId = 0;
const inputEvents = [];
function traceInput(type, event = {}) {
  inputEvents.push({type, code: event.code || '', repeat: Boolean(event.repeat),
    trusted: Boolean(event.isTrusted), at_ms: Math.round(performance.now()),
    focused: document.hasFocus(), hidden: document.hidden});
  if (inputEvents.length > 24) inputEvents.shift();
}
let authoritySocket = null;
const transportReady = fetch('/api/jog/config', {cache: 'no-store'})
  .then(response => response.ok ? response.json() : {})
  .then(config => {
    if (config.readonly_login && !config.preview_token) {
      $('login-note').textContent = '使用工控机操作台的访问令牌登录；当前为真机只读观测。';
    }
    if (config.authority_gateway) {
      $('primary-console').href = new URL('/arm', config.primary_console).href;
      $('login-note').textContent = config.keyboard_configured
        ? '使用主操作台的访问令牌登录；18121 与本页共用双臂互斥锁。'
        : '使用主操作台的访问令牌登录，两页共用双臂控制权。末端运动尚未开放。';
      authoritySocket = new AuthoritySocket(client, error => {
        connected = false; epoch = ''; armGeneration++;
        input.cancel(error.message); pointerCodes.clear();
        if (state) state = {...state, lease: 'none', armed: false, epoch: '', authority_available: false};
        if (error.loginRequired && !$('login').open) $('login').showModal();
        message(error.message); render();
      });
    }
  }).catch(() => {});
const armLabel = side => ({left: '左臂', right: '右臂', single: '当前臂'})[side] || side;
async function request(path, body, {timeout = 5000, keepalive = false, recoverSession = true} = {}) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeout);
  try {
    const response = await fetch(path, {method: body === undefined ? 'GET' : 'POST',
      headers: body === undefined ? {} : {'Content-Type': 'application/json'},
      body: body === undefined ? undefined : JSON.stringify(body),
      credentials: 'same-origin', signal: controller.signal, keepalive, cache: 'no-store'});
    if (response.status === 401 && authoritySocket && recoverSession && path !== '/api/login') {
      // Cookies are shared across ports. A login on 18121 can replace the cookie
      // while this page's authenticated socket still belongs to its predecessor.
      await response.body?.cancel();
      await authoritySocket.reconnect();
      if (body === undefined) return request(path, body, {timeout, keepalive, recoverSession: false});
      throw new Error('登录会话已更新，请重新获取控制权后操作');
    }
    const data = await response.json();
    if (!response.ok) {
      if (response.status === 401 && !$('login').open) $('login').showModal();
      throw new Error(data.message || data.error || `HTTP ${response.status}`);
    }
    return data;
  } finally { clearTimeout(timer); }
}
function message(text) { $('result').textContent = text; }
function command(action, extra = {}, options) {
  if (authoritySocket && ['acquire', 'release'].includes(action)) return authoritySocket.command(action);
  return request(`/api/jog/${action}`, {client, epoch, seq: action === 'heartbeat' ? ++heartbeatSequence : ++sequence,
    ...(['arm', 'velocity', 'halt', 'stop'].includes(action) ? {input_events: [...inputEvents]} : {}), ...extra}, options);
}
async function stop(reason = '已请求停止') {
  holdId++;
  notice = reason;
  epoch = ''; armGeneration++; localBusy = false;
  input.armed = false; input.held.clear(); pointerCodes.clear();
  message(reason); render();
  if (stopping || !connected) return;
  stopping = true;
  try { await command('stop', {reason}, {keepalive: true, timeout: 2000}); }
  catch (error) { message(`停止请求未确认：${error.message}。接管已退出。`); }
  finally { stopping = false; }
}
async function release(reason) {
  holdId++;
  const oldEpoch = epoch, generation = ++armGeneration;
  epoch = ''; input.armed = false; input.held.clear(); pointerCodes.clear();
  message(reason); render();
  if (!oldEpoch || !connected) return;
  try {
    const result = await request('/api/jog/halt', {client, epoch: oldEpoch, seq: ++sequence,
      reason, input_events: [...inputEvents]});
    if (generation !== armGeneration || document.hidden || !document.hasFocus()) return;
    armGeneration++; epoch = result.epoch; sequence = 0; heartbeatSequence = 0;
    state = {...state, armed: true, epoch}; input.arm(); render(); await poll();
  } catch (error) { await stop(`松键停止未确认：${error.message}`); }
}
const input = new JogInput({
  release,
  step: async direction => {
    if (!connected || !epoch || localBusy) return;
    const angular = isRotation(direction);
    if (angular && !state?.rotation_supported) { await stop('当前后端不支持 J7 姿态点动'); return; }
    const stepEpoch = epoch;
    const startState = state, speed = Number($(angular ? 'angular-speed' : 'speed').value), revision = ++feedbackRevision, motionId = holdId;
    const continuous = state?.input_mode === 'continuous';
    localBusy = true; input.busy = true; message(angular ? '正在启动 J7 局部旋转…' : continuous ? '正在启动连续移动…' : '正在检查直线路径…'); render();
    try {
      const plan = await command(continuous ? 'velocity' : 'step',
        motionCommand(direction, Number($('speed').value), Number($('angular-speed').value)));
      if (epoch === stepEpoch && revision === feedbackRevision) {
        motionFeedback.begin(startState, plan.direction, plan.continuous ? null : plan.distance_mm,
          plan.continuous ? null : plan.duration_s, speed, motionId, plan.rotation_axis);
        // Keep a continuous command busy until server feedback arrives. A stale
        // pre-command idle poll must not issue a second velocity start.
        if (plan.continuous) state = {...state, busy: true};
        feedbackRevision++;
      }
    }
    catch (error) { if (epoch === stepEpoch) await stop(error.message); }
    finally { localBusy = false; }
  },
  stop,
});
function render() {
  const readonly = state?.readonly === true;
  const mine = state?.lease === 'you';
  const armed = Boolean(connected && epoch && state?.armed && state.epoch === epoch);
  input.busy = localBusy || Boolean(state?.busy);
  $('lease').textContent = mine ? '释放控制权' : state?.lease === 'other' ? '其他页面控制中' : '获取控制权';
  $('lease').disabled = !connected || state?.mode === 'unavailable' || state?.lease === 'other';
  $('lease').disabled ||= switchBusy;
  $('lease').disabled ||= readonly && !state?.authority_available;
  if (readonly && !state?.authority_gateway) $('lease').textContent = '只读观测';
  $('arm').disabled = !connected || !mine || !state?.can_arm || !modelReady || state?.busy || Boolean(epoch);
  $('arm').textContent = epoch ? '键盘已接管' : '开启键盘接管';
  const sides = state?.arms || [];
  if (Array.from($('arm-side').options, option => option.value).join(',') !== sides.join(',')) {
    $('arm-side').replaceChildren(...sides.map(side => new Option(armLabel(side), side)));
  }
  $('arm-side').value = state?.selected_arm || '';
  $('arm-side').disabled = !connected || (!mine && !readonly) || !modelReady || Boolean(epoch) || Boolean(state?.busy) || switchBusy || sides.length < 2;
  $('arm-side-help').textContent = readonly ? '可切换左右臂进行观测；当前入口不提供运动控制。' : '获取控制权后可选择左右臂；切换前请按 Esc 停止并退出接管。';
  $('selected-arm').textContent = state?.selected_arm ? `${armLabel(state.selected_arm)} / TCP 轨迹` : '等待机械臂信息';
  $('arm').disabled ||= switchBusy;
  $('stop').disabled = !connected || state?.mode === 'unavailable';
  $('stop').disabled ||= readonly;
  if (readonly) $('stop').textContent = '只读入口无运动控制';
  document.querySelectorAll('.dir').forEach(button => {
    button.disabled = !armed || (isRotation(KEY_DIRECTIONS[button.dataset.code]) && !state?.rotation_supported);
    button.classList.toggle('active', input.held.has(button.dataset.code));
  });
  $('motion').textContent = state?.stopping ? '正在减速' : state?.busy ? state?.input_mode === 'continuous' ? '连续移动中' : '路径执行中' : '空闲';
  $('speed').disabled = localBusy || Boolean(state?.stopping) || Boolean(state?.busy && !state?.publication?.continuous);
  $('speed').disabled ||= readonly;
  $('angular-speed').disabled = $('speed').disabled || !state?.rotation_supported;
  $('rotation-note').textContent = state?.rotation_supported
    ? `旋转参考系 ${state.rotation_frame || '所选臂 J7'}：yaw/Z、pitch/Y、roll/X；正向遵循右手定则，旋转时保持 TCP 位置。`
    : '当前后端未开放 J7 姿态控制；真机 Mink → Rolling 支持以下旋转按键。';
  const speed = Number($('speed').value);
  $('speed-note').textContent = `所选速度 ${speed} mm/s；实际速度受加减速与关节限速约束，松键减速停止。`;
  $('keyboard-status').textContent = !armed ? '键盘未接管：获取控制权后点击“开启键盘接管”'
    : editable(document.activeElement) ? '当前焦点在选项框：选好后点击方向区空白处，再按方向键'
    : input.held.size ? '正在按住方向键 · 松开减速停止' : '键盘已接管 · 等待方向键';
  if (readonly) {
    document.querySelector('.controls h2').textContent = '真机观测';
    document.querySelector('.controls .pill').textContent = '只读';
    document.querySelector('label[for="arm-side"]').textContent = '观测机械臂';
    $('keyboard-status').textContent = '真机只读观测 · 方向键不发送运动';
    $('speed-note').textContent = '末端运动接口尚未开放，方向键不发送运动。';
    document.querySelector('.instruction').textContent = state.model_only
      ? '等待真机反馈；当前模型为零位预览，不代表实际姿态。'
      : '当前显示真机反馈计算的末端姿态与轨迹。';
  }
  const movement = state && motionFeedback.update(state);
  const angular = Boolean(movement?.angular);
  if (state?.busy && angular) $('motion').textContent = state.stopping ? '正在减速' : '连续旋转中';
  const unit = angular ? '°' : 'mm';
  $('movement-phase').textContent = movement?.label || '等待本页发出运动';
  $('movement-distance-label').textContent = angular ? '本次按住已转' : '本次按住已走';
  $('movement-distance').textContent = movement ? `${movement.holdTravelled.toFixed(4)} ${unit}` : '—';
  $('movement-target-speed').textContent = `${state?.busy && movement ? movement.requestedSpeed : angular ? Number($('angular-speed').value) : speed} ${unit}/s`;
  const hardware = state?.publication?.feedback_source === 'hardware_joint_states';
  const actual = hardware || state?.publication?.feedback_source === 'mujoco_actual';
  const measuredSpeed = angular
    ? actual ? state?.tcp_actual_angular_speed_deg_s : state?.tcp_command_angular_speed_deg_s
    : actual ? state?.tcp_actual_speed_mm_s : state?.tcp_command_speed_mm_s;
  $('movement-speed').textContent = Number.isFinite(measuredSpeed) ? `${measuredSpeed.toFixed(3)} ${unit}/s` : '—';
  $('movement-speed-label').textContent = actual ? '当前实际速度' : '当前命令速度';
  $('movement-feedback').textContent = hardware ? state.feedback_fresh ? '真机关节反馈' : '反馈已失效' : actual ? 'MuJoCo 实际状态' : '控制器命令反馈';
  $('movement-note').textContent = hardware ? 'TCP 与轨迹由真机实测关节角和当前 URDF 计算；不代表外部测量或 TCP 标定通过。'
    : actual ? '速度、位移和轨迹来自 MuJoCo 实际关节状态；运动受跟随误差、关节限速与避碰约束。'
    : '速度随加减速变化，受关节限速约束；这里显示命令反馈，物理跟随请看 MuJoCo。';
  $('metrics-title').textContent = actual ? angular ? '本次连续旋转' : '本次连续移动' : '本次路径检查';
  $('duration-label').textContent = actual ? '运行时长' : '轨迹时长';
  $('metrics-note').textContent = hardware ? readonly ? '只读观测未发出运动，不生成路径检查指标。'
    : '连续窗口经过关节动态限制与碰撞检查；TCP 位置与姿态来自实测关节反馈。'
    : actual ? '数值来自仿真模型实际状态，不能代表实机精度。' : '数值为模型离线计算结果，不能代表实机精度。';
  if (readonly) {
    $('metrics-title').textContent = '真机观测状态';
    $('motion').textContent = state.feedback_fresh ? '只读观测中' : '反馈已失效';
    $('movement-phase').textContent = state.feedback_fresh ? '观测当前姿态' : '显示最后有效姿态';
    $('movement-target-speed').textContent = '—';
  }
}
async function poll() {
  if (pollBusy || $('login').open) return;
  pollBusy = true;
  const generation = armGeneration;
  const revision = feedbackRevision;
  try {
    await transportReady;
    if (authoritySocket) await authoritySocket.connect();
    const next = await request(`/api/jog/state?client=${encodeURIComponent(client)}`, undefined, {timeout: 1200});
    if (generation !== armGeneration || revision !== feedbackRevision) return;
    renderDiagnostics(document, next);
    state = next; connected = true; lastState = performance.now();
    if (!modelTask && ['mock', 'real-observe', 'real-rolling'].includes(next.mode)) modelTask = loadModel();
    if (next.mode === 'unavailable') $('model-status').textContent = '实机模型等待运行服务接入';
    const simulation = next.publication?.backend === 'mujoco_ros_mock';
    const minkRolling = next.publication?.backend === 'mujoco_mink_rolling';
    const mink = minkRolling || next.publication?.backend === 'mujoco_mink';
    $('connection').textContent = mink ? '● Mink 仿真已连接' : simulation ? '● MuJoCo 联调已连接' : next.mode === 'mock' ? '● 本地 Mock 已连接' : '● 操作台已连接';
    $('connection').className = 'connection ok';
    $('mode').textContent = minkRolling ? 'MINK → Rolling / MuJoCo' : mink ? 'MINK / MuJoCo 连续控制' : simulation ? 'MuJoCo 联调 / Mock 控制器' : next.mode === 'mock' ? 'MOCK / 无硬件连接' : '实机点动未开放';
    $('mode').className = next.mode === 'mock' ? 'pill' : 'pill warning';
    if (next.readonly) {
      $('connection').textContent = next.feedback_fresh ? '● 真机只读反馈已连接' : '● 真机反馈已失效';
      $('connection').className = next.feedback_fresh ? 'connection ok' : 'connection bad';
      $('mode').textContent = '真机 / 只读观测';
    }
    if (next.mode === 'real-rolling') {
      $('connection').textContent = next.feedback_fresh ? '● 真机反馈与共享控制权已连接' : '● 等待真机反馈与共享控制权';
      $('mode').textContent = '真机 / Mink → Rolling';
    }
    const publication = next.publication;
    $('publication').textContent = minkRolling ? `Mink → Rolling 连续会话 · 批次 ${publication.sequence || 0} · 未来尾段替换 ${publication.replacements || 0} 次${next.fault ? ` · ${next.fault}` : ''}` : mink ? `Mink 连续求解 · ${Math.round(1 / publication.control_dt_s)} Hz · 显示模型实际末端轨迹${next.fault ? ` · ${next.fault}` : ''}` : publication?.mode === 'rolling'
      ? `${simulation ? 'ROS Rolling · 显示控制器命令姿态' : 'Rolling 发布'} · 批次 ${publication.sequence || 0}${publication.pending_sequence ? ' · 等待确认' : ''}${next.fault || publication.fault ? ` · ${next.fault || publication.fault}` : ''}`
      : next.mode === 'unavailable' ? '轨迹发布尚未接入' : '单步执行';
    if (next.readonly) $('publication').textContent = '真机关节反馈 → 当前 URDF → 模型与 TCP；运动接口未开放';
    const gravity = next.gravity_compensation;
    $('gravity-status').hidden = !gravity;
    if (gravity) {
      $('gravity-status').textContent = gravity.fault ? `重力补偿故障：${gravity.fault}` : gravity.mode === 'static'
        ? `静态重力补偿已启用 · 双臂 · ${Math.round(1 / gravity.period_s)} Hz`
        : gravity.mode === 'mujoco' ? 'MuJoCo 模型偏置补偿已启用' : '仿真重力补偿已关闭';
    }
    $('frame').textContent = `坐标 / ${next.frame || '待确认'}`;
    $('direction-note').textContent = `方向以机器人本体 ${next.direction_frame || '待确认'} 为准：左为左臂侧，右为右臂侧；两臂共用同一方向定义。`;
    $('tip').textContent = `TCP / ${next.tip || '待确认'}`;
    if (next.tcp_mm) {
      next.tcp_mm.forEach((v, i) => $(`pos-${'xyz'[i]}`).textContent = v.toFixed(3));
      $('rpy').textContent = `RPY ${next.rpy_deg.map(v => v.toFixed(2)).join(' / ')} °`;
      modelView?.update(next);

    }
    const metrics = next.last_plan?.metrics;
    if (metrics) {
      $('duration').textContent = `${next.last_plan.duration_s.toFixed(2)} s`;
      $('path-error').textContent = `${metrics.max_path_error_mm.toFixed(5)} mm`;
      $('orientation-error').textContent = `${metrics.max_orientation_error_deg.toFixed(5)} °`;
      $('joint-delta').textContent = `${metrics.max_joint_delta_deg.toFixed(3)} °`;
    } else {
      for (const id of ['duration', 'path-error', 'orientation-error', 'joint-delta']) $(id).textContent = '—';
    }
    $('blockers').replaceChildren(...(next.blockers || []).map(text => {
      const li = document.createElement('li'); li.textContent = text; return li;
    }));
    if (epoch && (!next.armed || next.epoch !== epoch)) {
      epoch = ''; input.cancel(next.result || '服务端已退出接管');
    }
    if (!localBusy && !stopping) message(notice || next.result || '末端点动暂不可用');
    render();
    input.resume();
  } catch (error) {
    if (generation !== armGeneration) return;
    connected = false; epoch = ''; input.cancel('连接中断，已退出接管');
    if (state?.readonly) state = {...state, feedback_fresh: false, hardware_connected: false,
      tcp_actual_speed_mm_s: null, tcp_actual_angular_speed_deg_s: null};
    $('connection').textContent = '● 连接中断'; $('connection').className = 'connection bad';
    message(error.message); render();
  } finally { pollBusy = false; }
}
async function loadModel() {
  try {
    const [{ModelView}, info] = await Promise.all([
      import('./model-view.mjs'), request('/api/jog/model'),
    ]);
    modelView = new ModelView($('model-view'), text => $('model-status').textContent = text);
    if (state) modelView.update(state);
    await modelView.load(info);
    if (state?.publication?.backend === 'mujoco_ros_mock') modelView.fit(true);
    modelReady = true; render();
  } catch (error) {
    $('model-status').textContent = `模型加载失败：${error.message}`;
    modelReady = false; render();
  }
}
$('view-arm').addEventListener('click', () => modelView?.fit());
$('view-selected').addEventListener('click', () => modelView?.fit(true));
$('view-tcp').addEventListener('click', () => modelView?.focusTCP());
$('clear-trail').addEventListener('click', () => modelView?.clear());
$('arm-side').addEventListener('change', async event => {
  const side = event.target.value;
  switchBusy = true; armGeneration++; render();
  try {
    await command('select_arm', {arm: side});
    armGeneration++;
    notice = ''; epoch = ''; sequence = 0; heartbeatSequence = 0;
    await poll();
  } catch (error) { message(error.message); }
  finally { switchBusy = false; render(); }
});
$('login-form').addEventListener('submit', async event => {
  event.preventDefault();
  const button = event.currentTarget.querySelector('button');
  if (button.disabled) return;
  button.disabled = true;
  try {
    await transportReady;
    await request('/api/login', {token: $('token').value});
    if (authoritySocket) await authoritySocket.reconnect();
    $('token').value = ''; $('login-error').textContent = ''; $('login').close(); await poll();
  } catch (error) { $('login-error').textContent = error.message; }
  finally { button.disabled = false; }
});
$('lease').addEventListener('click', async () => {
  try {
    if (state?.lease === 'you') { await stop('释放控制权'); await command('release'); }
    else await command('acquire');
    await poll();
  } catch (error) { message(error.message); }
});
$('arm').addEventListener('click', async () => {
  const generation = ++armGeneration;
  try {
    const result = await command('arm');
    notice = '';
    if (generation !== armGeneration || document.hidden || !document.hasFocus()) { await stop('页面已离开前台'); return; }
    armGeneration++; epoch = result.epoch; sequence = 0; heartbeatSequence = 0;
    state = {...state, armed: true, epoch}; input.arm(); render();
    await poll();
  } catch (error) { message(error.message); }
});
$('stop').addEventListener('click', () => stop('操作员停止'));
const editable = node => node instanceof Element && Boolean(node.closest('input,textarea,select,[contenteditable="true"]'));
for (const id of ['speed', 'angular-speed']) $(id).addEventListener('change', () => {
  render();
  if (state?.publication?.continuous && epoch && !state.stopping && input.held.size === 1) {
    const direction = KEY_DIRECTIONS[input.held.values().next().value];
    if (isRotation(direction) === (id === 'angular-speed')) input.step(direction);
  }
});
document.addEventListener('focusin', render);
document.addEventListener('focusout', () => queueMicrotask(render));
for (const pad of document.querySelectorAll('.pad, .rotation-pad')) {
pad.tabIndex = 0;
pad.addEventListener('pointerdown', event => {
  if (!event.target.closest('button')) pad.focus({preventScroll: true});
});
}
window.addEventListener('keydown', event => {
  if (isRotation(KEY_DIRECTIONS[event.code]) && !state?.rotation_supported) return;
  if (!editable(event.target) && (event.code in KEY_DIRECTIONS || event.code === 'Escape')) traceInput('keydown', event);
  if (event.code in KEY_DIRECTIONS && editable(event.target)) render();
  if (input.down(event.code, {repeat: event.repeat, editable: editable(event.target), modified: event.ctrlKey || event.altKey || event.metaKey})) {
    event.preventDefault(); render();
  }
});
window.addEventListener('keyup', event => {
  if (event.code in KEY_DIRECTIONS && (!editable(event.target) || input.held.has(event.code))) traceInput('keyup', event);
  if (input.up(event.code)) { event.preventDefault(); render(); }
});
document.querySelectorAll('.dir').forEach(button => {
  button.addEventListener('pointerdown', event => {
    if (event.button !== 0) return;
    traceInput('pointerdown', {code: button.dataset.code, isTrusted: event.isTrusted});
    button.focus({preventScroll: true});
    event.preventDefault(); button.setPointerCapture(event.pointerId);
    pointerCodes.set(event.pointerId, button.dataset.code);
    input.down(button.dataset.code); render();
  });
  button.addEventListener('lostpointercapture', event => {
    const code = pointerCodes.get(event.pointerId);
    if (code) traceInput('lostpointercapture', {code, isTrusted: event.isTrusted});
    if (code) { pointerCodes.delete(event.pointerId); input.up(code); }
  });
});
for (const type of ['pointerup', 'pointercancel']) window.addEventListener(type, event => {
  const code = pointerCodes.get(event.pointerId);
  if (code) traceInput(type, {code, isTrusted: event.isTrusted});
  if (code) { pointerCodes.delete(event.pointerId); input.up(code); render(); }
});
window.addEventListener('blur', () => { traceInput('blur'); armGeneration++; input.cancel('页面失焦'); });
document.addEventListener('visibilitychange', () => { traceInput('visibilitychange'); if (document.hidden) { armGeneration++; input.cancel('页面已进入后台'); } });
window.addEventListener('pagehide', () => { traceInput('pagehide'); armGeneration++; input.cancel('页面已关闭'); });
setInterval(async () => {
  if (performance.now() - lastState > 1500 && epoch) { await stop('状态反馈超时'); return; }
  if (!epoch || !input.armed || heartbeatBusy || document.hidden || !document.hasFocus()) return;
  heartbeatBusy = true;
  const heartbeatEpoch = epoch;
  try { await command('heartbeat', {}, {timeout: 1000}); }
  catch (error) { if (epoch === heartbeatEpoch) await stop(`保活失败：${error.message}`); }
  finally { heartbeatBusy = false; }
}, 100);
setInterval(poll, 200);
await poll();
