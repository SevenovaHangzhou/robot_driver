// Navigation velocity control and explicit, readback-confirmed operation tasks.
const $ = id => document.getElementById(id);
const labels = { leisure: '休闲档', sport: '运动档' };
const state = { ws:null, authed:false, lease:'none', enabled:false, connected:false,
  busy:false, driveAvailable:false, moving:false, gear:'leisure', gears:{},
  x:0, y:0, yaw:0, pointer:null, yawPointer:null, keys:new Set(), reconnect:500 };
const tokenKey = 'rt_operator_web_token';
const fmt = x => Number.isFinite(x) ? x.toFixed(3) : '--';
let toastTimer;
function toast(text) { $('toast').textContent=text; $('toast').classList.add('show'); clearTimeout(toastTimer); toastTimer=setTimeout(()=>$('toast').classList.remove('show'),2500); }
function send(message) { if(state.authed && state.ws?.readyState===WebSocket.OPEN) {state.ws.send(JSON.stringify(message));return true;} return false; }
function acquire() { if(state.lease==='other'){toast('其他终端正在控制');return false;} if(state.lease!=='you') send({type:'acquire'});return true; }
function drive() { if(state.driveAvailable && acquire()) send({type:'drive',x:state.x,y:state.y,yaw:state.yaw}); }
function drawStick() { const radius=$('stick').clientWidth*.34; $('stick-knob').style.transform=`translate(${-state.y*radius}px,${-state.x*radius}px)`; }
function resetInputs(transmit=true) { state.x=state.y=state.yaw=0;state.pointer=state.yawPointer=null;state.keys.clear();drawStick();document.querySelectorAll('.yaw-btn').forEach(b=>b.classList.remove('pressed'));if(transmit)send({type:'halt'}); }
function stop() { resetInputs(false);send({type:'soft_stop'}); }
function locks() {
  const blocked=!state.authed || !state.driveAvailable || state.busy || state.lease==='other';
  $('stick').classList.toggle('disabled',blocked || state.yaw!==0);
  document.querySelectorAll('.yaw-btn').forEach(b=>{b.disabled=blocked || state.pointer!==null;});
  $('enable-op').disabled=!state.authed || !state.connected || (state.busy && !state.enabled) || state.lease==='other' || (!state.enabled && !state.canEnable);
  $('enable-op').textContent=state.enabled?'失能底盘':state.busy?'操作中…':'使能底盘';
  $('enable-op').classList.toggle('enabled',state.enabled);
  $('reset-op').hidden=!state.canReset;
  $('reset-op').disabled=!state.authed || state.busy || state.lease==='other';
  $('steering-apply').disabled=!state.authed || !state.canTuneSteering || state.busy || state.lease==='other';
  $('steering-defaults').disabled=$('steering-apply').disabled;
  for(const mode of ['navigation','operation']) {
    const button=$(`mode-${mode}`);
    button.disabled=!state.authed || !state.canSwitchMode || state.busy || state.operationPending || state.lease==='other' || state.chassisMode===mode;
    button.setAttribute('aria-pressed',String(state.chassisMode===mode));
  }
  $('operation-fields').disabled=!state.authed || !state.canExecuteOperation || state.lease==='other' || state.operationPending;
  $('operation-execute').disabled=$('operation-fields').disabled;
  $('operation-cancel').disabled=!state.authed || !(state.operationActive || state.operationPending);
  $('lease-op').disabled=!state.authed || state.lease==='other';
  $('lease-op').textContent=state.lease==='you'?'释放控制权':state.lease==='other'?'其他终端控制中':'获取控制权';
  $('soft-stop').disabled=!state.authed;
  document.querySelectorAll('.gear-btn').forEach(b=>{b.disabled=state.moving || state.busy || state.lease==='other';b.classList.toggle('active',b.dataset.gear===state.gear);b.setAttribute('aria-checked',String(b.dataset.gear===state.gear));});
}
function renderGears() {
  $('gear-op').replaceChildren();
  for(const [name,gear] of Object.entries(state.gears)) {
    const b=document.createElement('button');b.type='button';b.className='gear-btn';b.dataset.gear=name;b.role='radio';b.textContent=labels[name]||name;
    b.addEventListener('click',()=>{if(state.moving){toast('请先停止操作再换档');return;}if(acquire())send({type:'gear',gear:name});});
    $('gear-op').appendChild(b);
  }
}
function status(message) {
  const backend=message.robot?.chassis || {};
  const wasAllowed=state.driveAvailable;
  state.lease=message.lease;state.gear=message.gear;state.moving=message.moving;
  state.connected=backend.connected===true;state.enabled=backend.enabled===true;
  state.canEnable=backend.can_enable===true;state.busy=backend.busy===true;
  state.canReset=backend.can_reset===true;state.canTuneSteering=backend.can_tune_steering===true;
  const controlMode=backend.chassis_mode||{};
  renderOperation(backend.operation||{});
  state.canSwitchMode=controlMode.can_switch===true;
  state.chassisMode=controlMode.mode;
  $('chassis-mode').textContent=controlMode.mode==='operation'?'作业模式（位置任务）':controlMode.mode==='navigation'?(controlMode.confirmed?'导航模式（速度控制）':'导航（速度入口）'):'实际模式待确认';
  $('chassis-mode-hint').textContent=controlMode.message||'等待控制器模式配置';
  $('drive-hint').textContent=controlMode.mode==='operation'?'作业模式使用相对位移任务，速度摇杆已锁定':'先使能底盘；拖动摇杆或按住 WASD 平移，Q/E 旋转';
  const steering=backend.steering||{};
  if(steering.max_steering_slew){
    const jerk=Number.isFinite(steering.max_steering_jerk)?`${steering.max_steering_jerk.toFixed(0)} °/s³`:'不限（梯形）';
    $('steering-current').textContent=`当前：速度 ${steering.max_steering_slew.toFixed(1)} °/s · 加速度 ${steering.max_steering_acceleration.toFixed(1)} °/s² · 加加速度 ${jerk}`;
    $('steering-limit').textContent=`上限 ${steering.max_steering_slew_limit.toFixed(0)} °/s · ${steering.max_steering_acceleration_limit.toFixed(0)} °/s² · ${steering.max_steering_jerk_limit.toFixed(0)} °/s³`;
    $('steering-jerk').max=steering.max_steering_jerk_limit.toFixed(0);
    const alignFirst=steering.drive_mode==='align_first';
    if(state.alignMode!==undefined&&state.alignMode!==alignFirst){state.steeringEdited=false;$('steering-token').value='';}
    state.alignMode=alignFirst;
    $('drive-mode').textContent=alignFirst?'对齐模式':steering.drive_mode==='steer_while_driving'?'连续模式':'未知';
    const fmt=v=>Number(v.toFixed(1)).toString();
    $('steering-current').textContent+=alignFirst
      ?` · 对齐门限 ${fmt(steering.align_first_threshold)}°/恢复 ${fmt(steering.align_first_release)}°`
      :` · 对齐门限 ${fmt(steering.alignment_threshold)}°`;
    $('steering-align-label').textContent=alignFirst?'对齐门限 °（超过则四轮停驱）':'对齐门限 °（90 = 不等待对齐）';
    $('steering-release-row').style.display=alignFirst?'':'none';
    const tuning=backend.steering_tuning||{};
    $('steering-source').textContent=tuning.source==='field'?`来源：现场调参（${tuning.updated_at}）`:tuning.source==='invalid'?`来源：现场调参文件无效（${tuning.updated_at}）`:'来源：默认值';
    $('steering-slew').max=steering.max_steering_slew_limit.toFixed(1);$('steering-accel').max=steering.max_steering_acceleration_limit.toFixed(1);
    if(!state.steeringEdited){$('steering-slew').value=steering.max_steering_slew.toFixed(1);$('steering-accel').value=steering.max_steering_acceleration.toFixed(1);$('steering-jerk').value=Number.isFinite(steering.max_steering_jerk)?steering.max_steering_jerk.toFixed(0):'';$('steering-align').value=fmt(alignFirst?steering.align_first_threshold:steering.alignment_threshold);if(alignFirst)$('steering-release').value=fmt(steering.align_first_release);}
  }
  state.driveAvailable=message.robot?.drive_available===true;
  if(wasAllowed && !state.driveAvailable)resetInputs(false);
  $('backend-badge').textContent=backend.backend_kind==='mock'?'软件测试':backend.backend_kind==='real'?'真实底盘':'等待底盘';
  $('ctrl-state').textContent=!state.connected?'底盘未连接':state.enabled?'底盘已使能':'底盘未使能';
  $('ctrl-detail').textContent=backend.message||'等待底盘运行栈启动';
  $('ctrl-dot').className='dot '+(!state.connected?'bad':state.enabled?'ok':'warn');
  $('endpoint').textContent=`速度接口 ${backend.command_topic || '/cmd_vel'}`;
  if(backend.last_error)$('last-result').textContent=backend.last_error;
  $('lease-chip').querySelector('b').textContent=state.lease==='you'?'本机控制':state.lease==='other'?'其他终端控制':'无控制权';
  $('rd-vx').textContent=fmt(message.command.vx);$('rd-vy').textContent=fmt(message.command.vy);$('rd-wz').textContent=fmt(message.command.wz);
  const gear=state.gears[state.gear];
  if(gear)$('gear-detail').textContent=`${labels[state.gear]}：平移 ${gear.linear} m/s · 旋转 ${gear.angular} rad/s`;
  document.body.classList.toggle('sport',state.gear==='sport');locks();
}
function connect() {
  const token=sessionStorage.getItem(tokenKey);if(!token){$('login').classList.remove('hidden');return;}
  const previous=state.ws;const socket=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws`);state.ws=socket;state.authed=false;if(previous && previous.readyState<2)previous.close();
  socket.addEventListener('open',()=>socket.send(JSON.stringify({type:'auth',token})));
  socket.addEventListener('message',event=>{
    if(state.ws!==socket)return;
    const data=JSON.parse(event.data);
    if(data.type==='auth_ok') {state.authed=true;state.reconnect=500;state.gears=data.gears;renderGears();$('login').classList.add('hidden');$('conn').className='sb-item sb-ok';$('conn').querySelector('b').textContent='网页已连接';}
    else if(data.type==='status')status(data);
    else if(data.type==='chassis_mode_result') {resetInputs(false);toast(data.message);$('last-result').textContent=data.message;}
    else if(data.type==='operation_result') {state.operationPending=false;toast(data.message);$('last-result').textContent=data.message;locks();}
    else if(data.type==='operation_cancel_result') {toast(data.message);$('last-result').textContent=data.message;}
    else if(data.type==='steering_params_result') {toast(data.message);$('last-result').textContent=data.message;if(data.ok)state.steeringEdited=false;$('steering-token').value='';}
    else if(data.type==='module_op_result') {resetInputs(false);$('last-result').textContent=data.op==='reset'?(data.stage||(data.ok?'故障已复位':'复位失败')):data.ok?(data.enabled?'底盘使能成功':'底盘已失能'):(data.stage||'操作失败');toast($('last-result').textContent);state.busy=false;locks();}
    else if(data.type==='error') {
      if(data.code==='auth_failed'){sessionStorage.removeItem(tokenKey);$('login-error').textContent=data.message;$('login').classList.remove('hidden');}
      else{toast(data.message);state.busy=false;state.operationPending=false;locks();}
    }
  });
  socket.addEventListener('close',event=>{
    if(state.ws!==socket)return;
    resetInputs(false);state.authed=state.enabled=state.connected=state.driveAvailable=false;state.lease='none';state.busy=false;
    state.canSwitchMode=false;state.chassisMode=null;$('chassis-mode').textContent='实际模式待确认';$('chassis-mode-hint').textContent='网页连接已断开';
    state.canExecuteOperation=false;state.operationPending=false;state.operationActive=false;
    $('operation-message').textContent='网页连接已断开，服务端将请求取消本网页作业；请核对实际停车状态。';
    $('conn').className='sb-item sb-bad';$('conn').querySelector('b').textContent='网页未连接';$('ctrl-state').textContent='底盘状态待确认';$('ctrl-detail').textContent='网页连接已断开';locks();
    if(event.code!==4401){setTimeout(connect,state.reconnect);state.reconnect=Math.min(state.reconnect*2,5000);}
  });
}
$('login-form').addEventListener('submit',event=>{event.preventDefault();sessionStorage.setItem(tokenKey,$('token').value.trim());$('token').value='';$('login-error').textContent='';connect();});
$('enable-op').addEventListener('click',()=>{
  if(!state.connected || (state.busy && !state.enabled))return;
  resetInputs();if(!acquire())return;
  const op=state.enabled?'disable':'enable';state.busy=true;locks();send({type:'module_op',op,modules:['chassis']});
});
$('reset-op').addEventListener('click',()=>{
  if(state.busy || !state.canReset)return;
  if(!acquire())return;
  state.busy=true;locks();send({type:'module_op',op:'reset',modules:['chassis']});
});
for(const mode of ['navigation','operation'])$(`mode-${mode}`).addEventListener('click',()=>{
  if(state.busy || !state.canSwitchMode)return;
  const name=mode==='operation'?'作业模式（位置任务）':'导航模式（速度控制）';
  if(!window.confirm(`确认切换到${name}？切换会停止当前速度操作，完成后不会恢复旧指令。`))return;
  resetInputs();if(!acquire())return;
  state.busy=true;state.driveAvailable=false;locks();
  send({type:'chassis_mode',mode,confirm:true});
});
for(const id of ['steering-slew','steering-accel','steering-jerk','steering-align','steering-release'])$(id).addEventListener('input',()=>{state.steeringEdited=true;});
$('steering-apply').addEventListener('click',()=>{
  const slew=Number($('steering-slew').value),accel=Number($('steering-accel').value),jerk=Number($('steering-jerk').value),alignment=Number($('steering-align').value),release=state.alignMode?Number($('steering-release').value):0,token=$('steering-token').value;
  if(!token){toast('请输入确认令牌');return;}
  if(!(slew>0)||!(accel>0)||!(jerk>0)||!(alignment>0&&alignment<=90)){toast('请输入有效的速度、加速度、加加速度和对齐门限');return;}
  if(state.alignMode&&!(release>0&&release<=alignment)){toast('恢复门限须大于 0 且不大于对齐门限');return;}
  if(!acquire())return;
  send({type:'steering_params',slew_deg_s:slew,accel_deg_s2:accel,jerk_deg_s3:jerk,alignment_deg:alignment,release_deg:release,token});
});
$('steering-defaults').addEventListener('click',()=>{
  const token=$('steering-token').value;
  if(!token){toast('请输入确认令牌');return;}
  if(!acquire())return;
  state.steeringEdited=false;send({type:'steering_defaults',token});
});
$('lease-op').addEventListener('click',()=>{resetInputs();if(state.lease==='you')send({type:'release'});else acquire();});
$('soft-stop').addEventListener('click',stop);
const operationFields=['dx','dy','dyaw_deg','max_translation_velocity','max_translation_acceleration','max_yaw_velocity','max_yaw_acceleration','max_wheel_velocity','max_wheel_acceleration','max_duration'];
const angularFields=new Set(['max_yaw_velocity','max_yaw_acceleration']);
function renderOperation(operation) {
  state.canExecuteOperation=operation.can_execute===true;
  state.operationActive=operation.active===true;
  const limits=operation.limits||{};
  if(Object.keys(limits).length) {
    const defaults={max_translation_velocity:.02,max_translation_acceleration:.05,max_yaw_velocity:.05,max_yaw_acceleration:.1,max_wheel_velocity:.5,max_wheel_acceleration:1,max_duration:30};
    for(const [key,limit] of Object.entries(limits)) {
      const input=$(`operation-${key}`);if(!input)continue;
      const factor=angularFields.has(key)?180/Math.PI:1;
      input.max=String(limit*factor);
      if(!state.operationEdited)input.value=String(Math.min(defaults[key],limit)*factor);
    }
    $('operation-limits').textContent=`控制器上限：平移 ${fmt(limits.max_translation_velocity)} m/s，旋转 ${fmt(limits.max_yaw_velocity*180/Math.PI)} °/s；任务超时 ≤ ${limits.max_duration} s。`;
  } else {$('operation-limits').textContent='运动上限未收到或已过期，不能执行';}
  const phases={0:'导航',1:'切换中',2:'转向对齐',3:'位移执行',4:'停车中',5:'位置保持',6:'故障'};
  const feedback=operation.feedback||{};
  $('operation-badge').textContent=operation.unknown?'结果待核对':operation.active?(phases[feedback.phase]||'提交中'):state.canExecuteOperation?'可配置执行':'等待作业就绪';
  $('operation-progress').value=Number.isFinite(feedback.progress)?Math.max(0,Math.min(1,feedback.progress)):0;
  $('operation-feedback').textContent=feedback.estimate_valid?`反馈计算位移：前后 ${fmt(feedback.actual_dx)} m · 左右 ${fmt(feedback.actual_dy)} m · 旋转 ${fmt(feedback.actual_dyaw_deg)}°`:'反馈计算位移：--（尚无有效反馈）';
  $('operation-message').textContent=operation.message||'先使能并切换到作业模式，等待实际确认。';
}
for(const field of operationFields)$(`operation-${field}`).addEventListener('input',()=>{state.operationEdited=true;});
$('operation-form').addEventListener('submit',event=>{
  event.preventDefault();
  if(!state.canExecuteOperation || state.operationPending)return;
  const values=Object.fromEntries(operationFields.map(key=>[key,Number($(`operation-${key}`).value)]));
  if(!Object.values(values).every(Number.isFinite) || Math.hypot(values.dx,values.dy)>.7 || Math.abs(values.dyaw_deg)>15){toast('请输入有效位移，平移合量≤0.7米，旋转≤15度');return;}
  if(!window.confirm(`确认执行作业运动？\n前后 ${values.dx} m，左右 ${values.dy} m，旋转 ${values.dyaw_deg}°\n相对任务开始时的底盘朝向。`))return;
  for(const key of angularFields)values[key]*=Math.PI/180;
  resetInputs();if(!acquire())return;
  state.operationPending=true;locks();
  if(!send({type:'operation_move',confirm:true,...values})){state.operationPending=false;locks();}
});
$('operation-cancel').addEventListener('click',()=>{send({type:'operation_cancel'});});
function pointerPosition(event) {
  const rect=$('stick').getBoundingClientRect(),r=$('stick').clientWidth*.34;
  let x=-(event.clientY-rect.top-rect.height/2)/r,y=-(event.clientX-rect.left-rect.width/2)/r;
  const norm=Math.max(1,Math.hypot(x,y));state.x=Math.trunc(x/norm*1000)/1000;state.y=Math.trunc(y/norm*1000)/1000;
  drawStick();drive();
}
$('stick').addEventListener('pointerdown',event=>{event.preventDefault();if(!state.driveAvailable || state.busy || state.yaw || state.lease==='other')return;state.pointer=event.pointerId;$('stick').setPointerCapture(event.pointerId);pointerPosition(event);});
window.addEventListener('pointermove',event=>{if(event.pointerId===state.pointer)pointerPosition(event);});
document.querySelectorAll('.yaw-btn').forEach(button=>button.addEventListener('pointerdown',event=>{event.preventDefault();if(button.disabled)return;state.yawPointer=event.pointerId;state.yaw=Number(button.dataset.yaw);button.setPointerCapture(event.pointerId);button.classList.add('pressed');locks();drive();}));
for(const name of ['pointerup','pointercancel'])window.addEventListener(name,event=>{if(event.pointerId===state.pointer || event.pointerId===state.yawPointer){resetInputs();locks();}});
function keyboardDrive() {
  state.yaw=(state.keys.has('KeyQ')?1:0)-(state.keys.has('KeyE')?1:0);
  state.x=state.yaw?0:(state.keys.has('KeyW')?1:0)-(state.keys.has('KeyS')?1:0);
  state.y=state.yaw?0:(state.keys.has('KeyA')?1:0)-(state.keys.has('KeyD')?1:0);
  const norm=Math.max(1,Math.hypot(state.x,state.y));state.x/=norm;state.y/=norm;drawStick();drive();locks();
}
window.addEventListener('keydown',event=>{
  if(event.target instanceof HTMLInputElement)return;
  if(event.code==='Space'){event.preventDefault();stop();return;}
  if(!['KeyW','KeyS','KeyA','KeyD','KeyQ','KeyE'].includes(event.code) || !state.driveAvailable || state.busy)return;
  event.preventDefault();state.keys.add(event.code);keyboardDrive();
});
window.addEventListener('keyup',event=>{if(state.keys.delete(event.code))keyboardDrive();});
window.addEventListener('blur',()=>resetInputs());
document.addEventListener('visibilitychange',()=>{if(document.hidden){resetInputs();send({type:'release'});}});
window.addEventListener('pagehide',()=>{resetInputs();send({type:'release'});});
setInterval(()=>{if(state.authed && state.driveAvailable && state.lease==='you' && !document.hidden)drive();},50);
locks();connect();
