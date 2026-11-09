import test from 'node:test';
import assert from 'node:assert/strict';
import {JogInput, KEY_DIRECTIONS, MotionFeedback, motionCommand} from '../web/input.mjs';
function fixture() {
  const events = [];
  const input = new JogInput({step: d => events.push(d), stop: r => events.push(`stop:${r}`), release: r => events.push(`release:${r}`)});
  return {events, input};
}
test('six directions, main and keypad +/-, unarmed has no effect', () => {
  for (const [code, direction] of Object.entries(KEY_DIRECTIONS)) {
    const {input, events} = fixture(); assert.equal(input.down(code), false);
    input.arm(); input.down(code); assert.deepEqual(events, [direction]);
  }
});
test('repeat does not queue; key release requests halt', () => {
  const {input, events} = fixture(); input.arm(); input.down('ArrowUp');
  input.down('ArrowUp', {repeat: true}); input.down('ArrowUp');
  assert.deepEqual(events, ['x+']); input.up('ArrowUp');
  assert.equal(input.armed, true); assert.match(events[1], /release/);
});
test('conflicting keys stop; no diagonal move', () => {
  const {input, events} = fixture(); input.arm(); input.down('ArrowUp'); input.down('ArrowLeft');
  assert.equal(events.length, 2); assert.match(events[1], /stop/); assert.equal(input.armed, false);
});
test('editing and modified shortcuts never move; busy never queues', () => {
  const {input, events} = fixture(); input.arm();
  input.down('ArrowUp', {editable: true}); input.down('ArrowLeft', {modified: true});
  input.busy = true; input.down('NumpadAdd'); assert.deepEqual(events, []);
  input.busy = false; assert.deepEqual(events, []);
});
test('focus cancellation and Escape require new explicit arm', () => {
  const {input, events} = fixture(); input.arm(); input.cancel('blur'); input.down('ArrowUp');
  assert.deepEqual(events, ['stop:blur']); input.arm(); input.down('Escape'); assert.equal(input.armed, false);
});

test('long hold dispatches another step only after completed feedback', () => {
  const {input, events} = fixture(); input.arm(); input.down('ArrowUp');
  for (let i = 0; i < 300; i++) input.down('ArrowUp', {repeat:true});
  assert.deepEqual(events, ['x+']); assert.equal(input.resume(), false);
  input.busy = false; assert.equal(input.resume(), true);
  assert.deepEqual(events, ['x+', 'x+']);
  input.up('ArrowUp'); input.busy = false;
  assert.equal(input.resume(), false); assert.match(events.at(-1), /release/);
});

test('feedback shows measured command movement even when a large step barely starts', () => {
  const feedback = new MotionFeedback();
  const state = {model_identity:'right',tcp_mm:[100,200,300],busy:true};
  feedback.begin(state,'x+',10,23.2);
  const update = feedback.update({...state,tcp_mm:[100.001,200,300]});
  assert.ok(Math.abs(update.travelled-.001)<1e-9);
  assert.ok(Math.abs(update.remaining-9.999)<1e-9);
  assert.match(update.label,/起步位移很小/);
  assert.equal(update.duration,23.2);
});

test('release feedback retains unexecuted distance without claiming full completion', () => {
  const feedback = new MotionFeedback();
  const state = {model_identity:'left',tcp_mm:[1,2,3]};
  feedback.begin(state,'z-',2,4.6);
  const result=feedback.update({...state,tcp_mm:[1,2,2.75],busy:false});
  assert.equal(result.travelled,.25);
  assert.equal(result.remaining,1.75);
  assert.equal(result.percent,12.5);
  assert.equal(result.label,'本段已结束');
  assert.equal(feedback.update({...state,model_identity:'right'}),null);
});

test('speed feedback accumulates held movement and resets for a new press or arm', () => {
  const feedback = new MotionFeedback();
  const state = {model_identity:'left', tcp_mm:[1,2,3], busy:true, tcp_command_speed_mm_s:4.25};
  feedback.begin(state,'x+',5,2,10,1);
  feedback.begin({...state,tcp_mm:[6,2,3]},'x+',5,2,10,1);
  let result=feedback.update({...state,tcp_mm:[7,2,3]});
  assert.equal(result.holdTravelled,6);
  assert.equal(result.requestedSpeed,10);
  assert.equal(result.commandSpeed,4.25);
  feedback.begin({...state,tcp_mm:[7,2,3]},'x+',5,2,2,2);
  result=feedback.update({...state,tcp_mm:[7.5,2,3]});
  assert.equal(result.holdTravelled,.5);
  assert.equal(result.requestedSpeed,2);
  assert.equal(feedback.update({...state,model_identity:'right'}),null);
  assert.equal(feedback.update({...state,tcp_command_speed_mm_s:undefined}).commandSpeed,null);
});

test('continuous feedback uses actual TCP movement without inventing a segment endpoint', () => {
  const feedback = new MotionFeedback();
  const start = {model_identity:'left',tcp_mm:[1,2,3],busy:true};
  feedback.begin(start,'x+',null,null,10,1);
  const result = feedback.update({...start,tcp_mm:[21,2,3],tcp_actual_speed_mm_s:9.8});
  assert.equal(result.holdTravelled,20);
  assert.equal(result.remaining,null);
  assert.equal(result.percent,null);
  assert.equal(result.commandSpeed,9.8);
  assert.equal(result.label,'连续移动');
  assert.equal(feedback.update({...start,busy:false}).label,'已停止');
});

test('QWE / ZXC rotate J7 yaw, pitch, roll with angular units', () => {
  for (const [code,direction] of Object.entries({KeyQ:'yaw+',KeyZ:'yaw-',KeyW:'pitch+',KeyX:'pitch-',KeyE:'roll+',KeyC:'roll-'})) {
    assert.equal(KEY_DIRECTIONS[code],direction);
    const {input,events}=fixture(); input.arm(); input.down(code);
    for(let i=0;i<300;i++)input.down(code,{repeat:true});
    assert.deepEqual(events,[direction]); input.up(code);
    assert.match(events.at(-1),/release/);
    assert.deepEqual(motionCommand(direction,30,1),{robot_rotation:direction,rotation_frame:'j7',speed_deg_s:1});
  }
  assert.deepEqual(motionCommand('x+',2,5),{robot_direction:'x+',speed_mm_s:2});
});
test('translation/rotation conflict stops; rotation in editable fields never moves', () => {
  const {input,events}=fixture(); input.arm();
  input.down('KeyQ',{editable:true}); input.down('KeyW',{modified:true});
  assert.deepEqual(events,[]);
  input.down('KeyQ'); input.down('ArrowUp');
  assert.match(events.at(-1),/stop/); assert.equal(input.armed,false);
});
const quat=(axis,degrees)=>{
  const q=[0,0,0,Math.cos(degrees*Math.PI/360)];q[axis]=Math.sin(degrees*Math.PI/360);return q;
};
const product=(p,q)=>[
  p[3]*q[0]+p[0]*q[3]+p[1]*q[2]-p[2]*q[1],
  p[3]*q[1]-p[0]*q[2]+p[1]*q[3]+p[2]*q[0],
  p[3]*q[2]+p[0]*q[1]-p[1]*q[0]+p[2]*q[3],
  p[3]*q[3]-p[0]*q[0]-p[1]*q[1]-p[2]*q[2]];
test('angular feedback measures moving J7 axes with a rotated TCP, including Euler singularities',()=>{
  for(const [direction,axis] of [['roll+',0],['pitch+',1],['yaw+',2]]){
    const initial=product(quat(0,73),quat(1,90));
    const state={model_identity:'left',tcp_mm:[1,2,3],rotation_orientation_xyzw:initial,
      tcp_orientation_xyzw:product(initial,quat(2,40)),busy:true};
    const feedback=new MotionFeedback();
    feedback.begin(state,direction,null,null,1,1,'xyz'[axis]+'+');
    const after={...state,rotation_orientation_xyzw:product(initial,quat(axis,2)),tcp_actual_angular_speed_deg_s:.8};
    const result=feedback.update(after);
    assert.ok(Math.abs(result.holdTravelled-2)<1e-10);
    assert.equal(result.commandSpeed,.8);assert.equal(result.angular,true);
    assert.equal(result.label,'连续旋转');
    feedback.begin(after,direction,null,null,2,1,'xyz'[axis]+'+');
    assert.ok(Math.abs(feedback.update({...after,rotation_orientation_xyzw:product(initial,quat(axis,3))}).holdTravelled-3)<1e-10);
  }
});
test('angular feedback crosses 180 degrees, quaternion sign flips, and resets on the next press',()=>{
  const state={model_identity:'right',tcp_mm:[0,0,0],rotation_orientation_xyzw:quat(2,179),busy:true};
  const feedback=new MotionFeedback(); feedback.begin(state,'yaw+',null,null,1,1,'z+');
  const after={...state,rotation_orientation_xyzw:quat(2,-179).map(v=>-v)};
  assert.ok(Math.abs(feedback.update(after).holdTravelled-2)<1e-10);
  feedback.begin(after,'yaw-',null,null,1,2,'z-');
  assert.ok(Math.abs(feedback.update({...state,rotation_orientation_xyzw:quat(2,179)}).holdTravelled-2)<1e-10);
  assert.equal(feedback.update({...state,model_identity:'left'}),null);
});
