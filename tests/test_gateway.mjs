import test from 'node:test';
import assert from 'node:assert/strict';
import {AuthoritySocket} from '../web/gateway.mjs';

class Socket extends EventTarget {
  static OPEN = 1;
  static CLOSED = 3;
  static instances = [];
  constructor() {
    super(); this.readyState = 0; this.sent = []; Socket.instances.push(this);
  }
  send(data) { this.sent.push(JSON.parse(data)); }
  close() { this.readyState = 2; }
  closed(code = 1000) {
    this.readyState = Socket.CLOSED;
    this.dispatchEvent(Object.assign(new Event('close'), {code}));
  }
  message(data) { this.dispatchEvent(Object.assign(new Event('message'), {data: JSON.stringify(data)})); }
  authenticate() {
    this.readyState = Socket.OPEN;
    this.message({type: 'auth_ok', protocol: 'shared_arm_authority_v1', entry: 'jog'});
  }
}

globalThis.WebSocket = Socket;
globalThis.location = {href: 'http://127.0.0.1:18131/jog'};
const flush = () => new Promise(resolve => setImmediate(resolve));
function setup() {
  Socket.instances = [];
  const closed = [];
  return {authority: new AuthoritySocket('browser-client-012345', error => closed.push(error)), closed};
}

test('login retires an outstanding unauthorized handshake before authenticating the new cookie', async () => {
  const {authority, closed} = setup();
  const first = authority.connect();
  const rejected = assert.rejects(first, /登录会话已更新/);
  const old = Socket.instances[0];
  const replacement = authority.reconnect();
  assert.equal(authority.connect(), replacement);
  assert.equal(authority.reconnect(), replacement);
  assert.equal(Socket.instances.length, 1); // Wait for the old connection's release.
  old.closed(4401);
  await rejected; await flush();
  const next = Socket.instances[1];
  assert.ok(next);
  old.dispatchEvent(new Event('error'));
  old.message({type: 'auth_ok', protocol: 'obsolete', entry: 'jog'});
  assert.equal(authority.authenticated, false);
  next.authenticate(); await replacement;
  assert.equal(authority.authenticated, true);
  assert.equal(closed.length, 1);
  assert.ok(closed.every(error => !error.loginRequired));
});

test('cookie replacement rejects pending operations and never replays or reacquires control', async () => {
  const {authority, closed} = setup();
  const first = authority.connect();
  const old = Socket.instances[0]; old.authenticate(); await first;
  const command = authority.command('acquire');
  const rejected = assert.rejects(command, /登录会话已更新/);
  await flush();
  assert.equal(old.sent.length, 1);
  const replacement = authority.reconnect();
  await rejected; old.closed(); await flush();
  const next = Socket.instances[1]; next.authenticate(); await replacement;
  assert.deepEqual(next.sent, []);
  assert.equal(authority.pending.size, 0);
  assert.equal(closed.length, 1);
  const explicit = authority.command('acquire'); await flush();
  next.message({type: 'result', id: next.sent[0].id, ok: true});
  assert.equal((await explicit).ok, true);
});

test('an expired replacement session requests login once; obsolete close events cannot reopen it', async () => {
  const {authority, closed} = setup();
  const first = authority.connect();
  const old = Socket.instances[0]; old.authenticate(); await first;
  const replacement = authority.reconnect();
  const rejected = assert.rejects(replacement, error => error.loginRequired === true);
  old.closed(4401); await flush();
  Socket.instances[1].closed(4401); await rejected;
  old.closed(4401);
  assert.equal(authority.authenticated, false);
  assert.equal(closed.filter(error => error.loginRequired).length, 1);
});
