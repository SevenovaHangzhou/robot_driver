// This socket owns no local lease. Every command is acknowledged by the primary console.
export class AuthoritySocket {
  constructor(client, onClose) {
    this.client = client;
    this.onClose = onClose;
    this.pending = new Map();
    this.sequence = 0;
    this.socket = null;
    this.connecting = null;
    this.reconnecting = null;
    this.authenticated = false;
  }
  connect() {
    return this.reconnecting || this.open();
  }
  open() {
    if (this.socket?.readyState === WebSocket.OPEN && this.authenticated) return Promise.resolve();
    if (this.connecting) return this.connecting;
    const connecting = new Promise((resolve, reject) => {
      const url = new URL('/ws/jog', location.href);
      url.protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
      url.searchParams.set('client', this.client);
      const socket = this.socket = new WebSocket(url);
      const timer = setTimeout(() => { reject(new Error('统一控制权连接超时')); socket.close(); }, 5000);
      this.cancelConnect = error => { clearTimeout(timer); reject(error); };
      socket.addEventListener('message', event => {
        if (socket !== this.socket) return;
        const message = JSON.parse(event.data);
        if (message.type === 'auth_ok') {
          if (message.protocol !== 'shared_arm_authority_v1' || message.entry !== 'jog') {
            reject(new Error('主操作台尚未接入共享控制权')); socket.close(); return;
          }
          clearTimeout(timer); this.authenticated = true; resolve();
        } else if (message.type === 'result') {
          const request = this.pending.get(message.id);
          if (!request) return;
          clearTimeout(request.timer); this.pending.delete(message.id);
          if (message.ok) request.resolve(message);
          else request.reject(new Error(message.message || message.error));
        }
      });
      socket.addEventListener('close', event => {
        clearTimeout(timer);
        if (socket !== this.socket) return;
        this.authenticated = false;
        const error = new Error(event.code === 4401 ? '请登录主操作台' : '统一控制权连接已断开');
        error.loginRequired = event.code === 4401;
        reject(error);
        this.rejectPending(error); this.onClose(error);
      });
      socket.addEventListener('error', () => {
        if (socket !== this.socket) return;
        clearTimeout(timer); reject(new Error('主操作台共享控制权接口不可用')); socket.close();
      });
    }).finally(() => {
      if (this.connecting === connecting) { this.connecting = null; this.cancelConnect = null; }
    });
    this.connecting = connecting;
    return connecting;
  }
  rejectPending(error) {
    for (const request of this.pending.values()) {
      clearTimeout(request.timer); request.reject(error);
    }
    this.pending.clear();
  }
  reconnect() {
    if (this.reconnecting) return this.reconnecting;
    const socket = this.socket;
    const error = new Error('登录会话已更新，请重新获取控制权和开启键盘接管');
    // Retire callbacks from the old cookie before opening its replacement.
    this.socket = null; this.authenticated = false;
    this.cancelConnect?.(error); this.cancelConnect = null; this.connecting = null;
    this.rejectPending(error); this.onClose(error);
    const closed = !socket || socket.readyState === WebSocket.CLOSED ? Promise.resolve() : new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error('旧控制权连接退出超时，请刷新页面')), 5000);
      socket.addEventListener('close', () => { clearTimeout(timer); resolve(); }, {once: true});
      socket.close();
    });
    const reconnecting = closed.then(() => this.open()).finally(() => {
      if (this.reconnecting === reconnecting) this.reconnecting = null;
    });
    this.reconnecting = reconnecting;
    return reconnecting;
  }
  async command(type) {
    await this.connect();
    const id = ++this.sequence;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error('控制权请求结果未确认，连接已关闭'));
        this.socket.close();
      }, 5000);
      this.pending.set(id, {resolve, reject, timer});
      this.socket.send(JSON.stringify({type, id}));
    });
  }
}
