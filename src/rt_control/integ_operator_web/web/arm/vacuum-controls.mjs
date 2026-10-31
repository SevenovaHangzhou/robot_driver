// Manual IO controls. Polling is read-only; only a click sends an explicit target.
export class VacuumControls {
  // send(body) issues the write over the page WebSocket; reads poll GET /api/arm/vacuum.
  constructor(root, send) {
    this.root = root;
    this.send = send;
    this.buttons = [...root.querySelectorAll('[data-vacuum]')];
    this.chip = root.querySelector('#vacuum-chip');
    this.message = root.querySelector('#vacuum-message');
    this.document = null;
    this.access = null;
    this.receivedAt = 0;
    this.revision = 0;
    this.pending = null;
    this.polling = false;
    this.started = false;
    this.lastMessage = '';
    this.failed = false;
    for (const button of this.buttons) {
      button.addEventListener('click', () => this.toggle(button.dataset.vacuum));
    }
  }

  start() {
    if (this.started) return;
    this.started = true;
    this.poll();
    this.timer = setInterval(() => this.poll(), 1000);
    this.expiryTimer = setInterval(() => this.render(), 500);
  }

  setAccess(status) {
    this.access = status;
    this.render();
  }

  fresh() {
    return this.document?.valid === true && performance.now() - this.receivedAt < 3000;
  }

  allowed() {
    return this.access?.lease === 'you' && !this.access.moving &&
      !this.access.sequence && !this.access.enable?.busy;
  }

  async request(method, body) {
    let timer;
    const timeout = new Promise((_, reject) => {
      timer = setTimeout(() => reject(new Error('请求超时')), 10000);
    });
    try {
      if (method !== 'GET') return await Promise.race([this.send(body), timeout]);
      const response = await Promise.race([fetch('/api/arm/vacuum'), timeout]);
      const data = await response.json();
      if (!response.ok || data.ok === false) {
        throw new Error(data.message || data.error || `HTTP ${response.status}`);
      }
      return data;
    } finally {
      clearTimeout(timer);
    }
  }

  async poll() {
    if (this.polling || this.pending) return;
    this.polling = true;
    const revision = this.revision;
    try {
      const data = await this.request('GET');
      // An older GET must not overwrite the outcome of a newer button click.
      if (!this.pending && revision === this.revision) {
        this.document = data;
        this.receivedAt = performance.now();
      }
    } catch (error) {
      if (!this.pending && revision === this.revision) {
        this.document = null;
        this.lastMessage = `无法读取开关状态：${error.message}`;
        this.failed = true;
      }
    } finally {
      this.polling = false;
      this.render();
    }
  }

  async toggle(output) {
    if (this.pending || !this.fresh() || !this.allowed() || this.document.busy) return;
    const expected = this.document.outputs[output];
    if (typeof expected !== 'boolean') return;
    this.pending = output;
    this.revision += 1;
    this.lastMessage = '';
    this.failed = false;
    this.render();
    try {
      const result = await this.request('POST', { output, enabled: !expected, expected });
      this.document = result.vacuum;
      this.receivedAt = performance.now();
      const name = { pump: '真空泵', left: '左臂阀', right: '右臂阀' }[output];
      this.lastMessage = `${name}${expected ? '已关闭' : '已开启'}，已读回确认`;
    } catch (error) {
      this.document = null;
      this.failed = true;
      this.lastMessage = error.name === 'AbortError'
        ? '操作结果未确认，正在重新读取状态；没有自动重发'
        : error.message;
    } finally {
      this.pending = null;
      this.render();
      this.poll();
    }
  }

  render() {
    const fresh = this.fresh();
    const busy = Boolean(this.pending || this.document?.busy);
    const configured = this.document?.configured !== false;
    for (const button of this.buttons) {
      const output = button.dataset.vacuum;
      const enabled = fresh ? this.document.outputs?.[output] : null;
      const known = typeof enabled === 'boolean';
      button.disabled = !known || !this.allowed() || busy;
      button.classList.toggle('on', known && enabled);
      button.setAttribute('aria-busy', String(this.pending === output));
      if (known) button.setAttribute('aria-pressed', String(enabled));
      else button.removeAttribute('aria-pressed');
      button.querySelector('strong').textContent = this.pending === output
        ? '执行中…' : known ? (enabled ? '已开启' : '已关闭') : '未知';
      button.querySelector('small').textContent = this.pending === output
        ? '等待读回' : known ? (enabled ? '点击关闭' : '点击开启') : '等待有效状态';
    }
    this.chip.textContent = busy ? '通信中' : fresh ? '已连接' : configured ? '状态不可用' : '未配置';
    this.chip.className = `chip ${fresh ? 'ok' : 'warn'}`;
    const accessMessage = !this.access ? '等待操作台连接' : this.access.lease !== 'you'
      ? '获取控制权后可操作' : !this.allowed() ? '等待当前运动、序列或使能操作结束' : '';
    this.message.textContent = this.pending ? '正在切换开关并读取确认…'
      : this.document?.message || this.lastMessage || accessMessage || '点击按钮切换对应开关';
    this.message.className = this.failed ? 'hint error' : 'hint';
  }
}
