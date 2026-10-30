// Only live transitions to successful idle states produce a completion sound.
export class CompletionSound {
  constructor(audio, report = () => {}) {
    this.audio = audio;
    this.report = report;
    this.previous = null;
    this.ready = false;
    this.unlocking = false;
  }

  reset() { this.previous = null; }

  // Called synchronously from a user gesture, so the browser can allow audio.
  async unlock() {
    if (this.ready || this.unlocking) return;
    this.unlocking = true;
    this.audio.muted = true;
    try {
      await this.audio.play();
      this.audio.pause();
      this.audio.currentTime = 0;
      this.ready = true;
    } catch (_) {
      // A later gesture can retry; show an actionable message if completion needs sound.
    } finally {
      this.audio.muted = false;
      this.unlocking = false;
    }
  }

  update(status) {
    const previous = this.previous;
    this.previous = status;
    if (!previous || status.moving || status.sequence) return;
    const result = status.last_result || '';
    const success = result === '已到达' || /^序列完成（\d+ 个位姿）$/.test(result);
    const changed = previous.moving || previous.sequence || previous.last_result !== result;
    if (!success || !changed) return;
    if (!this.ready) {
      this.report('动作已完成；提示音未启用，请点击页面后再执行下一次动作。');
      return;
    }
    try {
      this.audio.currentTime = 0;
      Promise.resolve(this.audio.play()).catch(() => {
        this.ready = false;
        this.report('动作已完成，但提示音播放失败，请检查浏览器声音权限。');
      });
    } catch (_) {
      this.ready = false;
      this.report('动作已完成，但提示音播放失败，请检查浏览器声音权限。');
    }
  }
}
