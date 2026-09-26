// Parking-radar beeper. Display/operator aid only; it never stops the robot.
//
// The beep period follows the nearest fresh ultrasonic distance continuously
// (not per display band) and the scheduler never restarts on every update:
// a closer obstacle can only pull the next beep earlier, so the rhythm speeds
// up smoothly as the distance shrinks.

export const TONE_BELOW_M = 0.3; // continuous tone below this distance
export const FASTEST_PERIOD_S = 0.08;
export const SLOWEST_PERIOD_S = 1.0;

// Seconds between beeps; 0 = continuous tone; null = silent.
export function beepPeriod(distance, farM) {
  if (typeof distance !== "number" || !Number.isFinite(distance) || distance < 0) return null;
  if (distance >= farM) return null;
  if (distance < TONE_BELOW_M) return 0;
  const t = (distance - TONE_BELOW_M) / (farM - TONE_BELOW_M); // 0 near .. 1 far
  return FASTEST_PERIOD_S + (SLOWEST_PERIOD_S - FASTEST_PERIOD_S) * Math.pow(t, 1.2);
}

// Higher pitch when closer adds a second urgency cue.
export function beepPitch(distance, farM) {
  const t = Math.min(Math.max((distance - TONE_BELOW_M) / (farM - TONE_BELOW_M), 0), 1);
  return 1400 + 600 * (1 - t);
}

export class ParkingBeeper {
  // audio: { now(): seconds, beep(freq, duration), toneOn(freq), toneOff() }
  // timers: { set(fn, ms) -> id, clear(id) }
  constructor(audio, timers) {
    this.audio = audio;
    this.timers = timers;
    this.period = null;
    this.pitch = 1400;
    this.lastBeep = -Infinity;
    this.timer = null;
    this.toneOn = false;
  }

  // distance: nearest fresh distance in metres, or null; farM: silent beyond.
  update(distance, farM) {
    const period = distance === null ? null : beepPeriod(distance, farM);
    if (period !== null && period > 0) this.pitch = beepPitch(distance, farM);
    const previous = this.period;
    this.period = period;
    if (period === null) { this.stop(); return; }
    if (period === 0) {
      this.clearTimer();
      if (!this.toneOn) { this.audio.toneOn(2000); this.toneOn = true; }
      return;
    }
    if (this.toneOn) { this.audio.toneOff(); this.toneOn = false; }
    const now = this.audio.now();
    const due = this.lastBeep + period;
    if (this.timer === null || previous === null || previous === 0) {
      this.schedule(Math.max(due - now, 0));
    } else if (period < previous) {
      // Closer: pull the pending beep earlier, never later.
      this.schedule(Math.max(due - now, 0));
    }
    // Farther: keep the pending beep; the next one uses the longer period.
  }

  fire() {
    this.timer = null;
    if (this.period === null || this.period === 0) return;
    this.lastBeep = this.audio.now();
    this.audio.beep(this.pitch, Math.min(0.07, this.period * 0.45));
    this.schedule(this.period);
  }

  schedule(seconds) {
    this.clearTimer();
    this.timer = this.timers.set(() => this.fire(), seconds * 1000);
  }

  clearTimer() {
    if (this.timer !== null) { this.timers.clear(this.timer); this.timer = null; }
  }

  stop() {
    this.clearTimer();
    if (this.toneOn) { this.audio.toneOff(); this.toneOn = false; }
    this.period = null;
  }
}
