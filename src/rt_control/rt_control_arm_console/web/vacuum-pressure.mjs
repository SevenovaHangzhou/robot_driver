// Display converted ROS pressures independently of the pump/valve controls.
export class VacuumPressure {
  constructor(element) {
    this.element = element;
    this.timer = null;
  }

  update(samples) {
    clearTimeout(this.timer);
    for (const side of ["left", "right"]) {
      const reading = this.element.querySelector(`[data-pressure="${side}"]`);
      const sample = samples?.[side];
      const valid = sample?.valid === true && sample.unit === "kPa"
        && Number.isFinite(sample.pressure_kpa);
      const rounded = valid ? Math.round(sample.pressure_kpa * 10) / 10 : null;
      reading.querySelector(".pressure-value").textContent = valid ? rounded.toFixed(1) : "—";
      const error = sample?.error || "waiting";
      const message = valid ? "" : ({
        waiting: "等待数据", stale: "数据超时", disconnected: "连接中断",
      }[error] || "数据无效");
      reading.querySelector(".pressure-status").textContent = message;
      reading.classList.toggle("unavailable", !valid);
      reading.title = valid ? "真空表压；大气压为 0 kPa" : `${message} (${error})`;
    }
    // Clear old numbers even if the HTTP event stream stalls without an error event.
    if (samples) this.timer = setTimeout(() => this.disconnect(), 1000);
  }

  disconnect() {
    this.update(Object.fromEntries(["left", "right"].map((side) => [
      side, { valid: false, error: "disconnected" },
    ])));
    clearTimeout(this.timer);
  }
}
