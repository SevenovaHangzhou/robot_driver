// Display constants for the status page. Change values here only.

// Chassis ultrasonic drawing (from the reference mockup).
export const PX = 56;       // drawing units per metre of range
export const MAXM = 1.5;    // range drawn at most, m
export const WARN = 1.0;    // at or below: orange, m
export const DANGER = 0.5;  // below: coral with glow, m

// Battery.
export const CURRENT_MAX_A = 15;  // fixed vertical scale of the discharge trace, A
export const CURRENT_SAMPLES = 32; // points in the discharge trace
// Low-charge colouring. null = not decided yet (no colour change). Percent of SOC.
export const SOC_WARN = 20;       // below: orange, % (user 2026-11-13)
export const SOC_DANGER = null;   // below: coral

// CPU.
export const CORE_HOT = 70;       // general core at or above: orange column, %
export const RT_TICK = 80;        // tick mark on real-time core bars, %
export const MATRIX_ROWS = 5;     // dot rows per general core
export const CPU_SAMPLES = 60;    // trend points (1 Hz → 60 s)

// Alerts.
export const RECOVERED_MAX = 1;   // recovered alerts shown at most

// Cameras: read-only MJPEG preview on the NX. The base URL comes from the node parameter
// `status_camera_base`, so deployment decides the address.
export const CAMERA_WIDTH = 640;         // 480 | 640 | 960
export const CAMERA_STATUS_PERIOD_MS = 2000;
