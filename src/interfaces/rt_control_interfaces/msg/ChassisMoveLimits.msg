# ELECTRI-133 Tier A; RT-Control private only. Not hardware defaults.
# Every field must be finite and strictly positive, including for a zero goal.
# Requested ceilings may tighten, never relax, required server configuration.
float64 max_translation_velocity  # m/s; <= 0.2
float64 max_translation_acceleration  # m/s^2
float64 max_yaw_velocity  # rad/s
float64 max_yaw_acceleration  # rad/s^2
# Wheel-side ceilings, applied to each wheel; server also enforces per-wheel limits.
float64 max_wheel_velocity  # rad/s
float64 max_wheel_acceleration  # rad/s^2
