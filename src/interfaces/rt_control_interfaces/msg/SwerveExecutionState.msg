# ELECTRI-127; RT-Control private only (see README.md). No other domain may depend on it yet.
# Read-only execution state of swerve_driver/SwerveController, published about every 10 ms from
# the realtime update loop while the controller is active. Never a command input.
#
# Arrays are indexed by wheel module number 1..4 (index 0 = module 1).
# Steering angles: module heading in the controller's base frame (base_footprint, x forward,
# y left, counterclockwise positive about +z), rad, unwrapped, i.e. the controller's
# kinematic angle after the steering joint heading sign (not the raw joint position).
# Wheel speeds: wheel-surface speed in m/s, signed along the module heading.

uint8 STATUS_INACTIVE=0
uint8 STATUS_RUNNING=1
uint8 STATUS_ALIGNMENT_GATED=2
uint8 STATUS_COMMAND_TIMEOUT=3
uint8 STATUS_FEEDBACK_FAULT=4
uint8 STATUS_INVALID_COMMAND=5
uint8 STATUS_STEERING_LIMIT=6
uint8 STATUS_STEERING_HOLD_FAULT=7

# Controller update time of the cycle that produced this sample.
builtin_interfaces/Time stamp
uint8 status
# False when the relative-move runtime owns the outputs; the profile fields then describe the
# navigation core only, not what was written to the drives.
bool core_owns_output

# Motor-derived steering angle read this cycle (rad).
float64[4] measured_steering_angle
# Final target of the current kinematic solution after the 180 deg branch choice (rad).
# Holding modules report their latched hold target. NaN before the first activation.
float64[4] goal_steering_angle
# S-curve profile target sent to the drive this cycle (rad), its rate (rad/s) and
# acceleration (rad/s^2).
float64[4] profile_steering_angle
float64[4] profile_steering_rate
float64[4] profile_steering_acceleration
# Commanded wheel-surface speed this cycle, including ramp-down to stop (m/s).
float64[4] commanded_wheel_speed
# Measured wheel-surface speed: drive velocity feedback times wheel radius (m/s).
float64[4] measured_wheel_speed
# True: the module points 180 deg from the requested direction and drives its wheel reversed.
bool[4] reversed
# True: the module holds its latched steering target (no requested wheel speed).
bool[4] holding
