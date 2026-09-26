# ELECTRI-133 Tier A; snapshot shared by action feedback and result.
# Quality flags are a bitmask, independent of encoder execution success.
uint32 QUALITY_NONE=0
uint32 QUALITY_POSE_RESIDUAL=1
uint32 QUALITY_YAW_DISCREPANCY=2
uint32 QUALITY_ESTIMATE_INVALID=4

builtin_interfaces/Time stamp
# ROS time captured with encoder/IMU origins, before alignment begins.
# Identifies the frozen execution-start base frame; no TF frame is published.
builtin_interfaces/Time execution_start
# ChassisState.PHASE_* and ChassisState.FAULT_* respectively.
uint8 phase
uint16 fault_code
# Commanded common scalar progress, [0,1]; cancellation can finish below 1.
float64 progress
# Estimated displacement of base origin, in the frozen execution-start frame (m).
float64 actual_dx
float64 actual_dy
# Primary estimated relative yaw = imu_yaw when estimate_valid (rad).
float64 actual_dyaw
# Continuous relative yaw from the same execution-start snapshot (rad).
float64 imu_yaw
float64 wheel_yaw
# Signed imu_yaw - wheel_yaw (rad), without wrapping at +/-pi.
float64 yaw_difference
# False after IMU/required odometry feedback loss; never repaired within this goal.
bool estimate_valid
uint32 quality_flags
