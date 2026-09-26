# ELECTRI-133 Tier A; RT-Control private only. See README.md.
# Mode is the confirmed backend mode, never merely a requested mode.
uint8 MODE_UNKNOWN=0
uint8 MODE_NAVIGATION=1
uint8 MODE_OPERATION=2
uint8 PHASE_IDLE=0
uint8 PHASE_SWITCHING=1
uint8 PHASE_ALIGNING=2
uint8 PHASE_EXECUTING=3
uint8 PHASE_STOPPING=4
uint8 PHASE_HOLDING=5
uint8 PHASE_FAULT=6
# First latched cause; remains until explicit, stationary, healthy software reset.
uint16 FAULT_NONE=0
uint16 FAULT_IMU_LOST=1
uint16 FAULT_FEEDBACK_INVALID=2
uint16 FAULT_SLIP=3
uint16 FAULT_STEERING_ERROR=4
uint16 FAULT_DRIVE=5
uint16 FAULT_BUS=6
uint16 FAULT_MODE_SWITCH=7
uint16 FAULT_TIMEOUT=8
uint16 FAULT_EXECUTION=9

builtin_interfaces/Time stamp
uint8 mode
uint8 phase
uint16 fault_code
# True only after backend mode readback, feedback/config and runtime gates pass.
bool ready
# Fresh drive/wheel AND steering velocities satisfy configured stationary dwell.
bool stationary
# Independent of goal activity; needed to explain admission refusal while idle.
bool imu_valid
bool goal_active
# Zero UUID when no active goal; ROS action goal UUID otherwise.
unique_identifier_msgs/UUID active_goal_id
