# RT-Control domain-internal handoff from enable_manager to the rolling
# controller. External domains consume SetJointControlMode.Response and
# RollingJointControlState, never this topic or type.

uint16 PROTOCOL_MAJOR=1
uint16 PROTOCOL_MINOR=0

uint16 protocol_major
uint16 protocol_minor
uint64 sequence
unique_identifier_msgs/UUID request_id
robot_rt_control_interfaces/RollingServiceResult result
robot_rt_control_interfaces/JointControlMode mode
unique_identifier_msgs/UUID controller_boot_id
bool source_controller_deactivated
bool target_controller_activated
bool restart_required
