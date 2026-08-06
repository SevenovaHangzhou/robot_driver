# R-OUT-06 /control/safety_state。软件可观测摘要，不代表硬安全链状态。

std_msgs/Header header
bool safe_to_start_motion
bool control_enabled
bool enable_manager_ok
bool ethercat_ok
bool canopen_ok
bool plc_ok
bool bms_ok
string state
string[] active_faults
