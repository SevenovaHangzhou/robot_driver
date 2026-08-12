std_msgs/Header header
bool connected
bool data_fresh
bool pump_enabled
VacuumChannelState[] channels
# 诊断信息。周期状态 topic 不承载重试判据，本字段只用于日志。
string error
