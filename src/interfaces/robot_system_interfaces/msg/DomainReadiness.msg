# 域就绪心跳。Q_LATCHED；状态变化立即发布，稳定时 1 Hz。
#
# 消费方按本地 steady clock 接收年龄派生 HEALTHY / DEGRADED / UNAVAILABLE，
# 判定规则见契约 6.6。

std_msgs/Header header

# 本域是否可接受新业务派发。
bool ready

# 域内状态标识，仅用于展示与日志，不参与控制分支。
string state

# 未就绪原因，人可读。ready=true 时应为空。
string reason

# 本域软件版本，便于现场核对部署版本。
string version

# 关键配置摘要，便于现场核对配置是否与预期一致。
string config_summary
