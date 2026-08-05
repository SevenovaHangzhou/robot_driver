# 跨域统一错误载荷。
#
# 重试判定：只有 N-01 / P-02 的 ControlledRetry 读取 retryable。
# code、message、origin 只用于诊断和日志，不得参与控制分支。

# 错误码。取值见 ErrorCode.msg 的集中常量定义（按域分段，SUCCESS=0）。
# 只用于诊断和日志，消费方不得按此分流。
uint32 code

# 调用方唯一被允许用于重试判定的字段。
bool retryable

# 人可读诊断信息。不得被解析用于控制分支。
string message

# 产生该错误的域名：rt_control / perception / motion / autonomy。
string origin
