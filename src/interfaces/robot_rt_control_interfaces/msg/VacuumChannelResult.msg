uint8 UNVERIFIED=0
uint8 ATTACHED_VERIFIED=1

string channel
bool command_accepted
bool valve_actuation_completed
uint8 verification_level
# 本通道诊断信息。逐通道错误不参与重试判定 —— 重试看 Action Result
# 的 ErrorInfo.retryable，本字段只用于日志。
string error
