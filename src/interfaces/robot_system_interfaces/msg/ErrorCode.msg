# 错误码分段常量。ErrorInfo.code 的取值来源，集中定义、单一事实源。
#
# 受众：Autonomy 域（派发决策）与现场运维（排障）。
# 本文件只承载常量，不作为消息发布；字段 unused 仅为满足 IDL 语法。
#
# ============ 编码规则：D R E E ============
#
#   D  第 1 位  域归属      0=全域通用 1=RT-Control 2=Perception 3=Motion 4=Autonomy
#   R  第 2 位  可自主恢复  0=不可机器自主恢复（需人工或换流程） 1=可机器自主恢复
#   EE 第 3~4 位 事件分类   跨域含义一致，见下表
#
# SUCCESS=0 是唯一例外，不参与编码。
#
# 事件分类（EE，各域含义相同，运维只需学一次）：
#
#   00~09  通信与接口误用
#   10~19  配置与版本
#   20~29  硬件与设备
#   30~39  环境与外部条件
#   40~49  数据质量
#   50~59  计算与求解
#   60~69  执行与运动
#   70~79  安全与联锁
#   80~89  任务与流程
#   90~99  内部错误
#
# 排障读法：先看第 1 位找责任部门，再看第 2 位决定是否等机器自己恢复，
# 再看后两位定位排查方向。
#
# ============ 使用约束 ============
#
# 第 2 位必须与 ErrorInfo.retryable 一致，由 CI 校验。
# Autonomy 的重试判定**只读 retryable**，不解析 code —— 第 2 位是给人看的冗余镜像。
# 各域在自己 D 段内自行扩充，非破坏性；改变某个码的第 2 位即改变其数值，
# 属于破坏性变更，需契约评审。

# ---------- SUCCESS ----------
uint32 SUCCESS=0

# ---------- 0 全域通用 · 不可自主恢复 ----------
# 无确定终态，禁止重发（契约 6.4）。
uint32 COMMUNICATION_UNKNOWN=1
uint32 TIMEOUT=2
# 前置条件不满足被拒；原样重发不会改变结果。
uint32 GOAL_REJECTED=3
uint32 INVALID_GOAL=4
# 契约 schema、模型或标定版本不一致。
uint32 VERSION_MISMATCH=10
# SafetyState 拒绝，或硬安全链已触发；需人工复位。
uint32 SAFETY_DENIED=70
uint32 CANCELED=80
uint32 INTERNAL_ERROR=90
uint32 UNKNOWN=91

# ---------- 0 全域通用 · 可自主恢复 ----------
# 目标域尚未就绪，等待 readiness 恢复即可。
uint32 NOT_READY=130

# ---------- 1 RT-Control · 不可自主恢复（1000~1099） ----------
# 轨迹不含完整 14 轴，或点内项数不符（契约 6.2）。上游生成有误。
uint32 RT_JOINT_SET_MISMATCH=1004
# 退避表/关节顺序与本域配置不一致。
uint32 RT_CONFIG_MISMATCH=1010
# 驱动器报故障，需人工检查后复位。
uint32 RT_DRIVE_FAULT=1020
uint32 RT_ENABLE_FAILED=1021
# 泵不可用（未上电、维护锁定或已强制停泵）。
uint32 RT_PUMP_UNAVAILABLE=1022
uint32 RT_TOLERANCE_VIOLATED=1060
uint32 RT_INTERNAL_ERROR=1090

# ---------- 1 RT-Control · 可自主恢复（1100~1199） ----------
# 总线瞬时掉线，等重连即可。持续掉线由域内升级为 RT_DRIVE_FAULT。
uint32 RT_FIELDBUS_OFFLINE=1120
# /cmd_vel_safe 超 500 ms 未更新，已本地停车；上游恢复发布即恢复。
uint32 RT_CMD_VEL_STALE=1130
# 任一目标通道的新鲜 attached 状态未建立；物理动作重试仍受契约 6.5 约束。
uint32 RT_VACUUM_NOT_ESTABLISHED=1160

# ---------- 2 Perception · 不可自主恢复（2000~2099） ----------
# 退避表与 wall_row 不匹配（契约 2.1）。感知侧配置过期。
uint32 PERC_STANDOFF_TABLE_MISMATCH=2010
uint32 PERC_CAMERA_FAULT=2020
# 未检出完整 25 箱或 15 序列；箱墙实况与预期不符，需人工确认。
uint32 PERC_INCOMPLETE_WALL=2040
uint32 PERC_INTERNAL_ERROR=2090

# ---------- 2 Perception · 可自主恢复（2100~2199） ----------
# TF 尚未就绪或暂时缺边。
uint32 PERC_TF_UNAVAILABLE=2100
# 采集期间底盘未静止，停稳后重采即可。
uint32 PERC_BASE_NOT_STATIONARY=2130
uint32 PERC_CAPTURE_FAILED=2140
# 目标箱本次未检出，可重新精定位。
uint32 PERC_BOX_NOT_FOUND=2141
# 位姿置信度不足。
uint32 PERC_POSE_UNRELIABLE=2142
# 定位丢失，等重定位收敛。
uint32 PERC_LOCALIZATION_LOST=2150

# ---------- 3 Motion · 不可自主恢复（3000~3099） ----------
# 目标超出可达域；指向标定或退避表配置，不是规划器问题。
uint32 MOTION_TARGET_UNREACHABLE=3010
# 手臂未在行走包络内，或存在活动手臂 goal（契约 6.7）。
uint32 MOTION_ARM_NOT_STOWED=3070
# 底盘未静止，或存在活动导航 goal（契约 6.7）。
uint32 MOTION_BASE_IN_MOTION=3071
uint32 MOTION_EXECUTION_FAILED=3060
uint32 MOTION_INTERNAL_ERROR=3090

# ---------- 3 Motion · 可自主恢复（3100~3199） ----------
# 取箱位姿超过 max_pose_age，重新精定位后可再试。
uint32 MOTION_POSE_STALE=3140
# 规划失败（含 IK 无解）；换目标或重规划可能成功。
uint32 MOTION_PLANNING_FAILED=3150
# 碰撞检出，需重规划。
uint32 MOTION_COLLISION_DETECTED=3151
# Nav2 本次未到达，属 N-01 三次预算内可重试。
uint32 MOTION_NAV_FAILED=3160

# ---------- 4 Autonomy · 不可自主恢复（4000~4099） ----------
# 已有活动 G-01。
uint32 AUTO_TASK_ALREADY_ACTIVE=4000
# 计划不可用或已失效。
uint32 AUTO_PLAN_UNAVAILABLE=4040
# 已用尽三次尝试预算（契约 6.4）。
uint32 AUTO_RETRY_BUDGET_EXHAUSTED=4080
uint32 AUTO_SEQUENCE_ABORTED=4081
uint32 AUTO_INTERNAL_ERROR=4090

# ---------- 4 Autonomy · 可自主恢复（4100~4199） ----------
# 某域 readiness 为 UNAVAILABLE，等其恢复。
uint32 AUTO_DOMAIN_UNAVAILABLE=4130

uint8 unused
