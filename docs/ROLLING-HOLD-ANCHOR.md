# Rolling 起步锚点和首段校验（2026-11-07）

Mink → Rolling 在每次新移动开始前，通过当前控制器的 `query_state` 只读服务获取实际保持的指令位置。主操作台仍负责双臂共享控制权和自动选模；18131 使用返回的 `hold_reference.positions` 初始化规划，实际反馈位置仍用于碰撞检查。首次连接、重新连接、正常完成和超时停车后的新起步不再依赖编码器位置来猜保持指令。

查询使用标准 `control_msgs/srv/QueryTrajectoryState`，仅 time=0 且已激活、静止、反馈有效时成功；非实时回调复用已有一致快照读取，实时更新路径不变。未激活或无法确认时返回失败，不退回实际位置发首窗。主操作台在查询前后及首窗发布前核对 HOLDING 的 epoch、序列、结果、停因和新鲜度；首点必须与指令保持值完全一致且速度为零，并校验首段 Hermite 加速度。规划器原有整窗位置、速度、加速度及碰撞检查继续生效。限制、跟随容差、256 点容量及运行时窗口参数未放宽。

验证：操作台 362 项通过、1 项扩展绑定测试跳过；键盘与规划相关 96 项通过；Rolling C++ 8 组、58 项通过，包含正常完成、超时停车、首批拒绝后的保持值及实时路径不分配内存。远端独立 Release 编译成功。实机只读确认新服务已加载、未激活时正确拒绝查询；尚未自动使能或进行 Rolling 运动复测。

远端沿用 `/home/ar/rt-control-arch-20261106/artifacts/bench_dual_arm_head_wrist_force`，ROS Domain 77；artifact 文件校验和保持一致。恢复前确认旧控制栈已在另一处完成失能和硬件清理；随后加载补丁并恢复原启动参数。当前 RT PID 758013，18121 PID 759972，18131 PID 758071。初次启动的检查早于 EtherCAT 初始化完成，后续已确认完整反馈、控制器和页面状态，无需再次重启控制栈。

同时清理了旧 `rt-control-electri127` 左臂实例遗留的能力发布进程 PID 1478031。它在同一 `/rt/capabilities` 主题广播 `bench_left_arm`，使新网页抢先绑定旧 action；清理后仅重启 18121，已正确绑定双臂模型。18121 和 18131 双向互斥获取/释放检查通过，未发布轨迹、未切运动模式、未使能驱动。交付时状态 IDLE、反馈有效、控制权空闲，两个轨迹控制器均已加载且 inactive。

远端回退备份：`/home/ar/gripper-jog/releases/shared-authority-20261107/rolling-hold-anchor-backup-20261107-221608.tar.gz`。本地改动前备份：`.runtime/rolling-hold-anchor-backup/`；部署哈希和验证证据：`.runtime/rolling-hold-anchor-deployment.json`、`.runtime/rolling-hold-anchor-health.json`、`.runtime/rolling-hold-anchor-web-check.json`。集成补丁和源码引用已同步，未提交或推送。刷新页面、重新获取控制权并使能后，由操作员选择最低速度复测重复起步。
