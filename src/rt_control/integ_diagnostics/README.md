# integ_diagnostics

RT-Control 诊断节点（设计稿：`domains/rt_control/docs/operator-web-and-diagnostics-design.md` §5）。

- 输入：制品中的 `diagnostics.yaml`（设备清单与厂家故障码表，组合器生成）、启动前自检报告
  `$RT_CONTROL_STATE_DIR/self_check/<作用域>.json`（阶段 A，运行栈启动文件写入）、内部状态广播
  `/rt_internal_state_broadcaster/dynamic_joint_states`。
- 阶段 B：硬件激活后收到第一份完整状态时检查一次（从站 OP、主站链路与从站数、驱动 0x603F 故障码
  与 CiA402 故障位），结果追加写回自检报告。
- 运行中：只在设备自己报出问题时更新，不重复自检。
- 输出：R-OUT-09 `/rt_control/readiness`（`DomainReadiness`，`Q_LATCHED`，变化即发、1 Hz）。
  `errors[]` 每项 `code` 为 DREE、`source` 为设备位置、`detail` 为厂家原始码与说明。
- 不下发任何命令，不参与使能准入（准入只由 `enable_manager` 决定），不替代急停/STO/安全链。
- DREE 数值取自锁定的 `robot_interfaces` `ErrorCode`；缺少所需常量时节点拒绝启动。

由 `integ_bringup` 的作用域运行栈在制品含 `diagnostics.yaml` 时启动。
