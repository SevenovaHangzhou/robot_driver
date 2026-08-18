---
id: lifecycle-20260819-04
area: lifecycle
title: ELECTRI-102 FJT 与 rolling 严格模式切换
date: 2026-08-19
type: feature
trigger: ELECTRI-102 / T-17 / E102-D22 / E102-D23
commits: [功能/视觉伺服-ELECTRI-102]
env: native-mock
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [src/rt_control/enable_manager/src/enable_manager_controller.cpp, src/rt_control/enable_manager/test/test_enable_manager_state_machine.cpp, src/rt_control/rt_control_bringup/config/controllers.yaml]
supersedes: []
related: [ELECTRI-102, lifecycle-20260819-03]
---

## 背景

ELECTRI-102 要求同一时刻只有一个 14 轴 command writer，并允许 Motion 在已取消且等待
FJT Action 终态后，将控制权在 `whole_body_jtc` 与 `rolling_trajectory_controller` 之间切换。
切换不是普通的 controller-manager 透传：在调用 STRICT switch 前还必须证明源控制器状态新鲜、
14 轴实际位置稳定、源期望与实际位置足够接近，并对超时、部分切换和安全抢占给出确定收敛。

## 改动

- `enable_manager` 实现公共 `/rt/joint_control/set_mode` 服务，严格匹配协议 1.0，采用
  `expected_mode` compare-and-set，并以 `client_instance_id + request_id` 保存 8 项定长幂等缓存。
- JTC 源证据取 `/whole_body_jtc/controller_state.output.positions`，rolling 源证据取公共状态中的
  `desired_positions`；两者都必须小于 100 ms。rolling 有活动 session 时禁止离开 rolling。
- 从 250 Hz 状态接口连续取得 5 个稳定区间，单次采样间隔不得超过 8 ms；13 个旋转轴稳定
  速度阈值为 `0.00872664626 rad/s`，升降轴为 `0.001 m/s`。接管位置容差分别为
  `0.00872664626 rad` 与 `0.005 m`。EtherCAT 完整过程数据年龄上限保持 500 ms。
- controller-manager 切换固定 `STRICT + activate_asap`，超时 500 ms；切换后再次查询两个
  controller，逐一回报 source INACTIVE、target ACTIVE 证据。FJT→rolling 还要求收到切换开始后
  的非零 `controller_boot_id`，才返回成功。
- 普通拒绝且源 writer 仍 ACTIVE 时保持源模式；两个 writer 均 INACTIVE 时进入普通失能；
  双 ACTIVE、成员缺失、超时、响应与实际状态矛盾等歧义锁存 `RESTART_REQUIRED` 并触发失能，
  不自动重试或声称事务回滚。
- 模式切换和已有 enable/disable/emergency controller-manager 操作共享互斥量。软件 disable
  请求可中止尚未进入 switch 的模式准入；group fault 立即接管 RT 安全状态机，非 RT 的
  controller deactivate 等当前 switch 退出后再执行，模式回调不得覆盖安全 owner。
- 所有门槛显式落在 `controllers.yaml`；JTC 状态发布率显式为 50 Hz。未修改 JTC 轨迹执行、
  CiA402 mode/PDO、joint-state broadcaster 配置或任何硬件限值。

## 验证

- 独立 Humble overlay 构建 `rt_control_interfaces` 与 `enable_manager`：PASS，无编译告警。
- `ctest --test-dir /tmp/e102-enable-build/build/enable_manager --output-on-failure`：PASS。
- enable-manager 套件共 46 个 gtest，覆盖模式识别、非事务结果分类、稳定/接管独立阈值、
  错误前置拒绝、幂等冲突、fake controller-manager 的真实 STRICT 成功路径、boot 证据、
  group-fault 抢占和 emergency/switch 互斥；46/46 通过。
- `ament_uncrustify` 对修改的头文件、实现和测试：PASS；`git diff --check`：PASS。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: FJT 与 rolling 任一成功模式切换都要求源状态新鲜、250 Hz 实际状态停稳、源期望与实际位置满足逐轴接管容差。
- F2: 模式切换只使用 controller-manager STRICT switch；响应和二次查询共同证明结果，任何歧义均进入 RESTART_REQUIRED。
- F3: disable/group fault 优先于模式请求；controller-manager 写操作由一个互斥量串行，RT 安全状态机不等待非 RT 切换完成。
- F4: 相同 client/request ID 的完全相同请求返回原结果；同 ID 不同 payload 返回 WRONG_REQUEST，不再次切换。

## 遗留

- `RollingJointControlState.last_mode_*` 的跨 controller 传播将在下一原子任务补齐；当前服务响应已完整返回切换结果和证据。
- 500 ms STRICT 切换超时与实际耗时分布仅在桌面 fake controller-manager 验证；目标工控机测量和节拍预算仍需无运动/受控现场验证。
- 本记录没有访问总线、reset、enable 或运动，不能解释为实机模式切换通过。
