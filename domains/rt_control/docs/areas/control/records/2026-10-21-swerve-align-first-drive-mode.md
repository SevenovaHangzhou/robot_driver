---
id: chassis-control-20261021-01
area: chassis-control
title: 舵轮控制器新增 drive_mode，默认先对齐后行走
date: 2026-10-21
type: feature
trigger: ELECTRI-127；用户 2026-10-21 口头需求（同事反馈边转边走影响较大）
commits: [feature/electri-127-swerve-align-first]
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [chassis-control-20261018-02]
---

## 背景

2026-10-18 起，控制器使用 90° 对齐门限：舵轮转向期间行走轮按 `cos(角度误差)` 降速继续驱动。
现场反馈起步和换向时轨迹偏离较大。用户要求默认改为先对齐后行走，并保留原方式作为启动参数。

## 改动

- `swerve_driver` 新增启动参数，在 configure 时读取，运行中不切换：
  - `drive_mode`：`align_first`（默认）或 `steer_while_driving`。
  - `align_first_threshold`：0.0873 rad（5°）。任一模块舵角误差超过该值，四个行走轮一起按
    加速度上限减速到 0。
  - `align_first_release`：0.0524 rad（3°）。停驱后，四个模块误差都不大于该值时才恢复驱动；
    5°/3° 回差避免在门限附近反复启停。
- 判断在起步和行驶中都生效；未进入门限时仍按 `cos(误差)` 降速。
- `steer_while_driving` 保持原行为：`alignment_threshold`（1.5708 rad），无回差。
- 两种模式的门限都可现场调参，下次使能生效：`align_first_threshold`、`align_first_release`
  加入控制器参数校验（(0, π/2]，释放角不大于进入角）、`scope_operator_tuning` 白名单和网页调参文件。
- 网页显示当前模式：`align_first` 显示“对齐模式”（对齐门限 + 恢复门限），
  `steer_while_driving` 显示“连续模式”（对齐门限）。模式本身只显示，不能在网页切换。
- `SwerveSetpointParameters` 新增末尾字段 `alignment_release_rad`，NaN 表示无回差（原行为）。
- `config/controller.draft.yaml` 写入上述默认值。

## 验证

已验证（T1，本地 Native）：

- `colcon build --packages-up-to swerve_driver`：通过。
- `colcon test --packages-select swerve_driver`：171 tests，0 failures。新增
  `AlignFirstGateHoldsAllWheelsUntilRelease`：4° 驱动、6° 四轮停驱、4° 保持停驱、2° 恢复、
  释放角大于进入角时拒绝。
- `tools/quality_gate.sh`：PASS；`rt_control_operator_web` 125 passed；`rt_control_bringup -k tuning` 8 passed。

已验证（T2，研华，底盘未使能）：

- 启动日志 `drive_mode align_first: gate 5.0 deg, release 3.0 deg`。
- 网页改为 6°/4°：控制器参数与调参文件一致，`alignment_threshold` 保持 90°；恢复默认后参数回到
  5°/3°，调参文件删除。

未验证：

- 研华实机：起步对齐等待时间、行驶中大幅转向的停顿、弧线行驶是否在门限附近启停。
- 导航连续指令下的整车表现。

本记录不授权使能或运动。
