---
id: chassis-control-20261029-02
area: chassis-control
title: 作业模式连续动作换向误报修复及三段实机验证
date: 2026-10-29
type: fix
trigger: 用户要求修复作业模式前后位移切换左右位移或旋转时报错，并依次执行前进0.3m、左移0.3m、逆时针10度
commits: []
env: native
risk: T4
writes: { reset: yes, enable: yes, motion: yes, plc: no }
verified: PARTIAL
evidence:
  - runs/operation-direction-20261029/results.json
  - runs/operation-direction-20261029/operator-evidence.jsonl
  - runs/operation-direction-20261029/tests.log
  - runs/operation-direction-20261029/quality.log
  - runs/operation-direction-20261029/deploy.log
supersedes: []
related: [integration-20261029-08, chassis-control-20261029-99]
---

## 背景

本任务针对同一作业模式会话内部的动作切换，不是要求在每段动作之间切换导航/作业模式。
验收条件：每段先停车及转向对齐，再行走；三段均收到成功结果、故障码为0；任一失败停止后续请求；完成后失能。
范围为 swerve_driver 和 Kinco cyclic adapter，直接消费者是现有作业 Action 和操作网页；不改公共接口、模型、标定或安全阈值，不提交或推送。

## 改动

- 作业模式本来已有 aligning 阶段，但 SwerveController 仍调用导航 ControlCore 的命令超时/保持路径。其 holding_modules 继续对比导航旧目标，作业换向超过保持偏差后误触发 steering_hold_fault，最终被 enable_manager 失能。
- 新增 ControlCore::observe_operation，作业持有控制权时只运行反馈、编码器、限位和里程计观察，转向跟踪仍由 RelativeMoveSession 对其活动斜坡检查。导航保持状态跟随最后实际写出的作业参考，不用实时测量持续重置目标，返回导航也不会恢复旧速度。
- Kinco 在失能状态撤销旧的位置及可切换模式命令，与既有 CSV 速度命令生命周期一致。保留启用后第一条位置命令的接管容差检查，避免旧作业目标在控制器重新激活前导致 unexpected_drive_state。固定启动模式不变。
- 沿用网页已有“确认并执行位移”按钮和“转向对齐/位移执行/位置保持”阶段显示。
- 新增连续三个动作、转向对齐期间轮位保持、作业观察安全检查、失能后 CSP 接管及模式恢复的回归测试。参数及阈值无变化。

## 验证

修复前新增回归分别复现导航旧保持误锁存、失能后旧 CSP 命令导致输出控制字0x0007。随后修复通过：

- 独立目录 /tmp/v3-operation-direction 构建 swerve_driver、robot_hw_ethercat，避免改写运行中库；colcon test --merge-install --return-code-on-test-failure 与 colcon test-result：349 tests，0 errors，0 failures，0 skipped。包含 Mock 进程和 controller_manager 生命周期验证；测试使用独立 ROS domain。
- PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 tools/quality_gate.sh：PASS，247 passed、13 skipped。
- 通过 bash -o pipefail v3-restart.sh swerve_driver robot_hw_ethercat 有序停机、构建和重启；停机日志确认 motion_inactive、drives_disabled、controllers_inactive、hardware cleaned；新启动确认硬件活动但未使能。保留库备份于 /tmp/v3-operation-direction/rollback。
- 实机通过现有网页 WebSocket API 取得控制权，使能后仅切换一次到作业模式。以平移0.02m/s、加速度0.05m/s²、旋转0.05rad/s、加速度0.1rad/s²、轮速0.5rad/s、轮加速度1rad/s²、每段45s上限依次提交三个目标，中间没有切回导航模式。

| 目标 | 实测dx (m) | 实测dy (m) | IMU相对角度 (°) | Action结果/故障码 |
| --- | --- | --- | --- | --- |
| 前进0.3m | 0.30000834 | 0.00039721 | 0.03448 | 成功 / 0 |
| 左移0.3m | -0.00835440 | 0.29989313 | 2.32650 | 成功 / 0 |
| 逆时针10° | -0.00005856 | -0.00004889 | 9.65365 | 成功 / 0 |

三个目标进度均为1，estimate_valid=true；每次收到成功并重新就绪后再提交下一目标。
最终 /rt/disable 返回成功，网页回读 enabled=false、busy=false、lease=none。以上是编码器/IMU估计，不是外部测量的定位精度验收。

本次实机操作经现场授权，操作人：本会话用户（确认现场安全）；执行入口：既有操作网页 WebSocket API，确认短语：“确认”，目标动作由用户本轮再次明确指定。

## 结论与冻结事实

- F1: 作业模式内部连续换向保留停轮、转向斜坡、反馈对齐及静止确认，导航旧保持目标不再否决作业参考；安全反馈和作业跟踪检查继续生效。
- F2: 正常失能结束速度、位置、可切换模式命令生产者生命周期；重新使能仍须新控制器接管，不能重播旧命令。
- F3: 本机三段作业连续执行及最终失能通过；不等于整机生产验收或高精度定位通过。

## 遗留

左移时IMU检测到约2.33°偏航、纵向约8.35mm偏差；10°目标实测约9.65°。现有协议将位姿残差作为质量信息而非Action执行故障，本次没有改变该语义或通过放宽阈值掩盖残差。机械/标定/地面条件和定位精度需单独验收；未执行长时间耐久、整机或Docker发布验收。所有既有用户工作树改动保留，未提交或推送。
