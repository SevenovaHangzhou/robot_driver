---
id: lifecycle-20260919-02
area: lifecycle
title: V3 rolling 激活握手与 provisional epoch 纠错
date: 2026-09-19
type: corrective
trigger: ELECTRI-102；二代 robot-ipc 实机验证暴露切换回调等待状态主题和 provisional epoch 重置失败
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [lifecycle-20260919-01, motion-20260919-01]
---

## 问题与修复

V3 `set_mode` 原先在 STRICT 切换和 controller_manager 事后确认成功后，仍在服务回调内
同步等待 rolling 状态主题及 boot ID。真实执行器时序可能让该状态回调无法在 500 ms 内
运行，于是误报 `RESTART_REQUIRED`。现在服务在控制器状态确认后返回；当新状态已到达时
仍附带 boot ID，否则返回零 ID。Motion 必须在本次切换后另等新鲜 `ROLLING_READY` 状态
与非零 boot ID，才允许 `open`；没有状态时不可发轨迹。STRICT 切换失败、writer 重叠、
生命周期抢占等原有失能收敛条件不变。

另一处失败发生在 provisional 包络的 `open` 之后：RT epoch reset 将该包络错当成
test-only 校验，首个 RT 更新返回 ERROR。现在按实际包络来源传入对应授权位，不修改
任一轴的限位、周期或安全阈值。

## 验证与边界

- 两条回归在修改前分别复现 `RESTART_REQUIRED(19)` 和 RT update ERROR，修改后通过。
- V3 `enable_manager` 与 `rolling_trajectory_controller` 原生增量构建和完整 CTest：
  192 tests，0 errors，0 failures；`tools/quality_gate.sh`：292 passed、13 skipped、
  门禁覆盖率 83%；V3 分支契约、仓库结构门禁及受影响 C++ 格式检查通过。
- 二代 robot-ipc 已完成公共使能、切 rolling、10 度往返、94 个 batch 接收、关闭会话及
  切回 whole_body；该记录是修复来源，不是 V3 硬件验收。
- V3 双七轴包络仍为 mock-only `ESTIMATED_NOT_MEASURED`；未对 V3 实机执行使能、
  运动或目标机部署。Motion 消费者须按更新后的两阶段握手实现并共同验收。
