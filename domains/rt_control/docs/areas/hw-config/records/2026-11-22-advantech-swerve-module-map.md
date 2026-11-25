---
id: hw-config-20261122-05
area: hw-config
title: 研华舵轮外置编码器与轮组对应、轮组坐标按实测与 Robot Model 更正
date: 2026-11-22
type: commissioning
trigger: 整机转向时电机推算航向与外置编码器差 1–4° 并随转向翻转，2°/4° 门限仍触发 feedback_fault；逐轮手转读回发现编码器对应错误（ELECTRI-233）
commits: [fix/swerve-module-map]
env: native
risk: T2
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: [hw-config-20261122-04]
related: [hw-config-20261122-04, hw-config-20261116-02, control-20261122-03]
---

## 背景

失能时用户逐个手转四个轮子的转向（左前、右前、左后、右后），同时记录每台 EtherCAT 转向驱动的推算航向与每个
CANopen 外置编码器（`scripts/steer_recorder.py`，研华 Robot_sev `runtime/evidence/steer-20261122T070134Z.csv`）：

| 物理轮 | 变化的转向驱动 | 变化的外置编码器 |
| --- | --- | --- |
| 左前 | 环位 3（module_1，steer1_joint） | 节点 14 |
| 右前 | 环位 5（module_2，steer2_joint） | 节点 12 |
| 左后 | 环位 7（module_3，steer3_joint） | 节点 11 |
| 右后 | 环位 9（module_4，steer4_joint） | 节点 13 |

每次只有一对电机/编码器同步变化，差小于 0.3°。EtherCAT 环位与 Robot Model `steer1..4_joint` 原点
（左前 (+0.305,+0.305)、右前 (+0.305,−0.305)、左后 (−0.305,+0.305)、右后 (−0.305,−0.305)，+X 为升降/双臂侧，
+Y 为左）一致；外置编码器节点与模块对应不一致（原配置 module_k = 节点 10+k）。用户 2026-11-22 确认前方为双臂侧，
q=0 时轮子朝前滚。

原 `module_xy` 为 2026-11-16 底盘单机的控制坐标（+X 为旧 base_link −Y，且四个取负），与 Robot Model 相差 90°。

## 改动

- `machines/Alfa_V3/swerve_encoders.yaml`：`steering_encoder_1..4`（module_1..4）节点 11/12/13/14 → 14/12/11/13。
- `units/Alfa_V3-Advantech-01/swerve_calibration.yaml`：
  - `module_x_m/module_y_m` 改为 Robot Model 原点（上表）；
  - `external_encoder_installation_offset_rad` 跟随物理编码器节点（方向均为 −1 未变）；
  - `steering_zero_counts` 按各模块自己的编码器重算：38164/78291/−28551/101892 → 88244/77701/41338/−19355。
    失能静止 500 样本：module_1 航向 −29.707° / 节点 14 −31.672°；module_2 −9.740° / 节点 12 −9.717°；
    module_3 −7.695° / 节点 11 −10.437°；module_4 −13.833° / 节点 13 −9.076°。新零点 = 旧零点 + 计数/rad·(编码器 − 航向)。
- 电机/编码器差门限保持 1°（control-20261122-03 的 2°/4° 不合并）。
- 部署后架空低速：原地旋转正确（菱形），前进时左前、右前行走轮反向（用户 2026-11-22）。`swerve_chassis.yaml`
  wheel1/wheel2 `direction` +1 → −1；驱动器 `607E` 未改（停栈后四台行走驱动停在 OP 且 AL 错误，SDO 被拒）。

## 验证

- T2：上述手转读回与换算。
- 未验证：新配置部署后失能读回各模块差值、使能后原地旋转与平移方向（架空时旋转应为左前/右后 −45°、右前/左后 +45°）。

## 结论与冻结事实

- F1: 研华 module_1..4（左前、右前、左后、右后）的外置编码器为节点 14/12/11/13。
- F2: 研华底盘控制坐标与 Robot Model base 一致：+X 双臂侧、+Y 左；轮组坐标为 steer1..4_joint 原点。
- F3: 研华 2026-11-22 转向零点为 88244/77701/41338/−19355。
- F4: 研华前轮（wheel1/wheel2）软件 `direction=−1`，后轮 +1；hw-config-20261116-01#F2 只对后轮仍成立。

## 遗留

- 10 月记录的“节点 11..14 对应 module_1..4”与实测不符的原因（接线或模块编号变化）未查明。
- 前进方向与里程计符号需实机复核；行走轮极性是否回到驱动器 `607E` 待定（需行走驱动可回 PREOP）。
- 停栈后行走驱动停在 OP（AL 错误）的原因未查。
