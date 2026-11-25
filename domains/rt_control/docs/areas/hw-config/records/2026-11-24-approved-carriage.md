---
id: hw-config-20261124-01
area: hw-config
title: 研华整机同步正式四盒装配模型
date: 2026-11-24
type: sync
trigger: ELECTRI-246
commits: [feature/sync-approved-carriage]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: [https://github.com/SevenovaHangzhou/robot_driver/issues/337]
supersedes: []
related: [hw-config-20261122-03]
---

## 改动

模型源版本为 fa636d3，正式入口为 `urdf/robot_alfav3_mobile.urdf`。根固定变换 `base_footprint → base_link` 平移为 (0.07478297093, 0, 0.370) m，零旋转。升降轴模型行程更新为 [-1.033, +0.017] m。规划侧配套资源为 `config/curobo/fixed_body_boxes.yaml` 的四个盒体。

## 验证

136 个上游文件逐一核对 Git blob 哈希和执行位。模型测试 22 项、组合金样 10 项及 TF 合同 2 项通过。CMake 临时安装后正式 URDF 的 47 个网格引用均可解析。

研华 `advantech_full` 的 mock 和 real 配置均完成离线组合，产物差异限于 `robot_description.xml`。现场发布时核对运行时 XML、Motion 的模型与四盒、导航外参及状态页轮位。现场验证待执行。
