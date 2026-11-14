---
id: integration-20261114-01
area: integration
title: 状态页底盘图改读运行模型；Robot Model 98d498e 的消费方核对
date: 2026-11-14
type: feature
trigger: ELECTRI-200（状态页）；robot_driver #263 把 Robot Model 升到 robot_description main 98d498e
commits: [feature/status-ultrasonic-frames]
env: software-only
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: [integration-20261114-model-sync]
---

## 背景

状态页原先把八个超声波探头的坐标写死在页面里，那是按用户给的对照表画的占位布局。模型同步到
`robot_description main@98d498e` 后，URDF 里已经有 `ultrasonic_channel_1..8_link`，页面应当以
运行中的模型为准，否则探头移位后图会骗人。

## 改动

- 新增 `integ_operator_web/status/model_geometry.py`：解析模型文本，取出
  `ultrasonic_channel_*` 固定关节的位姿、四个 `steer*_joint` 的轮位与底盘半长/半宽；
  模型缺失或没有这些坐标系时返回空，不是错误。
- `status/page.py` 把 `model_geometry` 放进推送快照；页面底盘图据此绘制：
  车体轮廓由轮距推出，探头按模型坐标放置，扇区朝向由关节 yaw 决定（REP-103，+X 前、+Y 左）。
- 模型里没有探头坐标系时仍用原占位布局，并在“最近障碍”一栏只显示通道号，不编造方位。

## 消费方核对（Robot Model 98d498e）

| 项 | 结论 |
| --- | --- |
| 静态 TF 重复发布 | 仓库内没有 `StaticTransformBroadcaster` 或 `static_transform_publisher` 发布 `lidar_main`/`lidar_rear`/`*_ft_sensor_link`/`ultrasonic_channel_*`，这些坐标系只由模型提供，无重复发布方 |
| 超声波 `Range` 的 frame_id | `periph_led_ultrasonic` 一直发布 `ultrasonic_channel_N_link`，此前模型里没有这些 frame；本次之后才真正可解析 |
| Home 预设与 J6 新限位（±110°） | `config/poses/home.yaml` 的 J6 为 ∓0.698 rad（40°），`sequence-presets.json` 中没有超过 ±1.92 rad 的 J6 目标，不受收紧影响 |
| operator web 三维视图 | 模型改为 canonical/visual/collision 三份产物，页面仍读运行产物里的 `robot_description.xml` 与安装的 mesh 目录；网格引用 80→86，加载量增加，需要在台架上确认加载时间与显存占用 |
| 导航/感知 | `base_link` 抬高 15 mm 且去掉了 90° yaw，属于跨域影响，本仓库只记录，不在本次判断 |

## 验证

- `pytest src/rt_control/integ_operator_web/test`：265 passed（新增 4 项：模型几何解析、
  缺坐标系退回占位、模型不可读、快照携带几何）。
- `tools/quality_gate.sh`、`tools/arch_check.py`：PASS。
- 本地桩服务 + 浏览器截图：1920×1080 与 2560×1440 下底盘图使用模型坐标，八个扇区方位与模型一致。

## 未验证

- 未上台架：真实 `/rt/ultrasonic/channelN/range` 与模型坐标的对应关系（尤其通道号与物理方位）
  仍需现场按线序核对。
- 三维视图在新模型下的加载时间、显存占用未实测。
- `base_link`/`tool0` 的坐标系变更对导航与轨迹的影响不在本记录范围。
