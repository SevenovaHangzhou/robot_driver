---
id: hw-config-20261125-01
area: hw-config
title: 研华轮组统一编号与模型零位前进方向
date: 2026-11-25
type: calibration
trigger: 用户确认转向环位3左前、5右前、7左后、9右后（ELECTRI-247）
commits: [fix/swerve-corner-order]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: [https://github.com/SevenovaHangzhou/robot_driver/issues/340, https://github.com/SevenovaHangzhou/robot_description/pull/27]
supersedes: [hw-config-20261116-02#F1]
related: [hw-config-20261122-04, hw-config-20261124-01]
---

## 背景与决定

用户确认本台研华整机在 `base_footprint` 下 +X 前、+Y 左，转向环位3/5/7/9对应左前/右前/左后/右后。旧标定的FR/RR/FL/RL方位被本次确认替代。保持模块编号与物理驱动器绑定，只修正方位与平面坐标：

| 模块 | 方位 | XY (m) | 转向/行走关节 | 转向/行走环位 | 编码器节点 |
|---|---|---|---|---|---|
| 1 | FL | +0.305,+0.305 | steer1 / wheel1 | 3 / 2 | 11 |
| 2 | FR | +0.305,-0.305 | steer2 / wheel2 | 5 / 4 | 12 |
| 3 | RL | -0.305,+0.305 | steer3 / wheel3 | 7 / 6 | 13 |
| 4 | RR | -0.305,-0.305 | steer4 / wheel4 | 9 / 8 | 14 |

关节全名带 `_joint` 后缀。转向零点38164/78291/-28551/101892、外置编码器方向和偏置、行走方向和原点、轮径全部沿用。不得因方位修正给电机零点另加90°。

## 模型与控制一致性

robot_description PR #27 将四个转向关节的固定安装偏航增加90°。零转角时四轮正轮速沿footprint +X滚动；控制和模型均满足航向=-转向关节角。转向轴原点、原后轮180°反装及轮轴符号保留。当前快照锁定该PR提交428351e，模型PR合入后再更新到对应main提交。

控制器仍使用相同关节、环位和编码器。模块坐标变化会改变旋转运动的逆运动学和里程计估计，不是仅改显示。逐轮观测广播器按坐标象限分配消息槽位，所以无需新增重排代码：FL/FR/RL/RR现在分别来自模块1/2/3/4。

## 验证

- 新几何断言在旧配置下失败，修正后底盘与整机real/mock组合测试通过。
- 15项底盘组合测试：绑定、环位、转向零点、CAN节点、模型安装点与控制XY一致。
- 广播器本地构建及5项C++测试通过；新测试对四个角落注入不同电机/编码器数据，验证槽位及正负号。
- 模型22项测试通过，含零转角、正负转角、±π/2的滚动方向；10项组合金样通过。
- 84个选择对照：前后均31个可生成，无新增失败；18个模型XML变化，4个底盘/整机controllers.yaml变化。研华real控制器差异仅swerve与观测的module_x/y及观测chassis_calibration_id。驱动运行YAML、CAN绑定和所有非XY单机标定值保持一致。
- 本地质量门通过。未执行现场启停、SDO写入、使能、运动或部署。

## 发布与消费者

先合模型PR #27，再将驱动快照锁更新为合并后的main SHA；两者作为同一次研华发布。停止运行栈由操作台完成，重新组合后核对运行XML及controllers.yaml；旧产物不能与新模型混用。记录当前产物以供整套回滚。

导航的观测槽位仍遵循公共FL/FR/RL/RR定义，来源模块随本次修正改变。chassis_calibration_id从86df8abe4f4b083a变为b4a2f7c60072b3f3，编码器部分96d372fd084b8828不变（均为当前组合结果）。导航应在切版时重建观测基线，不跨版本累计；动态TF、网页轮位与遥控显示按共同坐标系核对。

现场分阶段验证：首先禁能读取每轮与编码器对应及零位；经现场授权后低速前进/后退、横移、正负旋转，对照实体轮方向、轮观测槽位和里程计符号。出现不一致时停在当前阶段并核对标定，不借修正几何同时翻转电机方向。
