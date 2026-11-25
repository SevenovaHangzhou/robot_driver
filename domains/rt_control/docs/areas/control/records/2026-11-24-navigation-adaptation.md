---
id: chassis-control-20261124-01
area: chassis-control
title: 导航连续转舵、分场景限制、旋转授权与执行反馈
date: 2026-11-24
type: feature
trigger: 用户确认导航适配需求，模型以我方为准；GitHub robot_driver #333、#334、#335
commits: [feature/navigation-adaptation]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: [/tmp/navigation-adaptation-log, /tmp/navigation-adaptation-build/ctrl_swerve/test_results, /tmp/navigation-interfaces-log]
supersedes: []
related: [chassis-control-20261022-01, chassis-control-20261021-01]
---

## 背景

用户 2026-11-24 批准除 Robot Model 外按导航需求适配。模型、物理模块身份及标定以驱动权威配置为准，不套用导航仿真的统一转舵零位旋转。Linear 授权返回401；用户批准先建 GitHub Issue、后补互链，且接口 design Issue 暂放 robot_driver（robot_interfaces 已关闭 Issues）。

任务：[控制 #333](https://github.com/SevenovaHangzhou/robot_driver/issues/333)、[契约 #334](https://github.com/SevenovaHangzhou/robot_driver/issues/334)、[反馈 #335](https://github.com/SevenovaHangzhou/robot_driver/issues/335)。

## 改动

- 默认 `drive_mode` 从 `align_first` 改为 `steer_while_driving`；余弦补偿、90°舵角误差门控及保护保留。车体线速度上限2.0→1.0 m/s。
- 原联合 `max_body_acceleration` 改为独立 `max_linear_acceleration`（XY范数1.0 m/s²）与 `max_angular_acceleration`（π/8 rad/s²）；普通角速度0.8 rad/s。车体jerk保留4.0，对旋转使用等效半径。
- 新增旋转请求/真实授权状态。请求本地单调租约300ms；首次准入要求新鲜零命令、反馈正常、四轮<=0.01m/s、零输出保持0.05s。授权纯旋转1.5 rad/s、1.5 rad/s²。平移、撤销、超时、失效、故障及失活撤销；重激活不继承。高转速撤权后连续减速，到普通限值前不引入平移。
- 固定大小执行快照提供真实内部参考、最终目标、实际反馈、限制和几何；100Hz发布主/兼容91项状态及归一化JointState，50Hz发布授权状态。RT发布消息预分配，非RT定时器在失活时继续发布无效/未授权状态。
- `robot_interfaces` 独立工作树登记权威契约，91项为31个头字段与4×15轮组字段。用户不要求取得参考源码；不声称与未知副本索引兼容。执行状态轮速m/s，JointState轮速rad/s，模块采用物理接线顺序及中性名称；模型反馈不改。

## 验证

本机 ROS 2 Humble，隔离构建/安装目录；无真实设备访问：

- `colcon build/test` 仅 `ctrl_swerve`：构建成功，108项测试，0错误/0失败/0跳过。含核心约束/授权、ROS Topic租约超时/重激活、最终目标与参考分离、单位、失效反馈以及GenericSystem控制器加载释放测试；这是包级软件测试，不代表整机T1验收。
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q .../integ_composer/test/test_chassis.py .../test_advantech_full.py`：16 passed。禁用自动插件是为隔离宿主anyio与系统pytest版本冲突。
- 内联离线组合检查：chassis_only、advantech_full × mock/real 共4组生成配置，15项导航参数全部贯穿，旧参数不在生成结果中；real配置生成不等于实机启动。
- 接口工作树：生成视图检查、契约闭合、错误码、changelog门禁通过；67项工具测试通过；7个接口包全部构建成功。
- 首轮编译发现几何成员名及Humble不支持mutable定时lambda的错误，已修正后构建通过；首轮接口门禁发现重复N-06，新请求改用未占用N-07后通过。

未验证：导航消费端解析/录包迁移、目标机实际发布频率与RT时序、完整整机mock部署、硬件运动和停止距离。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 本地实现使用我方模型与硬件映射基线；不通过改零偏或模型关节原点适配导航副本。
- F2: 授权是驱动实际准入状态，不是请求回显；请求不改变已有cmd_vel看门狗或使能边界。
- F3: 91项索引权威属于 robot_interfaces 的 `contract/navigation-execution-v1.md`，需接口与消费域评审后统一发布，不假造已发布SHA。

## 遗留

未提交、未推送、未创建PR、未合并、未部署。接口契约尚待评审/发布，因此未修改 driver 的 deps.repos/source-lock pin；发布后同批升级驱动、导航及录包工具。Linear恢复后补建并互链。既有#326映射修正/方向实测与#328默认模式PR未擅自合并；合入本改动时需由集成负责人串行协调，不能以本机软件验证替代现场方向复测。
