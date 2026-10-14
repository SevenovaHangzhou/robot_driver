# 第二步字段来源与验收边界

状态：部分完成。手臂数据、策略、协议渲染和 opt-in 组合已实现。
六份旧输出规范化后零差异；原位使能 T1 接口占用对比尚未完成。

## 字段来源

| 新文件/字段 | 当前来源 | 迁移边界 |
| --- | --- | --- |
| `devices/zeroerr_erob{170h,142h,110h,90h,70h}.yaml` 的输出编码器计数 | 旧 `alfa_v3_arm_motion_calibration.draft.yaml:output_counts_per_revolution`；该字段引用 eRob 手册 V3.39 §9 | 524288；无型号到关节的映射证据 |
| 上述设备文件的减速比、额定/峰值力矩、电流、力矩常数 | 当前仓库无逐型号证据 | 全部 TBD；不进入输出侧位置换算 |
| `zeroerr_erob_model_tbd.yaml` | 同一已使用的输出编码器计数来源 | 表示型号未分配，不宣称某个型号装在某个关节 |
| `zeroerr_pp_gripper_model_tbd.yaml` | 原零位捕获的输出编码器口径及同一手册来源 | 夹爪型号、比率和额定参数 TBD；只保留 hold 行为 |
| `manifest.yaml` 的模块、profile、scope、硬件选项 | 旧 `rt_control_bringup/config/machines/alfa_v3.yaml` | 显式新增原位 scope；默认力传感选装移到 deployment |
| 模型 gripper/suction 的 xacro、limits、initial_positions | 当前 robot_description 中的两套文件及 source lock | 无模型、关节名或限位修改 |
| `layout.ethercat.cycle_hz` / `layout.canopen.sync_period_us` | 已确认设计口径与当前运行源码 | 1000 Hz / 4000 µs；不是头部 CAN 帧率 |
| 原位 scope 的周期覆盖与 BQ 引用 | `stationary_enable.py`；BQ-154 | 保留 250 Hz；不统一为 1000 Hz |
| `controllers.yaml` 的底盘/头部绑定 | 旧 manifest 的 controllers | 尚未迁移其运行入口 |
| 两个夹爪 forward 控制器绑定 | 旧 `alfa_v3_grippers.draft.yaml` 的 controller 段 | 保留绑定，不加入手臂 hold 入口的 spawner 集合 |
| 手臂、使能与广播器的绑定/role/order | 旧 `_controller_config()` 与两个 launch 的 spawner 顺序 | JTC/rolling 先 inactive，三个生命周期控制器后 active |
| `safety_policy.yaml:fault_dependencies` | 旧 manifest 的 safety_policy | 条目和值原样；需要用户亲自批准 PR；未新增运行参数 |
| `arms.yaml` 的分支、轴序、环位、模式、模型绑定、编码器策略选择 | 旧 `alfa_v3_arms_only.draft.yaml` | 设备型号仍未知；身份和错误码策略归协议 profile |
| `arms.yaml` 的方向、branch_mapping_verified、direction_status、verified | 旧 `alfa_v3_arm_motion_calibration.draft.yaml` | 方向为用户暂定 +1；运动 verified 仍 false |
| `wrist_force_sensors.yaml` 的 side、profile、frame_id、环位 | 旧 `bluepoint_force_sensors.draft.yaml` | 保留台架 2/4；全机环位与 frame_id 未确认，不外推台架拓扑 |
| deployment 的末端、force_sensors、master_id | 旧默认末端/选装与 arms_only 主站 | gripper、none、0；host_id TBD |
| `arms_zero.yaml` 全部 spec 字段 | 旧 `alfa_v3_mechanical_zero.yaml` | 16 轴计数、数字输出、确认人、时间、1.0 比率、0x6064 原样 |
| evidence 下协议读回 | 旧 `alfa_v3_protocol_readback.yaml` | 字节复制，只读归档，不作运行输入 |
| 两个 drive profile 的 driver_config | 旧运行入口实际消费的 `zeroerr_stationary.yaml` | 两项 RxPDO；CSP 在其上添加位置接口和运行时 factor/offset |
| drive identity、encoder_policies、coupler identities | 旧 arms inventory 的 identities/encoder_policies | 同一数值与允许码，不新增复位动作 |
| enable policy | 旧 `_controller_config()` 与 `build_stationary_enable()` | 16 个单轴批次；顺序明确；两个 runtime view 保留各自参数集合 |
| `service_result_timeout_ms` | 当前两个旧 builder | 两者均已为 90000；原指引所述 stationary 30000 与当前源码不同 |
| JTC、rolling、模式切换数值 | 旧 `_controller_config()` | 全部逐项迁移；数组顺序为旧 MOTION_JOINTS |
| arm envelope | 旧 `alfa_v3_rolling_envelope.mock.yaml` | 字节复制并精确改名，仍为 ESTIMATED_NOT_MEASURED |

其它模块保留带来源和 pending_facts 的迁移占位信封。
这些文件只保证清单引用闭合；它们没有协议渲染或运行资格。
第三、四步替换相应占位内容。

## 契约最小补齐

A0 没有表达下列当前必需数据。本次添加可选字段，旧合成样例仍合法：

- request `deployment`：复用现有 unit deployment schema，传递主站事实。
- manifest `policy_refs`：定位已有 owner 策略/包络文件。
- scope `render_mode`、`controller_order`：定位已有原位输出和顺序。
- controller `role`：按职责接线，避免 composer 按插件名分派。
- result `actuator_groups`、`resource_claims`：校验绑定与物理资源重复。

共享 request/result 信封在 registry 验证。协议 owner 校验私有数据。
协议包不 import composer；控制 owner 的代码 API 不进入 composer。

## 附录 §C：沿用与新增拒绝

本表记录实现状态，不代替用户逐条批准。
默认旧入口不变；新增路径仍 opt-in。以下“新增”未宣称已获批准。

| 阶段 | 条件 | 分类/依据 |
| --- | --- | --- |
| compose | 重复/合并键、锚点、别名、多文档、非有限数 | 清单重复键沿用；模块/单机/策略及其余 YAML 边界为新增 |
| compose | 包 share 绝对路径、父目录、符号链接越界、归档输入引用 | 新增；对应共享边界及不读 evidence 要求 |
| compose | 共享/私有 schema 形状与未知字段 | 旧轴/模式/零位等检查部分沿用；新增字段级检查需批准 |
| compose | variant/profile/scope 引用未知 | 旧清单路径沿用；新 machine_unit、跨文件信封一致性为新增 |
| compose | X1–X3 引用、唯一性与绑定 | 新增结构检查；旧入口 16 轴零位覆盖与唯一性仍沿用 |
| compose | X4 输出关节不在展开模型中 | 新增；commissioning 资源名仍按原位入口产生，不伪装模型关节 |
| compose | X5 direction / 正计数 / certified_modes | CSP real 的 ±1 沿用；共同 schema 和其它族/策略检查为新增 |
| compose | X7a 使能批次恰好覆盖一次、X7b 数组长度与绑定集合 | 新增；不计算批次、容差或重新排序 |
| compose | X8 device/family 引用、X9 同系统参数冲突 | 新增；系统不一致直接拒绝，不调参合并 |
| compose | 手臂 verified、branch_mapping_verified、零位覆盖 | 沿用；不检查无关模块 draft/TBD |
| compose | 原位 confirmed_by:user、external_transmission_ratio:1.0、position_object:0x6064、int32 计数 | 沿用 |
| compose | renderer 必需输出计数、真实 deployment 主站不能 TBD | 新增 required-field 条件；非必需型号额定参数 TBD 不阻断 |
| registry | API 非 1、非 Mapping、非法 XML/tail/namespace/未知字段 | 新增结构检查；不丢字段绕过 schema |
| launch | use_composer:true 配合旧 calibration_file | 新增；新入口读取机型/单机信封，不转换旧标定格式。旧路径仍接受原参数 |
| preflight | 原位主站 inactive、枚举/身份/PREOP、分支端口、6041/603F/6040、single_turn 允许码 | 沿用原位 verify_stationary_bus；只读注入测试通过 |
| preflight | 手臂运动入口新增总线检查 | 无；该 owner 入口保持无总线预检，调用次数接口仍保留 |

## 仍缺事实与验证

- 实际手臂运动文件 verified:false；方向尚未完成真实确认。
- eRob 型号映射、型号比率/额定参数、PP 夹爪型号仍未知。
- 全机腕传感拓扑和 frame_id 未确认。
- 原位 T1 查询未闭环；已观察到旧、新入口的三个控制器均 active，尚未取得完整接口占用对比。
- 完整镜像、目标机实时性、设备读回、使能、运动均未验证。
- 后续头部 D-b、底盘 N/身份/标定、BQ-154 不在此提交裁定。
