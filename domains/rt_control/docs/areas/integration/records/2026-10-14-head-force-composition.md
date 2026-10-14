---
id: integration-20261014-03
area: integration
title: 头部与力传感离线迁移及停止点三准入报告
date: 2026-10-14
type: feature
trigger: 用户要求保留原位T1验证缺口并继续第四步
commits: [feature/kkozia-17-head-force-composition]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence:
  - ../attachments/head-force/actual-unit-admission.json
  - ../attachments/head-force/head-composer-false.json
  - ../attachments/head-force/head-composer-true.json
supersedes: []
related: [integration-20261014-01, integration-20261014-02, BQ-154]
---

## 背景

在第二步原位T1根因未确认时，按用户要求继续开发。第四步仍保留默认旧入口，
头部D-b未确认时新真实路径拒绝。本记录同时给出第五步前的停止点三证据。

## 改动

- head_gimbal.yaml按damiao_head_controller/README.md §4示例保存协议参数：
  两台velocity_limit 0.2 rad/s、ACC 2.0 krad/s²、DEC -2.0 krad/s²、MAX_SPD 600.0 rad/s；
  configure/feedback/transition 100/50/250 ms、command_rate_hz 250、disabled poll 20 ms、
  max_rx_frames_per_cycle 32。can_id 1/2、master_id 17/18。direction和head_zero仍TBD。
  status:draft、verified:false；authorities确认人/日期TBD。示例值不冒充真实确认。
- deployment补head can2、bms can0。device仅声明DM-J4310-2EC_V1.1及未确认型号参数，
  不把本次单机寄存器读回当所有型号额定数据。限位仍取Robot Model当前文件±1.57 rad。
- robot_hw_can Mapping renderer复现旧Xacro的joint/gpio/参数集合，schema与唯一性/轴序检查
  复现旧head_can_config范围；D-b要求verified:true。渲染不访问CAN，不加入新设备预检。
- 蓝点左右profile合并，side/观察位置/ring_positions转为机型数据，runtime时按声明恢复旧字段。
  unit_codes、raw_per_unit、decimals与温度语义归协议profile，scale由其正有限raw_per_unit倒数计算。
  bench scope保持bluepoint_raw_bench及6响应者；全机位置独立TBD，不能用台架2/4准入全机。
- 原check_bench移到协议preflight，保留idle/link/responders/identity/PREOP/error flag/PDO校验。
  compose全程离线；GenericEcSlave被动片段无command interfaces，无startup SDO，固定输出0。
- integration生成话题与startup_id，sensor参数由owner返回，activation order与完整head controller文件保留。
  public request加可选hardware_system_id，result加可选controller_parameters。
  head CAN旧硬件参数没有control_frequency，共享schema不再要求每个transport都有该字段；
  EtherCAT仍按bus_cycle_hz提供，controller_manager主系统周期保持1000 Hz。
- HeadPhase枚举及原数值合法性检查迁到semantic_components头文件。robot_hw_can使用该枚举，
  damiao_head_controller只依赖semantic_components。状态值、接受范围与控制状态机不变。
- head/force launch增加use_composer默认false；新路径Node创建前compose/preflight。
  head_can_config只用于旧路径；新路径读取机型/单机信封，旧格式不自动转换。

## 验证

- 前置PR66修复后的源码CI通过；[CI](https://github.com/SevenovaHangzhou/robot_driver/actions/runs/38536170916)。
  不代替本增量CI，第二步原位T1缺口继续保留。
- 四份新金样在独立b7650e1旧检出生成，baseline head输入为旧test_v3_head_can_integration._config，
  force-real sync_mode=dc并注入归档CLI文本，只用于生成baseline。
  十份金样均规范化后零差异；元素、列表、批次、控制器顺序与文件集合保留。
  head测试限位/verified配置只在test临时输入中，不改生产数据。
- 八个构建包通过；实际测试包含robot_hw_can、robot_hw_ethercat、semantic_components、
  damiao_head_controller、composer、bringup及force broadcaster。colcon最终484 tests、0 errors、
  0 failures、0 skipped。新修改仅重跑composer，不重复未变C++测试。
- 第一次集成测试发现controller_manager误取最后CANopen模块周期250 Hz；已恢复主系统1000 Hz，
  底盘一致性用例通过。头部安装controller YAML链接越界已改为复制安装；本地隔离旧链接移除后
  重新安装，CI干净构建不依赖该本地处理。
- 头部旧/新Mock快照相同：head_position_controller inactive，damiao_head_manager与JSB active；
  command/state可用/占用与claimed_interfaces一致。日志中的非实时调度限制不构成目标机RT验收。
- force旧/新Mock本轮在加载/发现期限超时，未取得完整控制器/接口快照。保持T1未闭环，
  不依据环境假设标通过，不反复重跑。所有本次Mock进程已退出。
- 最终 `tools/quality_gate.sh` 通过：243 passed、13 skipped；repository gate无新违规。
  ruff/clang-format使用隔离固定工具，shellcheck本地缺失由CI强制。控制包grep robot_hw_can无命中。
- 合成C adapter、D-b拒绝、离线无CLI、录制CLI预检通过与不匹配拒绝有测试。
  full_ring_positions保持TBD，全机传感路径拒绝，不把bench拓扑复制为全机事实。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 头部D-b未确认，生产head verified:false，新真实入口拒绝，旧真实入口保留。
- F2: 十份离线golden等价不证明实际单机认证；真实golden用test配置，不替代以下准入结果。
- F3: 头部T1旧/新状态与占用相同；force及原位T1尚有缺口。
- F4: 同一HeadPhase与解码边界归semantic_components，控制包不再直接依赖robot_hw_can。

## 遗留与停止点三

实际单机文件离线real compose（未调用任何真实preflight）：

| 入口 | 离线结果 | 缺项或验证边界 |
| --- | --- | --- |
| arm | 拒绝 | arms.yaml verified:false；方向仍为未验证暂定值 |
| arm-jtc | 拒绝 | 同上 |
| stationary | 通过 | 仅离线；真实总线预检和当前设备状态未验证，T1查询缺口保留 |
| head | 拒绝 | D-b：head_gimbal.yaml verified:false、确认人和日期TBD |
| force-bench | 通过（dc） | 仅六响应者台架离线；真实预检未执行，T1未闭环；全机拓扑TBD |

D-b必须确认的具体资产：

- `rt_control_machine_config/machines/Alfa_V3/head_gimbal.yaml`：上述两台ID、速率、时序和ACC/DEC/MAX_SPD，
  逐项确认实际适用；填写authorities.confirmation_person和confirmation_date，再明确verified是否允许true。
- `robot_description/config/joint_limits_gripper.yaml`：两个头部±1.57 rad为当前模型限位，不能因README示例或
  golden测试将其替换成实测范围。新路径按当前模型消费；需要模型变化必须走模型owner流程。
- `rt_control_machine_config/units/Alfa_V3-01/head_zero.yaml`与机型direction：仍TBD，没有从历史近零读回猜模型正轴。
- `units/Alfa_V3-01/deployment.yaml`：head can2；确认适用于该单机。BMS can0来自旧配置，不改地址/帧协议。

本增量新增拒绝包括D-b verified门、机型/单机与API/XML结构检查、旧head_can_config参数在composer路径拒绝、
真实台架sync_mode与required拓扑边界。旧头部数值/ID/字段集合、力传感preserved PDO/no SDO/fixed0及bench预检沿用。
新增拒绝与safety_policy批准没有被提交/推送授权代替。

第五步尚未开始。必须由用户在停止点三批准模块默认切换；D-b未解决时头部旧入口保留。
完整镜像、目标机RT、设备通信、使能、运动、部署及当前增量CI仍未验证。
