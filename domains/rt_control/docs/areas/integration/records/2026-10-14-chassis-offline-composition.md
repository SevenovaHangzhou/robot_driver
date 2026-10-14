---
id: integration-20261014-02
area: integration
title: 底盘离线组合与联合测试边界迁移
date: 2026-10-14
type: feature
trigger: 用户要求提前开始四层解耦第三步；原 KKOZIA-11/12/13 底盘部分及 14 对照
commits: [feature/kkozia-11-chassis-composition]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [integration-20261014-01, BQ-154]
---

## 背景

第二步原位 T1 保持阻塞，用户明确要求继续第三步。此增量基于第二步 draft 分支，
只实现底盘数据、离线渲染和测试边界，不新增生产启动入口或真实准入。

## 改动

- 设备层迁入总成比率 27.48/140.0、外置编码器齿数 108/27、名义轮半径 0.100 m、
  机械端点 ±135° 和软件端点 ±120°。来源为旧机械文件引用的图纸和用户确认。
  打印参数标 as_printed；Kinco 有效 N、BRT 型号/计数、身份、环位和单机标定均 TBD。
- 机型文件拥有八轴的模块位置、角色、接口契约和待确认模型绑定；单机文件拥有零位、
  有效半径、背隙和外置编码器方向/偏置。公共 device spec 可表达 owner 私有型号事实。
- Kinco profile 保留四项 RxPDO/TxPDO 布局，512/1875 rpm 换算出处为 V3-DOC-2026-014
  p.107/108/154。阈值与超时的真实值全部 TBD；mock_adapter/mock_identity 来自既有 fixture，
  独立标 mock_only。渲染器计算 position/velocity factors，不把派生值写回事实文件。
- CANopen owner 安装 API 1 renderer、私有 schema、CiA406 protocol profile；节点声明顺序保留，
  输出 position/feedback_age_ms，没有 command interface。SYNC 取清单 4000 µs。
  原 cmake validator 保留旧校验 API，构建目标改指新 protocol profile，机型/单机校验交给 renderer。
- chassis_only 新离线 scope 只活动八轴与四编码器。原 suspension 仍在物理 profile 和 safety policy，
  没有 renderer，不纳入本次新 mock 控制集合；旧清单/入口不变。该选择差异显式记录。
- mock 模型从既有 chassis.urdf 去掉硬件片段后保存为明确的合成数据资产；真实模型绑定仍 TBD。
  mock_joint_name 使用既有 s0..s3/d0..d3；不是实机型号/关节映射。controller 仍为未验证 draft，
  输出只证明绑定和接口构成，不宣称控制器能以真实参数激活。
- 同硬件 id 合并，master health 在协议 owner 处只产生一次；peer_machines 提供同系统上下文。
  transport_layout 传递各总线参数；bus_cycle_hz 从显式 cycle 或 SYNC period 计算。
  unit_file/unit_module 显式引用同一单机标定，避免复制编码器标定。
- cyclic 联合测试移到 bringup，断言与 TIMEOUT 90 不变，KINCO_FIXTURE 来源不变。
  首次字节不变迁移触发远端“新路径”格式门禁；补做 clang-format 14 纯格式修正，
  与原文件经同一 formatter 的输出逐字节相等。
  两个辅助头仍供 swerve 其他测试使用，保留共享内容，bringup 测试直接引用，不建立副本。
- swerve package.xml/CMake 删除 protocol 测试依赖；R2/R3 增加 Python import 与 CMake
  find_package 检查。移除两处旧底盘 baseline，精确登记已有头部 find_package 到第四步。
- 删除无 runtime 消费者的 conversion.candidate；历史记录及冻结 golden 输入哈希不改写。

## 验证

- 五个受影响包构建通过，依赖按冻结 ros2_canopen SHA 与既有三份补丁在隔离目录构建。
  bringup 使用独立包发现范围构建联合测试，没有修改宿主或总线。
- `colcon test --packages-select rt_control_composer robot_hw_ethercat robot_hw_canopen swerve_driver rt_control_bringup`
  实际执行：五包通过，colcon 汇总 576 tests、0 errors、0 failures、0 skipped。
  后续只重跑修改后的底盘离线用例，5 passed；未反复运行全包测试。
- 现有手臂比较与底盘离线用例合计 22 passed。A5 六份冻结金样保持零差异。
  新数据文件、B 型合成 adapter、CAN profile、真实 TBD 拒绝均有实际检查。
- N={4096,65536}、ratio={27.48,140.0}、sign={-1,+1} 的位置/速度往返用例通过。
  real 路径不使用 fixture 参数；拒绝消息列出必需缺项，输出目录未创建。
- X9 合成两个模块以及手臂16轴+底盘8轴合并检查通过：一个系统、26个响应者、一个 master health。
  周期/插件不一致拒绝沿用第二步测试；每系统一次预检已有测试。
- 底盘输出与 motion/F3/F4/F7 的协议/控制边界相符：转向 CSP、驱动 CSV、编码器只读，
  renderer 不增加转向外环、rezero、MIT、力矩前馈、选支或 clamp。
- 清单 cycle_hz 与旧 validate_kinco_swerve_draft 的 1000 Hz 要求一致性用例通过。
- `tools/quality_gate.sh`：243 passed、13 skipped；repository gate 无新违规；两组覆盖率85%/86%。
  ruff 0.11.13 格式与检查通过；shellcheck 本地无，由 CI 强制检查。
- grep 两协议包没有 composer 引用。迁移前后格式化输出比较通过，断言与超时保留。
- 首次第三步远端 governance 因迁移路径的 C++ 格式检查失败，build 跳过。
  纯格式修正后本地质量门禁再次通过，修复后的远端 CI 待完成。
- 第二步 [PR65 CI](https://github.com/SevenovaHangzhou/robot_driver/actions/runs/38526655395) 已通过；
  不代替本次第三步 CI，也不闭合第二步的原位 T1 阻塞。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 实机 N、身份、拓扑、节点/EDS 和单机标定没有从 fixture 填值；real 仍拒绝。
- F2: 27.48/140 与 108/27 分属电机传动和外置编码器，不重复相乘。
- F3: 联合测试归 integration，协议与控制包不因该测试直接依赖。
- F4: 六份手臂 golden 不因底盘扩展更新；底盘没有旧运行 golden，只用合成和单位换算证据。

## 遗留

- 本增量的远端 CI 待完成；真实 CANopen master_config/master_bin/EDS 尚未确认，不生成或伪造真实 DCF。
- 全机共享物理从站未定义；相同 resource_claim 拒绝，不做共享从站兼容层。
- 原位 T1、第二步新增拒绝批准与 safety_policy 人工批准保持待办。
- chassis_only real 缺事实；mock controller 仍保留 calibration_verified:false 和真实阈值 TBD，不宣称可运行认证。
- D-b、BQ-154、完整传感拓扑与第五步默认切换不在此增量处理。
