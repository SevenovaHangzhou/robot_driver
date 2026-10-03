---
id: contract-20261002-01
area: contract
title: V3 公共接口与适配器实现
date: 2026-10-02
type: feature
trigger: ELECTRI-152 与桌面 RT-Control 实现 Spec/IDL 清单
commits:
  - feature/electri-152-interface-contract
  - feature/electri-152-rt-interface-implementation
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [RTC-01, RTC-02, RTC-05, RTC-09, RTC-13, RTC-14, RTC-15, RTC-17, RTC-18]
---

## 背景

> 完成边界已由 [2026-10-03 Draft 复核](2026-10-03-interface-dependent-draft.md) 更正：
> 这是部分软件实现，不能视为只剩硬件配置；接口候选 SHA 可以用于 Draft 验证，不代表发布。

ELECTRI-152 的 V3 接口评审要求将旧单体真空动作拆成 Autonomy 管泵、Motion 管左右阀，
并为位置执行、传感器质量、模块能力、事件和保护建立可编译公共协议。

## 改动

- 在独立 `robot_interfaces` 工作树实现 20 项 RT IDL 草案及 endpoint 注册，退役
  `VacuumGrip`、`/cmd_vel_safe` 和旧 LED／头部外部写入口；保留并合入当前 driver pin 的
  FJT/Rolling 类型，同时保留公共仓库 main 的 N-17。
- `control_api_adapter` 实现左右阀 Action、泵停机保护、双侧压力分类、真空变化事件、
  PP／升降／头部位置 Action、模块状态和触边保护状态机。
- 新增默认 `validation_only=true` 的 `rt_control_interface_runtime.launch.py`；Mock 模式只启动
  适配器，不访问 Modbus、CAN 或执行器。机械臂和头部入口可关闭各自 RSP，供组合运行保持单一 RSP。
- 位置 Action 的实物限位、容差、控制器映射缺失时 `configured=false`；触边、闭环力控和换电
  分别报告未安装／未准入／未实现，不伪造可用。

## 验证

- `robot_interfaces`：生成视图、contract/error/changelog gate、56 项工具测试、六包 ROS 2
  Humble 全量构建均 PASS。
- RT 聚焦：58 项 Python／合同测试 PASS；`control_api_adapter` 40 项包测试 PASS；
  validation-only launch PASS。
- `tools/quality_gate.sh`：PASS，207 passed、12 skipped、门禁覆盖率 83%。
- 默认全闭包构建已尝试，受本机缺少 `/usr/local/etherlab` 和
  `ros2_control_test_assets` 阻塞；改动包使用隔离 `/tmp` 构建闭包验证。
- Mock 四个适配器进程能够启动；本机 ROS 2 CLI 跨进程发现未取得 endpoint 列表，未写成
  cross-process smoke PASS，进程已清理。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 新公共真空职责为 Autonomy 设置泵、Motion 设置左右阀；阀输出确认不等于吸牢。
- F2: 位置 Action 同资源忙时拒绝、无队列／抢占，取消必须停止并保持；未配置资源 fail closed。
- F3: 底盘、双臂、升降和 PP 不建立跨组运动互斥；同资源唯一 writer 与 FJT/Rolling 互斥保留。
- F4: 公共 schema 已提交 PR #11；合并后的最终 main SHA 产生后，RT 实现与全部消费域才允许原子升级。

## 遗留

- PR #11 已创建，契约与全包构建 CI 通过；仍需至少两名批准和合并。合并后更新
  `deps.repos`、`source-lock.yaml` 并重跑 driver 全闭包；当前远端 main 不含 driver pin 中的 Rolling 合同。
- Motion、Perception、Autonomy 消费方尚未升级，跨域 smoke 未执行。
- 实物参数和后续功能集中见 BQ-152；本机完整构建环境缺失见本记录验证边界。
