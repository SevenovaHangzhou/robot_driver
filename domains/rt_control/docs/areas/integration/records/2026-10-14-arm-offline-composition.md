---
id: integration-20261014-01
area: integration
title: 手臂四层离线组合、冻结金样与 opt-in 入口
date: 2026-10-14
type: feature
trigger: 用户确认的四层解耦第二步（原 KKOZIA-7/8/9/10 对照）；后续提交推送已授权，暂不同步 Linear
commits: [feature/kkozia-7-arm-composition]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence:
  - ../attachments/arm-composition/arm-legacy.json
  - ../attachments/arm-composition/arm-composer.json
  - ../attachments/arm-composition/stationary-legacy-services.txt
  - ../attachments/arm-composition/stationary-composer-services.txt
  - ../../../../../../src/rt_control/rt_control_composer/test/golden/manifest.yaml
supersedes: []
related: [integration-20261013-01, platform-20261014-01, BQ-154]
---

## 背景

按用户已确认的四层边界实现手臂闭环。原运行入口与旧配置保留。
新增路径使用显式 opt-in；本记录不表示默认入口已切换或实机已认证。

## 改动

- 设备、机型、单机捕获和只读 evidence 分离；未知型号映射和额定参数保持 TBD。
  字段来源、契约补齐及逐条拒绝分类见[迁移明细](../arm-composition-sources.md)。
- EtherCAT owner 安装 Mapping API 1 renderer 和私有 schema。
  ZeroErr 输出侧换算使用设备输出计数，不乘减速比；PP 保持 hold。
  原位预检复现旧只读检查；手臂运动没有新增总线预检。
- 两个控制 owner 安装使能策略、容差和原样复制的 provisional 包络。
  16 个单轴批次、14 轴右先左后的容差顺序保留。主环 1000 Hz、原位 250 Hz 不变。
  当前源码两个入口的 service_result_timeout_ms 均为 90000；不按旧文本改成 30000。
- composer 注册器验证公共信封和无字段过滤的 XML 投影，私有字段归协议 owner。
  组合仅离线运行；每合并系统预检一次。原位 commissioning 名称和模型名称不混用。
- 两个 launch 增加 use_composer，默认 false。新路径在创建 Node 前调用 compose/preflight。
  旧 calibration_file 参数仍由旧路径消费；新路径拒绝旧标定格式，等待逐条批准。

## 验证

- 前置 PR #64 的修复提交 af24300 完整源码 CI 已通过：2175 tests、0 errors、0 failures、23 skipped。
  [CI](https://github.com/SevenovaHangzhou/robot_driver/actions/runs/38506917876)。完整镜像未构建。
- 在 b7650e1e7e88f0ed53794fa8e772b443d21d996a 的独立只读检出生成六份旧输出。
  arm-mock/real、各自 JTC-only、stationary-mock/real 全部规范化后零差异。
  源码与新实现没有混入同一 import 路径。输入、输出、测试标定哈希和精确改名表已归档。
- 首次归档的 XML 占位符转义及末尾换行有误。只用同一旧检出修正生成格式；
  修正前后六份规范化内容及来源输入哈希相同。没有用新实现重建旧输出。
- 本地隔离构建受影响包与安装资产通过。冻结上游只构建所需依赖，未改源码/补丁。
  本地链接既有 IgH userspace 缓存，仅为 Mock/构建环境证据，不冒充发布镜像身份。
- `colcon test --packages-select rt_control_composer robot_hw_ethercat enable_manager rolling_trajectory_controller`
  及受影响 bringup 测试执行；当前汇总 589 tests、0 errors、0 failures、0 skipped。
  bringup 使用独立包发现范围，避免构建其与此次 Mock 无关的完整硬件闭包。
- 样例 A conformance 包括缺失必需事实、verified:false、无写入/归档读取、无 composer import、
  未知 XML、API 版本与注入的原位预检。X9 冲突拒绝和一次预检已有合成检查。
- T1 手臂旧/新快照逐项相同：enable_manager、两个 JSB 为 active，JTC/rolling 为 inactive；
  30 个 command interfaces 的可用/占用及各控制器 claimed_interfaces 一致。
- 首次新手臂 Mock 发现 YAML 共享列表产生别名，Humble 参数解析器拒绝。
  已改为独立列表副本，值和顺序不变；现有金样测试同步检查生成 YAML 没有别名。
- T1 原位旧/新诊断均观察到三个控制器完成 active 加载，但后续控制服务查询超时。
  默认 CLI daemon 也存在本地端口连接超时；尝试官方 CLI direct-node 与临时 loopback UDP 后，
  原位接口占用查询仍未闭环。保存实际日志与服务目录，不标为完整 T1 通过。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 六份旧输出来自固定旧检出；新实现零差异仅证明离线等价，不证明实际单机认证。
- F2: 输出侧位置换算不乘减速比；未确认的型号参数没有变成真实值。
- F3: 使能批次、容差与控制器顺序显式保留；ROS 参数输出不含别名。
- F4: 默认入口未切换，旧 V3/V2 配置和冻结实现未删除；新增拒绝及 safety_policy 仍需人工批准。
- F5: 手臂 T1 旧/新状态与接口占用相同；原位 T1 仍为待验证，不能宣称第二步全部验收完成。

## 遗留

- 完成原位 T1 控制器/接口占用对比；定位本地发现/查询问题或在可验证环境重跑该部分。
- 用户逐条批准新增拒绝条件，并亲自批准 safety_policy 的原样迁移 PR。
- 新 PR 的 CI、完整镜像、目标机实时性与实机验证未完成。
- 实际手臂运动标定未验证；设备型号映射、完整腕传感拓扑、D-b 和 BQ-154 未解决。
- 不在此提交删除旧入口、清空基线或补 R2/R3 Python/CMake 扩展。
