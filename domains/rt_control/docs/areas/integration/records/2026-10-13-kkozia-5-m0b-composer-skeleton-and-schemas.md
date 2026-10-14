---
id: integration-20261013-01
area: integration
title: M0b 组合器与机型数据包骨架及共享 schema
date: 2026-10-13
type: feature
trigger: KKOZIA-5
commits: [agent/rt-control/d93b6ad1b934]
env: native
risk: T0
writes: {reset: no, enable: no, motion: no, plc: no}
verified: UNVERIFIED
evidence: []
supersedes: []
related: [governance-20261013-03]
---

## 背景

KKOZIA-5 按 SPEC v1.1 M0b 与架构师 ADR-18～21 提供 A0 评审输入。
基线为 M0a 合并后的 main `37c3242e087df86df81da7cf8dcf3a9cbc940621`。
记录编号由架构师在 issue 评论 `01a135ba-2475-7413-818e-6adc8b2dcafa` 发放。

## 改动

新增 `rt_control_composer` 与纯数据 `rt_control_machine_config`；只安装 schema、
契约文档、校验器和数据目录，不安装测试，不接 launch 或迁移运行入口。
九份 Draft 7 schema 覆盖 manifest、controllers、safety_policy、module envelope、
device、deployment、calibration envelope、render request 和 result projection。
公共字段逐字段引用一份 definitions；模块/标定私有段放 `spec`，由 owner 校验。
设备仅要求适用的六项型号信封字段与 family，不绑定某一机型/模块；物理字段可选，
正值或显式 TBD。机型文件另加 robot_variant，模块/标定另加 module。

新增离线 YAML 校验器，拒绝重复/合并键、锚点/别名、多文档、非有限值以及越界路径；
只从安装目录解析 schema 引用。`x-share-relative-path` 明确标注路径字段，私有 schema
也须标注；跨文件存在性与所有权仍归 M2d。协议 profile 与 Robot Model 原加载器不变。

结果 Mapping 保持 ET.Element，schema 校验 `{tag, attrib, text, children}` 投影；
新增可选 gpios，sensor 不允许 command interface。投影代码归 M2d；非 EtherCAT
受管轴按输入声明顺序，不编造环位。合成 A/B/C 改为 machines/devices/units 布局。
改名表仅含 rename_map；R5 排除整个 golden，运行时关节路径先保留同名映射。

跨路径变更只有本 issue 授权的机型数据包骨架，无其他跨路径机械性改动。
未修改公共接口、Robot Model、旧组合器、launch、安全参数、CI/Dockerfile 或补丁文件。

## 验证

Python 3.10 / ROS 2 Humble 原生 T0。jsonschema 3.2.0 与发行版依赖解包至隔离工具目录，
通过 PYTHONPATH 加载；sudo 需要密码，未改宿主 apt 安装状态。ruff 为 0.11.13。

- `colcon build --packages-select rt_control_composer rt_control_machine_config`：PASS。
- 同范围 `colcon test` / `colcon test-result --verbose`：152 项 pytest 通过；
  colcon 汇总 153（包括 CTest 包装测试），零失败、零跳过。
- 每份 schema 均有正例与缺字段、多字段、类型错误三种反例；测试使用安装后的 schema。
  另覆盖 YAML 边界、整数十六进制、路径/symlink 越界、外部 schema 引用拒绝、设备物理
  字段、scope 周期覆盖/BQ 配对、XML 属性/层级/sensor writer 拒绝和 CLI 退出结果。
- Python 编译、ruff check/format、R5（排除 golden）、diff 空白检查：PASS。
- `python3 tools/repository_gate.py`：PASS。
- `tools/quality_gate.sh`：PASS，227 passed / 10 skipped，门禁覆盖率 84%。早期 vendor
  未套冻结补丁的两项时钟测试失败已通过按 bootstrap 既定顺序应用原有补丁解决，未改补丁本身。
- 初次 colcon 测试因 APPEND_ENV 产生冒号前缀路径失败，改为 ENV 后全部通过。

测试数量大于校验器行为数量，原因是按 issue 验收对各 schema 的每份合成正例参数化
执行三种形状反例；未增加硬件或实现内部常量测试。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: M0b 新增两包与九份 Draft 7 草案，未改变运行入口；A0 以评审提交 SHA 为准。
- F2: 模块/标定私有字段位于 spec；型号物理事实可选且支持 TBD，结构通过不代表真实准入。
- F3: renderer API 1 保持 Mapping/Element；结果投影支持可选 GPIO，投影实现归 M2d。
- F4: 离线校验器在机型/策略 YAML 边界拒绝歧义输入与越界引用，不访问设备或网络 schema。

## 遗留

A0/消费者评审待架构师执行；实际协议渲染器、私有 schema、XML 投影、跨文件检查、
真实路径必需字段与 verified 门、Mock 启动/生命周期均未在本任务实现或验证。
金样本体由 M2d 生成，当前只有改名表草案；运行路径占位须展开为有限精确映射。
无 T2 及以上硬件操作或证据。M1 负责包注册及系统依赖；本地缺 clang-format/shellcheck，
本任务无 C++/shell 改动，完整 CI 仍需执行。三份合成 safety_policy.yaml 不改变生产策略，
仍按 R9 对文件路径的要求交由人工 reviewer 确认后合并。
