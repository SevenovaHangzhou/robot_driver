---
id: integration-20261029-05
area: integration
title: 网页作业位移配置、执行与取消
date: 2026-10-29
type: feature
trigger: 用户要求不使用命令行，在网页切换模式并配置运动，启动默认导航
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: [validation/web-operation-20261029]
supersedes: []
related: [integration-20261029-02, integration-20261029-04]
---

## 背景

验收：在现有操作站配置相对前后/左右位移和旋转、运动限制，显式确认后通过既有私有
Action执行；仅实际已使能、作业确认、静止、IMU有效且无故障时准入。限制从控制器回读，
不能超限；显示真实反馈/结果，取消不伪称完成。断网/释放控制权/停止按钮请求取消。
不发送自动任务、不自动使能/切换，启动保持导航；公共接口不变，仅网页包内扩展。

## 改动

网页新增相对位移表单、控制器限制回读、单位转换、显式执行确认、阶段和实际位移反馈。
非实时 ActionClient 管理单个任务，服务端检查使能/OP/静止/IMU/状态和参数时效。
重复提交、未确认结果和超限参数拒绝；迟到接受也请求取消。停止、释放、断网或失去准入
请求取消并等待终态。网页退出不自动恢复任务；NAV 启动约定不变。
改动限于网页包及文档，不增加公共接口、实时 writer 或硬件配置。

## 验证

- 加载工作区 `env.sh`，网页包 `colcon build --packages-select rt_control_operator_web` 通过。
- 网页包 pytest：179 passed；colcon：187 results、0 errors、0 failures。
- `node --check web/app.js`、Python 编译检查及 `git diff --check` 通过。
- `tools/quality_gate.sh`：247 passed、13 skipped。
- 隔离 ROS Domain 203，无设备模型：真实 WebSocket → ROS 模式服务/Action 完成 1 cm
  模拟任务；停止取消、断网取消、NAV 默认和 OP→NAV 返回均通过。未连接实机总线。
- Node VM 执行实际 app.js：无自动指令、导航禁用、作业表单角度换算、重复提交拒绝、取消及反馈显示通过。
- 已安装并仅启动原本停止的操作站服务；HTTP 18110 返回新表单，认证 WebSocket 只读确认
  当前实际 NAV、无作业任务且控制器上限回读有效。未向实机发送控制权/使能/模式/运动请求。
- 本机无可用浏览器，未进行真实浏览器视觉验收；实机位移未执行。

日志和无设备检查脚本归档于工作区 `validation/web-operation-20261029`。
本记录不授权使能或运动。

## 结论与冻结事实

F1：启动为 NAV，网页操作须显式确认；模式切换沿用控制器现有四轮交接条件。

F2：网页允许配置单个相对位移任务，限制从控制器回读、服务端校验，不绕开控制器准入。

F3：取消仅报告请求和实际终态；断网/释放/停止请求取消，未确认结果阻止重提。
上述结论已完成 T1 无设备验证，实机低速验收待用户操作。

## 遗留

实机位移验收须由用户明确操作。
