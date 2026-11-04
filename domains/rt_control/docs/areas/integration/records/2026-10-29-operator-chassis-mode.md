---
id: integration-20261029-02
area: integration
title: 操作站导航与作业模式切换服务对接
date: 2026-10-29
type: feature
trigger: 用户要求在18110操作站增加作业模式切换并核对执行条件
commits: []
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [BQ-145]
---

## 背景

任务/验收项 integration-20261029-02：RT-Control 域内操作站接入已有私有
ChassisSetMode 服务与 ChassisState 反馈；仅持有控制权的已认证终端可请求切换，
仅实际回读确认后报告成功；未配置、反馈过期、忙、故障时拒绝，作业模式锁定速度入口。
直接消费者为浏览器与 swerve_controller，不改变公共接口、实时控制环或硬件准入参数。
实现阶段为非实时 Web/ROS 适配，涉及运动入口的安全拒绝路径。

## 改动

操作站增加模式显示与切换按钮，独立于原有对齐/连续行走策略。
后端按配置及实际状态准入，切换前后清空速度操作，切换期间锁定速度请求。
当前生成配置未启用 relative.enabled、imu_enabled=false；保持硬件准入门禁。
用户授权复用导航参数并估计未知值后，新增 swerve_driver/config/relative.commissioning.yaml，
区分导航来源与待验估计。2 m/s 轮面速度按 0.1 m 半径换算成20 rad/s，
任务初始平移速度采用休闲档0.05 m/s。估计值不改写 profile_verified。
修复 lpms_nav3_can CMake 无条件统一警告基线缺失，保留 -Wsign-conversion；
tools 的旧IMU集成断言更新为现有只接收节点及有效配置检查，未删除实际的非法配置拒绝测试。

## 验证

- 两包使用现有v3工作区 merge-install 构建，BUILD_TESTING=ON，通过。
- `colcon test --base-paths robot_driver/src/rt_control/rt_control_operator_web robot_driver/src/rt_control/lpms_nav3_can --packages-select rt_control_operator_web lpms_nav3_can`：通过；
  分包 `colcon test-result`：操作站160项、IMU28项，均无失败。
- `tools/quality_gate.sh`：通过，247 passed / 13 skipped；未安装的clang-format/ruff按门禁提示保留为CI检查。
- `node --check .../web/app.js`、Python语法、git diff --check：通过。
- `/tmp/chassis-mode-offline-smoke.py`（ROS_DOMAIN_ID=201，ROS_LOCALHOST_ONLY=1）：
  将导航几何与新建议参数交给 explicit_no_device_mock=true 的独立模拟器；
  操作站真实ROS客户端往返切换NAV→OP→NAV成功，OP速度入口锁定，零位移Action成功。
  测试未加载任何真实硬件插件；模拟器NAV保留holding阶段且不执行速度遥控。
- 仅重启 rt-operator-web-v3.service；18110 HTTP 200、页面含两个模式按钮。
  认证后只读WebSocket确认 connected=true、enabled=true、drive_available=true；
  chassis_mode.can_switch=false，原因是现有部署未启用作业模式。
- 未重启底盘运行栈，未调用实机使能/失能/模式服务，未发送非零速度或实机Action任务；
  网页重启使用其原有零速收尾逻辑。实机四轮交接未验收。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 模式切换使用控制器已有私有服务，服务返回 OK 且实际模式匹配才报告成功。
- F2: 配置未启用或模式反馈过期时不能切换；作业模式不接受操作站速度遥控。
- F3: 调试建议配置不自动加载、不改变已验证标志；导航轮面单位与作业轮侧角速度需显式换算。

## 遗留

真实作业配置、逐轮限制/标定和 IMU 验证及四轮实际切换验收仍待完成；不以增加网页入口替代实机准入。
