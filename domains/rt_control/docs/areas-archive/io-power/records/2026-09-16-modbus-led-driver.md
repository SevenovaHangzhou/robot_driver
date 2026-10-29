---
id: io-power-20260916-01
area: io-power
title: 四路 Modbus LED 驱动与离线通信验证
date: 2026-09-16
type: feature
trigger: "robot_driver Issue #31; gitaccountwbt 授权修复、测试、提交与创建 PR"
commits: [feature/rt-control-modbus-led]
env: native
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: ["robot_driver#31"]
---

## 背景

按 Issue #31 收尾用户现有四路 LED 驱动。LED 是域内硬件 IO，归属 io-power；
不新增其他域业务、公共 endpoint 或安全指示保证。

## 改动

新增 `modbus_tcp_rtu485_led`，bringup 增加 `start_led` / `led_config` 与包依赖。
完整校验 MBAP 事务号、协议、长度、unit，以及写回显的功能码、地址、数量；
异常响应按失败处理。部分发送和分片接收共用连接至接收的总截止时间。
参数校验 IPv4、四路端口 1..65535、四路地址 1..247、超时 1..60000 ms。
代码默认地址从 [1,2,3,4] 对齐现有 YAML 的 [1,1,1,1]，队列深度从 10 降至 1。
Mock 默认不启动 LED；独立进程的 IO 不进入 ros2_control 实时环。

## 验证

环境：ROS 2 Humble；构建/安装/日志均位于 `/tmp/robot-led-pr-*`。

- `source /opt/ros/humble/setup.bash; colcon --log-base /tmp/robot-led-pr-log build --base-paths src/rt_control/modbus_tcp_rtu485_led src/rt_control/rt_control_bringup --build-base /tmp/robot-led-pr-build --install-base /tmp/robot-led-pr-install --packages-select modbus_tcp_rtu485_led rt_control_bringup`：PASS，2 包。
- `/tmp/robot-led-pr-build/modbus_tcp_rtu485_led/test_modbus_transport`：PASS；参数边界、编码、完整/分片响应、异常/非法回显、断连、后续请求恢复、接收/发送截止时间。仅使用本地 socketpair。
- `source /opt/ros/humble/setup.bash; ROS_LOG_DIR=/tmp/robot-led-pr-ros-log PYTHONPYCACHEPREFIX=/tmp/robot-led-pr-pycache PYTHONPATH=src/rt_control/rt_control_bringup:src/rt_control/x503_force_sensor:$PYTHONPATH python3 -m pytest src/rt_control/rt_control_bringup/test --ignore=src/rt_control/rt_control_bringup/test/test_mock_contract.py -q -p no:cacheprovider`：PASS，140 项，包括 Mock/真实配置的 LED 默认开关。
- `source /opt/ros/humble/setup.bash; ROS_LOG_DIR=/tmp/robot-led-pr-ros-log python3 /tmp/robot-led-node-smoke.py`：PASS，非法超时配置返回 1；两次 YAML 配置启动与 SIGINT 退出返回 0，未发布颜色。
- 相同节点验证已纳入 `test/test_node_lifecycle.py` 与 CTest `led_node_lifecycle`，使用临时日志目录与隔离 ROS 域。
- `source /opt/ros/humble/setup.bash; colcon --log-base /tmp/robot-led-pr-log test --build-base /tmp/robot-led-pr-build --install-base /tmp/robot-led-pr-install --packages-select modbus_tcp_rtu485_led; colcon test-result --test-result-base /tmp/robot-led-pr-build/modbus_tcp_rtu485_led --verbose`：PASS，2 tests / 0 errors / 0 failures / 0 skipped（获准离线 socket / ROS 测试）。
- `tools/quality_gate.sh`：PASS，282 passed / 13 skipped，门禁覆盖率 83%；本机缺 shellcheck，交由 CI 强制检查。
- `git diff --check`：PASS。
- 首次通信 CTest：FAIL（沙箱 socket IO 受限）；获准直接运行同一测试程序后 PASS。首次 bringup pytest 和节点 smoke：FAIL（默认 ROS 日志目录只读），显式 `/tmp` 日志路径并获准重跑后 PASS。
- 未运行完整整机 Mock 合同测试、Docker、HIL 或实机；不声明设备身份、寄存器映射或实际 LED 写入已通过。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: LED 颜色话题仅为 RT-Control 域内工程输入；外部消费者需另行审查公共契约。
- F2: 故障写入不自动重试；退出不发送未经授权的关灯/复位命令，硬件可能保留最后颜色。
- F3: supplied YAML 仅保留用户配置；四路身份、地址与寄存器映射仍需现场核验，不能以离线测试替代。

## 遗留

实际网关连接、四路 RGBW 写入、故障保持/退出行为与设备协议依据需授权现场验收。
完整整机 Mock 与实时隔离尚未复验。通信在独立非实时回调同步执行，退出可能等待当前
事务截止时间；无新增 CPU 绑核或容器权限。无硬件命令在本次验证中执行。
