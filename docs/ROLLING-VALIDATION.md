# 2026-10-31 Rolling 开发版验证

版本：0.2.0.dev0。工作源码默认启用 Rolling Mock 发布；原 v0.1.0-rc1 冻结包不变。

- Python 全量回归：79 passed，含权威模型左右臂各六方向 Rolling 路径、姿态、末端位移和另一臂保持检查。
- 新增 Rolling 用例：26 项，覆盖窗口容量/时序/确认、14 轴映射、控制器关节制动和 FINALIZE、拒包/重启/迟到/丢确认/状态超时/非有限反馈、保活退出、初始化失败清理、切臂和 HTTP 端到端链路。
- Node 键盘输入状态机：6 passed，含长按、重复键不积压、松键、冲突、输入框、失焦与 Esc。
- Python/前端模块语法检查通过。
- 更新后的 9 文件操作台 UI 补丁 git apply --check 通过；未实际应用到 robot_driver。
- 发布协议参考与 driver 当前源码 SHA 记录在 integration/rolling-reference.json；未复制公共 IDL。

本机缺少 ROS、robot_interfaces 权威生成包和 C++ 编译器。没有执行原生 Rolling 控制器测试、ROS 消息序列化/QoS/服务发现联调、控制器切换或任何硬件验证。ROS 适配器为待集成代码，不将本地协议模拟结果表述为 RT/实机验证。

本轮未重新做浏览器视觉验收或 ROS/colcon 构建。日志目录：`/tmp/gripper-jog-rolling-validation-20261031`。
