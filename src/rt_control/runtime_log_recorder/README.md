# runtime_log_recorder

Runtime-LOG 的 ros2_control 记录插件；每周期将已配置的状态接口复制到共享内存环。
插件名、参数、共享内存格式和既有运行行为保持不变，不提供运动或使能命令。

Python 归档服务、日志网页、部署脚本和使用说明见
[tools/runtime_log](../../../tools/runtime_log/README.md)。

从 robot_driver 根目录构建：

```bash
colcon build --symlink-install --base-paths src --packages-select runtime_log_recorder
colcon test --packages-select runtime_log_recorder
colcon test-result --verbose
```
