---
id: platform-20261029-01
area: platform
title: Runtime-LOG 归入 robot_driver 并保持部署行为
date: 2026-10-29
type: corrective
trigger: 用户要求将 rt-control/Runtime-LOG 纳入 robot_driver、保持功能并删除原目录
commits: []
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: []
---

## 背景

独立 Runtime-LOG 工作树含有未提交的波形读取、底盘反馈速度和加速改动，
systemd 实际使用另一份用户目录下的部署副本。用户要求合并维护位置并删除指定旧工作树。
本次不提交或推送 Git，不修改机器人控制参数，不启动底盘或发送使能/运动命令。

## 改动

- Python 归档、网页、配置、工具和测试迁至 `tools/runtime_log`；
  ros2_control 插件迁至 `src/rt_control/runtime_log_recorder`，列入 integration 层。
- 保留所有现有改动、插件名称、接口、共享内存 ABI、波形计算及缓存行为；
  仅修正安装、测试、基准工具和文档的目录引用。
- 部署脚本从实际目录渲染 systemd WorkingDirectory，保留既有用户环境与 realtime drop-in；
  服务仍为 `runtime-log.service`，归档仍为 `~/runtime-log/runs`，主机 overlay 不变。
- 定向构建重新生成当前 v3 安装中的插件清单/头文件链接和 CMake 源目录，
  后续构建不依赖旧的独立 ROS 包目录。
- 旧工作树在删除前保存 Git 历史和未提交文件的 tar 备份及 SHA256 清单；
  证据保存在外层工作区的 `records/runtime-log-migration-20261029/`。
- 新导入的测试样本清理行尾空白；CMake 对所有目标补齐仓库要求的告警基线；
  两个 trace 脚本注明 root 读取与用户写出重定向的原有语义。

## 验证

- 新路径 `python3 -m pytest -q -k 'not test_concurrent_reading_never_returns_torn_slots'`：
  119 passed、1 deselected。该并发压力用例此前已因运行时采样数不足而不稳定，本次未修改或运行。
- 独立 build/install 的 ROS 插件构建成功；`colcon test-result --verbose`：4 tests、0 errors、0 failures。
- 组合器 `test_runtime_log_recorder.py`：8 passed；安装后的 ament 插件清单解析到新源码目录。
- 当前 v3 工作区以 `--cmake-clean-cache --symlink-install` 定向构建成功；安装清单链接指向新源码。
- 相关 Shell 脚本 `bash -n`、`shellcheck -S warning` 通过；Python compileall 通过。
- 安装后 `systemd-analyze --user verify` 通过；日志服务从新 WorkingDirectory 运行。
- 索引页、live 页、历史实验页 HTTP 200；历史全程底盘波形返回 600001 个采样、
  491154 个有效反馈采样，gzip 41518 字节。
- `git diff --check` 和 `git diff --cached --check` 通过。
- `tools/quality_gate.sh` 迁移前后均被已有的
  `src/rt_control/lpms_nav3_can/CMakeLists.txt` 目录级告警基线缺失阻断；未改动该用户文件。
- 本次未重新启动底盘、未验证运动；底盘已停止时，日志服务验证后恢复停止，继续由 chassis 管理。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 日志站源码归属 `tools/runtime_log`；共享内存记录插件归属 `src/rt_control/runtime_log_recorder`。
- F2: 用户服务名称、18200 端口、归档目录、主机配置和插件 ABI 保持原行为。
- F3: `chassis start/status/stop/restart` 通过原有 service 名称联动两个网页，迁移不改变该联动。

## 遗留

仓库总门禁仍有既存 lpms_nav3_can 阻断。测试压力用例的稳定性与硬件运动验收不属于本次迁移。
基准工具在新增告警基线下存在 size_type 到 double 的已有转换告警，未改变其计算行为。
