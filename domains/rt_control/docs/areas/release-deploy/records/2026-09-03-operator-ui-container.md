---
id: release-deploy-20260903-01
area: release-deploy
title: 操作员 UI 最小权限容器封装
date: 2026-09-03
type: feature
trigger: contract-20260903-01
commits: [feature/rt-control-operator-console]
env: docker
risk: T1
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: []
supersedes: []
related: [contract-20260903-01, BQ-141]
---

## 背景

目标分支为 `main`，因此 Qt 工具不能只在宿主 symlink-install 中运行，必须进入同一可复现镜像，
同时避免把控制容器的设备和实时权限复制给显示进程。

## 改动

生产 Dockerfile 将 `rt_control_operator_ui` 加入显式包闭包，并在构建末尾实际导入 Qt binding；
新增 `docker/operator-ui.compose.yaml`，服务使用 UID/GID 1000、显式 housekeeping cpuset、同一可配置
ROS Domain、host network/ipc 和只读 X11 绑定，设置 `cap_drop: ALL`、`no-new-privileges`、
`restart: no`，不映射 EtherCAT/CAN、Docker socket 或其他设备。

## 验证

- `python3 -m pytest -q tools/tests/test_operator_ui_packaging.py`：PASS，2 tests。
- `docker compose -f docker/operator-ui.compose.yaml config --quiet`：PASS。
- rebase 到 `main@b80f6c1` 并补齐双 X503 兼容后，使用一次性构建代理完成 IgH
  fixed-PDO 补丁及 29 packages 构建；`a0f3286` 的最终审查镜像
  `rt-control:operator-ui-mainline-review` 为
  `sha256:53ce267e974b3e8297bd79257246330259062f0f58efda70476eb4d3e60436da`。
- 镜像内 `python_qt_binding` 导入、UI executable、TI5 catalog 及 PyQt5 ELF 文件尺寸检查：PASS。
- `--network none`、offscreen、Domain 232 UI 烟测：PASS；SIGTERM 后无 KILL、traceback 或残留容器。
- 镜像 runtime environment 与 history 检查：无代理环境变量，无本次代理 URL。

未执行：目标机 X11、UID/GID、housekeeping CPU 集和 DDS 联通；没有启动生产控制服务。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: UI 容器必须与 RT-Control 使用同一批准镜像版本，但独立启动且不获得任何硬件设备或实时 capability。
- F2: UI 服务默认非 root、`restart: no`、`cap_drop: ALL`，CPU 集必须由部署显式提供且只能选 housekeeping CPU。
- F3: 镜像构建必须实际导入 `python_qt_binding`，仅安装包或仅通过宿主测试不足以声明镜像可用。

## 遗留

该 tag 仅是本地评审镜像，不是发布版本。目标机仍需确认 X11 authority、UID/GID 1000、housekeeping
CPU 集和同 Domain DDS 联通；现有 Dockerfile 的可变 `ros:humble-ros-base` tag 本轮解析为
`sha256:75dd3aba34a3838dadbb31a9f7bef769bdfa8713e6cec686fc868db2981b0987`，发布前应由独立基线任务固定 digest。
