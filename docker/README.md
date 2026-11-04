# 容器、封装与发布

按需查阅。硬件与宿主授权底线见根 [`AGENTS.md`](../AGENTS.md) 第 5 节。

## 容器边界

- 生产容器必须以安装后的 `rt_control_start` 作为 PID 1；不得用 `ros2 run` 包裹，也不得直接启动 launch 文件绕过失能等待。
- Compose 一律通过 `tools/rt_control_compose.sh` 调用。`RT_CONTROL_CPUSET` 与 `RT_CONTROL_START_CPUSET`
  必须分别来自目标机验证，不得设置仓库默认值或从彼此推导。
- 不得增加 `privileged: true`，不得扩大设备映射、capability、主机目录写权限或 DDS 暴露范围，除非有明确需求和风险评审。
- 镜像依赖必须锁定且可追溯；仓库、镜像环境变量、构建日志和文档中不得出现代理凭据、令牌或私有地址。
- 正常停机走 `docker stop`；不得用 `docker kill` 或 `kill -9` 绕过 PID 1 的失能流程。

## 封装不随源码 PR

`main` 的普通源码 PR 不要求同步构建 Docker 镜像或提供容器启动证据。Dockerfile、Compose 和容器入口是发布资产，
不默认代表 `main` HEAD 的封装状态。只有人工封装任务明确记录 source SHA、镜像身份并完成容器验证后，该镜像才是部署候选。

改动 `docker/**`、Compose、镜像依赖或正式发布资产时，执行：

- 分别设置经验证的临时 `RT_CONTROL_CPUSET` 与 `RT_CONTROL_START_CPUSET`，用 `tools/rt_control_compose.sh config` 检查展开结果；
- 相关脚本 `bash -n`，可用时 `shellcheck`；
- 人工封装/发布任务另需完整镜像构建与容器内无硬件启动。

## 发布

版本号 `vMAJOR.MINOR.PATCH`：破坏性接口或 Robot Model 变更、域边界调整升 MAJOR；新增非破坏性能力升 MINOR；修复与文档升 PATCH。
阶段交付可加后缀（`-rc1`、`-demo`）。

1. 从选定 `main` SHA 切 `release/vX.Y`，冻结功能；
2. 由人工完成镜像构建与整机 Compose 启动；
3. 执行整机验收与 vendored `contract/views/rt_control.md` 边界验收；
4. 记录 release manifest：镜像 tag 与 digest、Robot Model 版本/哈希、interfaces 版本、标定版本、冻结上游 SHA；
5. 打 tag（打出后不得移动或删除）并归档证据：构建日志、启动与健康检查、实时循环时序实测、未验证项。
