# main 接入基线（2026-11-03）

2026-11-07：当前集成已迁移至 `cbf4116` / 公共接口 `aa82a77`，使用当前五字段 Rolling 状态与官方 RollingSender。远端网页已部署并验证共享控制权，已启用真机 Mink 键盘入口；当前实际 artifact 为 `bench_dual_arm_head_wrist_force`、Domain 77。未执行真机模式切换或运动验收。详见 [当前控制权与部署](CONTROL-AUTHORITY.md)。下文保留之前的对齐记录。

以下历史开发基线为 `origin/main 689ef343ded9488f452ba9c6dff3c1579e291c10`，
公共接口保持正式 pin `c82f0e40937453f1f8fb104822c2e91e9fb8d742`。
相较上次检查的 `69d7273`，新增诊断节点与离线错误码更新，网页/仲裁接入位置没有再次迁移。

## 当前交付物

主操作台补丁在 `integration/shared-arm-authority.patch`；
`shared-authority-reference.json` 固定 driver 基线、每个修改文件的前后 SHA-256、客户端文件及验证边界。
实际工作树为 `/home/zlz/Desktop/ing_obj/frame/robot_driver_shared_authority`，
分支 `codex/shared-arm-authority`。原 driver 的用户改动工作树未被覆盖；无提交、推送或远端部署。

```bash
.venv/bin/python -B tools/check_shared_authority.py
```

该命令在临时目录复原基线文件，检查并应用补丁，再比对全部生成文件与实际工作树；不操作 ROS 或运行服务。
若后续 main 改动这些文件，应先重做合并与验证，不能因旧补丁检查通过就覆盖新代码。

## 接入位置

新主分支将旧 `rt_control_arm_console`、底盘和头部网页整合为 `integ_operator_web`。
`/arm` 与新 `/ws/jog` 共享 `ArmConsole.authority`；18131 代理认证与权限请求，
真机反馈仍用于本地整机观测。详细协议与交接检查见 [统一控制权](CONTROL-AUTHORITY.md)。
旧 `console-ui.patch` 等补丁针对已删除的包，仅作历史保留。

现有关节页保留原关节执行器；末端运动没有接入 JTC。新末端入口只开放权限协议，
实际 Mink → Rolling 运动仍等待执行层验收、真机模型和硬件包络。

## 公共接口与历史验证

先前 `main-reference.json` 固定的是 `69d7273` 的 18 个 driver 文件和 15 个公共接口依赖，
保留其历史指纹与 181 项回归结果，不作为本次已修改源码的清单。
相比原 pin `ea582c8a711874813d2ad7f0801b366bd32ab67e`，新 pin 只改变公共错误码和 changelog；
记录的 Rolling 服务、消息与 QoS 均字节一致。

新公共类型在 `.runtime/interfaces-c82f0e4/` 直接从固定提交生成构建；未向源码复制或改写 IDL，
未将生成包安装到真机。原项目内的独立 `ArmAuthority` 草稿及 7 项测试已迁移到主操作台包，
因此旧验证命令中的 `tests/test_authority.py` 不再使用。

## 当前验证和部署边界

`integ_operator_web` 及其本机构建依赖构建完成，ROS bridge/node 导入通过；
CTest 297 项、无失败无跳过。代理、只读观测和 HTTP 27 项、键盘输入 10 项通过，
两个实际网页验证互斥、交接与完整模型渲染，无脚本错误。质量门禁结果及限制见统一控制权文档。

这些是本机验证。远端旧版 18121 与 Domain 78 的运行栈仍须按现场交接统一，
当前未申请真机控制权、使能、复位、切控制器或发送动作。
