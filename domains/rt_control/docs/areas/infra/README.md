# infra — 不依赖任何域包的共享叶子库

**范围**：被至少两个包复用、自身不依赖任何域包的共享库（`ARCHITECTURE.md` §3 进库条件、门禁 R11）。2026-10-31 新开（用户裁定）。

**Owner路径**：infra_modbus；infra_renderer；infra_shm_ring；infra_socketcan。未写完整前缀的包位于`src/rt_control/`。

**不属于本区**：调用这些库的协议实现归protocol，外设节点归peripheral-control，离线组合归integration；宿主 CAN/网络配置归platform。

更具体路径优先于整包归属；跨区改动按主责拆分事实并互引。

## 冻结事实

| 来源 | 事实 | 原始记录 | 验证状态 |
| --- | --- | --- | --- |
| D-002 | SocketCAN 库只管传输：显式关闭本机自收（`CAN_RAW_RECV_OWN_MSGS`），保留回环（`CAN_RAW_LOOPBACK`）；不做接口名白名单 | [BLOCKED-questions D-002](../../../BLOCKED-questions.md) | 台架头部 can2 复测通过；BMS/IMU 无实机 |
| D-003 | Modbus TCP 长连接；任一错误即关闭、不自动重试；只接受数字主机地址；超时按端点配置 | [BLOCKED-questions D-003](../../../BLOCKED-questions.md) | 本地假服务器测试通过；PLC/485 网关实机未验 |

## 跨区事实关联

`infra_shm_ring` 的共享内存布局是与 Runtime-LOG 仓库共享的契约（进库例外，见 `ARCHITECTURE.md` §3）。协议包用法见[protocol](../protocol/README.md)，外设用法见[peripheral-control](../peripheral-control/README.md)。
