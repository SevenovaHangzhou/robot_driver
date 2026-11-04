# rt_control_bringup：研华 V3 底盘现场启动

以下命令适用于本机 `/home/sev_v3/Robot/rt-control/v3` 的 `chassis_only` 部署。
启动运行栈不会自动使能驱动；使能仍由操作员在底盘网页显式执行。

## 启动 EtherCAT 与底盘

### 启动前提

2026-10-21 的故障日志曾显示 `CanController: Operation not permitted`。
实测发现 `can0` 的发送队列是 `qlen 10`，而本机 Lely CANopen 主站要求
`txqueuelen 128`；仅为运行进程增加 `net_raw` 后仍会报同样的错误。
现场已批准将队列设为 128，当前 `v3-restart.sh` 会在启动运行栈前检查并配置
`can0` 的 500 kbit/s、`qlen 128` 和 `ERROR-ACTIVE`，并保留运行时所需的
`sys_nice`、`net_raw` capability。以下步骤已于 2026-10-21 在本机启动验证。

在本机终端执行，先确认主站链路和从站数量：

```bash
sudo systemctl start ethercat.service
/usr/local/etherlab/bin/ethercat master -m 0
/usr/local/etherlab/bin/ethercat slaves -m 0
```

确认主站输出同时包含 `Link: UP` 和 `Slaves: 8`，从站列表有 0 到 7
共 8 台，再继续：

```bash
bash ~/Robot/rt-control/v3/v3-restart.sh
```

主站服务刚启动时可能需要数秒枚举从站。如果仍是 `Link: DOWN` 或
`Slaves: 0`，先检查 EtherCAT 网线、首台从站供电，并重新查询主站；此时不要运行
`v3-restart.sh`。该脚本会执行从站控制字预检，按需配置 `can0`，再启动
`chassis_only` 运行栈。脚本约 45 秒后检查新的启动日志和 ROS 进程；只有
输出 `Chassis runtime active; drives remain disabled until explicitly enabled.`
且退出码为 0，才表示启动检查通过。

查看最新启动日志：

```bash
run=$(ls -td ~/Robot/rt-control/v3/runs/chassis-* | head -1)
grep -E 'All configured EtherCAT slaves|RT_SCOPE_HARDWARE|Configured and activated|ERROR|CanController|Operation not permitted' "$run/launch.log"
ip -details link show can0
/usr/local/etherlab/bin/ethercat master -m 0
pgrep -af 'ros2_control_node|rt_control_start'
```

成功时应看到从站进入 OP、工作计数完整、`RT_SCOPE_HARDWARE_ACTIVE_DISABLED`，
以及 `enable_manager` 的 `Configured and activated`，且 `ros2_control_node`
仍在运行；主站为 `Phase: Operation`、`Slaves: 8`，`can0` 为
`ERROR-ACTIVE`、`bitrate 500000`、`qlen 128`。若出现
`CanController: Operation not permitted`、其他 `ERROR` 或 ROS 进程已退出，
先检查 `can0` 队列长度及最新日志，不要将网页可打开视为底盘已就绪：

```bash
cat ~/.cache/v3-chassis-up.out
tail -n 80 "$run/launch.log"
```

## 启动底盘网页

底盘运行栈与网页是独立进程。网页未运行时，在普通用户终端执行：

```bash
systemctl --user is-active --quiet rt-v3-operator-web.service || \
  systemd-run --user --unit=rt-v3-operator-web --collect \
    ~/Robot/rt-control/v3/start-operator-web.sh
curl --noproxy '*' -I http://192.168.100.28:18110/
```

网页地址是 <http://192.168.100.28:18110/>，登录需要现场配置的操作台令牌。
`rt-v3-operator-web.service` 是临时用户服务，主机重启后需再次启动。网页返回
HTTP 200 只证明入口可访问；底盘状态以运行栈日志和网页状态显示为准。
