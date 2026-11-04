# 宿主安装与验收

本目录脚本会修改 GRUB、systemd、内核模块、网络和总线。没有用户对**目标主机及具体动作**的单独明确授权，
只能做语法和静态审查（`bash -n`、可用时 `shellcheck`；systemd unit 用副本离线 `systemd-analyze verify`），
禁止执行安装、启停或重启。

目标机 CPU、网卡、USB 设备等事实只能来自该主机的验收记录，不得从旧机器复制后未经验证使用。
