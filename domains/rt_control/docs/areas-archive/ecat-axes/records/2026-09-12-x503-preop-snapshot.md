---
id: ecat-axes-20260912-01
area: ecat-axes
title: X503 参数在 PREOP 一次读取，运行期禁止 CoE 请求
date: 2026-09-12
type: fix
trigger: 用户要求初始化时确认换算参数，OP 后不再读写 SDO
commits: [fix/x503-preop-snapshot]
env: both
risk: T3
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence:
  - /home/kkozia/research_reviews/x503-preop-implementation-20260912/initialization.json
  - /home/kkozia/research_reviews/x503-preop-implementation-20260912/op-workbench-final.json
  - /home/kkozia/research_reviews/x503-preop-implementation-20260912/live-acceptance.json
  - /home/kkozia/research_reviews/x503-preop-implementation-20260912/kernel-installed.json
  - /home/kkozia/research_reviews/x503-preop-implementation-20260912/docker-tests-final.log
supersedes: []
related: [ELECTRI-116, BQ-143]
---

## 背景

两台 X503 在 OP 下读取 CoE 换算参数会间歇返回错误子索引、协议错误或卡满邮箱；
完整停止到 PREOP 后读取恢复。用户明确选择每次初始化读取一次，运行期仅使用 PDO。
本变更规避该访问路径，不把它等同于修复变送器内部 SPI/PDI 问题。

## 改动

- launch 的同步初始化阶段在创建任何控制节点之前，从已选择的 variant/profile 获取
  从站身份、环位和读取配置，在 Idle/Inactive、无错误 PREOP 条件下读取两侧
  `0x8005` 的十进制子索引 `6..17`。每项仅一次，核验 uint32 格式、单位码和小数位。
- 快照通过本次 launch 的只读参数传给桥节点，并绑定新的 startup_id；没有历史文件
  缓存回退。读取失败的传感器保持无效，不输出工程单位数据；状态或身份门禁失败则
  在控制节点创建之前终止初始化。
- 运行桥节点只有 PDO 状态订阅，持有本次快照并持续提供原 calibration topic 的
  transient-local 发布者。首次进入 OP 后若离开 OP、链路丢失或相应状态缺失，快照
  永久失效到下一次初始化，恢复 OP 不会自动重用旧快照。
- 退役原生 `x503_sdo_snapshot` C++ 轮询器及启动分支，兼容命令明确拒绝执行。
  主线包不再通过旧的 ioctl 头访问主站，避免该路径的 ABI 差异。
- IgH 补丁 `0003-preop-only-coe.patch` 在 SDO 传输入口返回 EPERM，并在 CoE 邮箱
  发送入口阻断非 PREOP 请求；自动对象字典读取也只在 PREOP 调度。
  此限制针对 EtherCAT/CoE，运行期 PDO 控制继续使用既有通路。
- Native/host setup 与 Docker 配方携带同一补丁哈希，Native 继续核对实际加载模块
  与磁盘模块的 build identity。工作台的备用读取路径改为仅等待并复用 RT-Control
  的快照，直接读取函数拒绝执行。

## 验证

- TDD：初始化顺序、OP 拒绝、单位/长度错误、旧启动标识、失联后失效及工作台禁止
  备用读取均先复现失败，再实现通过。未自动创建提交。
- 相关 Python 包及 launch 测试：88 passed，分支覆盖率 84%；工控机安装后包测试
  39 passed；工作台相关测试 45 passed。
- 从实际上游补丁重建函数并编译 C 测试，验证非 PREOP 不排队数据报、上传和下载
  都返回正确的权限错误：2 passed。候选内核编译成功，96 个导出符号 CRC 与原模块
  一致；新模块 SHA 为 `bcc7ec970b53f6d23b2bec3469734fe233db2c67f02f437871ae95639598c22f`。
- 固定已有镜像 `45fb2467a1f0...` 上增量构建两包及补丁身份，隔离网络运行容器单测
  和 Mock 契约测试：129 tests、0 errors、0 failures、0 skipped。
- 18:03:35 的 Native 不使能启动：Ti5 固定 1601/1A01 检查通过且无需恢复/重扫；
  控制线程仍为 CPU14/FIFO80、EtherCAT-OP 为 CPU14/FIFO79。
- 初始化内核追踪共 314 个 SDO 传输，全部发生在目标从站 PREOP；其中 X503 两侧
  各 12 项、合计 24 项各读取一次，全部成功。没有非 PREOP 请求。
- 6 秒状态采样中，右/左 Wrench 分别收到 301/300 条，两侧 snapshot_valid=true；
  14 个受管轴通过既有 disabled CiA 402 契约检查。
- 工作台以 mode=reused 接收到两侧有效快照。OP 观察窗口记录 6000 个发送帧，
  SDO/CoE/对象字典事件和邮箱数据报均为 0，追踪无 overrun/drop，域 WKC 为 48/48。
- 最后标准 stop 返回 already_disabled 和 STOPPED；临时追踪已清理。

本次实机操作经用户授权，通过标准 `rt_control_native.sh start/stop` 入口执行；
没有额外故障复位、使能或运动。标准启动原有的 CAN/PLC 初始化仍按原流程执行。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: X503 单位/小数位每次启动只在 PREOP 读取一次，运行期转换只消费本次快照与 PDO。
- F2: 标准 EtherCAT CoE 请求在非 PREOP 状态被主站拒绝，包含自动字典读取路径。
- F3: 失效快照只有下一次初始化能够更新，工作台不能在 OP 自行补读参数。
- F4: 两侧已实际读回小数位 `[1,1,1,3,3,3]`、单位码 `[5,5,5,7,7,7]`。

## 遗留

变送器内部 PDI/SPI 异常的具体固件/电气根因、测量精度、TF 现场标定、长期运行及
运动状态下的验证没有在本变更中闭合。参数更改、设备更换或复电后需要重新初始化。
验证镜像是固定基线的增量构建，完整 release 发布另行跟踪。
本次 PR 同时整理前一轮 Ti5 启动映射恢复改动，按独立提交保留其验证记录。
现场工作台位于独立目录，其已部署的订阅模式调整不属于本仓库提交内容。
