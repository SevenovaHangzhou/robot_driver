# XMC SW 5.11 Updown 固定 PDO 映射

本文冻结环位置 15 的 XMC Updown 在实机固件 SW 5.11 下的主站配置。运行时权威源是实机 SII/PDO 扫描，不是供应商提供的 `XMC_ESC_240909(a).xml`。

## 身份与周期

| 项目 | 固定值 |
| --- | --- |
| Alias / Position | `0 / 15` |
| Vendor ID | `0x0000034E` |
| Product code | `0x00445566` |
| Revision | `0x00000000` |
| Hardware / Software | `1.0 / 5.11` |
| 模式 | CiA 402 CSP (`0x6060=8`) |
| 周期 | 4 ms / 250 Hz |
| DC AssignActivate | `0x0300` |
| PDO 能力 | `PdoAssign=false`、`PdoConfig=false`，不得重映射 |

主站运行配置由 [`xmc_updown_sw511.yaml`](../../../src/rt_control/robot_hw_ethercat/config/slaves/xmc_updown_sw511.yaml) 直接描述，并由 [`ecat.ros2_control.xacro`](../../../src/rt_control/robot_hw_ethercat/urdf/ecat.ros2_control.xacro) 在 position 15 实例化。IgH/ICube 这条路径不需要把 XML 复制进类似 TwinCAT 的 ESI 目录；XML 仅作为供应商资料和差异证据保存。

## 固定 RxPDO `0x1600`（主站到驱动器）

| 顺序 | 对象 | 类型 | 位宽 | 本系统用途/周期值 |
| ---: | --- | --- | ---: | --- |
| 1 | `0x6040:00` Controlword | `uint16` | 16 | enable manager 控制字 |
| 2 | `0x6071:00` Target torque | `int16` | 16 | 未暴露，固定 `0` |
| 3 | `0x60FF:00` Target velocity | `int32` | 32 | 未暴露，固定 `0` |
| 4 | `0x607A:00` Target position | `int32` | 32 | JTC 的 Updown 位置命令 |
| 5 | `0x6081:00` Profile velocity | `uint32` | 32 | CSP 不使用，固定 `0` |
| 6 | `0x6060:00` Mode of operation | `int8` | 8 | 每周期保持 `8`（CSP） |
| 7 | `0x2302:00` Vendor control | `uint16` | 16 | 未暴露，固定 `0` |

总长度为 `152 bit = 19 byte`。

## 固定 TxPDO `0x1A00`（驱动器到主站）

| 顺序 | 对象 | 类型 | 位宽 | 本系统用途 |
| ---: | --- | --- | ---: | --- |
| 1 | `0x6041:00` Statusword | `uint16` | 16 | enable manager 状态字 |
| 2 | `0x603F:00` Error code | `uint16` | 16 | 注册到 process image，暂不导出 ros2_control 接口 |
| 3 | `0x6078:00` Current actual value | `int16` | 16 | 同上 |
| 4 | `0x606C:00` Velocity actual value | `int32` | 32 | 同上 |
| 5 | `0x6064:00` Position actual value | `int32` | 32 | Updown 位置反馈 |
| 6 | `0x6061:00` Mode display | `int8` | 8 | 注册到 process image，暂不导出 |
| 7 | `0x6000:00` Vendor status | `uint8` | 8 | 同上 |
| 8 | `0x2300:00` Vendor status | `uint16` | 16 | 同上 |
| 9 | `0x2301:00` Vendor status | `uint16` | 16 | 同上 |
| 10 | `0x60FD:00` Digital inputs | `uint32` | 32 | 同上 |

总长度为 `192 bit = 24 byte`。

“暂不导出”不等于从 PDO 删除：固定映射中的每个对象都必须按原顺序和位宽注册，否则后续字段会错位。当前业务只声明 position/statusword 两个只读状态接口，避免在未冻结语义前形成新的公共控制或诊断契约。

## 启动 SDO 与位置换算

启动进入周期控制前写入：

| 对象 | 类型 | 值 | 含义 |
| --- | --- | ---: | --- |
| `0x10F1:02` | `uint16` | `100` | 4 ms 周期下沿用约 400 ms 同步错误计数宽容策略 |
| `0x60C2:01` | `uint8` | `4` | 插补周期值 |
| `0x60C2:02` | `int8` | `-3` | 插补周期指数，`4 × 10^-3 s` |
| `0x6060:00` | `int8` | `8` | CSP |

机械换算由用户和供应商共同确认：65536 counts/rev、无减速、丝杆导程 10 mm/rev、`0x6064` 增大表示向上、raw 0 对应 0 m。因此：

- `6553600 counts/m`；
- `0.000000152587890625 m/count`；
- 目标范围 `[0.0,0.8] m`，对应 raw `[0,5242880]`；
- 多圈绝对位置掉电保持，抱闸由驱动器管理。

## 与供应商 XML 的已知差异

供应商 XML SHA-256 为 `4c295d61e87675652e9ba4df2b8e0970cd6bd7a4cbeae9927d04202add0a2b46`。它与 SW 5.11 实机至少有两处运行映射差异：

- XML 的 RxPDO 末项是 `0x607F:uint32`，实机是 `0x2302:uint16`；
- XML 的 TxPDO 额外包含 `0x6077:int16`，实机没有。

直接按 XML 生成会得到 21/26 byte，而实机需要 19/24 byte，造成 PDO 注册/Working Counter 门禁失败或字段错位。因此本版本明确按实机固定 PDO 生成。好处是与当前 SW 5.11 字节布局一致；弊端是固件升级、驱动器更换或供应商重新烧录后都必须重新读取 SII/PDO，不能假定此 profile 仍兼容。

## 实机放行门禁

本配置完成静态、构建和 Mock 验证不等于实机通过。按风险逐级完成：

1. 只读核对 position 15 身份、HW/SW 和 19/24 byte 固定 PDO；
2. 无使能启动，确认启动 SDO 成功、16 个位置响应、全部所需从站进入 OP 且 WC complete；
3. 确认当前位置预装载后，验证 `/rt/enable` 第五批 Turn + Updown；
4. 在机械防护和实体急停条件下执行单独批准的低速、小位移 14 轴完整 FJT；
5. 验证取消、整组 Fault 停车、整组 reset、失能和主站退出终态。

Updown 的 `0.3 m/s` 与 `0.5 m/s²` 是 motion 时间参数化和验收上限。当前 Humble JTC 不会从 `joint_limits.yaml` 自动执行这两个上限，机械安全链也不能由软件限值替代。
