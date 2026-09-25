# ELECTRI-136：机械臂与夹爪惯性参数候选来源

本目录接收机械提供的 `local_mudz3nqx_3qc2bv_urdf_stl` 导出，关联
[ELECTRI-136](https://linear.app/sevenova/issue/ELECTRI-136) 和
[MECHINE-35](https://linear.app/sevenova/issue/MECHINE-35)。

用户接受这批参数作为台架标定前的 CAD 初值。这里保留机械原值，不根据左右镜像推导修正
任何质量、质心或惯量；**接受初值不代表坐标映射已完成或实机已验证**。

## 内容与状态

- `robot.urdf`、`parts.json`、`user_model.json`、`export_report.json` 和 24 个 `meshes/*.stl`
  共 28 个原始文件按字节保留；身份见 `manifest.json` 的 SHA-256 清单。
- `inertial_candidates.json` 从原始数据提取 14 个臂 link 和 2 个 hand 的质量、质心和惯量，
  同时保留 source link 坐标（m）和 source CAD 坐标（mm）；惯量为 kg·m²。
- 每侧七连杆质量 43.146kg，含 hand 为 43.232kg。
- `target_link` 和 `target_from_source_link_transform` 已按用户授权的临时 FK 规则填写：
  左右臂分别对齐 J1/link1 零位 frame，再用两套 URDF 的零位 FK 计算其余变换。
  结果见 `mapping_result.json`；它是可复现的 CAD 初值映射，不是机械测量。
  `runtime_ready=false`、`hardware_verified=false` 保持不变。
- 本来源目录不随 CMake 安装；转换结果已写入 `urdf/` 中的惯性块，随模型正常安装。

## 临时映射与运行模型

接收来源时 RT-Control 构建副本锁定 `robot_description@17f5bdc46b8f2580ee81aed919da7b404da3bdaf`；
本任务分支基于公共模型分支 `robot_v3_suction_chassis@ec69ca04297896c1296720324d23cdb8f80d9e63`。
用户随后明确选择迁移公共 V3.1.1；驱动侧按该完整 SHA 同步构建副本与消费者校验。
该升级包含双臂零位、部分运动学、link 名称、升降零位和其他公共资产变化；
用户随后授权以确定性的临时 FK 映射将这些参数写入公共 V3.1.1 的惯性块：左右臂 J1–J6
替换对应 `*_link1..6`；夹爪变体的固定体和活动夹指分别使用 source link7/hand；吸盘末端不变。
该映射仍需台架重力标定，不改变 `hardware_verified=false`。

本次机械原始导出的 `*_linkN_joint` 与目标公共模型 `*_jointN` 的原点和坐标轴表达不同。
原始 14 臂关节限位统一为 ±1.57rad，不能替换公共模型已经确认的关节限位。
J1–J6 对应网格与公共 V3.1.1 网格的体积比约 0.80–1.33（新 link6 为多 part 组合），
不是可以通过全等网格唯一确认的简单刚体重定位。

临时映射已合入，实机验证前仍需确认：

1. 目标已确定为公共 V3.1.1；临时 FK 映射已应用，台架需确认实机编码器零位与逻辑模型对应关系。
2. source link 到 target link 的刚体坐标变换和部件归属依据。质心使用 `c_target=R*c_source+t`；
   若把惯性轴统一到 target link，则质心惯量使用 `I_target=R*I_source*R^T`。
   平移不会给“绕质心惯量”额外加平行轴项，只有合并/拆分部件时另做质量属性合成。
3. link7/hand 候选只用于这批夹爪装配。不得据此覆盖吸盘变体；不同夹爪零件归属及开度需要核实。

质量与质心精度可以通过台架拟合改善；台架须独立检查零位、单位及临时映射假设，
不能把拟合通过当成坐标定义正确的证明。`mapping_request.json` 保留原始待确认清单，
已采用的16个矩阵以 `mapping_result.json` 为准；两份记录的用途不同。

## 后续标定边界

第一版只补偿机械臂＋夹爪自重，不实现工件负载动态更新；带载提起和卸载仍需纳入后续
分级验收。台架装配完成后，先确认实际关节零位、力矩单位和方向，在纯 CSP 下采集多姿态
静态数据，再拟合质量一阶矩的可辨识组合并进行独立姿态验证。

该来源不解除 `0x6077/0x60B2` 单位与方向、PDO、抱闸时序及 active 准入限制。
本目录不包含上电、使能、运动或总线操作授权。
