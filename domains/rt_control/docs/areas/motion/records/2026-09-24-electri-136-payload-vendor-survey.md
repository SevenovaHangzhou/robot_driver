---
id: motion-20260924-02
area: motion
title: ELECTRI-136 商业机械臂负载切换、渐变与辨识资料调查
date: 2026-09-24
type: investigation
trigger: ELECTRI-136；用户要求扩大调查厂商是否实际支持抓放负载更新及平滑切换
commits: [feature/ELECTRI-136-gravity-ff]
env: none
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: UNVERIFIED
evidence: []
supersedes: []
related: [BQ-152, motion-20260919-01, motion-20260924-01]
---

## 背景

调查公开厂商资料是否支持以下能力：设置工件质量/质心、抓放时更新负载、参数渐变、负载辨识，
以及是否能够据此声称“自动识别接触/离地并实时分配承重”。截至 2026-09-24，读取了
UR、ABB、Doosan、KUKA、Yaskawa、FANUC、Franka 的官方资料；结论限于所列版本和接口。
不将其他厂商的接口、辨识结果或安全配置当成当前自研双臂的硬件事实。

## 改动

只新增调查记录及索引，无代码、配置、公共接口或机器人模型改动。
尚未授权采纳新的生产策略；以下实现建议是待评审建议。

证据对照：

| 厂商/版本或接口 | 官方资料可确认 | 本次不能据此确认 |
| --- | --- | --- |
| UR PolyScope SW5.20 `set_target_payload` | 质量、法兰系质心、惯量；抓放工件后负载属性变化需要更新；可指定 `transition_time`，默认 0 即立即切换 | 自动选择抓放切换时机、在线估计接触承重比例、内部插值曲线 |
| ABB RobotWare 6，3HAC050917-001 Rev.AC | `tooldata` 工具、`loaddata` 工件；`GripLoad piece1/load0` 连接/解除工件；`LoadId` 专门辨识程序 | `GripLoad` 有用户可设的渐变时间、自动接触识别或按实际承重比例渐变 |
| Doosan Programming Manual V3.2.0 | `set_workpiece_weight` 单独设置工件质量/质心、替换/增加/移除、延迟开始和渐变时间；`get_workpiece_weight` 测量读取 | 只调用 set 接口就自动称重、任意型号/安全状态均可调用、连续接触承重估计 |
| KUKA PLC mxAutomation / LoadDataDetermination ≥V7.2 | 负载数据读写；可选包辨识质量、质心、惯量，包含测试运动、测量和写入特定工具的步骤 | 全系列默认配备该选件、后台持续自动辨识、公开的统一负载渐变时间参数 |
| Yaskawa MotoLogix 2.1.0 | 工具质量、质心和惯量；负载含夹爪与工件；写入及选择工具档案有不同接口 | 写入接口可在任意运动阶段/安全配置使用、自动平滑或自动称重 |
| FANUC CRX 官方 Tech Transfer 课程介绍 | 四个推荐姿态的负载估算、schedule number、已知/未知质量选项、线缆影响 | 课程视频内容（未观看）、当前代际 `PAYLOAD[]` 的完整语法/切换执行语义或渐变保证 |
| Franka libfranka 0.15.0 | `Robot::setLoad` 设置工件质量、法兰系质心及惯量；末端执行器参数另设 | 自带可配置切换渐变、自动离地检测；该 API 没有提供上述承诺 |

## 验证

资料级 T0 调查，不是机器人实测或本项目实现验证。

**S1 — UR：直接证实重物抓放的平滑切换。**

[官方脚本手册 SW5.20](https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/set_target_payload.htm)
定义 `set_target_payload(m, cog, inertia=[0,0,0,0,0,0], transition_time=0)`。
质量单位 kg，质心相对 tool mount、单位 m，惯量为质心处 kg·m²。
官方原文：

> This function must be called when the payload mass, the mass displacement (CoG) or the inertia matrix changes - (i.e. when the robot picks up or puts down a workpiece).

> Setting a transition time larger than zero avoids the robot doing a small "jump" when payload changes. This is useful when picking up or releasing heavy objects.

默认 transition_time=0，省略时立即应用；质量和质心必填，惯量和时间可选。
文档另指出更新负载会重置内部 F/T 读数，最终重置发生在过渡结束，若此时正在加速则读数受影响。
因此不能直接套用“切换同时读取 F/T 就得到无偏称重”的设计。

**S2 — ABB：直接证实由抓取程序更新工件负载。**

[官方 RAPID 手册下载，3HAC050917-001](https://search.abb.com/library/Download.aspx?DocumentID=3HAC050917-001&LanguageCode=en&DocumentPartId=&Action=Launch)
实际取得 Rev.AC（©2004–2026）。印刷页 246–247 的 `GripLoad` 示例：

```text
夹紧输出 → 等待夹紧 → GripLoad piece1
释放输出 → 等待释放 → GripLoad load0
```

原示例使用 `WaitTime 0.3`，这里只记录为厂商示例，绝不作为本机抱闸、夹爪或前馈时序参数。
手册说明负载用于后续执行的运动指令，直到再次执行 GripLoad。
印刷页 1694–1696 的 `loaddata` 包含质量、质心、惯性主轴方向及主惯量；用于建立动力学模型。
质心可能在 tool 或 wrist 坐标系表达，受 `PayLoadsInWrist` 系统参数影响，不能统一假定都在 TCP。

印刷页 350–353 的 `LoadId` 是独立辨识过程：在满足机型/姿态条件后做 5/6 轴测量运动，
质量未知时还使用 3 轴运动；之后返回质量、质心及惯量。官方提醒辨识质量受机械差异影响，
需要高精度质量时应称重并按已知质量辨识。

手册还规定在若干程序重置/指针重定位事件中恢复 load0。这说明负载状态与程序恢复流程必须
一致；不能把“某次程序设置过有载”当成断点恢复后的持久真相。

**S3 — Doosan：同时提供工件增减、开始延迟与渐变。**

[官方 V3.2.0 set_workpiece_weight](https://manual.doosanrobotics.com/en/programming-manual/3.2.0/publish/set_workpiece_weight-weight-0-0-cog-0-0-0-0-0-0-co)
明确合成工具与工件的质量/质心，适用于工件类型频繁变化或需要动态改变重量的应用。
参数包括 weight（kg）、cog（mm）、cog_ref（TCP/FLANGE）、
add_up（DR_REPLACE/DR_ADD/DR_REMOVE）、start_time 和 transition_time（s）。
官方原文：

> It is possible to change the weight of the workpiece after the set time through start_time.

> Transition_time allows you to gradually change the weight of the workpiece through transition_time.

相继调用 `set_tool` 和 `set_workpiece_weight` 时，文档要求等待过渡完成，否则可能出错；
改变工具重量会把工件重量重置为 0。
该版本还对自动模式中工件重量修改规定了 Collision Detection 和 TCP SLF 的 mute/deactivated
条件，否则可能触发 SS1（清零有例外）。这是该厂商具体功能的适用条件，不是建议在本机关闭
保护；不能省略此条件而宣称任意状态均可动态改重量。

[官方 get_workpiece_weight](https://manual.doosanrobotics.com/en/programming-manual/3.2.0/publish/get_workpiece_weight)
单独提供工件重量测量读取，并明确 Non-FTS A 型号不可用。读取接口和设置接口是不同命令，
文档没有保证设置命令自动追踪实时接触承重比例。

**S4 — KUKA：负载辨识是有步骤、有选件要求的功能。**

[官方 mxAutomation 页面](https://www.kuka.com/en-gb/products/robotics-system/software/hub-technologies/kuka,-d-,plc-mxautomation)
列出 `KRC_ReadLoadData`、`KRC_WriteLoadData`，以及 LoadDataDetermination 的
`KRC_LDDconfig`、`KRC_LDDtestRun`、`KRC_LDDstart`、`KRC_LDDwriteLoad`。
页面明示测质量、质心和惯量，以优化加速度并避免过载；前提是安装 V7.2 或以上辨识选件。
没有从第三方 `$LOAD` 教程推导激活时序、连续辨识或插值行为。

**S5 — Yaskawa：改档案和选档案的运行约束不同。**

[官方 MotoLogix 2.1.0 Tools](https://motologix.yaskawa.eu/lib-2.1.0/features/tools/)
区分 `MLxRobotSetToolProperties`（写工具数据并选为活动工具）和
`MLxRobotSelectTool`（选择已存在的工具）。参数含 kg、mm、kg·m²，文档明确夹爪场景
应包括夹爪和其负载。写入接口要求 standstill；可先准备多个档案，之后选择已有档案，
避免因临时写入而中断轨迹。FSU 开启时写入接口不支持，而选择接口仍可用并有相应安全选择要求。
文档中对最不利负载档案/加速度的建议，不移植成自研力矩前馈“把质量设大更安全”的规则。

**S6 — FANUC：已确认辨识和档案，不夸大语法与自动化程度。**

[官方 CRX Payload Estimation 课程介绍](https://techtransfer.fanucamerica.com/tech-transfer/payload-estimation-on-crx-for-arc-welding)
描述四个推荐测量姿态、schedule number、已知或未知质量选项，以及线缆约束对结果的影响。
这里只读取公开课程介绍，未登录/观看视频。
[Payload Identification 官方产品页](https://www.fanucamerica.com/products/software/robot/payload-identification)
搜索索引摘要明确使用 controlled motion；正文直接获取返回 403，故不将其扩展细节列为全文核实。
非官方镜像的 HandlingTool 手册遇到人机验证，未用它验证 `PAYLOAD[]` 的代际语法。

**S7 — Franka：工件参数与末端执行器参数分开。**

[官方 libfranka 0.15.0 Robot::setLoad](https://frankarobotics.github.io/libfranka/0.15.0/classfranka_1_1Robot.html#afcb708df10f24563dbcf7d5b907b4a15)
参数为 load_mass（kg）、F_x_Cload（法兰至工件质心，m）、load_inertia（kg·m²）。
官方明确该命令不用于末端执行器参数，后者在管理员界面设置。没有给出可配置渐变时间，
因此不宣称它具有与 UR/Doosan 相同的切换功能。

检索中的非官方教程、论坛及专利仅作线索，不用来证明厂商已交付某项功能。
网页全文读取与 ABB PDF→文本检查完成；ABB 抽取器提示流长度/hints 警告，但上述相关章节完整可读。
本轮文档门禁：`tools/quality_gate.sh` PASS（286 passed、13 skipped，门禁覆盖率 83%；
本机无 ShellCheck，保留 CI 强制检查提示）。`git status --short --branch`、
`git diff --check`、`git diff --stat`、`git diff`、`git diff --cached --check`、
`git diff --cached` 已执行，新增记录已全文自审，YAML/五段模板及索引链接检查通过。
保留上一轮机械参数审查的全部未提交改动；本轮仅新增本记录并更新两份索引。
架构自审：域边界/依赖方向 PASS，公共模型/接口和运行路径无变更，无无关修改或生成物纳入交付。
未获提交/推送授权，未执行 commit/push/PR，也未向厂商或 Linear 发消息。
本次没有运行厂商 API、机器人辨识程序、ROS/Mock、生产进程、设备或硬件操作。
本记录不授权使能或运动。

## 结论与冻结事实

- F1: 工具/工件质量与质心参与动力学负载管理、由应用程序在抓放时更新，有明确商业实现证据。
- F2: 用户可设的负载渐变不只存在于 UR；Doosan V3.2.0 还提供开始延迟、工件增加/移除语义。
- F3: 负载测量/辨识、参数更新、按时间渐变和接触承重比例估计是不同能力。本次资料不能支持
  “所有厂商都自动实时判断离地并按承重比例补偿”的说法。
- F4: 以上公开 API 不能证明各家内部都采用 CSP + 0x60B2 或与本项目相同的前馈线程/公式。
  BQ-152、机械参数验证及本机驱动准入均不因此解除。

## 遗留

建议 ELECTRI-136 后续设计评审采用以下分层，尚未实现或裁决：

1. 保持机械臂/夹爪的已验证自重模型，独立维护工件质量、质心、参考 frame 和来源，避免把夹爪
   质量重复计入。商业接口有“总负载”和“工件增量”两种语义，接口不能只叫 payload 而不说明。
2. 由外部 Motion/抓取执行流程管理取放状态及切换请求；RT-Control 只验证并执行动力学参数更新、
   力矩限幅、渐变和反馈，不接管抓取编排。跨域请求须先在 robot_interfaces 定义版本化契约。
3. 对简单准静态搬运，可研究只渐变工件附加重力项；不把整臂自重补偿一起按抓放比例缩放。
   时间渐变是按计划变化，不证明实际承重，仍需与具体提放动作联合验证；复杂接触可能需要完整
   力/力矩估计，不能用一个质量比例表示任意接触。
4. 先支持已知工件的有载/空载档案及明确的恢复语义，再考虑未知重量测量和在线估计。
   停机/程序恢复后应核实仍在夹持的实际负载，不能无条件清零或继续使用过期值。
5. 两臂若共同搬同一物体，不能给两侧各加一份完整物重，也不能未经验证就固定平分；共同承重和
   内力是另一项建模/控制问题。这是力学分析提示，不是本次厂商文档验证出的统一实现方式。

调查未改变此前双臂机械导出的待确认结论，也没有新增生产负载数值、切换时间或安全阈值。
