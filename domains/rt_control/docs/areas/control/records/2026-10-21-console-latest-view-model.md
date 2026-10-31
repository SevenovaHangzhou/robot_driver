# 网页操作台接入指定最新显示模型

- 任务：R2S-VIEW-MODEL-20261021；所属域/功能区：RT-Control / arm-control。
- 包：rt_control_arm_console；直接消费者：浏览器操作台。未变更跨域 ROS 接口。
- 阶段：T0 假驱动验证与网页部署验证；风险：显示与控制模型版本暂时不同，需明确边界。
- 用户指定权威目录：`/home/kkozia/桌面/ing_obj/sim/robot_description`。
- 版本：`5538a004125756d38c73c859dd8cad9780153590`。
- URDF：`urdf/robot_alfav3_mobile.urdf`。
- SHA256：`db2522f4e82796571f16061a0babe75bb3f8e650b9ae2e00bb46cae27af457b6`。

## 行为与边界

原页面从 controller_manager 获取 URDF，更新磁盘文件不会替换运行中缓存。
新增 `--view-model-share` 与可选用户级链接
`~/.local/share/rt-control/view_robot_description`，让网页读取指定权威发行的几何与网格。
本地仅更新源文件和链接，没有启动常驻网页服务。
远端发行位置为 `/home/ar/robot-description-releases/5538a004125756d38c73c859dd8cad9780153590`。
网格不复制进网页目录，发行内容与权威源一致。

显示模型与运行模型分开：关节命令、运动限位、速度准入仍使用运行控制器模型；
不更新标定、PDO/SDO、生成产物、控制器或硬件参数，不执行实机运动。
最新模型含左 J5 朝向和右臂轴向等变化，本任务只更新显示消费者；
不据此宣称全域 Robot Model 升级或运动学联合验证完成。

`/api/info` 暴露显示模型版本；版本化网格路径保留 .stl/.dae 后缀，兼容 URDFLoader。
未收到运行关节数据时先显示整车，随后重试关节信息；重复加载前移除旧场景对象。
模型不兼容时返回 503，不回退旧几何。未配置新来源时保持旧行为。

## 验证

- 操作台包假驱动/HTTP 测试：78 passed，日志 `/tmp/arm-latest-model-tests.log`。
- 新测试覆盖显示模型与运行限位隔离、未提供运行模型时加载、资源认证、
  不兼容关节拒绝和网格清单变更导致缓存路径变化。
- 使用实际权威目录的临时 HTTP 服务：32 links，67 个唯一可视网格 URL 全部返回 200，
  内容逐一与权威源哈希一致，所有 joint XML 一致；无运动请求。
- 权威模型清单：84 个引用的网格已按清单校验。
- app.js/viewer.js 模块语法检查通过；git diff --check 通过。
- 权威模型自身测试：5 passed / 1 现存失败，`test_confirmed_zero_conventions`
  仍断言旧右 J5 旋转；保留用户指定 URDF 原样，未为测试修改模型。
- 全仓质量门禁受缺失 jsonschema 阻塞；未完成全域/实机运动验证。
- 无可用浏览器自动化工具，未声明 WebGL 视觉验收完成。

## 备份与回退

本地备份：`/home/kkozia/robot_driver-sync-backups/view-model-20261021-193955`，
`before/` 保留本次改动前文件。远端修改前另存逐文件备份与哈希。
回退仅恢复本次网页文件及用户级模型链接，确认安装路径链接关系；
无运动、无序列时仅重启网页 Python 进程，不能以 `arm console` 替代（它也重启 guard）。
历史位姿、预设、音效、控制配置与已有用户修改保留。未提交或推送。


## 远端部署结果（2026-10-21）

源码与安装入口经resolve核实：Python位于
`install/local/lib/python3.10/dist-packages/rt_control_arm_console`，
网页位于`install/share/rt_control_arm_console/web`，均链接到源码。
逐文件核对旧哈希后更新9个源/文档文件，并建立远端用户级模型链接。

重载前SSE确认 moving/sequence 均为空、无使能切换、使能状态DISCONNECTED、控制器不active。
仅网页进程从6696重载为44099，保留原环境、参数、日志并更新当前run的console.pid。
运动guard 6676的进程启动身份前后相同；未重启控制栈或发送运动/使能/复位命令。

远端18121登录后API返回上述URDF哈希，32连杆与全部joint XML一致，67个唯一可视网格
逐个HTTP 200且哈希与发行一致。资源版本为
`c63ef5d163b68be27e10d736c91f919e41087a6c2d918b7fcc276014bb75d2b6`。
运行关节数为0、状态DISCONNECTED，因此此时模型显示零位并等待运行反馈，不能宣称实测姿态就绪。
远端备份为`/home/ar/robot_driver-sync-backups/view-model-20261021-193955`，
包含manifest.json、before/、restart.json及verification.json；本地备份另存remote-verification.json。
操作台skill同步补充模型来源、显示/控制边界、安装路径和更新验证方式，quick_validate通过。
