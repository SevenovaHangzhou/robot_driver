# Mink → Rolling 本地验证（2026-11-03）

验证范围为本地 Python 客户端、本地协议模拟器、真实 Mink/DAQP 求解及 MuJoCo 物理模型；未连接硬件，未修改 robot_driver 或公共接口，未提交、推送或部署。

## 静态重力接入后的回归

默认执行端已改用模型匹配的 `static_gravity`，完整回归 364 passed、1 skipped（缺 ROS 生成类型），前端输入测试 10 项通过；同进程网页与原生 MuJoCo 联调通过。补偿在运动、正常减速和静止保持阶段持续更新，计算故障锁定后仍能完成 Rolling 会话清理。详情与实机边界见 [静态重力补偿](GRAVITY.md)。以下保留前次 Rolling 适配验收记录。

## 自动化

```bash
JOG_TEST_URDF=/home/zlz/Desktop/ing_obj/sim/robot_description/urdf/robot_alfav3_mobile.urdf \
  .venv/bin/python -B -m pytest -p no:cacheprovider -q
```

全量结果：**352 passed, 1 skipped，94.05 s**。跳过项是没有加载权威生成消息环境的旧 ROS transport 模块；本次不据此声称 ROS/DDS 或 C++ controller 联测通过。

浏览器联调后消除整条减速尾段的重复 TCP/几何检查；仍在每个窗口发布前检查该窗口。优化后的相关回归：

```bash
.venv/bin/python -B -m pytest -p no:cacheprovider -q \
  tests/test_mink_rolling.py tests/test_rolling_stream.py \
  tests/test_rolling_transport_recovery.py
```

结果：**64 passed，19.13 s**。Node 键盘状态机 **10 passed**；`web/jog.mjs` 语法检查、两个启动脚本的 `bash -n` 和变更 Python 文件的 AST 解析通过。

重点覆盖：

- 双臂六方向连续越过 5 mm；八档速度、14 轴映射及另一臂 hold。
- 两分钟单会话续流与有界缓存；至多一批未确认、ACK 前不提交候选。
- 改速/换向未来替换，保留旧前缀、q/v/a 接缝连续；段内极值、jerk 和同步制动空间拒绝检查。
- 正常松键在速度/加速度归零后 STOP/FINALIZE；输入过期、旧 epoch、拒包、迟到、boot 变化、未知服务结果不续动。
- 持续物理异常仍能完成协议清理；运动中 close 先关闭会话再退出 worker。
- 永久丢失 STOP 响应时，FINALIZE 确认后回收同会话服务请求；重复等待同一个 Future 不累积回调，其他会话不被回收。

## 浏览器与原生窗口

通过 `script/start_sim.sh` 启动默认 `mink-rolling`，网页为 `http://127.0.0.1:18131/jog`；原生 MuJoCo 窗口与网页使用同一进程及 MjModel/MjData。

独立 Chromium 自动化页面实际登录、加载完整机器人、获取控制权并开启键盘：

- 长按前方向约 2.2 秒，实际 TCP 前进 **11.4623 mm**，批次序号为 **28**；连续越过 5 mm，未重开会话。
- 长按期间将速度从 10 改为 5 mm/s，会话 UUID 保持一致，命令 revision 和未来替换次数增长。
- 松键后保持 armed，最终 busy=false、fault 为空、会话 complete=true；随后主动停止并释放测试页控制权。
- 网页反馈源为 `mujoco_actual`，完整模型加载成功，JavaScript pageerror 数量为 0。截图保存在 `.runtime/mink-rolling-browser.png`；本地复验脚本为 `.runtime/mink-rolling-browser-check.cjs`，依赖本机现有 Chromium/Playwright。

首次浏览器联调发现减速全尾段校验拖慢循环，已修复并复验；最终成功结果对应按发布窗口校验的实现。

## 证据边界

参考的 controller 源码及 public pin 记录在 `integration/mink-rolling-reference.json`；其中明确记录工作树包含用户既有改动及本次未验证 generated types/C++ 联测。历史 `integration/rolling-reference.json` 保留原样。

后续 main 对齐已改用提交对象核对：`69d7273` / 正式公共 pin `c82f0e4`，记录在 `integration/main-reference.json`。新生成类型下相关回归 181 项通过，仍未完成 C++ 控制器联测或真机 Cartesian 验证，详见 [main 对齐记录](MAIN-ALIGNMENT.md)。

客户端关节动态与制动包络仍是原离线候选限制。真实 controller 独立解析检查、方向性包络、RT 消费、动态低水位、反馈时效和现场碰撞/执行仲裁不由本地模拟器证明。正常规划段保持 C2 且 jerk 有界；异常 controller 制动不具有这一承诺。
