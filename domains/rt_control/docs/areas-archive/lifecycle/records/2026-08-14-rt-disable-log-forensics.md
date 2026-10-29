---
id: lifecycle-20260814-01
area: lifecycle
title: /rt/disable 缺陷声称的日志取证：症状真实、机制误诊（ELECTRI-92 第一步）
date: 2026-08-14
type: investigation
trigger: ELECTRI-92（独立复现被丢弃改动 T-DEV-NATIVE-003 声称的三项缺陷）
commits: []
env: native
risk: T2
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PASS
evidence: [工控机 /home/ar/rt-control-dev/log/native/（194 个日志，2026-07-30~08-13），重放命令清单见 ELECTRI-92 评论]
supersedes: []
related: [ELECTRI-92, ELECTRI-93]
---

## 背景

被丢弃的 T-DEV-NATIVE-003 声称"从目标机日志复现"三项缺陷。本次只读取证独立
核验该声称（journalctl、/var/log、~/.ros/log、194 个 native 启动日志、docker
容器日志全部覆盖）。

## 改动

无任何改动（严格只读取证，全程 nice/ionice，事后核验目标目录 mtime 未变）。

## 验证

对三项声称逐一核验：

1. **超 30s 预算（声称的串行叠加机制）——症状真实，机制被证伪**。
   `UNCLEAN_SHUTDOWN: /rt/disable did not return within shutdown deadline`
   在 4 个日志中存在，其中 2 个有服务端丢弃响应的对侧证据（真实挂起）。但
   `rt_disable_once.cpp:155` 本就计算一次绝对 deadline 并贯穿三阶段共享——
   不存在可复现的"串行叠加"；29 个失败日志全部呈现共享 deadline 行为
   （阶段 1 耗尽后阶段 2 立即 unavailable，1:1 无例外）；实测时间线约 30s 而
   非 >30s；且声称必经的 JTC 路径在全部 194 个日志中从未活跃过（whole_body_jtc
   由设计 --inactive，handleDisable 的 switchJtc 分支从未走到）。
   **真实病因指向：服务端 handler 在 waitForResult() 里自旋满 30000ms
   （service_result_timeout_ms），RT 循环未填充结果槽**——是"一个 30s 等待
   客户端等不完"，不是"两个预算相加"。
2. **超时 future 悬挂——代码属实，日志不可证**。switchJtc 超时后未
   remove_pending_request 属实（代码可核），但日志只能看到服务端丢响应的
   镜像证据，无法区分"future 悬挂"与"客户端退出"。定性为代码评审发现。
3. **结果槽无操作身份——结构弱点属实，零日志证据**。ResultSlot 无操作
   id/代际计数属实，但 194 个日志中无任何迟到/错配完成的症状。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `/rt/disable` 存在真实挂起症状（4 例日志证据），当前证据指向服务端
  waitForResult 30s 自旋 + RT 循环未填槽，而非串行预算叠加；ELECTRI-92 的
  修复设计必须以此病因假设为起点重新分析，不得沿用被丢弃改动的因果叙事。
- F2: rt_disable_once 的三阶段共享单一绝对 deadline 是已验证的现状行为
  （29 例日志 1:1 佐证），不是缺陷。

## 遗留

- 声称 1 的串行叠加若要证实/证伪到底，需要一次 drives 实际使能、JTC 活跃的
  实机运行（该路径在留存日志中从未被行使）——归入 ELECTRI-92 步骤 1 的
  剩余工作，需 L3 授权。
- 病因假设（waitForResult 自旋 + 结果槽未填）需在 ELECTRI-93 的 gtest 骨架上
  写单元复现，作为 ELECTRI-92 步骤 2 设计裁决的输入。
