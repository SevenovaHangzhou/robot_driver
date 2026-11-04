---
id: lifecycle-20261009-02
area: lifecycle
title: 达妙头部通信与调参会话归档
date: 2026-10-09
type: commissioning
trigger: 用户要求将此前内容写入README并将两台配置速度改为0.2rad/s
commits: []
env: native
risk: T0
writes: { reset: no, enable: no, motion: no, plc: no }
verified: PARTIAL
evidence: []
supersedes: []
related: [lifecycle-20261009-01]
---

## 背景

用户要求归档此前头部调试内容。本次仅整理已有会话证据，不进行新的硬件操作。

## 改动

扩展damiao_head_controller/README.md：通信接口、模式与ID、编译启动、域12环境、生命周期服务与位置话题、CAN寄存器/零位/Flash帧、故障排查、PI试验和实时曲线脚本。临时配置两台velocity_limit统一为0.2rad/s，修正注释；此前实验为0.1rad/s，不混作等条件数据。未重启运行实例。

## 验证

YAML读取确认两台速度均0.2rad/s；README示例通过配置schema离线校验；git diff --check通过。完整质量门禁仍受现有build_head/log_head生成物扫描问题阻塞，未删除用户构建目录。历史实机证据来自会话记录，不声明新实机验证。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: 用户请求的当前临时YAML两台速度限制为0.2rad/s，已运行实例需要重启加载。
- F2: 位置P54/48.6历史对比为0.1rad/s，未显示明显改善；速度环试验未完成，最后读回速度P仍为0.00372。

## 遗留

README列出了机械范围/轴向、0.2rad/s加载与验证、速度环试验、Flash断电保持和桌面绘图待验证项。临时脚本和数据仍在/tmp，不是永久部署资产。
