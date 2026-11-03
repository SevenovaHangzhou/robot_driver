---
id: integration-20261103-01
area: integration
title: 机械臂与头部共用操作页面
date: 2026-11-03
type: feature
trigger: 用户要求将头部控制和机械臂控制放在同一个网页
commits: [feature/arch-w5-unified-arm-head]
env: native
risk: T1
writes: {reset: no, enable: no, motion: no, plc: no}
verified: PARTIAL
evidence: []
supersedes: []
related: []
---

## 背景

用户要求同屏操作机械臂和头部，不通过独立页面切换。

## 改动

`/arm` 同时显示头部控制区和机械臂控制区，共用原有登录框。头部脚本改为可嵌入的模块，CSS 限定在头部区内；旧 `/head` 地址跳转到 `/arm#head-controls`。两组 WebSocket 和服务端运动校验沿用现状，未改控制器、设备参数或机械限位。

## 验证

从零构建 24 包、全量测试：1125 tests，0 errors，0 failures，1 skipped。真实 Chromium 与本地 aiohttp 测试服务验证：登录框唯一，两区同屏显示，旧地址跳转正确；机械臂模型缺失时头部区仍收到状态；390 像素宽度下头部区无横向溢出。页面初始化和登录没有发送控制命令，模拟环境显式取得及释放头部控制权正常。未发送实机运动目标。

本记录不授权使能或运动。

## 结论与冻结事实

- F1: `/arm` 是机械臂和头部的统一控制页面；`/head` 只保留跳转。
- F2: 页面合并不等于运行栈合并；两个控制区的可用性仍由运行栈实际发布的能力与反馈决定。

## 遗留

双臂与头部同时在线仍需对应的组合运行配置，本次未改变总线、实时线程或使能管理器。
