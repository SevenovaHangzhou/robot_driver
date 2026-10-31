# integ_diagnostics

RT-Control 自检报告与诊断（设计稿：`domains/rt_control/docs/operator-web-and-diagnostics-design.md` §5）。

- `integ_diagnostics.report`：自检报告 `$RT_CONTROL_STATE_DIR/self_check/<作用域>.json`（默认 `/tmp/rt-control/`）的唯一写入与路径规则。阶段 A 由 `integ_bringup` 的作用域运行栈启动文件在总线启动前写入；操作网页诊断页只读。
- 诊断节点（阶段 B、被动上报、`/rt_control/readiness`）在后续 PR 加入。
