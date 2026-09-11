# EtherCAT Timing Patch Promotion

The tested timing-support patch has moved to
[`../0011-dc-rate-diagnostics.patch`](../0011-dc-rate-diagnostics.patch).
Native and Docker both include it with the approved 1 kHz configuration,
continuous handoff, common shift 0 and explicit send interval. Diagnostic
activation delays remain opt-in and are not part of normal startup.
Patch 0012 follows 0011 to add license headers and satisfy the upstream
copyright/cpplint/uncrustify checks; it does not change timing settings.

`0011-dc-rate-diagnostics.patch` applies after ICube patches 0001 through
0010 on frozen commit 1390be742986f4e898ca112e49bb24805be9899a. It provides bounded, pre-activation diagnostic
settings and allocation-free cycle histograms exported only after stopping:

- `RT_CONTROL_ECAT_ACTIVATE_AT_NS`: absolute CLOCK_MONOTONIC activation
  deadline, strictly in the future and within 60 seconds at the wait point.
  Signals abort the wait before activation. The application time is sampled
  after waiting. This enables reproducible stop-to-activation experiments.
- `RT_CONTROL_ECAT_SYNC0_SHIFT_NS`: common explicit nonnegative SYNC0 shift,
  strictly smaller than the configured period. Unset retains the old phase.
- `RT_CONTROL_ECAT_EXPLICIT_SEND_INTERVAL=1`: supply the actual configured
  send period to IgH before activation. Unset retains prior behavior.

The existing raw send CSV still freezes after 256 control sends. Additional
`.stats.csv` and `.hist.csv` files cover the full period with tracing enabled.
Histogram bins are one microsecond wide, with an overflow bin at >= 16 ms.
These are software send-call intervals, not PHY timestamps or motion following
errors. DC curves and AL state transfers must be assessed separately.

Controller-manager update_rate, the EtherCAT Xacro control_frequency and
Ti5/Updown interpolation-time configuration must agree. The timing regression
in test_hardware_composition_contract.py checks that contract. This patch
alone does not qualify enabled motion or long-term 1 kHz operation.
