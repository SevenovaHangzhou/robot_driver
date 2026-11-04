# ros2_canopen patches

Listed explicitly, in this order, by `tools/bootstrap_native_dev.sh` and `docker/rt-control/Dockerfile`.
Numbers are never reused or renumbered, so existing references stay valid. The gaps are retired patches:
the previous `0001-rt-control-lifecycle-and-emcy-stop`, `0003-quiesce-callbacks-before-driver-removal`,
`0004-name-canopen-master-loop-thread` and `0005-derive-motor-topology-from-hardware-info` were
removed with the legacy track runtime in 8d61e8b (ELECTRI-127/133, #48); the current `0001` replaced
the old lifecycle patch.
