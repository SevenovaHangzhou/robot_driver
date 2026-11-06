# Source Provenance

This package migrates the project-owned ELECTRI-105 standalone staging driver
prepared on 2026-08-27/28 (migrated 2026-09-17: `src/decoder.cpp`, `src/ros_conversion.cpp`,
`src/socket_can.cpp`, `src/lpms_nav3_can_node.cpp`). The staging directory had no Git history.

The original Apache-2.0 package declaration and maintainer are retained. This is
not a copy of the manufacturer's ROS node: the original integration used the
provided EDS/DBC only to implement the default CANopen wire mapping.

Migration removes NMT transmit support, dedicated-host service paths and udev
identities; adds explicit commissioning admission, private topics, named QoS,
native/CI build inclusion and hardware-free node/launch tests.
