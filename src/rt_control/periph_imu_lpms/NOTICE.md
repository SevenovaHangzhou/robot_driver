# Source Provenance

This package migrates the project-owned ELECTRI-105 standalone staging driver
prepared on 2026-08-27/28. The staging directory had no Git history. Original source
SHA-256 values at migration (2026-09-17):

| File | SHA-256 |
| --- | --- |
| src/decoder.cpp | 92dd5143a52d5f2d52978b15271c10519d53cef005e8b74ff9b7090d5e541260 |
| src/ros_conversion.cpp | 5c44d7292ae525b17938f18dba3c04342b8ca6d98afed478e753b63626104b48 |
| src/socket_can.cpp | d81f321aed379fd8bb846f9b0dc3c1a8ace5ddbf782714ed984b9c940debeb8c |
| src/lpms_nav3_can_node.cpp | 77c92544f558fec34bb45c8c84b86e565a5708a7c5158eba20cc1e6e7e069edb |

The original Apache-2.0 package declaration and maintainer are retained. This is
not a copy of the manufacturer's ROS node: the original integration used the
provided EDS/DBC only to implement the default CANopen wire mapping.

Migration removes NMT transmit support, dedicated-host service paths and udev
identities; adds explicit commissioning admission, private topics, named QoS,
native/CI build inclusion and hardware-free node/launch tests.
