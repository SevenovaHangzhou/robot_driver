# modbus_tcp_rtu485_led

ROS 2 C++ driver for four WE-10x controllers through an RS485-ETH-M04:

```text
ROS 2 -> Modbus TCP -> RS485-ETH-M04 -> Modbus RTU/RS485 -> WE-10x -> LED strip
```

The node subscribes to `led0/color` through `led3/color` (`std_msgs/msg/ColorRGBA`).
RGB values are normalized to `0.0..1.0`; alpha is used as the W channel. Each
message writes WE-10x holding registers 40001..40004 with Modbus function `0x10`.

## 通信和地址测试

RS485-ETH-M04 的默认地址和端口如下：

```text
模块 IP：192.168.1.12
HC0：502
HC1：503
HC2：504
HC3：505
```

WE-10x 的 Modbus 地址保存在保持寄存器 40005。实际 PDU 地址为
`0x0004`，本驱动接受标准 Modbus 普通地址 `1..247`；地址 `0` 是广播地址，不能用于读取。

如果控制器当前地址为 0，需要通过广播写入改成 1。下面以 HC0 为例，
控制器必须已经接在 HC0，且该 RS485 总线上只有需要修改的控制器：

```bash
mbpoll -m tcp -a 0 -r 5 -t 4 -1 \
  -p 502 192.168.1.12 1
```

广播写入没有响应，`mbpoll` 可能显示超时，这是预期行为。写入后用新地址
读取 40005 验证：

```bash
mbpoll -m tcp -a 1 -r 5 -t 4 -1 \
  -p 502 192.168.1.12
```

预期结果为：

```text
[5]:    1
```

再读取 RGBW 输出寄存器 40001..40004：

```bash
mbpoll -m tcp -a 1 -r 1 -c 4 -t 4 -1 \
  -p 502 192.168.1.12
```

返回值依次表示 R、G、B、W。例如：

```text
[1]:    255
[2]:    0
[3]:    0
[4]:    0
```

表示控制器当前保存的是红色最大亮度。

四路 RS485 通道彼此独立，因此四个控制器保持出厂地址 1 时，配置可以是：

```yaml
controller_addresses: [1, 1, 1, 1]
```

如果要把 HC1、HC2、HC3 上的控制器从地址 0 改为 1，分别使用端口
`503`、`504`、`505` 重复广播写入和读取验证流程。

Default gateway ports are `502, 503, 504, 505` for HC0..HC3. The four RS485
channels are independent, so controllers may all keep the WE-10x factory
address `1`; configure different values only when the controllers were
actually re-addressed.

Run with:

```bash
ros2 run modbus_tcp_rtu485_led led_strip_node --ros-args \
  --params-file $(ros2 pkg prefix modbus_tcp_rtu485_led)/share/modbus_tcp_rtu485_led/config/led_strip.yaml
```

## Scope, timeouts and offline verification

The color topics are RT-Control-local engineering inputs, not new public
cross-domain endpoints. External consumers require a separately reviewed
`robot_interfaces` contract before integration. The node is a separate process;
its socket IO does not run in the ros2_control real-time update loop.

Each callback opens one connection and applies a single `response_timeout_ms`
deadline to connect, send and receive (valid range `1..60000`, default `500`).
The QoS depth is one to limit queued colors. Failed writes are logged and are
not retried automatically. Shutdown may wait for the current bounded callback;
the node does not send an unapproved reset/off command at exit. LED hardware
may retain its last color on disconnect or exit and must not represent a
hardware safety guarantee.

The four port values must be `1..65535`, and the four unit addresses `1..247`.
The driver checks MBAP transaction/protocol/length/unit and the complete
function/address/count acknowledgement; Modbus exceptions are failures.
Partial TCP sends and fragmented responses share the same deadline.

`rt_control.launch.py` supports `start_led` and `led_config`. Mock hardware
defaults `start_led` to false; explicitly enable only when intended. The gateway
and controller values in the supplied config describe the user's existing
setup, not evidence of device identity or successful commissioning. Verify
them against the actual devices before use. README `mbpoll` examples write real
hardware and require separate authorization; they are not offline tests.

Offline CTest `modbus_transport` uses local socket pairs only.
`led_node_lifecycle` also verifies invalid configuration, two configured starts
and SIGINT exits in an isolated local ROS domain without publishing colors.
Build and test using external build/install/log paths to avoid repository artifacts:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/robot-led-pr-log build \
  --base-paths src/rt_control/modbus_tcp_rtu485_led \
  --build-base /tmp/robot-led-pr-build --install-base /tmp/robot-led-pr-install
colcon --log-base /tmp/robot-led-pr-log test \
  --build-base /tmp/robot-led-pr-build --install-base /tmp/robot-led-pr-install \
  --packages-select modbus_tcp_rtu485_led
colcon test-result --test-result-base /tmp/robot-led-pr-build --verbose
```

Example:

```bash
ros2 topic pub --once /led0/color std_msgs/msg/ColorRGBA \
  '{r: 1.0, g: 0.0, b: 0.0, a: 0.0}'
```

发布后可以再次读取 40001..40004，确认颜色值已经写入控制器：

```bash
mbpoll -m tcp -a 1 -r 1 -c 4 -t 4 -1 \
  -p 502 192.168.1.12
```
