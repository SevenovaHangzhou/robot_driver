# 数据时间：收到本次 Modbus 响应后的 ROS 时间，不是压力表内部采样时间。
std_msgs/Header header

# MT3A-AE0830 的 16 位无符号输入值。电压模式下单位为 mV。
uint16 raw_millivolts

# 由 1~5 V 线性换算得到的真空压力，单位固定为 kPa。
float32 pressure_kpa
string unit

# true 表示本次通信成功、报文正确，且原始值位于配置的有效范围内。
bool valid
string error
