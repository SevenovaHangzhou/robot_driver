# The timestamp is assigned after the Modbus response is processed. It is not a
# device sample timestamp.
std_msgs/Header header
string channel
string sensor_id
uint16 raw_millivolts
float32 pressure_kpa
string unit
bool valid
string error
