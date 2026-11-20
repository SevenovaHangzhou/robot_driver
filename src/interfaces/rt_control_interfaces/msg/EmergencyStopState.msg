# RT-Control internal observation; not a safety-rated stop acknowledgement.
uint8 UNKNOWN=0
uint8 RELEASED=1
uint8 PRESSED=2
std_msgs/Header header
unique_identifier_msgs/UUID producer_instance_id
# Successful sample counter, scoped to producer_instance_id; does not advance on heartbeat.
uint64 sample_sequence
builtin_interfaces/Time sample_stamp
# Monotonic age at publication, -1 before first successful sample.
float64 sample_age_seconds
uint8 state
# False on startup, read error or stale sample. raw_level is meaningful only when true.
bool valid
bool raw_level
# Conservative consumer input: true for PRESSED or UNKNOWN. Not proof of physical stopping.
bool stop_requested
string detail
