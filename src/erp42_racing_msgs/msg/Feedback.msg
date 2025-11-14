# erp42_racing_msgs/msg/Feedback.msg

# Message header
std_msgs/Header header

# Control mode (Manual / Auto)
bool manual_mode

# Emergency stop (ON / OFF)
bool emergency_stop

# Gear (Drive / Neutral / Reverse)
uint8 GEAR_DRIVE   = 0
uint8 GEAR_NEUTRAL = 1
uint8 GEAR_REVERSE = 2
uint8 gear

# Vehicle speed (m/s)
float64 speed

# Steering angle (rad)
float64 steering

# Braking (0 - 100)
uint8 brake

# Encoder count (-2^31 - 2^31)
int32 encoder_count

# Battery (V)
float64 battery

# Health check counter (0 - 255)
uint8 heartbeat