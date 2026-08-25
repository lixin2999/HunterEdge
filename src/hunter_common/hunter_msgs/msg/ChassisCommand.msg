std_msgs/Header header
float32 target_velocity       # 目标线速度 m/s (范围 ±4.8)
float32 target_steering       # 目标转向角 rad (范围 ±0.4)
string control_mode           # CAN/REMOTE/ESTOP
bool emergency_stop           # 紧急制动标志
