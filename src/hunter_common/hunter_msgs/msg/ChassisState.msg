std_msgs/Header header
float32 velocity              # 实际线速度 m/s
float32 steering_angle        # 实际转向角 rad
float32 battery_voltage       # 电池电压 V
float32 battery_soc           # 电池SOC %
float32 battery_current       # 电池电流 A
float32 battery_temperature   # 电池温度 ℃
string control_mode           # CAN/REMOTE/ESTOP
string vehicle_state          # NORMAL/FAULT/ESTOP
uint32 fault_code
float32[] motor_rpm           # 电机转速
float32[] motor_current       # 电机电流
float32[] motor_temperature   # 电机温度
