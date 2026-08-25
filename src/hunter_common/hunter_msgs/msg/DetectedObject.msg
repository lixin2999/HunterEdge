std_msgs/Header header
int32 id
string class_name          # car, pedestrian, cyclist, obstacle
float32 confidence
geometry_msgs/Pose pose    # 目标中心位姿 (base_link坐标系)
geometry_msgs/Vector3 dimensions  # length, width, height
geometry_msgs/Twist velocity      # 目标速度
float32 tracking_age      # 跟踪持续时间(秒)
int32 track_id
