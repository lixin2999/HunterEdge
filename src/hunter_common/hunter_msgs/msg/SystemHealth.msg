std_msgs/Header header
float32 cpu_usage       # CPU 使用率 %
float32 gpu_usage       # GPU 使用率 %
float32 memory_usage    # 内存使用率 %
float32 disk_usage      # 磁盘使用率 %
float32 cpu_temp        # CPU 温度 ℃
float32 gpu_temp        # GPU 温度 ℃
string overall_status   # OK / WARNING / CRITICAL
string[] node_names     # 关键节点名
string[] node_states    # running / stopped / fault
