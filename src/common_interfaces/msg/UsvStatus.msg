# USV（无人船）状态消息
std_msgs/Header header          # 消息头，包含时间戳和帧 ID
string mode                     # 当前飞控模式（如 "OFFBOARD", "MANUAL"）
bool connected                  # 是否连接
bool armed                      # 是否解锁  
bool guided                     # 是否引导   
bool manual_input              # 是否手动输入
float32 battery_percentage      # 电池电量百分比（0.0 到 100.0）
geometry_msgs/Point position    # 位置（x, y, z）
geometry_msgs/TwistStamped velocity     # 速度信息（线速度和角速度）