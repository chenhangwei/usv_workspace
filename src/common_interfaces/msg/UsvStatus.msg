# UsvStatus.msg
std_msgs/Header header          # 消息头，包含时间戳和帧 ID
string mode                     # 当前飞控模式（如 "MANUAL", "GUIDED", "AUTO"）
bool connected                  # 是否连接（来自飞控心跳）
bool armed                      # 是否解锁  
bool guided                     # 是否引导
float32 battery_voltage         # 电池电压（单位：伏特）   
float32 battery_percentage      # 电池电量百分比（0.0 到 1.0）
uint8 power_supply_status            # 电池状态（0=未知, 1=充电, 2=放电, 3=未充电, 4=充满）
geometry_msgs/Point position    # 位置（x, y, z）
geometry_msgs/Twist velocity    # 速度信息（线速度和角速度）
float32 yaw                     # 偏航角（单位：弧度）