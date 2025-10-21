# UsvStatus.msg
# 该消息用于表示无人水面艇（USV）的状态信息
# 包含命名空间、飞控模式、连接状态、解锁状态、引导状态、电池电量、位置、速度和偏航角等信息
string usv_id                    # USV标识符（如 "usv_01"）
std_msgs/Header header           # 消息头，包含时间戳和帧ID
string mode                      # 当前飞控模式（如 "MANUAL", "GUIDED", "AUTO"）
bool connected                   # 是否连接（来自飞控心跳）
bool armed                       # 是否解锁  
bool guided                      # 是否处于引导模式
float32 battery_voltage          # 电池电压（单位：伏特）   
float32 battery_percentage       # 电池电量百分比（0.0 到 100.0）
uint8 power_supply_status        # 电源状态（0=未知, 1=充电, 2=放电, 3=未充电, 4=充满）
geometry_msgs/Point position     # 当前位置（x, y, z）
geometry_msgs/Twist velocity     # 当前速度（线速度和角速度）
float32 yaw                      # 当前偏航角（单位：弧度）
float32 temperature              # 当前温度（单位：摄氏度）