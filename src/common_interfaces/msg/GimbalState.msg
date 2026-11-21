# GimbalState.msg - 云台状态信息
#
# 由云台控制节点 (usv_gimbal_node) 发布，包含云台当前角度、
# 工作模式和跟踪目标信息。

std_msgs/Header header

# 当前 Pan 角度 (度)，0=正前方，正=右，负=左
float32 pan_angle

# 当前 Tilt 角度 (度)，0=水平，正=上仰，负=下俯
float32 tilt_angle

# 工作模式: 0=IDLE, 1=PATROL, 2=TRACK, 3=MANUAL
uint8 mode

uint8 MODE_IDLE = 0
uint8 MODE_PATROL = 1
uint8 MODE_TRACK = 2
uint8 MODE_MANUAL = 3

# 是否正在跟踪目标
bool tracking

# 跟踪目标类名 (跟踪模式下有效)
string track_target_class

# 跟踪目标距离 (米)
float32 track_target_distance
