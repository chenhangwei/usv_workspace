# 导航目标消息
# 用于发送导航目标到 USV

uint32 goal_id                         # 目标唯一ID,用于匹配反馈和结果
geometry_msgs/PoseStamped target_pose  # 目标位置和姿态

# 扩展控制字段
bool enable_yaw                        # 是否启用偏航角控制 (false=忽略yaw, true=根据maneuver_type决定)

# 特殊机动 (Maneuver) 定义
int8 MANEUVER_TYPE_NONE=0            # 默认: 普通导航(GOTO)
int8 MANEUVER_TYPE_ROTATE=1          # 原地旋转模式

int8 maneuver_type                     # 机动类型
float32 maneuver_param                 # 机动参数: 若type=ROTATE, 则表示旋转圈数(如 3.0 或 -3.0)

float32 timeout                        # 超时时间(秒)
builtin_interfaces/Time timestamp      # 发送时间戳
