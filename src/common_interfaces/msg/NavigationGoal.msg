# 导航目标消息
# 用于发送导航目标到 USV

uint32 goal_id                         # 目标唯一ID,用于匹配反馈和结果
geometry_msgs/PoseStamped target_pose  # 目标位置和姿态

# 扩展控制字段
bool enable_yaw                        # 是否启用偏航角控制 (false=忽略yaw, true=根据maneuver_type决定)

# ==================== 导航模式 (Navigation Mode) ====================
# 定义到达目标点后的行为模式
int8 NAV_MODE_ASYNC=0                  # 异步模式: 到达目标点后立即前往下一个目标点，不等待确认
int8 NAV_MODE_SYNC=1                   # 同步模式: 等待到达质量确认(80%到达判断)，超时自动切换异步
int8 NAV_MODE_ROTATE=2                 # 旋转模式: 到达目标点后原地旋转指定圈数
int8 NAV_MODE_TERMINAL=3               # 终止模式: 到达目标点后停止，没有后续任务

int8 nav_mode                          # 当前航点的导航模式
float32 sync_timeout                   # 同步模式超时时间(秒)，超时后自动切换为异步
float32 arrival_quality_threshold      # 到达质量阈值(0.0-1.0)，默认0.8表示80%

# ==================== 特殊机动 (Maneuver) 定义 ====================
int8 MANEUVER_TYPE_NONE=0              # 默认: 普通导航(GOTO)
int8 MANEUVER_TYPE_ROTATE=1            # 原地旋转模式

int8 maneuver_type                     # 机动类型
float32 maneuver_param                 # 机动参数: 若type=ROTATE, 则表示旋转圈数(如 3.0 或 -3.0)

float32 timeout                        # 超时时间(秒)
builtin_interfaces/Time timestamp      # 发送时间戳
