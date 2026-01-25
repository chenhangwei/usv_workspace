# 导航反馈消息
# USV 定期发送的导航进度反馈

uint32 goal_id                         # 对应的目标ID
float32 distance_to_goal               # 距离目标点(米)
float32 heading_error                  # 航向误差(弧度, 范围 -π 到 π)
float32 estimated_time                 # 预计剩余时间(秒)
builtin_interfaces/Time timestamp      # 反馈时间戳

# 航点队列状态 (用于 GS 决定是否预发送下一批航点)
uint8 queue_length                     # 当前队列中待执行的航点数
uint8 queue_capacity                   # 队列最大容量
bool smooth_navigation                 # 是否启用平滑导航模式

# 导航模式状态 (与 NavigationGoal.msg 中的模式对应)
int8 nav_mode                          # 当前航点的导航模式
float32 arrival_quality                # 同步模式: 当前到达质量 (0.0-1.0)
float32 rotate_progress                # 旋转模式: 旋转进度 (0.0-1.0)
