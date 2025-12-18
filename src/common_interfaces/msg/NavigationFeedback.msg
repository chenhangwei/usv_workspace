# 导航反馈消息
# USV 定期发送的导航进度反馈

uint32 goal_id                         # 对应的目标ID
float32 distance_to_goal               # 距离目标点(米)
float32 xy_distance_to_goal            # 水平距离(米)，用于 3D/2D 统一展示与 ETA 估算
float32 z_error                        # 高度误差(米)。约定：基于 PX4 NED Down 方向的误差(target_z - current_z)
float32 heading_error                  # 航向误差(度)
float32 estimated_time                 # 预计剩余时间(秒)
builtin_interfaces/Time timestamp      # 反馈时间戳
