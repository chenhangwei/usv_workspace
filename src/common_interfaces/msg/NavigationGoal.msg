# 导航目标消息
# 用于发送导航目标到 USV

uint32 goal_id                         # 目标唯一ID,用于匹配反馈和结果
geometry_msgs/PoseStamped target_pose  # 目标位置和姿态
float32 timeout                        # 超时时间(秒)

# 到达判定阈值（可选）：
# - reach_xy_threshold：水平距离阈值(米)
# - reach_z_threshold：高度误差阈值(米)
# 约定：<=0 表示不指定，由 USV 使用本地默认参数。
float32 reach_xy_threshold
float32 reach_z_threshold
builtin_interfaces/Time timestamp      # 发送时间戳
