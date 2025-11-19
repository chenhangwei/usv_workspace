# 导航目标消息
# 用于发送导航目标到 USV

uint32 goal_id                         # 目标唯一ID,用于匹配反馈和结果
geometry_msgs/PoseStamped target_pose  # 目标位置和姿态
float32 timeout                        # 超时时间(秒)
builtin_interfaces/Time timestamp      # 发送时间戳
