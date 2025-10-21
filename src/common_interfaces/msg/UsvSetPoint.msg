# UsvSetPoint.msg
# 该消息用于表示无人水面艇（USV）的目标点信息
string usv_id                    # USV标识符（如 "usv1"）
geometry_msgs/PoseStamped target_pose  # 目标位姿
float32 yaw                      # 目标偏航角（单位：弧度）
uint8 step                       # 步骤编号