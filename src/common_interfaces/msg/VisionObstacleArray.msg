# VisionObstacleArray.msg - 一帧内检测到的所有视觉障碍物
#
# 由双目视觉节点 (usv_vision_node) 发布，包含当前帧所有检测到的障碍物列表。

std_msgs/Header header

# 障碍物列表
VisionObstacle[] obstacles
