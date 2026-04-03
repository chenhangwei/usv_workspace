# VisionObstacle.msg - 视觉检测到的单个障碍物信息
#
# 由双目视觉节点 (usv_vision_node) 发布，包含障碍物的类别、置信度、
# 距离、方位角和尺寸信息，供避障节点进行融合决策。

std_msgs/Header header

# 目标类别 (boat, buoy, person, debris 等)
string class_name

# 检测置信度 [0.0, 1.0]
float32 confidence

# 障碍物距离 (米)，由双目视差计算得出
float32 distance

# 障碍物方位角 (弧度)，正前方为 0，右侧为正，左侧为负
float32 bearing

# 障碍物估计宽度 (米)
float32 width

# 像素级边界框 [x1, y1, x2, y2]
int32[4] bbox
