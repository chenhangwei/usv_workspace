# 集群状态消息
# 用于发布整个集群的聚合状态信息

std_msgs/Header header

# 总体统计
uint32 total_count          # 总 USV 数量
uint32 online_count         # 在线数量
uint32 armed_count          # 已解锁数量
uint32 in_mission_count     # 执行任务中数量
uint32 low_battery_count    # 低电量数量
uint32 error_count          # 错误/离线数量

# 分组信息
string[] group_ids          # 分组 ID 列表 (A, B, C, ...)
uint32[] group_counts       # 每组 USV 数量

# 告警信息
string[] warnings           # 告警消息列表
