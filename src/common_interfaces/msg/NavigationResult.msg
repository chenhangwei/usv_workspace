# 导航结果消息
# USV 完成导航任务后发送的结果

uint32 goal_id                         # 对应的目标ID
bool success                           # 是否成功到达
uint8 error_code                       # 错误码: 0=成功, 1=超时, 2=取消, 3=其他错误
string message                         # 结果描述信息
builtin_interfaces/Time timestamp      # 完成时间戳
