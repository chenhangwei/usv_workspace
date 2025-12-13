# 导航应答消息（收到/接受层）
# USV 在接收到导航目标后立即发送，用于让 GS 停止 step_timeout 级别的重发。
#
# 设计要点：
# - 与 NavigationResult（完成层）解耦：ack 表示“已收到并开始执行”，不代表到达。
# - execute_mask 用于表达平台能力与执行限制：2D 只执行位置+yaw；3D 可执行 roll/pitch/yaw。

uint32 goal_id

bool accepted

# 执行能力掩码（bitmask）
uint8 EXECUTE_POS = 1
uint8 EXECUTE_YAW = 2
uint8 EXECUTE_ROLL = 4
uint8 EXECUTE_PITCH = 8
uint8 execute_mask

string message

builtin_interfaces/Time timestamp
