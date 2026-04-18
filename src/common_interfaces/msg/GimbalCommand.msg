# GimbalCommand.msg - 云台控制指令
#
# 发送给云台控制节点 (usv_gimbal_node) 以控制云台运动和模式切换。

# 命令类型: 0=SET_MODE, 1=SET_ANGLE, 2=SET_VELOCITY
uint8 command_type

uint8 CMD_SET_MODE = 0
uint8 CMD_SET_ANGLE = 1
uint8 CMD_SET_VELOCITY = 2

# SET_MODE 时的目标模式 (0=IDLE, 1=PATROL, 2=TRACK, 3=MANUAL)
uint8 target_mode

# SET_ANGLE 时的目标角度 (度)
float32 pan_angle
float32 tilt_angle

# SET_VELOCITY 时的角速度 (度/秒)
float32 pan_velocity
float32 tilt_velocity
