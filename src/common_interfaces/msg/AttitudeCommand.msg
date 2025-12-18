# AttitudeCommand.msg
# 单向姿态动作指令（不要求完成度反馈）
#
# 单位约定：弧度（rad）
# 坐标系约定：与 USV 侧参数 coordinate_system 一致（默认 NED）；
# 本消息只携带 roll/pitch（以及可选的 yaw），不涉及 ENU/NED 位置转换。
#
# duration:
# - >0  : 指令保持 duration 秒后自动失效（回到仅导航/位置控制输出）
# - <=0 : 持续生效，直到下一条 AttitudeCommand 覆盖

builtin_interfaces/Time timestamp

float32 roll
float32 pitch

# 可选：如不需要可设置为 NaN，USV 端将回退为当前 heading
float32 yaw

float32 duration
