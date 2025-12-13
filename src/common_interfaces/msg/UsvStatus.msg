# UsvStatus.msg
# 无人飞行球（USV - Unmanned Sphere Vehicle）聚合状态消息
# 适用于 8 旋翼无人飞行器，基于 PX4 uXRCE-DDS 架构
#
# 使用 ROS 2 标准消息类型以确保生态系统兼容性

# 消息头（包含时间戳和坐标系）
std_msgs/Header header

# USV 标识符
string usv_id

# ==================== 飞控状态 ====================
# 使用 PX4 导航状态常量
uint8 NAV_STATE_MANUAL = 0
uint8 NAV_STATE_ALTCTL = 1
uint8 NAV_STATE_POSCTL = 2
uint8 NAV_STATE_AUTO_MISSION = 3
uint8 NAV_STATE_AUTO_LOITER = 4
uint8 NAV_STATE_AUTO_RTL = 5
uint8 NAV_STATE_ACRO = 10
uint8 NAV_STATE_DESCEND = 12
uint8 NAV_STATE_TERMINATION = 13
uint8 NAV_STATE_OFFBOARD = 14
uint8 NAV_STATE_STAB = 15
uint8 NAV_STATE_AUTO_TAKEOFF = 17
uint8 NAV_STATE_AUTO_LAND = 18

# 解锁状态常量
uint8 ARMING_STATE_INIT = 0
uint8 ARMING_STATE_STANDBY = 1
uint8 ARMING_STATE_ARMED = 2

uint8 nav_state                  # 当前导航状态
uint8 arming_state               # 当前解锁状态
string mode                      # 模式名称字符串（用于显示）
bool connected                   # 飞控连接状态
bool armed                       # 是否解锁
bool in_air                      # 是否在空中
bool landed                      # 是否处于着陆状态
bool pre_flight_checks_pass      # 预检通过
bool failsafe                    # 失控保护激活

# ==================== 位姿信息 ====================
# 使用标准 geometry_msgs 类型
geometry_msgs/Pose pose          # 位置和姿态（四元数）
geometry_msgs/Twist twist        # 线速度和角速度

# 欧拉角（弧度）
# 语义：与 pose.orientation 一致，使用 ROS 约定（世界 ENU，机体 FLU）。
float64 roll                     # 横滚角 [rad]
float64 pitch                    # 俯仰角 [rad]
float64 yaw                      # 偏航角 [rad]（ENU: 0=East，逆时针为正，范围通常为 [-π, π]）

# 航向和高度
# 航向角用于“罗盘/航向显示”，沿用 PX4 的 heading 语义：NED 航向（0=North，顺时针为正），发布时归一化到 [0, 2π)
float64 heading                  # 航向角 [rad, 0-2π)
float64 altitude_relative        # 相对高度 [m]
float64 altitude_amsl            # 海拔高度 AMSL [m]

# ==================== 电池信息 ====================
# 使用标准 sensor_msgs/BatteryState 兼容字段
float32 battery_voltage          # 电压 [V]
float32 battery_current          # 电流 [A]（放电为正）
float32 battery_percentage       # 电量百分比 [0-100]

# 电池状态常量（与 sensor_msgs/BatteryState 兼容）
uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4
uint8 power_supply_status

bool low_voltage_mode            # 低电压警告

# ==================== 导航信息 ====================
float64 distance_to_target       # 到目标距离 [m]
bool target_reached              # 是否到达目标

# ==================== 健康状态 ====================
uint64 health_warning_flags      # 健康警告标志位
uint64 health_error_flags        # 健康错误标志位
uint64 arming_check_error_flags  # 解锁检查错误标志位
bool gcs_connection_lost         # 地面站连接丢失

# ==================== 传感器状态 (来自 PX4 estimator_status_flags) ====================
bool sensor_gyro_ok              # 陀螺仪正常 (IMU)
bool sensor_accel_ok             # 加速度计正常 (IMU)
bool sensor_mag_ok               # 磁罗盘正常
bool sensor_baro_ok              # 气压计正常
bool sensor_gps_ok               # GPS 正常

# ==================== 飞控消息 ====================
string last_status_text          # 最近的飞控状态消息
uint8 last_status_severity       # 消息严重性级别 (0-7, 0最严重)

# ==================== 系统健康 ====================
float32 cpu_percent              # CPU 使用率 [%]
float32 memory_percent           # 内存使用率 [%]
float32 temperature              # 温度 [°C]
uint32 uptime_sec                # 运行时间 [s]

# ==================== 数据质量 ====================
bool data_valid                  # 数据有效
builtin_interfaces/Duration data_age  # 数据年龄
uint32 sequence                  # 消息序号
