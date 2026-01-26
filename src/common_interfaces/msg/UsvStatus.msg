# =============================================================================
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Message definition for UsvStatus.
#
# Author: chenhangwei
# Date: 2026-01-26
# =============================================================================
# UsvStatus.msg
# 该消息用于表示无人水面艇（USV）的状态信息
# 包含命名空间、飞控模式、连接状态、解锁状态、引导状态、电池电量、位置、速度和偏航角等信息
string usv_id                    # USV标识符（如 "usv_01"）
std_msgs/Header header           # 消息头，包含时间戳和帧ID

# ==================== 飞控状态 ====================
string mode                      # 当前飞控模式（如 "MANUAL", "GUIDED", "AUTO"）
bool connected                   # 是否连接（来自飞控心跳）
bool armed                       # 是否解锁  
bool guided                      # 是否处于引导模式
string system_status             # 系统状态（STANDBY, ACTIVE, CRITICAL, EMERGENCY, POWEROFF, FLIGHT_TERMINATION）

# ==================== 电池信息 ====================
float32 battery_voltage          # 电池电压（单位：伏特）   
float32 battery_percentage       # 电池电量百分比（0.0 到 100.0）
float32 battery_current          # 电池电流（单位：安培，放电为正）
float32 battery_remaining        # 剩余电量（单位：安时 Ah）
uint8 power_supply_status        # 电源状态（0=未知, 1=充电, 2=放电, 3=未充电, 4=充满）
int32 battery_cell_count         # 电池串联节数
bool low_voltage_mode            # 低电压模式（电压低于 10.5V 时为 true）

# ==================== 位置和姿态 ====================
geometry_msgs/Point position     # 当前位置（x, y, z）在 local frame
geometry_msgs/Twist velocity     # 当前速度（线速度和角速度）
float32 yaw                      # 当前偏航角（单位：弧度，范围 -π 到 π）
float32 pitch                    # 俯仰角（单位：弧度）
float32 roll                     # 横滚角（单位：弧度）
float32 altitude_relative        # 相对起飞点高度（单位：米）
float32 altitude_amsl            # 海拔高度 AMSL（单位：米）

# ==================== 速度和性能 ====================
float32 ground_speed             # 地速（单位：米/秒）
float32 air_speed                # 空速（单位：米/秒，水面艇通常为0）
float32 climb_rate               # 爬升率（单位：米/秒）
float32 heading                  # 航向角（单位：度，0-360）

# ==================== GPS 信息 ====================
uint8 gps_fix_type               # GPS 定位类型（0=无, 1=无定位, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed）
uint8 gps_satellites_visible     # 可见卫星数量
float32 gps_eph                  # 水平位置精度因子 HDOP（单位：米）
float32 gps_epv                  # 垂直位置精度因子 VDOP（单位：米）

# ==================== 系统健康 ====================
float32 temperature              # CPU温度（单位：摄氏度）
float32 cpu_percent              # CPU 使用率（0.0-100.0）
float32 memory_percent           # 内存使用率（0.0-100.0）
uint32 uptime_sec                # 系统运行时间（单位：秒）

# ==================== 数据质量指标 ====================
float32 data_age_sec             # 数据年龄（单位：秒，距离最后更新的时间）
bool data_valid                  # 数据有效性标志
uint32 message_count             # 已发布消息计数（用于检测消息丢失）