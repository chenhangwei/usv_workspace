# =============================================================================
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Message definition for FormationConfig.
#
# Author: chenhangwei
# Date: 2026-02-13
# =============================================================================
# FormationConfig.msg
# 编队配置消息，用于描述编队模式的配置信息

# ==================== 编队类型常量 ====================
int8 FORMATION_V_SHAPE=0         # 人字形 (V-Shape)
int8 FORMATION_LINE=1            # 一字形 (Line)
int8 FORMATION_DIAMOND=2         # 菱形 (Diamond)
int8 FORMATION_TRIANGLE=3       # 三角形 (Triangle)
int8 FORMATION_COLUMN=4         # 纵列 (Column)
int8 FORMATION_S_SHAPE=5        # S形 (S-Shape)
int8 FORMATION_ESCORT=6         # 护卫 (Escort)

# ==================== 编队配置 ====================
string leader_id                  # 领队 USV ID (如 "usv_02")
string[] follower_ids             # 跟随者 USV ID 列表 (按编队位置排序)
int8 formation_type               # 编队类型 (使用上述常量)
float32 spacing_along             # 前后间距 (米，沿领队航向方向)
float32 spacing_cross             # 左右间距 (米，垂直于领队航向方向)

# ==================== 控制参数 ====================
float32 update_rate               # 跟随者目标更新频率 (Hz，建议 5-20)
float32 leader_timeout            # 领队状态超时时间 (秒，超时后跟随者自动 HOLD)
float32 follower_speed            # 跟随者巡航速度 (m/s，0 表示使用默认)

# ==================== 状态标志 ====================
bool active                       # 编队是否激活
builtin_interfaces/Time timestamp # 配置时间戳
