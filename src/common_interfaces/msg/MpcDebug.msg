# =============================================================================
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Message definition for MpcDebug.
#
# Author: chenhangwei
# Date: 2026-01-29
# Updated: 2026-02-03 - v5 添加一阶惯性模型相关字段
# Updated: 2026-02-09 - v8 添加 AMPC 在线辨识相关字段
# Updated: 2026-02-27 - v14 添加 ORCA/APF 避障状态字段
# =============================================================================
# MPC 控制器调试信息

builtin_interfaces/Time timestamp

# 性能指标
float32 solve_time_ms           # 求解耗时 (ms)
float32 cost                    # 最终代价值

# 预测状态 (N步后的预测)
float32 mpc_pred_x              # 预测位置 X
float32 mpc_pred_y              # 预测位置 Y
float32 mpc_pred_theta          # 预测航向

# v5 新增: 一阶惯性模型状态
float32 omega_actual            # 估计的实际角速度 (rad/s)
float32 omega_cmd               # 角速度命令 (rad/s)
float32 cross_track_error       # 横向偏差 CTE (m)
float32 path_theta              # 当前航线方向 (rad)

# 内部状态
int32 solver_status             # 求解器状态 (0=Success)
string active_controller        # 当前激活的控制器类型
float32 ref_curvature           # 参考路径曲率 (1/R)

# MPC 参数配置 (用于日志记录和调试)
float32 param_q_pos             # 位置权重 Q_pos
float32 param_q_theta           # 航向权重 Q_theta  
float32 param_r_w               # 角速度权重 R_w
float32 param_r_dw              # 角加速度权重 R_dw
float32 param_w_max             # 最大角速度限制 (rad/s)
int32 param_n_steps             # 预测步数

# v5 新增参数
float32 param_tau_omega         # 转向时间常数 (s)
float32 param_q_cte             # CTE 权重

# v6 新增: 速度自适应 tau_omega 参数
string usv_id                   # USV 标识 (如 "usv_02", "usv_03")
bool adaptive_tau_enabled       # 是否启用速度自适应 tau_omega
float32 tau_omega_low_speed     # 低速时的 tau_omega (s)
float32 tau_omega_high_speed    # 高速时的 tau_omega (s)
float32 tau_speed_threshold_low # 低速阈值 (m/s)
float32 tau_speed_threshold_high # 高速阈值 (m/s)
float32 current_tau_omega       # 当前实际使用的 tau_omega (s) - 自适应实时值

# v8 新增: AMPC 自适应 MPC 在线辨识信息
bool ampc_enabled               # 是否启用 AMPC 在线辨识
float32 ampc_tau_estimated      # RLS 在线估计的 tau_omega (s)
float32 ampc_tau_confidence     # tau 估计置信度 (0-1)
float32 ampc_omega_measured     # 从航向变化实测的 omega_actual (rad/s)
float32 ampc_saturation_ratio   # 角速度饱和率 (0-1)
float32 ampc_heading_noise      # 航向噪声标准差 (rad/s)
int32 ampc_rebuild_count        # MPC 求解器重建次数
bool ampc_converged             # RLS 估计是否已收敛

# v14 新增: ORCA/APF 避障状态
bool orca_active                    # ORCA 是否处于激活状态
float32 orca_closest_distance       # 最近邻居距离 (m), 无邻居时为 -1
string orca_encounter_type          # 遭遇类型: head_on/crossing/overtaking/none
string orca_encounter_type_raw      # 遭遇类型(原始, 未平滑)
int8 orca_commit_side               # 选边方向: +1=左, -1=右, 0=无
float32 orca_linear_correction      # 线速度修正量 (m/s)
float32 orca_angular_correction     # 角速度修正量 (rad/s)
bool orca_hard_brake_active         # 硬刹车是否激活
int8 apf_neighbor_count             # 有效邻居数量
float32 orca_rel_bearing_deg        # 相对方位角(本船体系, deg)
float32 orca_rel_course_deg         # 相对航向差(相对航向-本船航向, deg)
float32 orca_rel_speed              # 相对速度模值 (m/s)
float32 orca_tcpa                   # 时间最近点 (s), 无效时为 -1
float32 orca_dcpa                   # 最近点距离 (m), 无效时为 -1
