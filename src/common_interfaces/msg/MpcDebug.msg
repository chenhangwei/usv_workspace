# =============================================================================
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Message definition for MpcDebug.
#
# Author: chenhangwei
# Date: 2026-01-29
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
