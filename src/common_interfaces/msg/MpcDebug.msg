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
