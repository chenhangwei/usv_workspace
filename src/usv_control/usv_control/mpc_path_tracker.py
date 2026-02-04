#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of MPC path tracker using CasADi.
#
# Author: chenhangwei
# Date: 2026-01-22
# Updated: 2026-02-03 - v5 升级为一阶惯性转向模型，根治S形振荡
"""
MPC 路径跟踪控制器 (v5: 一阶惯性转向模型)

基于模型预测控制 (MPC) 的路径跟踪器。
使用 CasADi 进行非线性优化求解。

特点:
- 一阶惯性转向模型: 模拟真实船的转向延迟，从根本上消除S形振荡
- Cross Track Error (CTE) 控制: 沿航线追踪，而非点对点追踪
- 平滑的控制输出，保护舵机
- 直接处理运动学约束 (最大速度、最大角速度)

核心改进 (v5):
1. 状态扩展: [x, y, θ] (3D) → [x, y, θ, ω_actual] (4D)
2. 动力学改进: θ̇ = ω_cmd → θ̇ = ω_actual, τ·ω̇ + ω = ω_cmd
3. 代价函数改进: 点对点追踪 → CTE + 航线方向追踪

模型升级历史:
- v1-v4: Unicycle 运动学模型 (假设无转向延迟) → 持续S形振荡
- v5 (2/03): 一阶惯性转向模型 + CTE控制 → 根治振荡

"""

import casadi as ca
import numpy as np
import math
import time
import sys
import threading

class MpcPathTracker:
    # 参数调整历史:
    # v1 (1/30): w_max=0.4, q_pos=10, q_theta=1, r_w=5, r_dw=10 → 大振幅低频振荡
    # v2 (2/02): w_max=0.3, q_pos=10, q_theta=1, r_w=10, r_dw=20 → 小振幅高频振荡
    # v3 (2/03): 降低q_pos，增加q_theta → 仍有振荡
    # v4 (2/03): q_pos=10, q_theta=12, r_w=6, r_dw=12, w_max=0.5 → 仍有振荡
    # v5 (2/03): 一阶惯性模型 + CTE控制 → 根治振荡
    
    def __init__(self, prediction_steps=20, dt=0.1, v_max=0.4, w_max=0.5,
                 q_pos=10.0, q_theta=8.0, r_vel=0.1, r_w=5.0, r_acc=2.0, r_dw=10.0,
                 tau_omega=0.4, q_cte=15.0):
        """
        初始化 MPC 控制器 (一阶惯性转向模型 v5)
        
        核心改进:
        - 状态扩展为 4 维: [x, y, θ, ω_actual]
        - 新增 tau_omega 参数模拟转向惯性
        - 新增 q_cte 参数控制横向偏差
        
        Args:
            prediction_steps: 预测步数 N
            dt: 预测步长 (s)
            v_max: 最大线速度 (m/s)
            w_max: 最大角速度 (rad/s)
            q_pos: 沿航线进度权重
            q_theta: 航向误差权重
            r_vel: 速度误差权重
            r_w: 角速度惩罚权重
            r_acc: 加速度变化率惩罚
            r_dw: 角加速度变化率惩罚
            tau_omega: 转向时间常数 (s) - 核心新参数！建议 0.3-0.6
            q_cte: Cross Track Error 权重 - 控制横向偏差
        """
        self.lock = threading.Lock()
        self.v_max = v_max
        self.w_max = w_max
        self.dt = dt
        self.N = prediction_steps

        # MPC 权重参数
        self.Q_pos   = q_pos      # 沿航线进度权重
        self.Q_theta = q_theta    # 航向误差权重
        self.R_vel   = r_vel      # 速度误差权重
        self.R_w     = r_w        # 角速度惩罚
        self.R_acc   = r_acc      # 加速度变化率惩罚
        self.R_dw    = r_dw       # 角加速度变化率惩罚
        
        # ==================== v5 新增参数 ====================
        
        # 1. 转向时间常数 tau_omega (一阶惯性模型核心参数)
        # 物理含义: 船从命令发出到实际角速度达到 63.2% 的时间
        # 推荐范围: 0.3 ~ 0.6 秒 (需要根据实船系统辨识)
        # 调整效果:
        #   - 调大: MPC 预测船转向更慢，会提前开始转向，输出更激进
        #   - 调小: MPC 预测船转向更快，接近原 Unicycle 模型
        # 如果不确定，先用 0.4s，然后观察：
        #   - 还在画龙 → 调大到 0.5-0.6
        #   - 响应太慢/迟钝 → 调小到 0.3
        self.tau_omega = tau_omega
        
        # 2. Cross Track Error (CTE) 权重
        # 物理含义: 控制船与航线的横向偏差
        # 推荐范围: 10.0 ~ 30.0
        # 调整效果:
        #   - 调大: 船严格沿航线走，横向偏差小，但可能震荡
        #   - 调小: 允许较大横向偏差，轨迹更平滑
        self.Q_cte = q_cte
        
        # 上一个航点 (用于计算航线方向)
        self._prev_waypoint = None
        
        # 上一次估计的实际角速度
        self._last_omega_actual = 0.0
        
        self.setup_mpc()

    def setup_mpc(self):
        """
        建立 MPC 优化问题 (一阶惯性转向模型 v5)
        
        状态变量: [x, y, theta, omega_actual] (4维)
        控制变量: [v, w_cmd] (2维)
        
        动力学模型:
            ẋ = v·cos(θ)
            ẏ = v·sin(θ)
            θ̇ = ω_actual (实际角速度，不是命令！)
            ω̇_actual = (ω_cmd - ω_actual) / τ  (一阶惯性)
            
        离散化: ω_actual[k+1] = α·ω_actual[k] + (1-α)·ω_cmd[k]
        其中 α = exp(-dt/τ)
        """
        # 1. 定义状态变量 (4维: x, y, theta, omega_actual)
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        omega_actual = ca.SX.sym('omega_actual')  # 新增: 实际角速度状态
        states = ca.vertcat(x, y, theta, omega_actual)
        n_states = states.size1()  # 4

        # 2. 定义控制变量 (2维: v, w_cmd)
        v = ca.SX.sym('v')
        w_cmd = ca.SX.sym('w_cmd')  # 角速度命令 (不是实际角速度!)
        controls = ca.vertcat(v, w_cmd)
        n_controls = controls.size1()  # 2

        # 3. 动力学模型 (一阶惯性转向 - 核心改进!)
        # 离散化: omega_actual[k+1] = alpha * omega_actual[k] + (1-alpha) * w_cmd[k]
        # 其中 alpha = exp(-dt / tau_omega)
        alpha = ca.exp(-self.dt / self.tau_omega)
        omega_next = alpha * omega_actual + (1 - alpha) * w_cmd
        
        # 状态导数 (注意: θ̇ = omega_actual, 不是 w_cmd!)
        rhs = ca.vertcat(
            v * ca.cos(theta),           # ẋ = v·cos(θ)
            v * ca.sin(theta),           # ẏ = v·sin(θ)
            omega_actual,                 # θ̇ = ω_actual (关键!)
            (omega_next - omega_actual) / self.dt  # ω̇_actual
        )
        
        # 离散化状态转移函数
        f = ca.Function('f', [states, controls], [states + rhs * self.dt])

        # 4. 优化变量
        U = ca.SX.sym('U', n_controls, self.N)      # 未来 N 步的控制量
        X = ca.SX.sym('X', n_states, self.N+1)      # 状态轨迹
        
        # 参数向量 P: [当前状态(4), 目标点(2), 航线起点(2), 航线方向(1)]
        # P = [x0, y0, theta0, omega0, target_x, target_y, start_x, start_y, path_theta]
        P = ca.SX.sym('P', n_states + 5)

        # 5. 构建代价函数与约束
        obj = 0
        g = []
        
        # 初始状态约束: X[:,0] == P[:4]
        st = X[:, 0]
        g = ca.vertcat(g, st - P[:n_states])

        # 解析参数
        target_x = P[4]
        target_y = P[5]
        start_x = P[6]
        start_y = P[7]
        path_theta = P[8]  # 航线方向 (固定，不随船位置变化!)
        
        # 计算航线向量 (归一化)
        path_vec_x = target_x - start_x
        path_vec_y = target_y - start_y
        path_len = ca.sqrt(path_vec_x**2 + path_vec_y**2 + 1e-6)
        path_nx = path_vec_x / path_len  # 航线单位向量 x
        path_ny = path_vec_y / path_len  # 航线单位向量 y

        for k in range(self.N):
            st = X[:, k]
            con = U[:, k]
            
            # 当前预测位置
            px = st[0]
            py = st[1]
            ptheta = st[2]
            # pomega = st[3]  # 实际角速度 (本轮不直接用，但参与状态转移)
            
            # ==================== 代价计算 (改进版: CTE + 航线方向) ====================
            
            # 1. Cross Track Error (横向偏差) - 核心改进!
            # 船到航线起点的向量
            dx_to_start = px - start_x
            dy_to_start = py - start_y
            
            # 横向误差 = 叉积 (垂直于航线的距离)
            cross_track_error = dx_to_start * path_ny - dy_to_start * path_nx
            obj += self.Q_cte * cross_track_error**2
            
            # 2. 沿航线进度 (纵向位置)
            # 点积 = 沿航线走了多远
            along_track = dx_to_start * path_nx + dy_to_start * path_ny
            # 惩罚尚未到达终点的距离
            remaining = path_len - along_track
            obj += self.Q_pos * remaining**2
            
            # 3. 航向误差 - 使用航线方向，不是船到目标点方向!
            heading_error = ptheta - path_theta
            obj += self.Q_theta * heading_error**2
            
            # 4. 控制输入惩罚
            obj += self.R_w * con[1]**2           # 惩罚大角速度命令
            obj += self.R_vel * (con[0] - self.v_max)**2  # 激励保持速度

            # 5. 变化率惩罚 (丝滑的关键)
            if k < self.N - 1:
                con_next = U[:, k+1]
                obj += self.R_acc * (con[0] - con_next[0])**2   # 加速度平滑
                obj += self.R_dw * (con[1] - con_next[1])**2    # 角加速度平滑

            # 状态更新约束
            st_next = X[:, k+1]
            st_next_pred = f(st, con)
            g = ca.vertcat(g, st_next - st_next_pred)

        # 6. 配置求解器 (IPOPT)
        nlp_prob = {
            'f': obj, 
            'x': ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1)), 
            'p': P, 
            'g': g
        }
        
        opts = {
            'ipopt.print_level': 0, 
            'ipopt.sb': 'yes', 
            'print_time': 0,
            'ipopt.max_iter': 150,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': 1e-4,
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
        
        # 保存维度
        self.n_states = n_states        # 4
        self.n_controls = n_controls    # 2
        self.n_x_all = (self.N+1) * n_states
        self.n_u_all = self.N * n_controls

        # 热启动缓存
        self.last_sol_x = [0.0] * (self.n_x_all + self.n_u_all)

    def reset(self):
        """重置 MPC 状态"""
        self.last_sol_x = [0.0] * (self.n_x_all + self.n_u_all)
        self._last_omega_actual = 0.0
        self._prev_waypoint = None

    def set_prev_waypoint(self, prev_wp):
        """
        设置上一个航点 (用于计算航线方向)
        
        Args:
            prev_wp: [x, y] 上一个航点坐标
        """
        if prev_wp is not None:
            self._prev_waypoint = [float(prev_wp[0]), float(prev_wp[1])]

    def compute_velocity_command(self, current_pose, target_waypoint, target_speed=0.4,
                                  prev_waypoint=None):
        """
        计算控制指令 (一阶惯性转向模型 v5)
        
        Args:
            current_pose: [x, y, yaw] 当前位姿
            target_waypoint: [x, y] 目标航点
            target_speed: 期望线速度 (m/s)
            prev_waypoint: [x, y] 上一个航点 (可选，用于计算航线方向)
            
        Returns:
            (v, w): 速度命令
        """
        # ==================== 输入校验 ====================
        if any(math.isnan(x) or math.isinf(x) for x in current_pose):
            print(f"MPC Error: Invalid current_pose: {current_pose}")
            return 0.0, 0.0
            
        if any(math.isnan(x) or math.isinf(x) for x in target_waypoint):
            print(f"MPC Error: Invalid target_waypoint: {target_waypoint}")
            return 0.0, 0.0

        # 类型转换
        current_pose = [float(current_pose[0]), float(current_pose[1]), float(current_pose[2])]
        target_waypoint = [float(target_waypoint[0]), float(target_waypoint[1])]
        target_speed = float(target_speed)
        
        # ==================== 航线起点确定 ====================
        # 优先使用传入的 prev_waypoint，否则使用缓存，最后使用当前位置
        if prev_waypoint is not None:
            start_point = [float(prev_waypoint[0]), float(prev_waypoint[1])]
            self._prev_waypoint = start_point
        elif self._prev_waypoint is not None:
            start_point = self._prev_waypoint
        else:
            # 第一次调用，使用当前位置作为起点
            start_point = [current_pose[0], current_pose[1]]
            self._prev_waypoint = start_point
        
        # ==================== 计算航线方向 (固定，不随船位置变化!) ====================
        path_dx = target_waypoint[0] - start_point[0]
        path_dy = target_waypoint[1] - start_point[1]
        path_theta = math.atan2(path_dy, path_dx)
        
        # 处理角度周期性
        while path_theta - current_pose[2] > math.pi: path_theta -= 2*math.pi
        while path_theta - current_pose[2] < -math.pi: path_theta += 2*math.pi

        # ==================== 估计当前实际角速度 ====================
        # 使用一阶惯性模型估计 (基于上一次的状态)
        omega_actual_est = self._last_omega_actual

        # ==================== 组装参数 P (9维) ====================
        # P = [x0, y0, theta0, omega0, target_x, target_y, start_x, start_y, path_theta]
        p_val = [
            current_pose[0], current_pose[1], current_pose[2], omega_actual_est,
            target_waypoint[0], target_waypoint[1],
            start_point[0], start_point[1],
            path_theta
        ]

        # ==================== 跳变检测 ====================
        if isinstance(self.last_sol_x, list) and len(self.last_sol_x) > 4:
            last_x = self.last_sol_x[0]
            last_y = self.last_sol_x[1]
            dist_jump = math.hypot(current_pose[0] - last_x, current_pose[1] - last_y)
            if dist_jump > 1.0:
                self.reset()

        # ==================== 构造约束边界 ====================
        lbx_list = [-float('inf')] * self.n_x_all
        ubx_list = [float('inf')] * self.n_x_all
        
        # 控制变量约束
        for k in range(self.N):
            lbx_list.append(0.0)                    # v >= 0
            ubx_list.append(max(float(self.v_max), 0.1))  # v <= v_max
            lbx_list.append(-float(self.w_max))     # w >= -w_max
            ubx_list.append(float(self.w_max))      # w <= w_max

        try:
            # 准备热启动
            if hasattr(self.last_sol_x, 'tolist'):
                x0_list = self.last_sol_x.flatten().tolist()
            elif isinstance(self.last_sol_x, list):
                x0_list = self.last_sol_x
            else:
                x0_list = [0.0] * (self.n_x_all + self.n_u_all)
            
            target_len = self.n_x_all + self.n_u_all
            if len(x0_list) != target_len:
                x0_list = [0.0] * target_len

            t0 = time.time()

            # 调用求解器
            with self.lock:
                res = self.solver(
                    x0=x0_list, 
                    p=p_val, 
                    lbg=0.0, 
                    ubg=0.0, 
                    lbx=lbx_list, 
                    ubx=ubx_list
                )
            t1 = time.time()
            
            if res['x'].is_empty():
                raise RuntimeError("Solver returned empty solution")
            
            res_full = res['x'].full().flatten().tolist()
            
            if any(math.isnan(x) for x in res_full):
                raise RuntimeError("Solver returned NaNs")
                 
            self.last_sol_x = res_full
            
            # 记录调试信息
            solve_time = (t1 - t0) * 1000.0
            status = self.solver.stats()['return_status']
            cost = float(res['f']) if 'f' in res else 0.0
            
            # 解析结果
            u_start_idx = self.n_x_all
            v_cmd = float(res_full[u_start_idx])
            w_cmd = float(res_full[u_start_idx + 1])
            
            # 更新实际角速度估计 (取 X[1] 的 omega_actual)
            if len(res_full) > self.n_states + 3:
                self._last_omega_actual = float(res_full[self.n_states + 3])

            # 取最后一步预测位置
            pred_final_idx = self.n_x_all - self.n_states
            pred_final = res_full[pred_final_idx : pred_final_idx + self.n_states]
            
            # 计算实际 CTE (当前位置到航线的横向偏差)
            dx_to_start = current_pose[0] - start_point[0]
            dy_to_start = current_pose[1] - start_point[1]
            path_len = math.hypot(path_dx, path_dy)
            if path_len > 1e-6:
                path_nx = path_dx / path_len
                path_ny = path_dy / path_len
                cte = dx_to_start * path_ny - dy_to_start * path_nx
            else:
                cte = 0.0
            
            self.debug_info = {
                'solve_time_ms': solve_time,
                'cost': cost,
                'pred_x': float(pred_final[0]),
                'pred_y': float(pred_final[1]),
                'pred_theta': float(pred_final[2]),
                'pred_omega': float(pred_final[3]) if len(pred_final) > 3 else 0.0,
                'omega_actual_est': omega_actual_est,
                'path_theta_deg': math.degrees(path_theta),
                'cte': cte,
                'status': 0 if status == 'Solve_Succeeded' else 1
            }

            # 安全限幅
            effective_v_limit = min(self.v_max, target_speed)
            v_cmd = np.clip(v_cmd, 0.0, effective_v_limit) 
            w_cmd = np.clip(w_cmd, -self.w_max, self.w_max)
            
            return v_cmd, w_cmd
            
        except Exception as e:
            print(f"MPC Crash/Exception: {e}")
            self.reset()
            self.debug_info = {'status': -1}
            return 0.0, 0.0

    def set_max_speed(self, v_max):
        """动态更新最大航速"""
        if v_max > 0:
            self.v_max = v_max
    
    def set_tau_omega(self, tau):
        """
        动态更新转向时间常数
        
        注意: 修改此参数需要重新 setup_mpc()
        
        Args:
            tau: 新的转向时间常数 (s)，建议 0.3-0.6
        """
        if tau > 0.01:
            self.tau_omega = tau
            self.setup_mpc()
            print(f"MPC: tau_omega updated to {tau:.2f}s, solver rebuilt.")
    
    def get_omega_actual(self):
        """获取当前估计的实际角速度"""
        return self._last_omega_actual
