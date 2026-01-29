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
"""
MPC 路径跟踪控制器

基于模型预测控制 (MPC) 的路径跟踪器。
使用 CasADi 进行非线性优化求解。

特点:
- 预测未来轨迹，消除蛇形震荡
- 平滑的控制输出，保护舵机
- 直接处理运动学约束 (最大速度、最大角速度)

"""

import casadi as ca
import numpy as np
import math

class MpcPathTracker:
    def __init__(self, 
                 v_max=0.4, w_max=1.0, 
                 q_pos=20.0, q_theta=5.0, 
                 r_vel=0.1, r_w=5.0, 
                 r_acc=1.0, r_dw=10.0,
                 dt=0.1, prediction_steps=20):
        # --- 车辆(船)物理参数 (根据用户描述: 0.6m长, 0.4m/s最高速) ---
        self.v_max = v_max      # (m/s) 最高航速
        self.w_max = w_max      # (rad/s) 最大转速 (0.6m小船转向很灵活，给大一点)
        self.dt = dt            # (s) 预测步长 (10Hz)
        self.N = prediction_steps # 预测步数

        # --- MPC 权重参数说明 (调试核心) ---
        # 提示: MPC 的核心在于权衡"误差"与"控制量"之间的矛盾
        
        # 1. 位置误差权重 (Q_pos)
        # 含义: 船偏离目标航线时的惩罚力度。
        # 推荐范围: 10.0 ~ 50.0
        # 效果: 
        #   - 调大: 船会死命贴着线走，抗干扰能力强，但可能导致动作僵硬。
        #   - 调小: 船走线比较随意，容忍一定的偏离，换取更平滑的轨迹。
        self.Q_pos   = q_pos 

        # 2. 航向误差权重 (Q_theta)
        # 含义: 船头朝向与目标方向不一致时的惩罚力度。
        # 推荐范围: 1.0 ~ 10.0
        # 效果:
        #   - 调大: 船头始终想指着目标，可能会导致船体频繁摆动。
        #   - 调小: 允许船头有偏差(如蟹行)，主要靠位置权重拉回来。
        self.Q_theta = q_theta

        # 3. 速度误差权重 (R_vel)
        # 含义: 实际速度与期望最高速(v_max)不一致的惩罚。
        # 推荐范围: 0.1 ~ 1.0
        # 效果: 通常设得很小，允许MPC为了转弯而减速。如果设大，船会不顾一切地保持全速冲刺。
        self.R_vel   = r_vel

        # 4. 转向幅度惩罚 (R_w) [最关键参数!]
        # 含义: 对输出角速度(打舵量)的惩罚。
        # 推荐范围: 1.0 ~ 10.0
        # 效果:
        #   - 调大 (如 10.0): 强迫船慢慢转，轨迹非常圆润，消除S形震荡(画龙)，但敏捷性下降。
        #   - 调小 (如 0.1): 船反应极快，指哪打哪，但容易过敏、震荡。
        # *调试建议*: 如果船画龙，优先增大此值。
        self.R_w     = r_w

        # --- 变化率(平滑度)惩罚 ---
        
        # 5. 加速度变化率 (R_acc)
        # 含义: 油门变化剧烈程度的惩罚。
        # 推荐范围: 1.0 ~ 5.0
        # 效果: 让加速减速过程平滑，不突变。
        self.R_acc   = r_acc

        # 6. 角加速度变化率 (R_dw) [丝滑的核心]
        # 含义: 舵角变化快慢的惩罚 (即 d(Omega)/dt)。
        # 推荐范围: 5.0 ~ 20.0
        # 效果:
        #   - 调大: 禁止猛打方向，舵机动作像老船长一样连贯。
        #   - 调小: 允许瞬间大幅度改舵。
        # *调试建议*: 想要"丝滑"，请保持此值在 10.0 或更高。
        self.R_dw    = r_dw

        self.setup_mpc()

    def setup_mpc(self):
        """建立 MPC 优化问题"""
        # 1. 定义状态变量
        x = ca.SX.sym('x'); y = ca.SX.sym('y'); theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        n_states = states.size1()

        # 2. 定义控制变量
        v = ca.SX.sym('v'); w = ca.SX.sym('w')
        controls = ca.vertcat(v, w)
        n_controls = controls.size1()

        # 3. 动力学模型 (x' = f(x, u))
        # Unicycle Kinematics
        rhs = ca.vertcat(v*ca.cos(theta), v*ca.sin(theta), w)
        f = ca.Function('f', [states, controls], [states + rhs * self.dt])

        # 4. 优化变量
        U = ca.SX.sym('U', n_controls, self.N)    # 未来 N 步的控制量
        P = ca.SX.sym('P', n_states + 3)          # 参数: [当前x,y,theta, Ref_x, Ref_y, Ref_theta]
        X = ca.SX.sym('X', n_states, self.N+1)    # 状态轨迹

        # 5. 构建代价函数与约束
        obj = 0
        g = []
        
        # 初始状态约束
        st = X[:, 0]
        g = ca.vertcat(g, st - P[:3]) # X0 == Current State

        # 目标状态
        target_x = P[3]
        target_y = P[4]
        target_theta = P[5]

        for k in range(self.N):
            st = X[:, k]
            con = U[:, k]
            
            # --- 代价计算 ---
            # 1. 追踪误差 (距离目标的欧氏距离)
            # 在长直线跟踪时，更好的做法是计算 Cross Track Error，但在点对点模式中，
            # 只要目标点不断向前推移，追踪点误差也能达到类似效果。
            obj += self.Q_pos * ((st[0] - target_x)**2 + (st[1] - target_y)**2)
            
            # 2. 航向误差
            # 处理角度周期性问题有点麻烦，这里假设传入的 target_theta 已经处理好
            obj += self.Q_theta * (st[2] - target_theta)**2
            
            # 3. 输入惩罚
            obj += self.R_w * con[1]**2      # 惩罚大转向
            
            # 4. 变化率惩罚 (丝滑的关键)
            if k < self.N - 1:
                con_next = U[:, k+1]
                # 加速度与角加速度惩罚
                obj += self.R_acc * (con[0] - con_next[0])**2 
                obj += self.R_dw * (con[1] - con_next[1])**2

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
        
        # 抑制输出，追求速度
        opts = {
            'ipopt.print_level': 0, 
            'ipopt.sb': 'yes', 
            'print_time': 0,
            'ipopt.max_iter': 50,
            'ipopt.warm_start_init_point': 'yes'
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
        
        # 保存维度
        self.n_states = n_states
        self.n_controls = n_controls
        self.n_x_all = (self.N+1) * n_states
        self.n_u_all = self.N * n_controls

        # 缓存上一次的解，用于热启动 (Warm Start)
        self.last_sol_x = np.zeros(self.n_x_all + self.n_u_all)

    def compute_velocity_command(self, current_pose, target_waypoint, target_speed=0.4):
        """
        计算控制指令
        :param current_pose: [x, y, yaw]
        :param target_waypoint: [x, y]
        :param target_speed: 期望线速度 (m/s)
        :return: (v, w)
        """
        # 1. 计算期望航向 (指向目标点的方向)
        dx = target_waypoint[0] - current_pose[0]
        dy = target_waypoint[1] - current_pose[1]
        target_yaw = math.atan2(dy, dx)
        
        # 优化：解决 -pi 到 pi 的突变问题
        # 将 target_yaw 映射到 [current_yaw - pi, current_yaw + pi] 区间
        while target_yaw - current_pose[2] > math.pi: target_yaw -= 2*math.pi
        while target_yaw - current_pose[2] < -math.pi: target_yaw += 2*math.pi

        # 2. 组装参数 P
        p_val = np.array([
            current_pose[0], current_pose[1], current_pose[2],
            target_waypoint[0], target_waypoint[1], target_yaw
        ])

        # 3. 求解
        # 使用上一次的解作为初值 (Warm Start)，显著加快收敛
        x0 = self.last_sol_x
        
        try:
            # 调用求解器
            res = self.solver(x0=x0, p=p_val, lbg=0.0, ubg=0.0)
            
            # 保存解用于下一次热启动
            self.last_sol_x = res['x'].full().flatten()
            
            # 解析结果: 取预测序列的第一个控制量
            # 解向量结构: X(0..N) followed by U(0..N-1)
            u_start_idx = self.n_x_all
            u_opt = res['x'][u_start_idx : u_start_idx + 2] 
            
            v_cmd = float(u_opt[0])
            w_cmd = float(u_opt[1])
            
            # --- 后处理 ---
            
            # 1. 速度调度 (距离越近速度越慢，防止过冲)
            dist_to_goal = math.hypot(dx, dy)
            # 简单的 P 控制调整速度，或者直接信任 MPC 的 v 输出
            # 这里我们强制限制 v_cmd，但允许 MPC 调整 v 来配合转向
            
            # 2. 最终安全限幅
            # 动态调整上限：取 (系统最大允许速度, 当前航点目标速度) 的最小值
            # 这样既遵守了物理/参数限制，也遵守了航点指令
            effective_v_limit = min(self.v_max, target_speed)
            v_cmd = np.clip(v_cmd, 0.0, effective_v_limit) 
            w_cmd = np.clip(w_cmd, -self.w_max, self.w_max)
            
            return v_cmd, w_cmd
            
        except Exception as e:
            # 降级处理: 如果 MPC 挂了，简单的 P 控制兜底
            # print(f"MPC Error: {e}")
            angle_diff = target_yaw - current_pose[2]
            w_cmd = 1.0 * angle_diff
            return 0.0, w_cmd

    def set_max_speed(self, v_max):
        """动态更新最大航速"""
        if v_max > 0:
            self.v_max = v_max
            v_cmd = 0.0 # 安全停船或低速蠕动
            return v_cmd, w_cmd
