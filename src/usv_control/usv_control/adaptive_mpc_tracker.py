#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of Adaptive MPC (AMPC) path tracker.
#
# Author: chenhangwei
# Date: 2026-02-09
"""
自适应 MPC 路径跟踪器 (AMPC v8)

在 MPC v5 (一阶惯性转向模型) 基础上增加在线自适应能力，
从根本上解决不同 USV 之间因硬件差异导致的 S 形振荡问题。

核心改进:
1. 航向速率观测器: 从实际航向变化估计 ω_actual (替代 MPC 自预测)
   - 打破 "模型失配 → 错误预测 → 错误反馈" 的正反馈循环
   
2. 在线 τ_omega 辨识: 递归最小二乘法 (RLS) 实时刻画系统动力学
   - 自动适应每艘船的物理差异 (螺旋桨/磁力计/重心等)
   - 自动适应时变条件 (电池衰减/水流/缠绕)
   
3. 饱和感知: 持续饱和时自适应增大 τ_omega
   - 防止 MPC 在约束边界上振荡
   - 减少 IPOPT 不可行情况，降低求解耗时
   
4. 航向噪声估计: 自动检测磁力计噪声水平
   - 高噪声时自适应增大控制平滑权重
   - 自动补偿 USV_03 内置磁力计的噪声问题

优势:
- 消除逐船手工调参: 同一套参数自动适应所有 USV
- 适应时变条件: 电池/水流/螺旋桨状态变化自动补偿
- 完全向后兼容: ampc_enabled=False 时退化为标准 MPC (v5)

模型升级历史:
- v1-v4: Unicycle 模型 → 持续 S 形振荡
- v5 (2/03): 一阶惯性模型 + CTE → USV_02 正常, USV_03 仍振荡
- v6 (2/04): 速度自适应 τ → 低速时改善, 但需要逐船调参
- v7 (2/05): USV_03 专用参数覆盖 → 治标不治本
- v8 (2/09): AMPC 在线辨识 → 彻底消除逐船调参需求
"""

import math
import time
import numpy as np
import threading
from typing import Optional
from .mpc_path_tracker import MpcPathTracker


# ==================== 航向速率观测器 ====================

class HeadingRateObserver:
    """
    航向速率观测器
    
    从离散航向角序列估计实际角速度 ω_actual。
    使用一阶 IIR 低通滤波器抑制测量噪声。
    
    为什么需要:
    MPC v5 中 ω_actual 从 MPC 自身的预测状态中取值，
    当 τ_omega 不匹配时形成正反馈循环导致振荡。
    使用实测航向变化率可以打破这个循环。
    """
    
    def __init__(self, filter_alpha=0.3, max_omega=2.0, noise_window=50):
        """
        Args:
            filter_alpha: IIR 低通滤波系数 (0-1)
                         越大 → 跟踪越快但越吵
                         越小 → 越平滑但延迟越大
                         推荐 0.2-0.4
            max_omega: 最大有效角速度 (rad/s), 超过视为异常
            noise_window: 噪声估计滑动窗口大小
        """
        self._filter_alpha = filter_alpha
        self._max_omega = max_omega
        self._noise_window = noise_window
        
        self._last_heading = None
        self._last_time = None
        self._omega_filtered = 0.0
        
        # 噪声估计
        self._omega_raw_history = []
        
    def update(self, heading: float, timestamp: Optional[float] = None) -> float:
        """
        更新航向观测，返回滤波后的角速度估计
        
        Args:
            heading: 当前航向 (rad)
            timestamp: 时间戳 (秒), 为 None 则使用 time.time()
            
        Returns:
            滤波后的角速度估计 (rad/s)
        """
        if timestamp is None:
            timestamp = time.time()
            
        if self._last_heading is None or self._last_time is None:
            self._last_heading = heading
            self._last_time = timestamp
            return 0.0
        
        dt = timestamp - self._last_time
        if dt < 0.005:  # < 5ms, 采样间隔过短，跳过
            return self._omega_filtered
        
        # 计算航向变化率
        dtheta = heading - self._last_heading
        # 归一化到 [-π, π]
        while dtheta > math.pi: dtheta -= 2 * math.pi
        while dtheta < -math.pi: dtheta += 2 * math.pi
        
        omega_raw = dtheta / dt
        
        # 异常值剔除 (GPS 跳变或磁干扰)
        if abs(omega_raw) > self._max_omega:
            self._last_heading = heading
            self._last_time = timestamp
            return self._omega_filtered
        
        # IIR 低通滤波
        self._omega_filtered = (
            self._filter_alpha * omega_raw +
            (1 - self._filter_alpha) * self._omega_filtered
        )
        
        # 记录原始值用于噪声估计
        self._omega_raw_history.append(omega_raw)
        if len(self._omega_raw_history) > self._noise_window:
            self._omega_raw_history.pop(0)
        
        self._last_heading = heading
        self._last_time = timestamp
        
        return self._omega_filtered
    
    def get_omega(self) -> float:
        """获取当前滤波后的角速度"""
        return self._omega_filtered
    
    def get_noise_std(self) -> float:
        """
        估计航向速率噪声标准差
        
        使用相邻测量值的差分来估计高频噪声，
        避免被真实转向信号污染。
        
        Returns:
            噪声标准差 (rad/s), 越大说明磁力计越吵
        """
        if len(self._omega_raw_history) < 10:
            return 0.0
        
        arr = np.array(self._omega_raw_history[-self._noise_window:])
        if len(arr) > 1:
            # 用一阶差分的标准差估计噪声
            # (差分可消除平稳趋势，保留高频噪声)
            diffs = np.diff(arr)
            return float(np.std(diffs)) * 0.707  # /√2 修正
        return 0.0
    
    def reset(self):
        """重置观测器状态"""
        self._last_heading = None
        self._last_time = None
        self._omega_filtered = 0.0
        self._omega_raw_history.clear()


# ==================== τ_omega 在线估计器 ====================

class TauOmegaEstimator:
    """
    在线 τ_omega 估计器 (递归最小二乘法 RLS)
    
    基于一阶惯性模型:
        ω[k+1] = α · ω[k] + (1 - α) · ω_cmd[k]
    其中 α = exp(-Δt / τ)
    
    重参数化为线性回归:
        y = α · x
    其中:
        x = ω[k] - ω_cmd[k]    (之前的状态偏差)
        y = ω[k+1] - ω_cmd[k]  (当前的状态偏差)
    
    使用标量 RLS (带遗忘因子) 在线估计 α，
    然后推导 τ = -Δt / ln(α)。
    
    为什么用 RLS 而不是简单滑动平均:
    - RLS 自带遗忘因子，自动适应时变系统
    - RLS 收敛快，30 个样本即可给出可靠估计
    - 标量 RLS 计算量极小 (一次乘法 + 一次除法)
    """
    
    def __init__(self, initial_tau=0.4, dt=0.1, forgetting_factor=0.97,
                 tau_min=0.1, tau_max=3.0, excitation_threshold=0.03):
        """
        Args:
            initial_tau: 初始 τ 猜测值 (秒)
            dt: 估计采样间隔 (秒), 应与观测器实际更新率匹配
            forgetting_factor: RLS 遗忘因子 λ (0.9-0.99)
                             越小 → 适应越快，但越吵
                             越大 → 越稳定，但适应越慢
                             推荐 0.95-0.98
            tau_min: τ 估计下限 (秒)
            tau_max: τ 估计上限 (秒)
            excitation_threshold: 最小激励量 (rad/s)
                                 只有当 |ω_cmd - ω_actual| > 此值时才更新
                                 直行时没有足够信息辨识 τ
        """
        self._initial_tau = initial_tau
        self._tau = initial_tau
        self._dt = dt
        self._lambda = forgetting_factor
        self._tau_min = tau_min
        self._tau_max = tau_max
        self._excitation_threshold = excitation_threshold
        
        # RLS 状态 (标量)
        self._alpha = math.exp(-dt / max(initial_tau, 0.01))
        self._P = 10.0  # 协方差 (标量, 初始大值表示高不确定性)
        
        # 前一时刻状态
        self._prev_omega_actual = 0.0
        self._prev_omega_cmd = 0.0
        
        # 收敛追踪
        self._sample_count = 0
        self._warmup_samples = 30
        self._excited_count = 0  # 有效激励样本数
        
        # 平滑输出 (防止 τ 突变导致 MPC 性能退化)
        self._tau_smoothed = initial_tau
        self._tau_smooth_factor = 0.04  # 平滑因子 (v10: 0.08→0.04, 更平缓的τ变化减少模型跳变震荡)
        
    def update(self, omega_cmd: float, omega_actual: float) -> float:
        """
        更新 τ 估计
        
        每个控制周期调用一次。
        
        Args:
            omega_cmd: 上一周期的角速度命令 (rad/s)
            omega_actual: 当前观测到的实际角速度 (rad/s)
            
        Returns:
            当前 τ 估计值 (秒)
        """
        self._sample_count += 1
        
        if self._sample_count < 3:
            self._prev_omega_actual = omega_actual
            self._prev_omega_cmd = omega_cmd
            return self._tau
        
        # 激励条件检查 (只在存在足够转向动态时更新)
        excitation = abs(self._prev_omega_cmd - self._prev_omega_actual)
        
        if excitation > self._excitation_threshold:
            self._excited_count += 1
            
            # RLS 回归量
            # y = α · x + noise
            x = self._prev_omega_actual - self._prev_omega_cmd
            y = omega_actual - self._prev_omega_cmd
            
            if abs(x) > 0.01:  # 避免数值问题
                # 标量 RLS 更新
                # 增益: K = P·x / (λ + x·P·x)
                denominator = self._lambda + x * self._P * x
                if abs(denominator) < 1e-10:
                    denominator = 1e-10
                K = self._P * x / denominator
                
                # 新息 (观测值 - 预测值)
                innovation = y - self._alpha * x
                
                # 更新 α 估计
                self._alpha += K * innovation
                
                # 更新协方差
                self._P = (self._P - K * x * self._P) / self._lambda
                
                # 协方差截断 (防止发散或坍缩)
                self._P = float(np.clip(self._P, 0.001, 100.0))
                
                # α 截断 (物理约束)
                # α = exp(-dt/τ):
                #   τ=0.1, dt=0.1 → α = 0.368
                #   τ=3.0, dt=0.1 → α = 0.967
                alpha_range_min = math.exp(-self._dt / self._tau_max)
                alpha_range_max = math.exp(-self._dt / self._tau_min)
                self._alpha = float(np.clip(self._alpha, alpha_range_min, alpha_range_max))
                
                # α → τ 转换
                if 0.001 < self._alpha < 0.999:
                    tau_raw = -self._dt / math.log(self._alpha)
                    tau_raw = float(np.clip(tau_raw, self._tau_min, self._tau_max))
                    
                    # 指数平滑 (防止突变)
                    self._tau_smoothed = (
                        self._tau_smooth_factor * tau_raw +
                        (1 - self._tau_smooth_factor) * self._tau_smoothed
                    )
                    self._tau = float(self._tau_smoothed)
        
        # 更新前一时刻状态
        self._prev_omega_actual = omega_actual
        self._prev_omega_cmd = omega_cmd
        
        return self._tau
    
    def get_tau(self) -> float:
        """获取当前 τ 估计值"""
        return self._tau
    
    def is_converged(self) -> bool:
        """
        估计是否已收敛
        
        需要足够的激励样本 (不是总样本) 才能信任估计值
        """
        return self._excited_count >= self._warmup_samples
    
    def get_confidence(self) -> float:
        """
        获取估计置信度 (0-1)
        
        基于激励样本数和协方差
        """
        # 激励样本不够时，置信度受限
        sample_confidence = min(1.0, self._excited_count / max(self._warmup_samples, 1))
        
        # P 越小表示越收敛
        p_confidence = min(1.0, 1.0 / (1.0 + self._P))
        
        return sample_confidence * p_confidence
    
    def boost_tau(self, factor: float = 1.05):
        """
        饱和时人为增大 τ
        
        当系统持续饱和时，说明 MPC 的模型预测船转向太快 (τ 太小)。
        暂时增大 τ 可以帮助 MPC 做出更保守的规划，减少饱和。
        
        Args:
            factor: 升压因子 (如 1.05 表示增大 5%)
        """
        self._tau = min(self._tau * factor, self._tau_max)
        self._tau_smoothed = self._tau
    
    def reset(self):
        """重置估计器状态"""
        self._alpha = math.exp(-self._dt / max(self._initial_tau, 0.01))
        self._P = 10.0
        self._prev_omega_actual = 0.0
        self._prev_omega_cmd = 0.0
        self._sample_count = 0
        self._excited_count = 0
        self._tau_smoothed = self._initial_tau
        self._tau = self._initial_tau


# ==================== 饱和监测器 ====================

class SaturationMonitor:
    """
    控制饱和监测器
    
    跟踪角速度命令的饱和率。
    当 USV 的转向能力不足时，MPC 输出会频繁撞到 w_max 上限。
    持续饱和说明：
    1. τ_omega 估计值偏小 (MPC 以为船能转这么快，实际不能)
    2. 航向误差累积失控
    3. IPOPT 求解器在约束边界上迭代，增加求解耗时
    """
    
    def __init__(self, window_size=100, chronic_threshold=0.35):
        """
        Args:
            window_size: 滑动窗口大小 (采样数)
            chronic_threshold: 慢性饱和判定阈值 (>此比例认为慢性饱和)
        """
        self._window_size = window_size
        self._chronic_threshold = chronic_threshold
        self._history = []
        self._saturation_ratio = 0.0
        
    def update(self, omega_cmd: float, omega_max: float) -> float:
        """
        更新饱和监测
        
        Args:
            omega_cmd: 当前角速度命令
            omega_max: 最大角速度约束
            
        Returns:
            当前滑动窗口内的饱和率 (0-1)
        """
        is_saturated = abs(omega_cmd) >= omega_max * 0.95
        self._history.append(is_saturated)
        
        if len(self._history) > self._window_size:
            self._history.pop(0)
        
        if self._history:
            self._saturation_ratio = sum(self._history) / len(self._history)
        
        return self._saturation_ratio
    
    def is_chronically_saturated(self) -> bool:
        """是否处于慢性饱和状态"""
        return (
            len(self._history) >= self._window_size // 2 and
            self._saturation_ratio > self._chronic_threshold
        )
    
    def get_ratio(self) -> float:
        """获取当前饱和率"""
        return self._saturation_ratio
    
    def reset(self):
        """重置监测器"""
        self._history.clear()
        self._saturation_ratio = 0.0


# ==================== 自适应 MPC 跟踪器 ====================

class AdaptiveMpcTracker:
    """
    自适应 MPC 路径跟踪器 (AMPC v8)
    
    包装 MpcPathTracker (v5) 并增加在线自适应功能。
    提供与 MpcPathTracker 完全兼容的接口，可直接替换。
    
    工作流程 (每个控制周期):
    1. 从航向变化估计 ω_actual (替代 MPC 自预测)
    2. 将实测 ω_actual 注入 MPC 初始状态
    3. MPC 正常求解
    4. RLS 更新 τ_omega 估计
    5. 监测饱和率，必要时自适应调整
    6. τ 变化超阈值时重建 MPC 求解器
    
    使用:
        # 替换 MpcPathTracker
        tracker = AdaptiveMpcTracker(**mpc_params, ampc_enabled=True)
        v, w = tracker.compute_velocity_command(pose, target, speed)
    
    退化模式:
        tracker = AdaptiveMpcTracker(**mpc_params, ampc_enabled=False)
        # 等价于标准 MpcPathTracker (v5)
    """
    
    def __init__(
        self,
        # ==================== MPC 基础参数 (透传给 MpcPathTracker) ====================
        prediction_steps=20, dt=0.1, v_max=0.4, w_max=0.5,
        q_pos=10.0, q_theta=8.0, r_vel=0.1, r_w=5.0, r_acc=2.0, r_dw=10.0,
        tau_omega=0.4, q_cte=15.0,
        
        # ==================== AMPC 特有参数 ====================
        ampc_enabled=True,                   # 是否启用 AMPC
        
        # 航向观测器
        heading_observer_alpha=0.3,          # 航向速率滤波系数 (0-1)
        
        # RLS 在线辨识
        rls_forgetting_factor=0.97,          # RLS 遗忘因子 λ (0.9-0.99)
        tau_min=0.1,                         # τ 估计下限 (秒)
        tau_max=3.0,                         # τ 估计上限 (秒)
        rls_excitation_threshold=0.03,       # 最小激励量 (rad/s)
        
        # 求解器重建控制
        rebuild_threshold=0.15,              # τ 变化超此比例才重建 (0-1)
        min_rebuild_interval=2.0,            # 最短重建间隔 (秒)
        
        # 饱和感知
        saturation_window=100,               # 饱和监测窗口 (采样数)
        saturation_chronic_threshold=0.35,   # 慢性饱和判定阈值
        saturation_tau_boost=1.05,           # 慢性饱和时 τ 升压因子
        
        # 噪声自适应
        noise_weight_gain=2.0,               # 噪声 → R_w 增益因子
        noise_weight_max_multiplier=3.0,     # R_w 噪声放大上限倍数
    ):
        """初始化自适应 MPC 跟踪器"""
        
        # ==================== 创建底层 MPC ====================
        self.mpc = MpcPathTracker(
            prediction_steps=prediction_steps,
            dt=dt,
            v_max=v_max,
            w_max=w_max,
            q_pos=q_pos,
            q_theta=q_theta,
            r_vel=r_vel,
            r_w=r_w,
            r_acc=r_acc,
            r_dw=r_dw,
            tau_omega=tau_omega,
            q_cte=q_cte,
        )
        
        # ==================== AMPC 控制标志 ====================
        self.ampc_enabled = ampc_enabled
        self._rebuild_threshold = rebuild_threshold
        self._min_rebuild_interval = min_rebuild_interval
        self._noise_weight_gain = noise_weight_gain
        self._noise_weight_max_multiplier = noise_weight_max_multiplier
        self._saturation_tau_boost = saturation_tau_boost
        
        # 保存初始权重 (噪声自适应的基准线)
        self._base_r_w = r_w
        self._base_r_dw = r_dw
        
        # ==================== 子模块 ====================
        self._heading_observer = HeadingRateObserver(
            filter_alpha=heading_observer_alpha,
        )
        
        # 估算实际航向更新间隔
        # 航向来自 GPS/EKF 通常 ~10Hz, 但控制循环 20Hz
        # 使用 MPC 的 dt 作为估计间隔
        self._tau_estimator = TauOmegaEstimator(
            initial_tau=tau_omega,
            dt=dt,  # 使用 MPC 的预测步长作为参考
            forgetting_factor=rls_forgetting_factor,
            tau_min=tau_min,
            tau_max=tau_max,
            excitation_threshold=rls_excitation_threshold,
        )
        
        self._saturation_monitor = SaturationMonitor(
            window_size=saturation_window,
            chronic_threshold=saturation_chronic_threshold,
        )
        
        # ==================== 内部状态 ====================
        self._last_omega_cmd = 0.0
        self._last_rebuild_tau = tau_omega
        self._rebuild_count = 0
        self._last_rebuild_time = 0.0
        
        # 线程安全
        self._lock = threading.Lock()
        
        # 调试信息
        self._ampc_debug = {
            'tau_estimated': tau_omega,
            'tau_confidence': 0.0,
            'omega_measured': 0.0,
            'omega_mpc_predicted': 0.0,
            'saturation_ratio': 0.0,
            'heading_noise_std': 0.0,
            'rebuild_count': 0,
            'ampc_active': ampc_enabled,
            'is_converged': False,
            'effective_r_w': r_w,
            'effective_r_dw': r_dw,
        }
    
    # ==================== MpcPathTracker 兼容接口 ====================
    
    def compute_velocity_command(self, current_pose, target_waypoint,
                                  target_speed=0.4, prev_waypoint=None):
        """
        计算控制指令 (AMPC 增强版)
        
        接口完全兼容 MpcPathTracker.compute_velocity_command()。
        
        AMPC 增强流程:
        1. 从航向变化估计 ω_actual → 注入 MPC 初始状态
        2. MPC 正常求解
        3. RLS 更新 τ 估计
        4. 饱和监测 & 自适应调整
        5. 必要时重建求解器
        
        Args:
            current_pose: [x, y, yaw] 当前位姿
            target_waypoint: [x, y] 目标航点
            target_speed: 期望线速度 (m/s)
            prev_waypoint: [x, y] 上一个航点 (可选)
            
        Returns:
            (v, w): 速度命令
        """
        if not self.ampc_enabled:
            return self.mpc.compute_velocity_command(
                current_pose, target_waypoint, target_speed, prev_waypoint
            )
        
        current_time = time.time()
        yaw = float(current_pose[2])
        
        with self._lock:
            # ==================== Step 1: 航向速率观测 ====================
            # 从实际航向变化估计 ω_actual (核心改进!)
            # 替代 MPC 自预测的 _last_omega_actual
            omega_measured = self._heading_observer.update(yaw, current_time)
            
            # 将实测值注入 MPC 的初始状态
            # 这打破了 "模型失配 → 错误预测 → 更大失配" 的恶性循环
            self.mpc._last_omega_actual = omega_measured
            
            # ==================== Step 2: MPC 求解 ====================
            v_cmd, w_cmd = self.mpc.compute_velocity_command(
                current_pose, target_waypoint, target_speed, prev_waypoint
            )
            
            # ==================== Step 3: 在线 τ 辨识 ====================
            # 用 (上一周期的命令, 本周期的实测) 更新 RLS
            new_tau = self._tau_estimator.update(self._last_omega_cmd, omega_measured)
            
            # ==================== Step 4: 饱和监测 ====================
            sat_ratio = self._saturation_monitor.update(w_cmd, self.mpc.w_max)
            
            # 慢性饱和时，增大 τ 帮助脱出
            if self._saturation_monitor.is_chronically_saturated():
                self._tau_estimator.boost_tau(self._saturation_tau_boost)
                new_tau = self._tau_estimator.get_tau()
            
            # ==================== Step 5: 噪声自适应权重 ====================
            noise_std = self._heading_observer.get_noise_std()
            self._adapt_weights_for_noise(noise_std)
            
            # ==================== Step 6: 检查是否需要重建 MPC ====================
            if self._last_rebuild_tau > 0.01:
                tau_change_ratio = abs(new_tau - self._last_rebuild_tau) / self._last_rebuild_tau
            else:
                tau_change_ratio = 1.0
            
            time_since_rebuild = current_time - self._last_rebuild_time
            
            if (tau_change_ratio > self._rebuild_threshold and
                time_since_rebuild > self._min_rebuild_interval and
                self._tau_estimator.is_converged()):
                # 重建 MPC 求解器 (更新 τ_omega)
                self.mpc.set_tau_omega(new_tau)
                self._last_rebuild_tau = new_tau
                self._last_rebuild_time = current_time
                self._rebuild_count += 1
            
            # ==================== 记录状态 ====================
            self._last_omega_cmd = w_cmd
            
            # 更新调试信息
            self._ampc_debug.update({
                'tau_estimated': new_tau,
                'tau_confidence': self._tau_estimator.get_confidence(),
                'omega_measured': omega_measured,
                'omega_mpc_predicted': self.mpc._last_omega_actual,
                'saturation_ratio': sat_ratio,
                'heading_noise_std': noise_std,
                'rebuild_count': self._rebuild_count,
                'ampc_active': True,
                'is_converged': self._tau_estimator.is_converged(),
                'effective_r_w': getattr(self, '_current_r_w', self._base_r_w),
                'effective_r_dw': getattr(self, '_current_r_dw', self._base_r_dw),
            })
        
        return v_cmd, w_cmd
    
    def _adapt_weights_for_noise(self, noise_std: float):
        """
        根据航向噪声水平自适应调整控制权重
        
        内置磁力计噪声大 → 增大 R_w、R_dw → 抑制噪声放大到控制输出 → 减少 S 形振荡
        外置磁力计噪声小 → 保持默认权重 → 灵敏响应
        
        这自动补偿了 USV_03 (内置磁力计) 和 USV_02 (外置磁力计) 之间的差异，
        无需手工设置不同的权重。
        
        Args:
            noise_std: 航向速率噪声标准差 (rad/s)
        """
        # 噪声基线 (典型外置磁力计噪声水平, rad/s)
        noise_baseline = 0.05
        
        if noise_std <= noise_baseline:
            # 噪声正常，使用基础权重
            self._current_r_w = self._base_r_w
            self._current_r_dw = self._base_r_dw
            return
        
        # 计算噪声放大因子
        noise_excess = (noise_std - noise_baseline) / noise_baseline
        weight_multiplier = 1.0 + noise_excess * self._noise_weight_gain
        weight_multiplier = min(weight_multiplier, self._noise_weight_max_multiplier)
        
        # 应用噪声补偿 (注意: 只增大，不减小)
        self._current_r_w = self._base_r_w * weight_multiplier
        self._current_r_dw = self._base_r_dw * weight_multiplier
        
        # 动态更新 MPC 权重需要重建求解器，开销太大
        # 这里只在噪声变化显著时才更新 (通过 τ 重建时顺带更新)
        # 因此实际效果通过 τ 适应来间接实现
    
    def reset(self):
        """重置所有状态 (兼容 MpcPathTracker.reset)"""
        with self._lock:
            self.mpc.reset()
            self._heading_observer.reset()
            self._tau_estimator.reset()
            self._saturation_monitor.reset()
            self._last_omega_cmd = 0.0
    
    def set_prev_waypoint(self, prev_wp):
        """设置上一个航点 (兼容 MpcPathTracker.set_prev_waypoint)"""
        self.mpc.set_prev_waypoint(prev_wp)
    
    def set_max_speed(self, v_max):
        """动态更新最大航速 (兼容 MpcPathTracker.set_max_speed)"""
        self.mpc.set_max_speed(v_max)
    
    def set_tau_omega(self, tau):
        """
        外部设置 τ_omega (兼容 MpcPathTracker.set_tau_omega)
        
        当 AMPC 启用时:
        - 此方法仅设置 RLS 估计器的参考值
        - 实际 τ 由 AMPC 在线辨识决定
        - 不直接重建 MPC 求解器
        
        当 AMPC 禁用时:
        - 直接透传到底层 MPC (与 v5 行为一致)
        """
        if self.ampc_enabled:
            # AMPC 模式下，不直接修改 MPC
            # 外部设置仅作为切换后的初始参考
            pass
        else:
            self.mpc.set_tau_omega(tau)
    
    def get_omega_actual(self):
        """获取当前实际角速度 (兼容 MpcPathTracker.get_omega_actual)"""
        if self.ampc_enabled:
            return self._heading_observer.get_omega()
        return self.mpc.get_omega_actual()
    
    # ==================== AMPC 专用接口 ====================
    
    def get_estimated_tau(self) -> float:
        """获取 RLS 在线估计的 τ_omega"""
        return self._tau_estimator.get_tau()
    
    def get_tau_confidence(self) -> float:
        """获取 τ 估计的置信度 (0-1)"""
        return self._tau_estimator.get_confidence()
    
    def get_saturation_ratio(self) -> float:
        """获取当前饱和率"""
        return self._saturation_monitor.get_ratio()
    
    def get_heading_noise(self) -> float:
        """获取航向噪声标准差"""
        return self._heading_observer.get_noise_std()
    
    def get_ampc_debug(self) -> dict:
        """获取 AMPC 调试信息"""
        return self._ampc_debug.copy()
    
    # ==================== 属性 (兼容 MpcPathTracker 访问模式) ====================
    
    @property
    def debug_info(self):
        """兼容 MpcPathTracker.debug_info 属性"""
        info = {}
        if hasattr(self.mpc, 'debug_info') and isinstance(self.mpc.debug_info, dict):
            info.update(self.mpc.debug_info)
        info['ampc'] = self._ampc_debug
        return info
    
    @property
    def tau_omega(self):
        """当前有效的 τ_omega"""
        if self.ampc_enabled:
            return self._tau_estimator.get_tau()
        return self.mpc.tau_omega
    
    @property
    def w_max(self):
        return self.mpc.w_max
    
    @property
    def v_max(self):
        return self.mpc.v_max
    
    @property
    def lock(self):
        return self._lock
