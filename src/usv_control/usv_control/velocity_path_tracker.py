#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of velocity path tracker.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
USV 速度模式路径跟踪控制器

算法: Pure Pursuit + Stanley 混合控制
特点: 
- 远距离快速收敛 (Pure Pursuit)
- 近距离精确跟踪 (Stanley)
- 不依赖飞控减速逻辑，彻底解决航点附近减速问题

参考: 
- PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics
- Stanley: Stanford DARPA Challenge

作者: Auto-generated
日期: 2026-01-22
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Deque
from enum import Enum
from collections import deque
import time


# ==================== 安全常量 ====================
MAX_VALID_DISTANCE = 500.0      # 最大有效航点距离 (m)
MAX_VALID_SPEED = 5.0           # 最大有效速度 (m/s)
MIN_VALID_SPEED = 0.01          # 最小有效速度 (m/s)
MAX_VALID_ANGULAR = 2.0         # 最大有效角速度 (rad/s)
HEADING_DEADZONE = 0.02         # 航向死区 (rad), ~1度
POSITION_DEADZONE = 0.05        # 位置死区 (m)
NAN_REPLACEMENT = 0.0           # NaN 替换值


@dataclass
class Pose2D:
    """2D 位姿"""
    x: float
    y: float
    yaw: float  # 弧度
    
    def distance_to(self, other: 'Pose2D') -> float:
        """计算到另一个位置的距离"""
        return math.hypot(other.x - self.x, other.y - self.y)
    
    def is_valid(self) -> bool:
        """检查位姿是否有效"""
        # 检查 NaN
        if math.isnan(self.x) or math.isnan(self.y) or math.isnan(self.yaw):
            return False
        # 检查 Inf
        if math.isinf(self.x) or math.isinf(self.y) or math.isinf(self.yaw):
            return False
        # 检查合理范围 (GPS 坐标转换后通常在 ±10000m 以内)
        if abs(self.x) > 100000 or abs(self.y) > 100000:
            return False
        return True


@dataclass
class Waypoint:
    """航点"""
    x: float
    y: float
    speed: float = 0.5  # 期望速度 (m/s)
    goal_id: int = 0    # 目标 ID
    is_final: bool = False  # 是否是最终航点
    
    def to_pose(self) -> Pose2D:
        """转换为 Pose2D (yaw=0)"""
        return Pose2D(self.x, self.y, 0.0)
    
    def is_valid(self) -> bool:
        """检查航点是否有效"""
        if math.isnan(self.x) or math.isnan(self.y):
            return False
        if math.isinf(self.x) or math.isinf(self.y):
            return False
        if abs(self.x) > 100000 or abs(self.y) > 100000:
            return False
        if self.speed < MIN_VALID_SPEED or self.speed > MAX_VALID_SPEED:
            return False
        return True
    
    def distance_from(self, pose: Pose2D) -> float:
        """计算到指定位置的距离"""
        return math.hypot(self.x - pose.x, self.y - pose.y)


@dataclass
class VelocityCommand:
    """速度指令"""
    linear_x: float   # 前向速度 (m/s)
    linear_y: float   # 侧向速度 (m/s), 对于差速船通常为 0
    angular_z: float  # 角速度 (rad/s)
    
    @staticmethod
    def stop() -> 'VelocityCommand':
        """停止指令"""
        return VelocityCommand(0.0, 0.0, 0.0)
    
    def sanitize(self) -> 'VelocityCommand':
        """
        清理无效值，确保指令安全
        
        处理 NaN、Inf 和超限值
        """
        def safe_value(v: float, max_abs: float, default: float = 0.0) -> float:
            if math.isnan(v) or math.isinf(v):
                return default
            return np.clip(v, -max_abs, max_abs)
        
        return VelocityCommand(
            linear_x=safe_value(self.linear_x, MAX_VALID_SPEED),
            linear_y=safe_value(self.linear_y, MAX_VALID_SPEED),
            angular_z=safe_value(self.angular_z, MAX_VALID_ANGULAR)
        )
    
    def is_valid(self) -> bool:
        """检查指令是否有效"""
        for v in [self.linear_x, self.linear_y, self.angular_z]:
            if math.isnan(v) or math.isinf(v):
                return False
        return True


class ControllerType(Enum):
    """控制器类型"""
    PURE_PURSUIT = 1   # 纯追踪
    STANLEY = 2        # Stanley 控制器
    HYBRID = 3         # 混合控制（推荐）


class VelocityPathTracker:
    """
    速度模式路径跟踪控制器
    
    使用方法:
        tracker = VelocityPathTracker()
        tracker.set_waypoint(waypoint)  # 或 add_waypoint
        
        while not tracker.is_goal_reached():
            cmd = tracker.compute_velocity(current_pose)
            publish_velocity(cmd)
    
    特点:
    - 混合控制: 远距离用 Pure Pursuit, 近距离用 Stanley
    - 航点队列: 支持连续航点，平滑切换
    - 自适应速度: 曲率大时减速，接近目标时减速
    - 安全保护: 位姿无效时返回停止指令
    """
    
    def __init__(
        self,
        # Pure Pursuit 参数
        lookahead_distance: float = 2.0,      # 前视距离 (m)
        min_lookahead: float = 1.0,           # 最小前视距离
        max_lookahead: float = 5.0,           # 最大前视距离
        lookahead_gain: float = 0.5,          # 前视距离增益 (L = gain * v + min)
        
        # Stanley 参数
        stanley_gain: float = 2.5,            # Stanley 增益 k
        stanley_softening: float = 0.1,       # 软化系数，防止低速时不稳定
        
        # 混合控制参数
        controller_type: ControllerType = ControllerType.HYBRID,
        hybrid_switch_distance: float = 2.0,  # 切换距离阈值
        
        # 速度参数
        cruise_speed: float = 0.5,            # 巡航速度 (m/s)
        max_angular_velocity: float = 0.5,    # 最大角速度 (rad/s)
        min_speed: float = 0.05,              # 最小速度 (m/s)
        
        # 到达判断
        goal_tolerance: float = 0.5,          # 目标容差 (m)
        switch_tolerance: float = 1.5,        # 中间航点切换容差 (m)
        
        # 平滑参数
        angular_velocity_filter: float = 0.3, # 角速度低通滤波系数 (0-1, 越小越平滑)
        
        # 航点队列
        waypoint_queue_size: int = 10,        # 航点队列大小
    ):
        # ==================== Pure Pursuit 参数 ====================
        self.lookahead_distance = lookahead_distance
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_gain = lookahead_gain
        
        # ==================== Stanley 参数 ====================
        self.stanley_gain = stanley_gain
        self.stanley_softening = stanley_softening
        
        # ==================== 混合控制 ====================
        self.controller_type = controller_type
        self.hybrid_switch_distance = hybrid_switch_distance
        
        # ==================== 速度参数 ====================
        self.cruise_speed = cruise_speed
        self.max_angular_velocity = max_angular_velocity
        self.min_speed = min_speed
        
        # ==================== 到达判断 ====================
        self.goal_tolerance = goal_tolerance
        self.switch_tolerance = switch_tolerance
        
        # ==================== 滤波 ====================
        self.angular_velocity_filter = angular_velocity_filter
        self._last_angular_velocity = 0.0
        
        # ==================== 航点队列 ====================
        self._waypoint_queue: Deque[Waypoint] = deque(maxlen=waypoint_queue_size)
        self._current_waypoint: Optional[Waypoint] = None
        self._goal_reached = True
        
        # ==================== 安全与诊断 ====================
        self._last_pose_time: float = 0.0
        self._pose_timeout: float = 2.0  # 位姿超时时间 (秒)
        self._consecutive_errors: int = 0
        self._max_consecutive_errors: int = 10  # 连续错误阈值
        self._last_valid_distance: float = 0.0
        self._stall_detection_enabled: bool = True
        self._stall_distance_threshold: float = 0.1  # 停滞检测阈值 (m)
        self._stall_check_interval: float = 5.0  # 停滞检测间隔 (秒)
        self._last_stall_check_time: float = 0.0
        self._last_stall_check_distance: float = 0.0
        
        # ==================== 路径历史 (用于 Stanley) ====================
        self._path_history: List[Waypoint] = []
        self._max_path_history = 50
    
    # ==================== 公共接口 ====================
    
    def set_waypoint(self, waypoint: Waypoint):
        """
        设置单个航点（清空队列）
        
        Args:
            waypoint: 目标航点
        """
        # 验证航点
        if not waypoint.is_valid():
            # 尝试修复速度
            waypoint.speed = np.clip(waypoint.speed, MIN_VALID_SPEED, MAX_VALID_SPEED)
        
        self._waypoint_queue.clear()
        self._path_history.clear()
        self._current_waypoint = waypoint
        self._goal_reached = False
        self._last_angular_velocity = 0.0
        self._last_pose_time = time.time()  # 记录时间
        
        # 注意：不再强制设置 is_final = True
        # is_final 由调用者根据 nav_mode 决定
        # 这样可以支持平滑导航，让 navigate_to_point_node 统一管理到达判断
    
    def add_waypoint(self, waypoint: Waypoint):
        """
        添加航点到队列末尾
        
        Args:
            waypoint: 目标航点
        """
        # 验证航点
        if not waypoint.is_valid():
            waypoint.speed = np.clip(waypoint.speed, MIN_VALID_SPEED, MAX_VALID_SPEED)
        
        # 如果当前没有航点，直接设置
        if self._current_waypoint is None:
            self._current_waypoint = waypoint
            self._goal_reached = False
            self._last_pose_time = time.time()
        else:
            self._waypoint_queue.append(waypoint)
        
        # 更新最终航点标记
        self._update_final_waypoint_flags()
    
    def clear_waypoints(self):
        """清空所有航点"""
        self._waypoint_queue.clear()
        self._path_history.clear()
        self._current_waypoint = None
        self._goal_reached = True
        self._last_angular_velocity = 0.0
    
    def is_goal_reached(self) -> bool:
        """是否到达最终目标"""
        return self._goal_reached
    
    def get_current_waypoint(self) -> Optional[Waypoint]:
        """获取当前目标航点"""
        return self._current_waypoint
    
    def get_queue_length(self) -> int:
        """获取队列中剩余航点数量"""
        return len(self._waypoint_queue)
    
    def get_distance_to_goal(self, pose: Pose2D) -> float:
        """获取到当前目标的距离"""
        if self._current_waypoint is None:
            return 0.0
        return math.hypot(
            self._current_waypoint.x - pose.x,
            self._current_waypoint.y - pose.y
        )
    
    def compute_velocity(self, pose: Pose2D) -> VelocityCommand:
        """
        计算速度指令（主接口）
        
        包含完整的安全检查:
        - 位姿有效性验证
        - NaN/Inf 值过滤
        - 死区处理
        - 异常情况恢复
        
        Args:
            pose: 当前位姿
            
        Returns:
            VelocityCommand: 经过安全校验的速度指令
        """
        current_time = time.time()
        
        # ==================== 安全检查 1: 空值 ====================
        if pose is None:
            self._consecutive_errors += 1
            return VelocityCommand.stop()
        
        # ==================== 安全检查 2: 位姿有效性 ====================
        if not pose.is_valid():
            self._consecutive_errors += 1
            if self._consecutive_errors > self._max_consecutive_errors:
                # 连续错误过多，停止导航
                self._goal_reached = True
            return VelocityCommand.stop()
        
        # 重置错误计数
        self._consecutive_errors = 0
        self._last_pose_time = current_time
        
        # ==================== 安全检查 3: 目标存在性 ====================
        if self._current_waypoint is None or self._goal_reached:
            return VelocityCommand.stop()
        
        # ==================== 安全检查 4: 距离合理性 ====================
        distance_to_waypoint = self.get_distance_to_goal(pose)
        
        # 距离异常检测 (突然跳变)
        if self._last_valid_distance > 0:
            distance_change = abs(distance_to_waypoint - self._last_valid_distance)
            if distance_change > MAX_VALID_DISTANCE:
                # 距离突变，可能是定位跳变，暂停等待
                return VelocityCommand.stop()
        self._last_valid_distance = distance_to_waypoint
        
        # 更新路径历史 (用于 Stanley)
        self._update_path_history(pose)
        
        # 检查是否需要切换航点或到达
        if self._check_waypoint_transition(distance_to_waypoint):
            # 已经到达最终目标
            if self._goal_reached:
                return VelocityCommand.stop()
        
        # 获取当前目标（可能已切换）
        target = self._current_waypoint
        if target is None:
            return VelocityCommand.stop()
        
        distance_to_target = self.get_distance_to_goal(pose)
        
        # ==================== 死区处理 ====================
        # 非常接近目标且是最终航点时，应用位置死区
        if target.is_final and distance_to_target < POSITION_DEADZONE:
            self._goal_reached = True
            return VelocityCommand.stop()
        
        # ==================== 大航向误差处理 ====================
        # 计算航向误差
        dx = target.x - pose.x
        dy = target.y - pose.y
        target_yaw = math.atan2(dy, dx)
        heading_error = self._normalize_angle(target_yaw - pose.yaw)
        
        # 当航向误差 > 90° 时，Pure Pursuit 会失效（目标在后方）
        # 此时应该原地转向而非前进
        LARGE_HEADING_ERROR_THRESHOLD = math.pi / 2  # 90°
        
        if abs(heading_error) > LARGE_HEADING_ERROR_THRESHOLD:
            # 大航向误差模式：减速 + 最大角速度转向
            # 确定转向方向（正误差左转，负误差右转）
            if heading_error > 0:
                angular_velocity = self.max_angular_velocity
            else:
                angular_velocity = -self.max_angular_velocity
            
            # 减速因子：航向误差越大，速度越慢
            # 180° → 0.1 倍速, 90° → 0.5 倍速
            heading_factor = 1.0 - (abs(heading_error) - LARGE_HEADING_ERROR_THRESHOLD) / LARGE_HEADING_ERROR_THRESHOLD
            heading_factor = max(0.1, min(0.5, heading_factor))
            linear_speed = target.speed * heading_factor
            
            # 应用滤波
            angular_velocity = (
                self.angular_velocity_filter * angular_velocity +
                (1 - self.angular_velocity_filter) * self._last_angular_velocity
            )
            self._last_angular_velocity = angular_velocity
            
            cmd = VelocityCommand(
                linear_x=linear_speed,
                linear_y=0.0,
                angular_z=angular_velocity
            )
            return cmd.sanitize()
        
        # ==================== 计算控制指令 ====================
        # 根据控制器类型计算角速度
        if self.controller_type == ControllerType.PURE_PURSUIT:
            angular_velocity = self._pure_pursuit(pose, target)
        elif self.controller_type == ControllerType.STANLEY:
            angular_velocity = self._stanley(pose, target)
        else:  # HYBRID
            angular_velocity = self._hybrid_control(pose, target, distance_to_target)
        
        # ==================== 输出安全校验 ====================
        # 检查 NaN
        if math.isnan(angular_velocity):
            angular_velocity = 0.0
        
        # 限制角速度
        angular_velocity = np.clip(
            angular_velocity, 
            -self.max_angular_velocity, 
            self.max_angular_velocity
        )
        
        # 航向死区 (非常小的角速度归零，避免抖动)
        if abs(angular_velocity) < HEADING_DEADZONE:
            angular_velocity = 0.0
        
        # ==================== 自适应低通滤波 ====================
        # 航向误差大时减小滤波系数（快速响应），误差小时增大滤波（平稳）
        adaptive_filter = self._compute_adaptive_filter(heading_error)
        angular_velocity = (
            adaptive_filter * angular_velocity +
            (1 - adaptive_filter) * self._last_angular_velocity
        )
        self._last_angular_velocity = angular_velocity
        
        # 计算自适应线速度
        linear_speed = self._compute_adaptive_speed(
            target.speed, 
            angular_velocity, 
            distance_to_target,
            target.is_final
        )
        
        # 构建并清理指令
        cmd = VelocityCommand(
            linear_x=linear_speed,
            linear_y=0.0,
            angular_z=angular_velocity
        )
        
        # 最终安全校验
        return cmd.sanitize()
    
    # ==================== 内部方法 ====================
    
    def _update_final_waypoint_flags(self):
        """更新所有航点的 is_final 标记"""
        # 当前航点和队列中的航点
        all_waypoints = []
        if self._current_waypoint:
            all_waypoints.append(self._current_waypoint)
        all_waypoints.extend(self._waypoint_queue)
        
        # 只有最后一个是 final
        for i, wp in enumerate(all_waypoints):
            wp.is_final = (i == len(all_waypoints) - 1)
    
    def _update_path_history(self, pose: Pose2D):
        """更新路径历史"""
        if self._current_waypoint:
            # 添加当前航点到历史
            if not self._path_history or (
                self._path_history[-1].x != self._current_waypoint.x or
                self._path_history[-1].y != self._current_waypoint.y
            ):
                self._path_history.append(self._current_waypoint)
                
                # 限制历史长度
                if len(self._path_history) > self._max_path_history:
                    self._path_history.pop(0)
    
    def _check_waypoint_transition(self, distance: float) -> bool:
        """
        检查是否需要切换航点或到达
        
        Returns:
            True 如果已到达最终目标
        """
        if self._current_waypoint is None:
            return True
        
        # 判断使用哪个阈值
        if self._current_waypoint.is_final:
            # 最终航点：使用到达阈值
            threshold = self.goal_tolerance
        else:
            # 中间航点：使用切换阈值（更大）
            threshold = self.switch_tolerance
        
        if distance < threshold:
            if self._current_waypoint.is_final:
                # 到达最终目标
                self._goal_reached = True
                return True
            else:
                # 切换到下一个航点
                self._switch_to_next_waypoint()
                return False
        
        return False
    
    def _switch_to_next_waypoint(self):
        """切换到下一个航点"""
        if self._waypoint_queue:
            # 保存当前航点到历史
            if self._current_waypoint:
                self._path_history.append(self._current_waypoint)
            
            # 从队列获取下一个航点
            self._current_waypoint = self._waypoint_queue.popleft()
            
            # 更新 final 标记
            self._update_final_waypoint_flags()
        else:
            # 队列为空，当前航点变成最终航点
            if self._current_waypoint:
                self._current_waypoint.is_final = True
    
    def _pure_pursuit(self, pose: Pose2D, target: Waypoint) -> float:
        """
        Pure Pursuit 算法
        
        原理: 追踪前视点，计算转向曲率
        κ = 2 * sin(α) / L
        ω = v * κ
        
        Args:
            pose: 当前位姿
            target: 目标航点
            
        Returns:
            angular_velocity: 角速度 (rad/s)
        """
        # 计算自适应前视距离
        L = self.lookahead_gain * self.cruise_speed + self.min_lookahead
        L = np.clip(L, self.min_lookahead, self.max_lookahead)
        
        # 目标点作为前视点
        lookahead_x = target.x
        lookahead_y = target.y
        
        # 计算到前视点的向量 (在世界坐标系)
        dx = lookahead_x - pose.x
        dy = lookahead_y - pose.y
        
        # 转换到车身坐标系
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        
        # 计算到前视点的距离
        ld = math.hypot(local_x, local_y)
        if ld < 0.1:
            return 0.0
        
        # 计算曲率: κ = 2 * y / L²
        # 这里用实际距离代替固定 L
        curvature = 2.0 * local_y / (ld * ld)
        
        # 角速度 = 速度 * 曲率
        angular_velocity = target.speed * curvature
        
        return angular_velocity
    
    def _stanley(self, pose: Pose2D, target: Waypoint) -> float:
        """
        Stanley 控制器
        
        原理: 考虑横向误差和航向误差
        δ = θ_e + arctan(k * e_y / (v + ε))
        
        Args:
            pose: 当前位姿
            target: 目标航点
            
        Returns:
            angular_velocity: 角速度 (rad/s)
        """
        # 计算到目标的方向
        dx = target.x - pose.x
        dy = target.y - pose.y
        dist = math.hypot(dx, dy)
        
        if dist < 0.1:
            return 0.0
        
        # 目标航向（从当前位置指向目标）
        target_yaw = math.atan2(dy, dx)
        
        # 航向误差
        heading_error = self._normalize_angle(target_yaw - pose.yaw)
        
        # 如果有路径历史，计算横向误差
        cross_track_error = 0.0
        if len(self._path_history) >= 1:
            # 使用最近的路径段计算横向误差
            prev_wp = self._path_history[-1]
            cross_track_error = self._compute_cross_track_error(
                pose, prev_wp, target
            )
        
        # Stanley 公式
        velocity = max(target.speed, self.min_speed)
        stanley_term = math.atan2(
            self.stanley_gain * cross_track_error,
            velocity + self.stanley_softening
        )
        
        # 总转向角
        steering_angle = heading_error + stanley_term
        
        # 转换为角速度 (假设船长 ~1m)
        angular_velocity = steering_angle * velocity
        
        return angular_velocity
    
    def _hybrid_control(
        self, 
        pose: Pose2D, 
        target: Waypoint,
        distance_to_target: float
    ) -> float:
        """
        混合控制: 远距离用 Pure Pursuit，近距离用 Stanley
        
        过渡区使用加权混合，避免控制指令跳变
        """
        pure_pursuit_omega = self._pure_pursuit(pose, target)
        stanley_omega = self._stanley(pose, target)
        
        # 计算混合权重 (alpha: Pure Pursuit 权重)
        if distance_to_target > self.hybrid_switch_distance:
            # 远距离: 主要使用 Pure Pursuit (90%)
            alpha = 0.9
        elif distance_to_target < self.hybrid_switch_distance / 2:
            # 近距离: 主要使用 Stanley (20% Pure Pursuit)
            alpha = 0.2
        else:
            # 过渡区: 线性插值
            ratio = (distance_to_target - self.hybrid_switch_distance / 2) / \
                    (self.hybrid_switch_distance / 2)
            alpha = 0.2 + 0.7 * ratio  # 从 0.2 到 0.9
        
        return alpha * pure_pursuit_omega + (1 - alpha) * stanley_omega
    
    def _compute_cross_track_error(
        self, 
        pose: Pose2D, 
        wp1: Waypoint, 
        wp2: Waypoint
    ) -> float:
        """
        计算横向误差 (有符号距离)
        
        正值表示在路径左侧，负值表示在路径右侧
        """
        # 路径向量
        dx = wp2.x - wp1.x
        dy = wp2.y - wp1.y
        
        # 位置向量 (从 wp1 到 pose)
        px = pose.x - wp1.x
        py = pose.y - wp1.y
        
        # 叉积得到有符号距离
        cross = dx * py - dy * px
        path_length = math.hypot(dx, dy)
        
        if path_length < 0.001:
            return 0.0
        
        return cross / path_length
    
    def _compute_adaptive_speed(
        self, 
        target_speed: float,
        angular_velocity: float,
        distance_to_goal: float,
        is_final: bool
    ) -> float:
        """
        计算自适应速度
        
        - 曲率大时减速（急转弯时降速）
        - 仅在最终航点附近减速（中间航点不减速）
        """
        speed = target_speed
        
        # 1. 曲率减速 (角速度大时减速)
        if abs(angular_velocity) > self.max_angular_velocity * 0.5:
            # 角速度超过最大值的一半时开始减速
            curvature_factor = 1.0 - (abs(angular_velocity) - self.max_angular_velocity * 0.5) / self.max_angular_velocity
            curvature_factor = np.clip(curvature_factor, 0.3, 1.0)
            speed *= curvature_factor
        
        # 2. 接近减速 (仅对最终航点)
        if is_final and distance_to_goal < self.goal_tolerance * 2:
            # 在到达阈值 2 倍范围内开始减速
            approach_factor = distance_to_goal / (self.goal_tolerance * 2)
            approach_factor = np.clip(approach_factor, 0.2, 1.0)
            speed *= approach_factor
        
        # 确保不低于最小速度
        return max(speed, self.min_speed)
    
    def _compute_adaptive_filter(self, heading_error: float) -> float:
        """
        计算自适应滤波系数
        
        航向误差大时减小滤波系数（快速响应），误差小时增大滤波（平稳）
        
        Args:
            heading_error: 航向误差 (rad)
            
        Returns:
            滤波系数 (0.3 ~ 0.7)
        """
        # 基础滤波系数
        base_filter = self.angular_velocity_filter  # 默认 0.4
        
        # 根据航向误差调整
        # 误差 0° → 保持基础值
        # 误差 90° → 增加到 0.7（快速响应）
        error_ratio = min(abs(heading_error) / (math.pi / 2), 1.0)
        
        # 误差大时，增加新值权重（减小滤波）
        adaptive_filter = base_filter + 0.3 * error_ratio
        
        return np.clip(adaptive_filter, 0.3, 0.7)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """归一化角度到 [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    # ==================== 参数更新接口 ====================
    
    def set_cruise_speed(self, speed: float):
        """更新巡航速度"""
        if speed > 0:
            self.cruise_speed = speed
    
    def set_goal_tolerance(self, tolerance: float):
        """更新到达阈值"""
        if tolerance > 0:
            self.goal_tolerance = tolerance
    
    def set_switch_tolerance(self, tolerance: float):
        """更新切换阈值"""
        if tolerance > 0:
            self.switch_tolerance = tolerance
    
    def set_controller_type(self, controller_type: ControllerType):
        """设置控制器类型"""
        self.controller_type = controller_type
    
    def set_stanley_gain(self, gain: float):
        """设置 Stanley 增益"""
        if gain > 0:
            self.stanley_gain = gain
    
    def set_lookahead_distance(self, distance: float):
        """设置前视距离"""
        if distance > 0:
            self.lookahead_distance = distance
