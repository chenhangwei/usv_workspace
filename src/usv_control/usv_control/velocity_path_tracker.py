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
from .mpc_path_tracker import MpcPathTracker


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
    yaw: float  # 船头朝向 (Heading), 弧度
    course: Optional[float] = None  # 航迹角 (Course over Ground), 弧度
    speed: Optional[float] = None   # 对地速度 (m/s)
    
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
    MPC = 4            # MPC 模型预测控制


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
        
        # MPC 参数 (推荐值)
        # v_max: 最大速度 (m/s). 限制 MPC 输出的线速度上限.
        mpc_v_max: float = 0.4,
        
        # w_max: 最大角速度 (rad/s). 限制 MPC 输出的旋转速度.
        mpc_w_max: float = 1.0,
        
        # Q_pos: 位置权重 (推荐 10.0 ~ 50.0). 
        # 越大越贴线，抗风流干扰强; 越小越平滑，允许适当偏离.
        mpc_q_pos: float = 20.0,
        
        # Q_theta: 航向权重 (推荐 1.0 ~ 10.0).
        # 保持船头朝向目标的意愿强度.
        mpc_q_theta: float = 5.0,
        
        # R_w: 转向平滑度权重 (推荐 1.0 ~ 10.0). [核心参数]
        # 越大越平滑(消除画龙)，越小反应越快.
        mpc_r_w: float = 5.0,
        
        # N: 预测步数 (推荐 10 ~ 30).
        # 20步 * 0.1s = 预测未来2秒. 太长增加计算量，太短容易短视.
        mpc_prediction_steps: int = 20,
        
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
        
        # ==================== MPC 参数 ====================
        self.mpc_params = {
            'v_max': mpc_v_max,
            'w_max': mpc_w_max,
            'q_pos': mpc_q_pos,
            'q_theta': mpc_q_theta,
            'r_w': mpc_r_w,
            'prediction_steps': mpc_prediction_steps
        }

        # ==================== 航点队列 ====================
        self._waypoint_queue: Deque[Waypoint] = deque(maxlen=waypoint_queue_size)
        self._current_waypoint: Optional[Waypoint] = None
        self._goal_reached = True

        # ==================== MPC 控制器 ====================
        self.mpc_tracker = None
        if self.controller_type == ControllerType.MPC:
            self.mpc_tracker = MpcPathTracker(**self.mpc_params)
        
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
        # 传入 pose 用于支持过站检测逻辑，防止绕圈
        if self._check_waypoint_transition(distance_to_waypoint, pose):
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
        
        # ==================== 优先转向逻辑 (Pivot Turn) ====================
        # 当航向误差 > 30° 时，优先进行原地或低速转向，减小转弯半径
        # 这样可以避免在目标点附近绕大圈 (Orbiting)
        LARGE_HEADING_ERROR_THRESHOLD = math.radians(30)  # 30°
        
        if abs(heading_error) > LARGE_HEADING_ERROR_THRESHOLD:
            # 大航向误差模式：急减速 + 最大角速度转向
            # 确定转向方向（正误差左转，负误差右转）
            if heading_error > 0:
                angular_velocity = self.max_angular_velocity
            else:
                angular_velocity = -self.max_angular_velocity
            
            # 减速策略：
            # 误差 > 45°: 极低速/原地旋转 (0.05x 速度)
            # 誤差 30°-45°: 线性过渡 (0.4x -> 0.05x)
            
            PIVOT_THRESHOLD = math.radians(45) # 45度以上视为需要大幅调整
            
            if abs(heading_error) > PIVOT_THRESHOLD:
                heading_factor = 0.05 # 几乎停车，优先转向
            else:
                # 30~45度之间插值: 0.4 -> 0.05
                ratio = (abs(heading_error) - LARGE_HEADING_ERROR_THRESHOLD) / (PIVOT_THRESHOLD - LARGE_HEADING_ERROR_THRESHOLD)
                heading_factor = 0.4 - 0.35 * ratio
            
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
        # MPC 模型预测控制
        if self.controller_type == ControllerType.MPC:
            if self.mpc_tracker is None:
                self.mpc_tracker = MpcPathTracker(**self.mpc_params)
            
            # L1 改进：使用有效航向(可能基于速度向量)
            # 当速度 > 0.3m/s 时使用 Course (航迹角)，否则使用 Heading (罗盘角)
            effective_yaw = self._get_control_heading(pose)
            
            v_cmd, w_cmd = self.mpc_tracker.compute_velocity_command(
                [pose.x, pose.y, effective_yaw], 
                [target.x, target.y], 
                target_speed=target.speed
            )
            # MPC 自带平滑和限幅，直接返回
            return VelocityCommand(v_cmd, 0.0, w_cmd).sanitize()

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
        # 修改：拆分为基础逻辑 + 过弯减速逻辑
        # 1. 基础速度（考虑转弯曲率和距离终点减速）
        base_speed = self._compute_adaptive_speed(
            target.speed, 
            angular_velocity, 
            distance_to_target,
            target.is_final
        )
        
        # 2. 过弯前瞻减速（根据下一个航点的角度提前减速）
        corning_limit = self._calculate_corner_speed_limit(pose, target)
        
        # 取最小值
        linear_speed = min(base_speed, corning_limit)
        
        # 构建并清理指令
        cmd = VelocityCommand(
            linear_x=linear_speed,
            linear_y=0.0,
            angular_z=angular_velocity
        )
        
        # 最终安全校验
        return cmd.sanitize()
    
    # ==================== 内部方法 ====================

    def _calculate_corner_speed_limit(self, current_pose: Pose2D, current_wp: Waypoint) -> float:
        """
        计算过弯限速
        
        逻辑：
        1. 检查队列里有没有下一个航点。如果没有，说明是终点，不限制（由距离逻辑控制）。
        2. 如果有，计算当前航段和下一航段的夹角。
        3. 夹角越尖锐（急弯），允许通过的速度越低。
        """
        # 1. 终点判断 (队列为空)
        if not self._waypoint_queue:
            # 没有下一个点，说明是最终终点。返回最大许可速度，让 _compute_adaptive_speed 处理减速
            return self.max_valid_speed_limit()
        
        next_wp = self._waypoint_queue[0]
        
        # 2. 计算向量
        # vec1: 船 -> 当前点
        dx1 = current_wp.x - current_pose.x
        dy1 = current_wp.y - current_pose.y
        
        # vec2: 当前点 -> 下一点
        dx2 = next_wp.x - current_wp.x
        dy2 = next_wp.y - current_wp.y
        
        len1 = math.hypot(dx1, dy1)
        len2 = math.hypot(dx2, dy2)
        
        if len1 < 0.1 or len2 < 0.1:
            return self.max_valid_speed_limit()

        # 3. 计算夹角 (0~180度, 0=掉头, 180=直行)
        # 使用 atan2 计算每个向量的绝对角度
        angle1 = math.atan2(dy1, dx1)
        angle2 = math.atan2(dy2, dx2)
        
        # 转向角 delta (0: 直行, pi: 掉头) 
        # 我们定义 turn_angle 为偏转角度，0表示直行，pi表示反向
        delta = abs(self._normalize_angle(angle2 - angle1))
        
        # 4. 映射到速度因子
        # 修改：采用更激进的减速策略，90度弯时即降至最低速
        
        # 允许最大转弯速度比例 (20% 巡航速度)
        MIN_CORNER_RATIO = 0.2
        
        # 计算转弯因子 (turn_factor)
        # 0度 -> 1.0 (全速)
        # 90度 (pi/2) -> MIN_CORNER_RATIO (最低速)
        # >90度 -> MIN_CORNER_RATIO
        
        if delta >= (math.pi / 2.0):
            corner_factor = MIN_CORNER_RATIO
        else:
            # 线性插值: 1.0 -> MIN_CORNER_RATIO
            ratio = delta / (math.pi / 2.0)
            corner_factor = 1.0 - ratio * (1.0 - MIN_CORNER_RATIO)
        
        # 计算距离影响: 离当前拐点越近，限速越严格
        # 增大影响半径，提前减速
        # 动态调整：基于当前巡航速度，预留至少 5秒 的减速时间/距离
        # 如 2m/s -> 10m. 0.5m/s -> 2.5m. 最小 5.0m
        influence_radius = max(8.0, self.cruise_speed * 5.0)
        
        dist_factor = min(1.0, len1 / influence_radius)
        
        # 使用幂函数平滑过渡，这会让减速更早发生
        # dist_factor < 1 时，快速下降
        dist_factor = math.pow(dist_factor, 0.7)
        
        target_corner_speed = self.cruise_speed * corner_factor
        current_limit = target_corner_speed + (self.cruise_speed - target_corner_speed) * dist_factor
        
        return max(current_limit, self.min_speed)

    def max_valid_speed_limit(self):
        return 10.0 # 足够大的数，不做限制

    def _get_control_heading(self, pose: Pose2D) -> float:
        """
        获取用于控制的航向 (Course vs Heading)
        
        借鉴 L1 导航算法：
        当有足够速度时，优先使用航迹角(Course)代替船头朝向(Heading)。
        
        关于 UWB 定位 (误差0.4m) 的特殊调整:
        - UWB 绝对误差虽大(0.4m)，但相对抖动(Jitter)通常较小(5-10cm)。
        - 对于最高速仅 0.4m/s 的船，如果阈值设太高，该功能永远无法激活。
        - 这里设为 0.2 m/s (最高速的一半)，只要动起来就信任 UWB 计算出的航向。
        """
        # 降低阈值以适配低速船 (0.4m/s max)
        COURSE_VALID_SPEED = 0.2
        
        # 只要上层节点计算出了pose.course (说明已经过了Node层的校验)
        # 且速度满足最低要求，就使用 Course
        if (pose.course is not None and 
            pose.speed is not None and 
            pose.speed > COURSE_VALID_SPEED):
            return pose.course
        return pose.yaw
    
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
    
    def _check_waypoint_transition(self, distance: float, pose: Pose2D = None) -> bool:
        """
        检查是否需要切换航点或到达
        
        策略:
        1. 距离圈检测: 进入目标点周围半径内
        2. 过站检测: 即使未进入半径，如果已经越过目标点所在的法线面，也强制切换(防止回头绕圈)
        
        Returns:
            True 如果已到达最终目标
        """
        if self._current_waypoint is None:
            return True
        
        # 判断使用哪个阈值
        if self._current_waypoint.is_final:
            # 最终航点：使用到达阈值
            threshold = self.goal_tolerance
            # 最终点必须严格到达，不启用过站切换
            allow_pass_by = False
        else:
            # 中间航点：使用切换阈值
            threshold = self.switch_tolerance
            
            # 针对 MPC 优化：智能调整切换半径
            if self.controller_type == ControllerType.MPC:
                # 默认保持 1.0m 宽松切换，追求丝滑
                base_threshold = max(threshold, 1.0)
                
                # 智能判断：如果是急转弯（夹角 < 90度），强制缩小半径，防止绕大圈
                # 获取下个航点检测拐角
                if self._waypoint_queue and self._path_history:
                    prev = self._path_history[-1]
                    curr = self._current_waypoint
                    next_wp = self._waypoint_queue[0]
                    
                    # 向量1: prev -> current
                    v1_x, v1_y = curr.x - prev.x, curr.y - prev.y
                    # 向量2: current -> next
                    v2_x, v2_y = next_wp.x - curr.x, next_wp.y - curr.y
                    
                    # 计算夹角余弦
                    dot_prod = v1_x*v2_x + v1_y*v2_y
                    norm1 = math.hypot(v1_x, v1_y)
                    norm2 = math.hypot(v2_x, v2_y)
                    
                    if norm1 > 0.1 and norm2 > 0.1:
                        cos_angle = dot_prod / (norm1 * norm2)
                        
                        # cos_angle > 0 说明夹角 < 90度 (锐角/直角转向)
                        # cos_angle < 0 说明夹角 > 90度 (钝角/平缓转向)
                        # 注意：这里是向量夹角。
                        # 直行: 向量角0度(cos=1); 掉头: 向量角180度(cos=-1)
                        # 我们关心的是航线之间的折角？
                        # 不，向量夹角 定义为: 前进方向的偏转.
                        # 直行: v1, v2 同向 -> angle=0, cos=1
                        # 90度右转: angle=90, cos=0
                        # 掉头: angle=180, cos=-1
                        
                        # 修正: 上述逻辑反了。
                        # 我们希望: 
                        # - 偏转角小 (直行/缓弯): cos -> 1.0. 此时 threshold = 1.0m (大半径切弯)
                        # - 偏转角大 (急弯/掉头): cos -> -1.0. 此时 threshold = 0.3m (小半径，逼近顶点)
                        
                        # 简单的线性映射:
                        # cos = 1  (0度偏转) -> threshold = 1.0
                        # cos = 0  (90度偏转) -> threshold = 0.5
                        # cos = -1 (180度掉头) -> threshold = 0.2
                        
                        # 映射公式:
                        # factor = (cos_angle + 1) / 2.0  # 0.0 ~ 1.0
                        # threshold = 0.2 + 0.8 * factor
                        
                        factor = (cos_angle + 1.0) / 2.0 
                        threshold = 0.2 + (base_threshold - 0.2) * factor
                        
                        # 再次保底，不小于 0.2m
                        threshold = max(threshold, 0.2)
                
                else:
                    threshold = base_threshold
                    
            # 中间点允许过站切换
            allow_pass_by = True
        
        # 1. 距离圈检测 (Circle Check)
        if distance < threshold:
            return self._handle_arrival_or_switch()
            
        # 2. 过站检测 (Pass-by Check)
        # 仅当允许过站、且位姿有效、且有历史路径时进行
        if allow_pass_by and pose is not None and self._path_history:
            prev_wp = self._path_history[-1]
            
            # 确保稍微有一点距离才算航线，防止重合点除零
            path_len = math.hypot(self._current_waypoint.x - prev_wp.x, 
                                self._current_waypoint.y - prev_wp.y)
            
            if path_len > 0.5:
                # 投影计算进度 t
                _, _, t = self._project_to_path(pose, prev_wp, self._current_waypoint)
                
                # t >= 1.0 表示已经越过了目标点
                if t >= 1.0:
                    # 为了安全，限制最大偏离距离。如果偏离太远(比如漏了20米)，可能还是应该回头或报错
                    # 这里放宽阈值：只要横向距离在 2.5倍 阈值以内，就算“擦肩而过”成功
                    # 比如阈值1m，只要在2.5m范围内路过，都算过
                    pass_by_threshold = max(threshold * 2.5, 3.0)
                    if distance < pass_by_threshold:
                        # 记录日志或调试信息通常在这里，但在纯算法类通过返回值处理
                        return self._handle_arrival_or_switch()

        return False

    def _handle_arrival_or_switch(self) -> bool:
        """执行到达或切换逻辑"""
        if self._current_waypoint.is_final:
            # 到达最终目标
            self._goal_reached = True
            return True
        else:
            # 切换到下一个航点
            self._switch_to_next_waypoint()
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
        # L1 改进：如果使用航迹角，local_x aligned with velocity vector
        effective_yaw = self._get_control_heading(pose)
        cos_yaw = math.cos(effective_yaw)
        sin_yaw = math.sin(effective_yaw)
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
        
        # 航向误差 (L1 改进：使用有效航向/航迹)
        effective_yaw = self._get_control_heading(pose)
        heading_error = self._normalize_angle(target_yaw - effective_yaw)
        
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
        混合控制: 远距离用 Pure Pursuit (航线跟踪)，近距离用 Stanley
        
        改进：在有前序航点时，Pure Pursuit 使用航线上的前视点，
        实现真正的直线跟踪，类似飞控 L1 算法。
        
        过渡区使用加权混合，避免控制指令跳变。
        """
        # 尝试使用航线跟踪版 Pure Pursuit
        pure_pursuit_omega, used_path_tracking = self._pure_pursuit_on_path(pose, target)
        
        # Stanley 始终基于航线计算（如果有历史）
        stanley_omega = self._stanley(pose, target)
        
        # 定义安全切换距离：接近目标时必须用航点跟踪模式
        SAFE_DISTANCE = 3.0  # 距离目标 3m 内回退到航点跟踪
        
        # 计算混合权重 (alpha: Pure Pursuit 权重)
        if distance_to_target > self.hybrid_switch_distance:
            # 远距离: 主要使用 Pure Pursuit (航线跟踪模式)
            alpha = 0.9
        elif distance_to_target < self.hybrid_switch_distance / 2:
            # 近距离: 主要使用 Stanley (20% Pure Pursuit)
            alpha = 0.2
        else:
            # 过渡区: 线性插值
            ratio = (distance_to_target - self.hybrid_switch_distance / 2) / \
                    (self.hybrid_switch_distance / 2)
            alpha = 0.2 + 0.7 * ratio  # 从 0.2 到 0.9
        
        # 如果接近目标 (< 3m) 且未使用航线跟踪，完全切换到 Stanley
        # 确保能精确到达目标点
        if distance_to_target < SAFE_DISTANCE and not used_path_tracking:
            alpha = min(alpha, 0.3)  # 限制 PP 权重
        
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
    
    def _project_to_path(self, pose: Pose2D, wp1: Waypoint, wp2: Waypoint) -> Tuple[float, float, float]:
        """
        计算船在航线上的投影点
        
        Args:
            pose: 当前位姿
            wp1: 航线起点 (前序航点)
            wp2: 航线终点 (当前目标)
            
        Returns:
            (proj_x, proj_y, t): 投影点坐标和参数 t
            t < 0: 投影点在 wp1 之前
            t > 1: 投影点在 wp2 之后
            0 <= t <= 1: 投影点在航线段上
        """
        # 航线向量
        dx = wp2.x - wp1.x
        dy = wp2.y - wp1.y
        path_len_sq = dx * dx + dy * dy
        
        if path_len_sq < 0.001:
            # 两点重合，返回 wp2
            return wp2.x, wp2.y, 1.0
        
        # 计算投影参数 t (0~1 表示在航线段上)
        t = ((pose.x - wp1.x) * dx + (pose.y - wp1.y) * dy) / path_len_sq
        
        # 投影点坐标
        proj_x = wp1.x + t * dx
        proj_y = wp1.y + t * dy
        
        return proj_x, proj_y, t
    
    def _get_lookahead_on_path(
        self, 
        pose: Pose2D, 
        wp1: Waypoint, 
        wp2: Waypoint,
        lookahead_dist: float
    ) -> Tuple[float, float, bool]:
        """
        在航线上计算前视点 (L1 风格)
        
        Args:
            pose: 当前位姿
            wp1: 航线起点
            wp2: 航线终点
            lookahead_dist: 前视距离
            
        Returns:
            (lh_x, lh_y, valid): 前视点坐标和是否有效
        """
        # 1. 计算投影点
        proj_x, proj_y, t = self._project_to_path(pose, wp1, wp2)
        
        # 2. 检查投影是否在有效范围
        # 如果 t > 1 (船过了目标点)，不使用航线跟踪
        if t > 1.0:
            return wp2.x, wp2.y, False
        
        # 如果 t < -0.5 (船在航线起点后方太远)，不使用航线跟踪
        if t < -0.5:
            return wp2.x, wp2.y, False
        
        # 3. 计算到投影点的横向距离
        cross_track = math.hypot(pose.x - proj_x, pose.y - proj_y)
        
        # 如果横向误差太大 (>5m)，先回归航线
        if cross_track > 5.0:
            return wp2.x, wp2.y, False
        
        # 4. 沿航线方向前进 lookahead_dist
        path_dx = wp2.x - wp1.x
        path_dy = wp2.y - wp1.y
        path_len = math.hypot(path_dx, path_dy)
        
        if path_len < 0.1:
            return wp2.x, wp2.y, False
        
        # 单位方向向量
        dir_x = path_dx / path_len
        dir_y = path_dy / path_len
        
        # 前视点 = 投影点 + 前视距离 * 方向
        lh_x = proj_x + lookahead_dist * dir_x
        lh_y = proj_y + lookahead_dist * dir_y
        
        # 5. 检查前视点是否超出航线终点
        # 计算前视点到起点的距离比例
        lh_t = ((lh_x - wp1.x) * path_dx + (lh_y - wp1.y) * path_dy) / (path_len * path_len)
        
        if lh_t > 1.0:
            # 前视点超出终点，限制在终点
            lh_x = wp2.x
            lh_y = wp2.y
        
        return lh_x, lh_y, True
    
    def _pure_pursuit_on_path(self, pose: Pose2D, target: Waypoint) -> Tuple[float, bool]:
        """
        航线跟踪版 Pure Pursuit
        
        前视点在航线上，而不是直接指向目标航点。
        这样可以让船先回到航线再沿线行驶，实现真正的直线。
        
        Args:
            pose: 当前位姿
            target: 目标航点
            
        Returns:
            (angular_velocity, used_path_tracking): 角速度和是否使用了航线跟踪
        """
        # 检查是否有前序航点
        if not self._path_history:
            # 没有历史航点，无法构成航线
            return self._pure_pursuit(pose, target), False
        
        prev_wp = self._path_history[-1]
        
        # 计算自适应前视距离
        L = self.lookahead_gain * self.cruise_speed + self.min_lookahead
        L = np.clip(L, self.min_lookahead, self.max_lookahead)
        
        # 获取航线上的前视点
        lh_x, lh_y, valid = self._get_lookahead_on_path(pose, prev_wp, target, L)
        
        if not valid:
            # 无法使用航线跟踪，回退到普通模式
            return self._pure_pursuit(pose, target), False
        
        # 使用航线上的前视点计算曲率
        dx = lh_x - pose.x
        dy = lh_y - pose.y
        
        # 转换到车身坐标系
        effective_yaw = self._get_control_heading(pose)
        cos_yaw = math.cos(effective_yaw)
        sin_yaw = math.sin(effective_yaw)
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        
        # 计算到前视点的距离
        ld = math.hypot(local_x, local_y)
        if ld < 0.1:
            return 0.0, True
        
        # 计算曲率
        curvature = 2.0 * local_y / (ld * ld)
        
        # 角速度 = 速度 * 曲率
        angular_velocity = target.speed * curvature
        
        return angular_velocity, True
    
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
        
        修改：更早介入减速，更平滑的停车
        """
        speed = target_speed
        
        # 1. 曲率减速 (角速度大时减速)
        # 修改：阈值降低到 20% (原 50%)，意味着只要开始转向就会减速
        threshold = self.max_angular_velocity * 0.2
        
        if abs(angular_velocity) > threshold:
            excess = abs(angular_velocity) - threshold
            available_range = self.max_angular_velocity * 0.8
            
            if available_range > 1e-3:
                ratio = excess / available_range
                # 能够降低到 30% 速度
                curvature_factor = 1.0 - ratio * 0.7
                curvature_factor = np.clip(curvature_factor, 0.3, 1.0)
                speed *= curvature_factor
        
        # 2. 接近减速 (仅对最终航点)
        if is_final:
            # 用户需求：距离目标点越近速度越慢，例如8米时开始递减
            # 取 8.0m 作为基础减速距离，高速时自动延长
            stop_distance = max(8.0, speed * 4.0)
            
            if distance_to_goal < stop_distance:
                # 归一化距离因子 (0.0 ~ 1.0)
                ratio = distance_to_goal / stop_distance
                
                # 使用幂函数平滑减速响应
                # 0.8 次幂：在减速初期(8m处)速度下降较缓，接近目标时下降较快，比较自然
                approach_factor = math.pow(ratio, 0.8)
                
                # 限制最小因子，防止过早停转 (由 min_speed 兜底)
                approach_factor = np.clip(approach_factor, 0.1, 1.0)
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
            # 同步更新 MPC 的最大速度约束
            if self.mpc_tracker is not None:
                self.mpc_tracker.set_max_speed(speed)
    
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
