#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# ROS 2 Node implementation: Velocity Controller Node.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
USV 速度模式控制节点

基于 MPC（模型预测控制）的速度模式导航节点。
直接发送速度指令给飞控，绕过飞控的减速逻辑，实现平滑连续导航。

订阅:
- /{ns}/set_usv_nav_goal: 导航目标 (NavigationGoal)
- /{ns}/local_position/pose_from_gps: 当前位姿 (PoseStamped)
- /{ns}/state: 飞控状态 (State)
- /{ns}/cancel_navigation: 暂停导航请求 (Bool)
- /{ns}/stop_navigation: 停止导航请求 (Bool)

发布:
- /{ns}/setpoint_raw/local: 速度指令 (PositionTarget)
- /{ns}/velocity_controller/status: 控制器状态 (String)

参数:
- control_mode: 控制模式 ('velocity' 或 'position')
- cruise_speed: 巡航速度 (m/s)
- max_angular_velocity: 最大角速度 (rad/s)
- goal_tolerance: 到达阈值 (m)
- switch_tolerance: 切换阈值 (m)

作者: Auto-generated
日期: 2026-01-22
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.exceptions import ParameterUninitializedException
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import String, Float32, Bool
from common_interfaces.msg import (
    NavigationGoal,
    NavigationFeedback,
    NavigationResult,
    MpcDebug,
    FleetNeighborPoses,
)

import math
import time
import threading
from typing import Dict, Optional, Tuple
from enum import Enum, auto

from .velocity_path_tracker import (
    VelocityPathTracker, 
    Pose2D, 
    Waypoint,
    VelocityCommand,
    ControllerType
)


class NavigationState(Enum):
    """
    导航任务状态枚举
    
    用于精确控制导航任务的生命周期，支持更灵活的模式保护策略。
    """
    IDLE = auto()        # 空闲 - 无任务，等待新目标
    ACTIVE = auto()      # 进行中 - 正在执行导航
    PAUSED = auto()      # 暂停 - 被遥控器/HOLD模式打断，可自动恢复GUIDED
    COMPLETED = auto()   # 已完成 - 正常到达目标，不恢复GUIDED
    CANCELLED = auto()   # 用户取消 - 用户主动取消(点击HOLD/MANUAL)，不恢复GUIDED
    FAILED = auto()      # 失败 - 超时或异常，不恢复GUIDED


class VelocityControllerNode(Node):
    """
    USV 速度模式控制节点
    
    使用 MPC（模型预测控制）算法计算速度指令，
    直接发送给飞控，避免飞控的位置模式减速逻辑。
    """
    
    def __init__(self):
        super().__init__('velocity_controller_node')
        
        # USV ID (从命名空间提取，如 /usv_02 -> usv_02)
        ns = self.get_namespace()
        self._usv_id = ns.strip('/') if ns and ns != '/' else 'unknown'
        
        # 回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 导航目标回调锁 - 防止 ReentrantCallbackGroup 下并发执行导致竞态条件
        # 当 navigate_to_point_node 快速连发多个目标时（如乱序修正），
        # 确保 set_waypoint 调用按消息到达顺序串行执行
        self._nav_goal_lock = threading.Lock()
        
        # ==================== 参数声明 ====================
        # 控制模式
        self.declare_parameter('control_mode', 'velocity')  # 'position' 或 'velocity'
        
        # MPC 参数
        self.declare_parameter('mpc_prediction_steps', 20)
        self.declare_parameter('mpc_weight_pos', 10.0)
        self.declare_parameter('mpc_weight_heading', 8.0)
        self.declare_parameter('mpc_weight_steering', 5.0)
        self.declare_parameter('mpc_weight_steering_rate', 10.0)  # R_dw: 角加速度惩罚
        
        # v5 新增参数 (一阶惯性转向模型)
        self.declare_parameter('mpc_tau_omega', 0.55)             # 转向时间常数 (秒) v11: 0.4→0.55, 更接近实船收敛值
        self.declare_parameter('mpc_weight_cte', 15.0)            # Cross Track Error 权重
        
        # v6 新增: 速度自适应 tau_omega 参数 (解决低速S形振荡)
        self.declare_parameter('adaptive_tau_enabled', True)      # 是否启用速度自适应 tau_omega
        self.declare_parameter('tau_omega_low_speed', 0.8)        # 低速时的 tau_omega (秒)
        self.declare_parameter('tau_omega_high_speed', 0.4)       # 高速时的 tau_omega (秒)
        self.declare_parameter('tau_speed_threshold_low', 0.15)   # 低速阈值 (m/s)
        self.declare_parameter('tau_speed_threshold_high', 0.35)  # 高速阈值 (m/s)
        
        # v8 新增: AMPC (自适应MPC) 参数 - 在线辨识 tau_omega，消除逐船调参
        self.declare_parameter('ampc_enabled', True)                   # 是否启用 AMPC
        self.declare_parameter('ampc_rls_forgetting_factor', 0.97)     # RLS 遗忘因子
        self.declare_parameter('ampc_heading_observer_alpha', 0.3)     # 航向速率滤波系数
        self.declare_parameter('ampc_tau_min', 0.1)                    # τ 估计下限 (秒)
        self.declare_parameter('ampc_tau_max', 3.0)                    # τ 估计上限 (秒)
        self.declare_parameter('ampc_rebuild_threshold', 0.15)         # τ 变化重建阈值
        self.declare_parameter('ampc_saturation_tau_boost', 1.05)      # 饱和时 τ 升压因子
        
        # 速度参数
        self.declare_parameter('cruise_speed', 0.5)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('min_speed', 0.05)
        
        # 到达判断
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('switch_tolerance', 1.5)
        
        # 控制参数
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('angular_velocity_filter', 0.3)
        
        # 安全参数
        self.declare_parameter('require_guided_mode', True)
        self.declare_parameter('require_armed', True)
        
        # L1 风格航向估计参数
        self.declare_parameter('use_velocity_based_heading', True)  # 使用速度方向估计航向
        self.declare_parameter('min_speed_for_velocity_heading', 0.20)  # 使用速度航向的最小速度 (m/s)
        self.declare_parameter('heading_fusion_speed_range', 0.15)  # 航向融合过渡速度范围 (m/s)
        
        # v14 新增: 模式保护和渐进减速参数
        self.declare_parameter('consecutive_hold_threshold', 3)      # 连续 HOLD 强制暂停阈值
        self.declare_parameter('soft_decel_duration', 3.0)           # 渐进减速持续时间 (秒)

        # 分布式 APF 参数（本地计算，多船排斥避碰）
        self.declare_parameter('apf_enabled', False)
        self.declare_parameter('apf_use_fleet_neighbors_topic', True)
        self.declare_parameter('apf_fleet_neighbors_topic', 'apf/neighbors')
        self.declare_parameter(
            'apf_neighbor_pose_topics',
            [],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter('apf_safe_distance', 3.0)
        self.declare_parameter('apf_repulsion_gain', 0.8)
        self.declare_parameter('apf_lateral_to_yaw_gain', 0.8)
        self.declare_parameter('apf_max_linear_correction', 0.50)
        self.declare_parameter('apf_max_angular_correction', 0.60)
        self.declare_parameter('apf_neighbor_timeout', 0.8)
        self.declare_parameter('apf_goal_slow_speed_threshold', 0.25)
        self.declare_parameter('apf_goal_relax_min_scale', 0.20)
        self.declare_parameter('apf_orca_enabled', True)
        self.declare_parameter('apf_orca_time_horizon', 6.0)
        self.declare_parameter('apf_orca_min_separation', 1.5)   # v11: 1.0→1.5, 增大安全距离避免过近接触
        self.declare_parameter('apf_orca_influence_distance', 3.5)  # v15: 4.0→3.5, 减少远场"other"类型的不必要ORCA激活(原77%激活率过高)
        self.declare_parameter('apf_orca_side_commit_enabled', True)
        self.declare_parameter('apf_orca_commit_policy', 'starboard')
        self.declare_parameter('apf_orca_side_commit_distance', 1.2)
        self.declare_parameter('apf_orca_side_commit_hold_time', 2.5)
        self.declare_parameter('apf_orca_side_bias_lateral', 0.12)
        self.declare_parameter('apf_orca_min_angular_near', 0.12)
        self.declare_parameter('apf_orca_min_angular_distance', 0.7)
        self.declare_parameter('apf_orca_near_apf_fallback_enabled', True)
        self.declare_parameter('apf_orca_near_apf_distance', 0.8)
        self.declare_parameter('apf_orca_near_apf_weight', 1.0)
        self.declare_parameter('apf_orca_enforce_commit_direction', True)
        self.declare_parameter('apf_orca_enforce_min_angular', 0.12)
        self.declare_parameter('apf_orca_shared_turn_enabled', True)
        self.declare_parameter('apf_orca_shared_turn_distance', 1.2)
        self.declare_parameter('apf_orca_shared_turn_min_angular', 0.18)
        self.declare_parameter('apf_orca_deadlock_enabled', True)
        self.declare_parameter('apf_orca_deadlock_distance', 1.0)
        self.declare_parameter('apf_orca_deadlock_hold_time', 3.0)
        self.declare_parameter('apf_orca_deadlock_turn_boost', 0.08)
        self.declare_parameter('apf_orca_deadlock_speed_reduction', 0.10)
        self.declare_parameter('apf_orca_lane_guidance_enabled', True)
        self.declare_parameter('apf_orca_lane_guidance_distance', 2.5)
        self.declare_parameter('apf_orca_lane_offset', 1.0)
        self.declare_parameter('apf_orca_lane_gain', 0.20)
        self.declare_parameter('apf_orca_overlap_guard_enabled', True)
        self.declare_parameter('apf_orca_overlap_ahead_distance', 2.2)
        self.declare_parameter('apf_orca_overlap_lateral_threshold', 0.8)
        self.declare_parameter('apf_orca_overlap_speed_cut', 0.12)
        self.declare_parameter('apf_orca_overlap_turn_scale', 0.65)
        self.declare_parameter('apf_orca_coupling_guard_enabled', True)
        self.declare_parameter('apf_orca_coupling_distance', 1.2)
        self.declare_parameter('apf_orca_coupling_release_distance', 1.6)
        self.declare_parameter('apf_orca_coupling_lateral_threshold', 0.45)
        self.declare_parameter('apf_orca_coupling_rel_speed_max', 0.12)
        self.declare_parameter('apf_orca_coupling_hold_time', 8.0)
        self.declare_parameter('apf_orca_coupling_speed_min', 0.20)
        self.declare_parameter('apf_orca_coupling_speed_cut', 0.12)
        self.declare_parameter('apf_orca_coupling_turn_boost', 0.10)
        self.declare_parameter('apf_orca_coupling_lane_push', 0.10)
        self.declare_parameter('apf_orca_coupling_stationary_distance', 1.6)
        self.declare_parameter('apf_orca_coupling_stationary_speed_max', 0.30)
        self.declare_parameter('apf_orca_coupling_hard_brake_scale', 0.60)
        self.declare_parameter('apf_orca_coupling_speed_floor', 0.12)
        self.declare_parameter('apf_orca_predictive_early_enabled', True)
        self.declare_parameter('apf_orca_predictive_tcpa_threshold', 12.0)
        self.declare_parameter('apf_orca_predictive_cpa_threshold', 2.0)
        self.declare_parameter('apf_orca_predictive_max_distance', 6.0)
        self.declare_parameter('apf_orca_predictive_angular_max', 0.10)
        self.declare_parameter('apf_orca_predictive_speed_cut_max', 0.06)
        self.declare_parameter('apf_orca_cpa_enabled', True)
        self.declare_parameter('apf_orca_cpa_distance_threshold', 1.2)
        self.declare_parameter('apf_orca_tcpa_threshold', 6.0)
        self.declare_parameter('apf_orca_urgency_scale_max', 2.0)
        self.declare_parameter('apf_orca_linear_slowdown_max', 0.12)
        self.declare_parameter('apf_orca_turn_budget_enabled', True)
        self.declare_parameter('apf_orca_turn_budget_ratio', 0.85)
        self.declare_parameter('apf_orca_turn_budget_same_side_scale', 0.25)
        self.declare_parameter('apf_orca_smart_side_enabled', True)
        self.declare_parameter('apf_orca_smart_side_horizon', 2.5)
        self.declare_parameter('apf_orca_smart_side_ref_yaw_rate', 0.25)
        self.declare_parameter('apf_orca_smart_side_clearance_weight', 1.2)
        self.declare_parameter('apf_orca_smart_side_colreg_weight', 0.5)
        self.declare_parameter('apf_orca_smart_side_goal_weight', 0.3)
        self.declare_parameter('apf_orca_encounter_rule_enabled', True)
        self.declare_parameter('apf_orca_encounter_guidance_distance', 3.0)
        self.declare_parameter('apf_orca_encounter_bias_angular', 0.08)
        self.declare_parameter('apf_orca_encounter_head_on_deg', 20.0)
        self.declare_parameter('apf_orca_encounter_head_on_bearing_deg', 15.0)
        self.declare_parameter('apf_orca_encounter_head_on_course_deg', 165.0)
        self.declare_parameter('apf_orca_encounter_crossing_deg', 112.5)
        self.declare_parameter('apf_orca_encounter_overtake_deg', 22.5)
        self.declare_parameter('apf_orca_encounter_hold_time', 0.6)
        # P5: 静止船舶识别 — COLREGs 仅适用于"在航"船舶 (Rule 3)
        # 静止/锚泊船舶不应触发 crossing_give_way 等强制避让方向
        self.declare_parameter('apf_orca_stationary_speed_threshold', 0.05)   # 邻船速度低于此值视为静止 (m/s)
        self.declare_parameter('apf_orca_stationary_correction_scale', 1.0)  # v16: 0.65→1.0, 不再缩减静止船修正(v15数据: stationary仍是最危险类型,min=0.39m)
        # P6: 会遇类型锁定 — 防止船舶转向时 bearing 符号翻转导致 stand_on↔give_way 振荡
        # 参考: IMO COLREGs Rule 15/17 在同一次会遇中不应因本船转向而改变让路/直航权关系
        # Tam & Bucknall (2010), Eriksen et al. (2019) 均采用首次评估锁定策略
        self.declare_parameter('apf_orca_encounter_lock_enabled', True)       # 启用会遇类型锁定
        self.declare_parameter('apf_orca_encounter_lock_timeout', 30.0)      # 锁定最大持续时间 (秒), 超时后重新评估
        self.declare_parameter('apf_orca_encounter_lock_release_ratio', 1.5) # 距离增大到初始距离的N倍时解锁
        self.declare_parameter('apf_orca_crossing_dead_zone_deg', 10.0)      # bearing 死区: |bearing|<此值时不判crossing, 避免船头附近翻转
        # P2: ORCA 迟滞防振荡 (防止 MPC↔ORCA 快速切换)
        self.declare_parameter('apf_orca_hysteresis_margin', 1.0)      # v15: 1.5→1.0, 缩小迟滞余量(原effective_exit=5.5m导致77%激活率过高)
        self.declare_parameter('apf_orca_min_hold_time', 3.5)          # v15: 5.0→3.5, 缩短最小保持(原值导致远场ORCA持续过久)
        # P1: COLREGS 强化 (使航行规则成为方向选择主决策)
        self.declare_parameter('apf_orca_colregs_override', True)      # COLREGS 方向覆盖其他选边逻辑
        self.declare_parameter('apf_orca_colregs_standon_scale', 0.3)  # 直航权船 ORCA 修正缩减比
        self.declare_parameter('apf_orca_colregs_head_on_bias', 0.25)  # 对遇强制右转角速度 (rad/s)
        # P4: NH-ORCA 非完整运动学约束
        self.declare_parameter('apf_orca_nh_enabled', True)            # 启用非完整运动学约束投影
        self.declare_parameter('apf_orca_nh_reachable_horizon', 2.0)   # 可达航向弧计算时域 (秒)
        self.declare_parameter('apf_health_monitor_enabled', True)
        self.declare_parameter('apf_health_freeze_pos_epsilon', 0.01)
        self.declare_parameter('apf_health_min_speed_for_freeze', 0.15)
        self.declare_parameter('apf_health_own_freeze_warn_duration', 0.8)
        self.declare_parameter('apf_health_neighbor_freeze_warn_duration', 0.8)
        self.declare_parameter('apf_health_warn_interval', 2.0)
        
        # ==================== 获取参数 ====================
        self.control_mode = str(self.get_parameter('control_mode').value or 'velocity')
        self.get_logger().info(f'🔍 正在初始化速度控制器... 模式: MPC')
        
        # 控制器类型固定为 MPC
        controller_type = ControllerType.MPC
        
        self.require_guided_mode = bool(self.get_parameter('require_guided_mode').value)
        self.require_armed = bool(self.get_parameter('require_armed').value)

        # ==================== APF 配置 ====================
        self._apf_enabled = bool(self.get_parameter('apf_enabled').value)
        self._apf_use_fleet_topic = bool(self.get_parameter('apf_use_fleet_neighbors_topic').value)
        self._apf_fleet_topic = str(self.get_parameter('apf_fleet_neighbors_topic').value or 'apf/neighbors')
        try:
            neighbor_topics = self.get_parameter('apf_neighbor_pose_topics').value
        except ParameterUninitializedException:
            neighbor_topics = []
        if isinstance(neighbor_topics, str):
            neighbor_topics = [neighbor_topics]
        self._apf_neighbor_topics = [str(topic) for topic in (neighbor_topics or []) if str(topic)]
        self._apf_safe_distance = float(self.get_parameter('apf_safe_distance').value or 3.0)
        self._apf_repulsion_gain = float(self.get_parameter('apf_repulsion_gain').value or 0.8)
        self._apf_lateral_to_yaw_gain = float(self.get_parameter('apf_lateral_to_yaw_gain').value or 0.8)
        self._apf_max_linear_correction = float(self.get_parameter('apf_max_linear_correction').value or 0.50)
        self._apf_max_angular_correction = float(self.get_parameter('apf_max_angular_correction').value or 0.60)
        self._apf_neighbor_timeout = float(self.get_parameter('apf_neighbor_timeout').value or 0.8)
        self._apf_goal_slow_speed_threshold = float(
            self.get_parameter('apf_goal_slow_speed_threshold').value or 0.25
        )
        self._apf_goal_relax_min_scale = float(
            self.get_parameter('apf_goal_relax_min_scale').value or 0.20
        )
        self._apf_orca_enabled = bool(self.get_parameter('apf_orca_enabled').value)
        self._apf_orca_time_horizon = float(self.get_parameter('apf_orca_time_horizon').value or 6.0)
        self._apf_orca_min_separation = float(self.get_parameter('apf_orca_min_separation').value or 1.0)
        self._apf_orca_influence_distance = float(self.get_parameter('apf_orca_influence_distance').value or 4.0)
        self._apf_orca_side_commit_enabled = bool(self.get_parameter('apf_orca_side_commit_enabled').value)
        self._apf_orca_commit_policy = str(
            self.get_parameter('apf_orca_commit_policy').value or 'starboard'
        ).strip().lower()
        self._apf_orca_side_commit_distance = float(self.get_parameter('apf_orca_side_commit_distance').value or 1.2)
        self._apf_orca_side_commit_hold_time = float(self.get_parameter('apf_orca_side_commit_hold_time').value or 2.5)
        self._apf_orca_side_bias_lateral = float(self.get_parameter('apf_orca_side_bias_lateral').value or 0.12)
        self._apf_orca_min_angular_near = float(self.get_parameter('apf_orca_min_angular_near').value or 0.12)
        self._apf_orca_min_angular_distance = float(self.get_parameter('apf_orca_min_angular_distance').value or 0.7)
        self._apf_orca_near_apf_fallback_enabled = bool(self.get_parameter('apf_orca_near_apf_fallback_enabled').value)
        self._apf_orca_near_apf_distance = float(self.get_parameter('apf_orca_near_apf_distance').value or 0.8)
        self._apf_orca_near_apf_weight = float(self.get_parameter('apf_orca_near_apf_weight').value or 1.0)
        self._apf_orca_enforce_commit_direction = bool(self.get_parameter('apf_orca_enforce_commit_direction').value)
        self._apf_orca_enforce_min_angular = float(self.get_parameter('apf_orca_enforce_min_angular').value or 0.12)
        self._apf_orca_shared_turn_enabled = bool(self.get_parameter('apf_orca_shared_turn_enabled').value)
        self._apf_orca_shared_turn_distance = float(self.get_parameter('apf_orca_shared_turn_distance').value or 1.2)
        self._apf_orca_shared_turn_min_angular = float(self.get_parameter('apf_orca_shared_turn_min_angular').value or 0.18)
        self._apf_orca_deadlock_enabled = bool(self.get_parameter('apf_orca_deadlock_enabled').value)
        self._apf_orca_deadlock_distance = float(self.get_parameter('apf_orca_deadlock_distance').value or 1.0)
        self._apf_orca_deadlock_hold_time = float(self.get_parameter('apf_orca_deadlock_hold_time').value or 3.0)
        self._apf_orca_deadlock_turn_boost = float(self.get_parameter('apf_orca_deadlock_turn_boost').value or 0.08)
        self._apf_orca_deadlock_speed_reduction = float(self.get_parameter('apf_orca_deadlock_speed_reduction').value or 0.10)
        self._apf_orca_lane_guidance_enabled = bool(self.get_parameter('apf_orca_lane_guidance_enabled').value)
        self._apf_orca_lane_guidance_distance = float(self.get_parameter('apf_orca_lane_guidance_distance').value or 2.5)
        self._apf_orca_lane_offset = float(self.get_parameter('apf_orca_lane_offset').value or 1.0)
        self._apf_orca_lane_gain = float(self.get_parameter('apf_orca_lane_gain').value or 0.20)
        self._apf_orca_overlap_guard_enabled = bool(self.get_parameter('apf_orca_overlap_guard_enabled').value)
        self._apf_orca_overlap_ahead_distance = float(self.get_parameter('apf_orca_overlap_ahead_distance').value or 2.2)
        self._apf_orca_overlap_lateral_threshold = float(self.get_parameter('apf_orca_overlap_lateral_threshold').value or 0.8)
        self._apf_orca_overlap_speed_cut = float(self.get_parameter('apf_orca_overlap_speed_cut').value or 0.12)
        self._apf_orca_overlap_turn_scale = float(self.get_parameter('apf_orca_overlap_turn_scale').value or 0.65)
        self._apf_orca_coupling_guard_enabled = bool(self.get_parameter('apf_orca_coupling_guard_enabled').value)
        self._apf_orca_coupling_distance = float(self.get_parameter('apf_orca_coupling_distance').value or 1.2)
        self._apf_orca_coupling_release_distance = float(
            self.get_parameter('apf_orca_coupling_release_distance').value or 1.6
        )
        self._apf_orca_coupling_lateral_threshold = float(
            self.get_parameter('apf_orca_coupling_lateral_threshold').value or 0.45
        )
        self._apf_orca_coupling_rel_speed_max = float(
            self.get_parameter('apf_orca_coupling_rel_speed_max').value or 0.12
        )
        self._apf_orca_coupling_hold_time = float(self.get_parameter('apf_orca_coupling_hold_time').value or 8.0)
        self._apf_orca_coupling_speed_min = float(self.get_parameter('apf_orca_coupling_speed_min').value or 0.20)
        self._apf_orca_coupling_speed_cut = float(self.get_parameter('apf_orca_coupling_speed_cut').value or 0.12)
        self._apf_orca_coupling_turn_boost = float(self.get_parameter('apf_orca_coupling_turn_boost').value or 0.10)
        self._apf_orca_coupling_lane_push = float(self.get_parameter('apf_orca_coupling_lane_push').value or 0.10)
        self._apf_orca_coupling_stationary_distance = float(
            self.get_parameter('apf_orca_coupling_stationary_distance').value or 1.6
        )
        self._apf_orca_coupling_stationary_speed_max = float(
            self.get_parameter('apf_orca_coupling_stationary_speed_max').value or 0.30
        )
        self._apf_orca_coupling_hard_brake_scale = float(
            self.get_parameter('apf_orca_coupling_hard_brake_scale').value or 0.60
        )
        self._apf_orca_coupling_speed_floor = float(
            self.get_parameter('apf_orca_coupling_speed_floor').value or 0.12
        )
        self._apf_orca_predictive_early_enabled = bool(self.get_parameter('apf_orca_predictive_early_enabled').value)
        self._apf_orca_predictive_tcpa_threshold = float(
            self.get_parameter('apf_orca_predictive_tcpa_threshold').value or 12.0
        )
        self._apf_orca_predictive_cpa_threshold = float(
            self.get_parameter('apf_orca_predictive_cpa_threshold').value or 2.0
        )
        self._apf_orca_predictive_max_distance = float(
            self.get_parameter('apf_orca_predictive_max_distance').value or 6.0
        )
        self._apf_orca_predictive_angular_max = float(
            self.get_parameter('apf_orca_predictive_angular_max').value or 0.10
        )
        self._apf_orca_predictive_speed_cut_max = float(
            self.get_parameter('apf_orca_predictive_speed_cut_max').value or 0.06
        )
        self._apf_orca_cpa_enabled = bool(self.get_parameter('apf_orca_cpa_enabled').value)
        self._apf_orca_cpa_distance_threshold = float(self.get_parameter('apf_orca_cpa_distance_threshold').value or 1.2)
        self._apf_orca_tcpa_threshold = float(self.get_parameter('apf_orca_tcpa_threshold').value or 6.0)
        self._apf_orca_urgency_scale_max = float(self.get_parameter('apf_orca_urgency_scale_max').value or 2.0)
        self._apf_orca_linear_slowdown_max = float(self.get_parameter('apf_orca_linear_slowdown_max').value or 0.12)
        self._apf_orca_turn_budget_enabled = bool(self.get_parameter('apf_orca_turn_budget_enabled').value)
        self._apf_orca_turn_budget_ratio = float(self.get_parameter('apf_orca_turn_budget_ratio').value or 0.85)
        self._apf_orca_turn_budget_same_side_scale = float(
            self.get_parameter('apf_orca_turn_budget_same_side_scale').value or 0.25
        )
        self._apf_orca_smart_side_enabled = bool(self.get_parameter('apf_orca_smart_side_enabled').value)
        self._apf_orca_smart_side_horizon = float(self.get_parameter('apf_orca_smart_side_horizon').value or 2.5)
        self._apf_orca_smart_side_ref_yaw_rate = float(
            self.get_parameter('apf_orca_smart_side_ref_yaw_rate').value or 0.25
        )
        self._apf_orca_smart_side_clearance_weight = float(
            self.get_parameter('apf_orca_smart_side_clearance_weight').value or 1.2
        )
        self._apf_orca_smart_side_colreg_weight = float(
            self.get_parameter('apf_orca_smart_side_colreg_weight').value or 0.5
        )
        self._apf_orca_smart_side_goal_weight = float(
            self.get_parameter('apf_orca_smart_side_goal_weight').value or 0.3
        )
        self._apf_orca_encounter_rule_enabled = bool(self.get_parameter('apf_orca_encounter_rule_enabled').value)
        self._apf_orca_encounter_guidance_distance = float(
            self.get_parameter('apf_orca_encounter_guidance_distance').value or 3.0
        )
        self._apf_orca_encounter_bias_angular = float(
            self.get_parameter('apf_orca_encounter_bias_angular').value or 0.08
        )
        self._apf_orca_encounter_head_on_deg = float(
            self.get_parameter('apf_orca_encounter_head_on_deg').value or 20.0
        )
        self._apf_orca_encounter_head_on_bearing_deg = float(
            self.get_parameter('apf_orca_encounter_head_on_bearing_deg').value or 15.0
        )
        self._apf_orca_encounter_head_on_course_deg = float(
            self.get_parameter('apf_orca_encounter_head_on_course_deg').value or 165.0
        )
        self._apf_orca_encounter_crossing_deg = float(
            self.get_parameter('apf_orca_encounter_crossing_deg').value or 112.5
        )
        self._apf_orca_encounter_overtake_deg = float(
            self.get_parameter('apf_orca_encounter_overtake_deg').value or 22.5
        )
        self._apf_orca_encounter_hold_time = float(
            self.get_parameter('apf_orca_encounter_hold_time').value or 0.6
        )
        # P5: 静止船舶识别
        self._apf_orca_stationary_speed_threshold = float(
            self.get_parameter('apf_orca_stationary_speed_threshold').value or 0.05
        )
        self._apf_orca_stationary_correction_scale = float(
            self.get_parameter('apf_orca_stationary_correction_scale').value or 1.0
        )
        # P6: 会遇类型锁定
        self._apf_orca_encounter_lock_enabled = bool(
            self.get_parameter('apf_orca_encounter_lock_enabled').value
        )
        self._apf_orca_encounter_lock_timeout = float(
            self.get_parameter('apf_orca_encounter_lock_timeout').value or 30.0
        )
        self._apf_orca_encounter_lock_release_ratio = float(
            self.get_parameter('apf_orca_encounter_lock_release_ratio').value or 1.5
        )
        self._apf_orca_crossing_dead_zone_deg = float(
            self.get_parameter('apf_orca_crossing_dead_zone_deg').value or 10.0
        )
        # P2: ORCA 迟滞
        self._apf_orca_hysteresis_margin = float(
            self.get_parameter('apf_orca_hysteresis_margin').value or 1.5
        )
        self._apf_orca_min_hold_time = float(
            self.get_parameter('apf_orca_min_hold_time').value or 5.0
        )
        # P1: COLREGS 强化
        self._apf_orca_colregs_override = bool(self.get_parameter('apf_orca_colregs_override').value)
        self._apf_orca_colregs_standon_scale = float(
            self.get_parameter('apf_orca_colregs_standon_scale').value or 0.3
        )
        self._apf_orca_colregs_head_on_bias = float(
            self.get_parameter('apf_orca_colregs_head_on_bias').value or 0.25
        )
        # P4: NH-ORCA
        self._apf_orca_nh_enabled = bool(self.get_parameter('apf_orca_nh_enabled').value)
        self._apf_orca_nh_reachable_horizon = float(
            self.get_parameter('apf_orca_nh_reachable_horizon').value or 2.0
        )
        self._apf_health_monitor_enabled = bool(self.get_parameter('apf_health_monitor_enabled').value)
        self._apf_health_freeze_pos_epsilon = float(
            self.get_parameter('apf_health_freeze_pos_epsilon').value or 0.01
        )
        self._apf_health_min_speed_for_freeze = float(
            self.get_parameter('apf_health_min_speed_for_freeze').value or 0.15
        )
        self._apf_health_own_freeze_warn_duration = float(
            self.get_parameter('apf_health_own_freeze_warn_duration').value or 0.8
        )
        self._apf_health_neighbor_freeze_warn_duration = float(
            self.get_parameter('apf_health_neighbor_freeze_warn_duration').value or 0.8
        )
        self._apf_health_warn_interval = float(
            self.get_parameter('apf_health_warn_interval').value or 2.0
        )
        
        # ==================== 初始化路径跟踪器 ====================
        self.get_logger().info(f'🛠️ 正在创建 VelocityPathTracker (MPC)...')
        
        # 保存 MPC 参数供日志记录使用
        self._mpc_params = {
            'q_pos': float(self.get_parameter('mpc_weight_pos').value or 10.0),
            'q_theta': float(self.get_parameter('mpc_weight_heading').value or 8.0),
            'r_w': float(self.get_parameter('mpc_weight_steering').value or 5.0),
            'r_dw': float(self.get_parameter('mpc_weight_steering_rate').value or 10.0),
            'w_max': float(self.get_parameter('max_angular_velocity').value or 0.5),
            'n_steps': int(self.get_parameter('mpc_prediction_steps').value or 20),
            # v5 新增
            'tau_omega': float(self.get_parameter('mpc_tau_omega').value or 0.4),
            'q_cte': float(self.get_parameter('mpc_weight_cte').value or 15.0),
        }
        
        # v8: AMPC 参数
        self._ampc_enabled = bool(self.get_parameter('ampc_enabled').value)
        self._ampc_params = {
            'ampc_enabled': self._ampc_enabled,
            'ampc_rls_forgetting_factor': float(self.get_parameter('ampc_rls_forgetting_factor').value or 0.97),
            'ampc_heading_observer_alpha': float(self.get_parameter('ampc_heading_observer_alpha').value or 0.3),
            'ampc_tau_min': float(self.get_parameter('ampc_tau_min').value or 0.1),
            'ampc_tau_max': float(self.get_parameter('ampc_tau_max').value or 3.0),
            'ampc_rebuild_threshold': float(self.get_parameter('ampc_rebuild_threshold').value or 0.15),
            'ampc_saturation_tau_boost': float(self.get_parameter('ampc_saturation_tau_boost').value or 1.05),
        }
        
        try:
            self.tracker = VelocityPathTracker(
                cruise_speed=float(self.get_parameter('cruise_speed').value or 0.5),
                max_angular_velocity=float(self.get_parameter('max_angular_velocity').value or 0.5),
                
                # MPC 参数传递
                mpc_v_max=float(self.get_parameter('cruise_speed').value or 0.4),
                mpc_w_max=self._mpc_params['w_max'],
                mpc_q_pos=self._mpc_params['q_pos'],
                mpc_q_theta=self._mpc_params['q_theta'],
                mpc_r_w=self._mpc_params['r_w'],
                mpc_r_dw=self._mpc_params['r_dw'],
                mpc_prediction_steps=self._mpc_params['n_steps'],
                # v5 新增参数
                mpc_tau_omega=self._mpc_params['tau_omega'],
                mpc_q_cte=self._mpc_params['q_cte'],
                
                # v8 AMPC 参数
                **self._ampc_params,
                
                min_speed=float(self.get_parameter('min_speed').value or 0.05),
                goal_tolerance=float(self.get_parameter('goal_tolerance').value or 0.5),
                switch_tolerance=float(self.get_parameter('switch_tolerance').value or 1.5),
                angular_velocity_filter=float(self.get_parameter('angular_velocity_filter').value or 0.3),
            )
            ampc_status = 'AMPC在线辨识' if self._ampc_enabled else 'MPC v5 标准'
            self.get_logger().info(f'✅ VelocityPathTracker 初始化成功 (模式: {ampc_status})')
        except Exception as e:
            self.get_logger().fatal(f'❌ VelocityPathTracker 初始化失败: {e}')
            raise e
        
        # ==================== QoS 配置 ====================
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # ==================== 状态变量 ====================
        self.current_pose: Optional[Pose2D] = None
        self.current_state: Optional[State] = None
        self._current_goal_id: Optional[int] = None
        self._last_velocity_cmd: Optional[VelocityCommand] = None
        self._control_active = False
        
        # ==================== 避障状态 ====================
        self._avoidance_active = False           # 避障模式是否激活
        self._avoidance_position: Optional[Pose2D] = None  # 避障目标位置
        
        # ==================== 避障停滞检测与后退状态 ====================
        self._avoidance_best_dist: float = float('inf')  # 避障期间最优（最小）目标距离
        self._avoidance_no_progress_start: float = 0.0   # 开始无进展的时间戳
        self._avoidance_stall_timeout: float = 60.0      # 无进展超时 (秒)
        self._retreat_active: bool = False                # 是否正在执行后退动作
        self._retreat_start_time: float = 0.0             # 后退开始时间
        self._retreat_duration: float = 10.0              # 后退持续时间 (秒)
        
        # ==================== 旋转机动状态 ====================
        self._rotation_active = False            # 是否正在执行旋转
        self._rotation_target_yaw = 0.0          # 目标旋转总角度 (rad)
        self._rotation_accumulated = 0.0         # 累计旋转角度 (rad)
        self._rotation_last_yaw = 0.0            # 上一次记录的航向
        self._rotation_initialized = False       # 是否已初始化旋转
        self._rotation_yaw_rate = 0.5            # 旋转角速度 (rad/s)
        self._rotation_goal_id: Optional[int] = None  # 旋转任务的 goal_id
        
        # ==================== 健壮性增强 ====================
        self._last_pose_time: float = 0.0
        self._last_state_time: float = 0.0
        self._pose_timeout: float = 5.0   # 位姿超时 (秒), 适配非实时仿真
        self._state_timeout: float = 10.0  # 飞控状态超时 (秒), 适配非实时仿真(~0.2x)
        self._consecutive_timeout_count: int = 0
        self._max_timeout_before_stop: int = 5  # 连续超时次数阈值
        
        # ==================== 模式保护 ====================
        # 设计原则: 导航任务进行中 GUIDED 模式为最高优先级
        # 任何非 GUIDED 模式(HOLD/MANUAL/LOITER等)被检测到时立即恢复 GUIDED
        # 不允许任何停顿影响导航效果
        self._mode_protection_enabled: bool = True  # 导航中自动恢复 GUIDED 模式
        self._last_mode_restore_time: float = 0.0   # 上次恢复模式的时间
        self._mode_restore_cooldown: float = 0.2    # 恢复模式冷却时间 (秒) - 极短，仅防消息风暴
        
        # ====== 连续HOLD强制暂停机制 ======
        # 地面站手动HOLD需要连续触发多次才能强制暂停任务
        # 单次HOLD(无论来源)不会暂停导航，会立即恢复GUIDED
        # 这样飞控自动HOLD、意外HOLD都不会中断导航
        self._manual_hold_requested: bool = False
        self._manual_hold_request_time: float = 0.0  # 最近一次请求时间戳
        self._manual_hold_timeout: float = 3600.0    # 手动请求有效期 (1小时，实际由新任务清除)
        self._consecutive_hold_count: int = 0        # 连续HOLD请求计数
        self._consecutive_hold_window: float = 5.0   # 连续HOLD计数窗口 (秒)
        self._consecutive_hold_threshold: int = int(self.get_parameter('consecutive_hold_threshold').value or 3)
        self._last_hold_request_time: float = 0.0    # 上一次HOLD请求的时间
        
        # PAUSED 状态保护 - 仅在强制暂停后使用
        self._paused_state_enter_time: float = 0.0   # 进入 PAUSED 状态的时间
        self._paused_state_grace_period: float = 0.5  # 暂停状态保护期 (秒) - 极短
        
        # GUIDED 切换保护期 - 发送 GUIDED 切换命令后等待生效
        self._guided_switch_request_time: float = 0.0  # 上次请求切换 GUIDED 的时间
        self._guided_switch_grace_period: float = 1.0  # 切换保护期 (秒) - 缩短
        
        # 延迟 HOLD 切换 - 防止多步任务中每 2 步频繁 HOLD/GUIDED 切换
        self._delayed_hold_pending: bool = False      # 是否有待执行的延迟 HOLD
        self._delayed_hold_deadline: float = 0.0      # 延迟 HOLD 的执行截止时间
        self._delayed_hold_delay: float = 3.0         # 延迟时间 (秒)
        
        # 延迟软停止 - COMPLETED 状态下不立即停止，等待后续航点
        # 解决多 batch 导航中 batch 间隙(~1s)被强制停止的问题
        self._delayed_stop_pending: bool = False      # 是否有待执行的延迟停止
        self._delayed_stop_deadline: float = 0.0      # 延迟停止的执行截止时间
        self._delayed_stop_delay: float = 1.5         # 延迟时间 (秒)，大于典型 batch 间隔 (~1s)
        
        # 渐进减速 - 导航完成后平滑减速至零，替代急刹车
        # 在 _end_navigation(COMPLETED) 后立即启动，线性降低速度
        # 如果减速期间收到新航点，自动取消减速并恢复导航
        self._soft_decel_active: bool = False           # 渐进减速是否激活
        self._soft_decel_start_time: float = 0.0        # 渐进减速开始时间
        self._soft_decel_duration: float = float(self.get_parameter('soft_decel_duration').value or 3.0)
        self._soft_decel_start_vx: float = 0.0          # 减速开始时的线速度
        self._soft_decel_start_omega: float = 0.0       # 减速开始时的角速度
        
        # 导航状态管理 (使用枚举替代简单布尔值)
        self._navigation_state: NavigationState = NavigationState.IDLE
        self._navigation_active: bool = False       # 兼容性：是否有活跃的导航任务
        
        self._last_valid_pose: Optional[Pose2D] = None  # 用于跳变检测
        self._pose_jump_threshold: float = 3.0  # 位姿跳变阈值 (m) - 降低以检测小幅漂移

        # APF 邻船状态缓存: topic/usv_id -> (x, y, vx, vy, timestamp)
        self._apf_neighbor_states: Dict[str, Tuple[float, float, float, float, float]] = {}
        self._health_last_own_pose: Optional[Tuple[float, float]] = None
        self._health_own_freeze_start: float = 0.0
        self._health_own_last_warn: float = 0.0
        self._health_neighbor_freeze: Dict[str, Dict[str, float]] = {}
        # P2: ORCA 迟滞状态
        self._orca_active: bool = False                # ORCA 当前是否激活
        self._orca_activate_time: float = 0.0          # 上次激活时间
        self._orca_committed_side: int = 0
        self._orca_commit_deadline: float = 0.0
        self._orca_close_enter_time: float = 0.0
        self._orca_coupling_enter_time: float = 0.0
        self._orca_coupling_active: bool = False
        # 制动迟滞状态: 避免通信延迟导致的交替制动振荡
        self._orca_soft_brake_active: bool = False
        self._orca_hard_brake_active: bool = False
        # 输出平滑: 限制每 tick 的速度变化率，避免急停急走
        self._orca_smooth_vx: float = 0.0    # 上一帧输出的 cmd_vx
        self._orca_smooth_omega: float = 0.0 # 上一帧输出的 cmd_omega
        self._orca_smooth_initialized: bool = False
        # 安全锁: 诸船距离记录，近距时禁止 ORCA 退出
        self._orca_last_closest_distance: float = float('inf')
        # v14 新增: ORCA 调试状态 (每 tick 更新，供 debug_msg 发布)
        self._orca_debug_closest_distance: float = -1.0
        self._orca_debug_encounter_type: str = 'none'
        self._orca_debug_encounter_type_raw: str = 'none'
        self._orca_debug_commit_side: int = 0
        self._orca_debug_linear_correction: float = 0.0
        self._orca_debug_angular_correction: float = 0.0
        self._orca_debug_neighbor_count: int = 0
        self._orca_debug_rel_bearing_deg: float = -1.0
        self._orca_debug_rel_course_deg: float = -1.0
        self._orca_debug_rel_speed: float = -1.0
        self._orca_debug_tcpa: float = -1.0
        self._orca_debug_dcpa: float = -1.0
        self._orca_debug_primary_neighbor_id: str = ''
        self._orca_debug_escape_active: bool = False
        self._orca_debug_escape_phase: int = 0
        self._orca_debug_escape_direction: int = 0
        self._orca_debug_escape_count: int = 0
        self._orca_encounter_type_smoothed: str = 'none'
        self._orca_encounter_candidate: Optional[str] = None
        self._orca_encounter_candidate_since: float = 0.0
        # P6: 会遇类型锁定状态
        self._orca_encounter_locked_type: str = 'none'    # 锁定的会遇类型
        self._orca_encounter_locked_side: int = 0          # 锁定的避让方向
        self._orca_encounter_lock_time: float = 0.0        # 锁定开始时间
        self._orca_encounter_lock_distance: float = 0.0    # 锁定时的初始距离

        # ==================== ORCA 脱困机制 ====================
        # 检测 ORCA 导致的长时间导航停滞（如两船近距振荡死锁），
        # 通过后退+转向脱离死锁区域，再以削弱 ORCA 的方式绕行通过。
        self._orca_escape_active: bool = False              # 脱困模式是否激活
        self._orca_escape_start_time: float = 0.0           # 脱困开始时间
        self._orca_escape_phase: int = 0                    # 脱困阶段: 1=后退转向, 2=削弱绕行
        self._orca_escape_direction: int = -1               # 脱困转向方向: 1=左转, -1=右转
        self._orca_escape_phase1_duration: float = 8.0      # 阶段1持续时间 (秒)
        self._orca_escape_phase2_duration: float = 15.0     # 阶段2持续时间 (秒)
        self._orca_stall_start_time: float = 0.0            # ORCA停滞检测开始时间
        self._orca_stall_best_dist: float = float('inf')    # ORCA期间到目标最优距离
        self._orca_stall_timeout: float = 60.0              # ORCA停滞超时 (秒)
        self._orca_stall_progress_threshold: float = 1.0    # 有效进展阈值 (米)
        self._orca_stall_start_pose: tuple = None            # 停滞开始时的位置 (x, y)
        self._orca_stall_displacement_threshold: float = 0.6 # 位移阈值: 超过此值视为仍在移动 (米)
        self._orca_escape_count: int = 0                    # 连续脱困次数 (用于升级策略)
        self._orca_escape_max_count: int = 5                # 最大连续脱困次数

        self._apf_neighbor_subs = []
        self._apf_fleet_sub = None
        self._recovery_enabled: bool = True  # 启用自动恢复
        self._was_timed_out: bool = False  # 是否曾经超时
        
        # 位姿跳变恢复机制 - 用于处理定位源切换导致的稳定偏移
        self._consecutive_jump_count: int = 0  # 连续检测到跳变的次数
        self._jump_recovery_threshold: int = 5  # 连续N次跳变后接受新位姿（认为是定位源切换）
        self._last_jump_pose: Optional[Pose2D] = None  # 上一次检测到跳变时的新位姿
        self._jump_consistency_tolerance: float = 1.0  # 连续跳变位置一致性容差 (m)
        
        # ==================== L1 风格航向估计 ====================
        # 使用飞控 EKF 融合的速度向量计算实际航向（类似 L1 算法）
        self._use_velocity_heading = bool(self.get_parameter('use_velocity_based_heading').value)
        self._min_speed_for_velocity_yaw = float(self.get_parameter('min_speed_for_velocity_heading').value)
        self._velocity_based_yaw: float = 0.0  # 基于速度向量的航向
        self._velocity_yaw_valid: bool = False  # 速度航向是否有效
        self._current_speed: float = 0.0  # 当前速度 (m/s)
        self._current_vx: float = 0.0  # 当前速度 x (map)
        self._current_vy: float = 0.0  # 当前速度 y (map)
        
        # ==================== v6: 速度自适应 tau_omega ====================
        # 低速时舵效差，转向惯性相对更大，需要更大的 tau_omega
        self._adaptive_tau_enabled = bool(self.get_parameter('adaptive_tau_enabled').value or True)
        self._tau_omega_low = float(self.get_parameter('tau_omega_low_speed').value or 0.8)
        self._tau_omega_high = float(self.get_parameter('tau_omega_high_speed').value or 0.4)
        self._tau_speed_low = float(self.get_parameter('tau_speed_threshold_low').value or 0.15)
        self._tau_speed_high = float(self.get_parameter('tau_speed_threshold_high').value or 0.35)
        self._current_tau_omega = self._mpc_params['tau_omega']  # 当前使用的 tau_omega
        self._last_tau_update_time: float = 0.0
        self._tau_update_interval: float = 0.5  # tau_omega 更新间隔 (秒，避免频繁重建求解器)
        
        # ==================== 订阅者 ====================
        # 位姿订阅
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose_from_gps',
            self._pose_callback,
            qos_best_effort,
            callback_group=self.callback_group
        )
        
        # 飞控状态订阅
        self.state_sub = self.create_subscription(
            State,
            'state',
            self._state_callback,
            qos_best_effort,
            callback_group=self.callback_group
        )
        
        # 速度订阅 (MAVROS EKF 融合后的速度向量，用于 L1 风格航向估计)
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            'local_position/velocity_local',
            self._velocity_callback,
            qos_best_effort,
            callback_group=self.callback_group
        )
        
        # 导航目标订阅
        self.nav_goal_sub = self.create_subscription(
            NavigationGoal,
            'set_usv_nav_goal',
            self._nav_goal_callback,
            qos_reliable,
            callback_group=self.callback_group
        )
        
        # 导航结果订阅 (来自 navigate_to_point_node 的到达通知)
        self.nav_result_sub = self.create_subscription(
            NavigationResult,
            'navigation_result',
            self._nav_result_callback,
            qos_reliable,
            callback_group=self.callback_group
        )
        
        # 暂停导航订阅 (来自地面站 HOLD 按钮)
        self.cancel_nav_sub = self.create_subscription(
            Bool,
            'cancel_navigation',
            self._cancel_navigation_callback,
            qos_reliable,
            callback_group=self.callback_group
        )
        
        # 停止导航订阅 (来自地面站集群 STOP 按钮)
        self.stop_nav_sub = self.create_subscription(
            Bool,
            'stop_navigation',
            self._stop_navigation_callback,
            qos_reliable,
            callback_group=self.callback_group
        )
        
        # 参数更新订阅
        self.cruise_speed_sub = self.create_subscription(
            Float32,
            'set_velocity_cruise_speed',
            self._cruise_speed_callback,
            qos_reliable,
            callback_group=self.callback_group
        )
        
        self.goal_tolerance_sub = self.create_subscription(
            Float32,
            'set_velocity_goal_tolerance',
            self._goal_tolerance_callback,
            qos_reliable,
            callback_group=self.callback_group
        )
        
        self.switch_tolerance_sub = self.create_subscription(
            Float32,
            'set_velocity_switch_tolerance',
            self._switch_tolerance_callback,
            qos_reliable,
            callback_group=self.callback_group
        )
        
        # ==================== 与 navigate_to_point_node 阈值同步 ====================
        # 订阅 navigate_to_point_node 的阈值设置话题，保持两个节点阈值一致
        self.nav_arrival_threshold_sub = self.create_subscription(
            Float32,
            'set_nav_arrival_threshold',
            self._goal_tolerance_callback,  # 复用相同的回调
            qos_reliable,
            callback_group=self.callback_group
        )
        
        self.nav_switch_threshold_sub = self.create_subscription(
            Float32,
            'set_nav_switch_threshold',
            self._switch_tolerance_callback,  # 复用相同的回调
            qos_reliable,
            callback_group=self.callback_group
        )
        
        # 最大角速度
        self.max_angular_sub = self.create_subscription(
            Float32,
            'set_velocity_max_angular',
            self._max_angular_callback,
            qos_reliable,
            callback_group=self.callback_group
        )
        
        # ==================== 避障订阅 ====================
        # 避障目标位置
        self.avoidance_position_sub = self.create_subscription(
            PositionTarget,
            'avoidance_position',
            self._avoidance_position_callback,
            qos_best_effort,
            callback_group=self.callback_group
        )
        
        # 避障标志
        self.avoidance_flag_sub = self.create_subscription(
            Bool,
            'avoidance_flag',
            self._avoidance_flag_callback,
            qos_reliable,
            callback_group=self.callback_group
        )

        # ==================== 分布式 APF 邻船位姿订阅 ====================
        if self._apf_enabled:
            if self._apf_use_fleet_topic:
                self._apf_fleet_sub = self.create_subscription(
                    FleetNeighborPoses,
                    self._apf_fleet_topic,
                    self._apf_fleet_neighbors_callback,
                    qos_best_effort,
                    callback_group=self.callback_group
                )
            elif self._apf_neighbor_topics:
                for topic_name in self._apf_neighbor_topics:
                    sub = self.create_subscription(
                        PoseStamped,
                        str(topic_name),
                        lambda msg, t=str(topic_name): self._apf_neighbor_pose_callback(msg, t),
                        qos_best_effort,
                        callback_group=self.callback_group
                    )
                    self._apf_neighbor_subs.append(sub)
        
        # ==================== 发布者 ====================
        # 速度指令发布
        self.velocity_pub = self.create_publisher(
            PositionTarget,
            'setpoint_raw/local',
            qos_best_effort
        )
        
        # 状态发布
        self.status_pub = self.create_publisher(
            String,
            'velocity_controller/status',
            qos_reliable
        )
        
        # 导航结果发布 (用于通知上层节点)
        self.result_pub = self.create_publisher(
            NavigationResult,
            'velocity_controller/result',
            qos_reliable
        )
        
        # 导航反馈发布 (实时状态)
        self.feedback_pub = self.create_publisher(
            NavigationFeedback,
            'velocity_controller/feedback',
            qos_best_effort
        )
        
        # 模式切换发布 (用于自动恢复 GUIDED 和任务完成后切换 HOLD)
        self.mode_pub = self.create_publisher(
            String,
            'set_usv_mode',
            qos_reliable
        )
        
        # 调试信息发布
        self.debug_pub = self.create_publisher(
            MpcDebug,
            'velocity_controller/debug',
            qos_best_effort
        )

        # ==================== 控制循环 ====================
        control_rate = float(self.get_parameter('control_rate').value or 20.0)
        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(
            control_period, 
            self._control_loop,
            callback_group=self.callback_group
        )
        
        # 状态发布定时器 (1Hz)
        self.status_timer = self.create_timer(
            1.0,
            self._publish_status,
            callback_group=self.callback_group
        )
        
        # 日志计数器
        self._log_counter = 0
        
        # 启动日志
        self.get_logger().info('='*60)
        self.get_logger().info('USV 速度控制器节点已启动')
        self.get_logger().info(f'  控制模式: {self.control_mode}')
        if self.control_mode == 'velocity':
            self.get_logger().info('  功能: 常规导航 + 避障 + 旋转机动')
            self.get_logger().info('  输出: 速度指令 → setpoint_raw/local')
            if self._apf_enabled:
                if self._apf_use_fleet_topic:
                    self.get_logger().info(
                        f'  APF避碰: 已启用 (统一邻船话题={self._apf_fleet_topic}, 安全半径={self._apf_safe_distance:.2f}m)'
                    )
                else:
                    self.get_logger().info(
                        f'  APF避碰: 已启用 (邻船话题数={len(self._apf_neighbor_topics)}, 安全半径={self._apf_safe_distance:.2f}m)'
                    )
                if self._apf_orca_enabled:
                    self.get_logger().info(
                        f'  ORCA避让: 已启用 (T={self._apf_orca_time_horizon:.1f}s, 最小分离={self._apf_orca_min_separation:.2f}m, '
                        f'左右承诺={self._apf_orca_side_commit_enabled}, 策略={self._apf_orca_commit_policy})'
                    )
                    if self._apf_orca_encounter_rule_enabled:
                        self.get_logger().info(
                            f'  会遇规则: 已启用 (引导距离={self._apf_orca_encounter_guidance_distance:.1f}m, '
                            f'head_on={self._apf_orca_encounter_head_on_deg:.1f}°, crossing={self._apf_orca_encounter_crossing_deg:.1f}°, '
                            f'静止阈值={self._apf_orca_stationary_speed_threshold:.2f}m/s, '
                            f'静止缩减={self._apf_orca_stationary_correction_scale:.2f})'
                        )
            else:
                self.get_logger().info('  APF避碰: 未启用')
            if self._use_velocity_heading:
                self.get_logger().info('  航向估计: L1风格（飞控EKF速度向量优先，低速回退磁力计）')
            else:
                self.get_logger().info('  航向估计: 磁力计')
        else:
            self.get_logger().info('  功能: 待机 (由 usv_control_node 处理)')
        self.get_logger().info('  控制器类型: MPC')
        if self._ampc_enabled:
            self.get_logger().info('  🧠 AMPC 在线辨识: 已启用 (τ_omega 自动适应)')
        else:
            self.get_logger().info('  AMPC 在线辨识: 未启用 (使用 v6 速度自适应 tau)')
        self.get_logger().info(f'  巡航速度: {self.tracker.cruise_speed} m/s')
        self.get_logger().info(f'  最大角速度: {self.tracker.max_angular_velocity} rad/s')
        self.get_logger().info(f'  到达阈值: {self.tracker.goal_tolerance} m')
        self.get_logger().info(f'  切换阈值: {self.tracker.switch_tolerance} m')
        self.get_logger().info('='*60)
    
    # ==================== 回调函数 ====================
    
    def _velocity_callback(self, msg: TwistStamped):
        """
        速度回调 - L1 风格航向估计 + 自适应 tau_omega
        
        使用飞控 EKF 融合后的速度向量计算实际航向，
        类似飞控 L1 算法，自动适应坐标系偏移。
        
        v6 新增: 根据当前速度自适应调整 MPC 的 tau_omega 参数，
        解决低速时 S 形振荡问题。
        """
        import time
        
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self._current_vx = vx
        self._current_vy = vy
        speed = math.sqrt(vx * vx + vy * vy)
        self._current_speed = speed
        
        # 只有速度足够时才使用速度方向估计航向
        if speed > self._min_speed_for_velocity_yaw:
            self._velocity_based_yaw = math.atan2(vy, vx)
            self._velocity_yaw_valid = True
        else:
            # 速度太低，速度航向不可靠
            self._velocity_yaw_valid = False
        
        # ==================== v6: 速度自适应 tau_omega ====================
        # v8: AMPC 启用时，跳过 v6 的速度自适应 tau，因为 AMPC 通过 RLS 在线辨识真实 tau
        # 两套自适应逻辑不应同时工作，否则会互相干扰
        if self._adaptive_tau_enabled and not self._ampc_enabled:
            self._update_adaptive_tau_omega(speed)
    
    def _pose_callback(self, msg: PoseStamped):
        """位姿回调 - 包含数据验证、跳变检测和航向选择"""
        import time
        
        current_time = time.time()
        
        # 从四元数提取 yaw（磁力计航向，仅作为初始回退）
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        magnetometer_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # ==================== 航向选择 ====================
        # L1 改进：分离航向(Heading)和航迹(Course)
        # yaw 始终使用磁力计/EKF融合的船头朝向 (Heading)
        # course 使用速度向量方向 (Course over Ground)
        
        heading_yaw = magnetometer_yaw
        course_yaw = None
        current_speed = self._current_speed
        
        if self._use_velocity_heading and self._velocity_yaw_valid:
            course_yaw = self._velocity_based_yaw
            # 记录最后有效速度航向供调试
            self._last_valid_velocity_yaw = course_yaw
            
        new_pose = Pose2D(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            yaw=heading_yaw,
            course=course_yaw,
            speed=current_speed
        )
        
        # 数据有效性检查
        if not new_pose.is_valid():
            self.get_logger().warn('收到无效位姿数据，已忽略')
            return
        
        # 位姿跳变检测 (GPS 跳变保护) - 带恢复机制
        if self._last_valid_pose is not None:
            jump_distance = new_pose.distance_to(self._last_valid_pose)
            if jump_distance > self._pose_jump_threshold:
                # 检测到跳变，判断是否是持续性偏移（定位源切换）
                if self._last_jump_pose is not None:
                    # 检查新位姿与上次跳变位姿是否一致（说明是稳定的定位源切换）
                    consistency_distance = new_pose.distance_to(self._last_jump_pose)
                    if consistency_distance < self._jump_consistency_tolerance:
                        # 新位姿与上次跳变位姿一致，累加计数
                        self._consecutive_jump_count += 1
                    else:
                        # 新位姿与上次跳变位姿不一致，重置计数
                        self._consecutive_jump_count = 1
                else:
                    self._consecutive_jump_count = 1
                
                self._last_jump_pose = new_pose
                
                # 检查是否达到恢复阈值
                if self._consecutive_jump_count >= self._jump_recovery_threshold:
                    self.get_logger().warn(
                        f'⚠️ 定位源切换检测: 连续{self._consecutive_jump_count}次检测到稳定偏移 '
                        f'({jump_distance:.2f}m)，接受新位姿作为新参考点'
                    )
                    # 重置跳变检测状态
                    self._consecutive_jump_count = 0
                    self._last_jump_pose = None
                    # 继续执行，接受新位姿
                else:
                    self.get_logger().warn(
                        f'位姿跳变检测: {jump_distance:.2f}m > {self._pose_jump_threshold}m，'
                        f'暂停更新 (连续{self._consecutive_jump_count}/{self._jump_recovery_threshold})'
                    )
                    # 不更新位姿，等待稳定或达到恢复阈值
                    return
            else:
                # 位姿正常，重置跳变检测状态
                self._consecutive_jump_count = 0
                self._last_jump_pose = None
        
        self.current_pose = new_pose
        self._last_valid_pose = new_pose
        self._last_pose_time = current_time
        self._consecutive_timeout_count = 0  # 重置超时计数
        self._update_own_freeze_health(new_pose, current_time)
    
    def _update_adaptive_tau_omega(self, current_speed: float):
        """
        根据当前速度自适应调整 MPC 的 tau_omega 参数
        
        核心原理:
        - 高速时 (>0.35 m/s): 舵效好，tau_omega = 0.4s (响应快)
        - 低速时 (<0.15 m/s): 舵效差，tau_omega = 0.8s (响应慢)
        - 中间速度: 线性插值
        
        这解决了 MPC 使用固定 tau_omega 时，低速下模型失配导致的 S 形振荡问题。
        所有 USV 无需单独调参，因为舵效-速度关系是船舶动力学的普遍规律。
        
        Args:
            current_speed: 当前速度 (m/s)
        """
        import time
        
        current_time = time.time()
        
        # 限制更新频率，避免频繁重建 MPC 求解器
        if current_time - self._last_tau_update_time < self._tau_update_interval:
            return
        
        # 计算自适应 tau_omega
        if current_speed <= self._tau_speed_low:
            # 低速区: 使用大 tau (转向慢)
            new_tau = self._tau_omega_low
        elif current_speed >= self._tau_speed_high:
            # 高速区: 使用小 tau (转向快)
            new_tau = self._tau_omega_high
        else:
            # 过渡区: 线性插值
            ratio = (current_speed - self._tau_speed_low) / (self._tau_speed_high - self._tau_speed_low)
            new_tau = self._tau_omega_low + ratio * (self._tau_omega_high - self._tau_omega_low)
        
        # 检查是否需要更新 (变化超过 5% 才更新，避免频繁重建)
        if abs(new_tau - self._current_tau_omega) / self._current_tau_omega > 0.05:
            self._current_tau_omega = new_tau
            self._last_tau_update_time = current_time
            
            # 动态更新 MPC 控制器的 tau_omega
            try:
                self.tracker.mpc_tracker.set_tau_omega(new_tau)
                self.get_logger().debug(
                    f'🔧 Adaptive tau_omega: speed={current_speed:.2f} m/s -> tau={new_tau:.2f}s'
                )
            except Exception as e:
                self.get_logger().warning(f'Failed to update tau_omega: {e}')
    
    def _state_callback(self, msg: State):
        """飞控状态回调"""
        import time
        self.current_state = msg
        self._last_state_time = time.time()
    
    def _nav_result_callback(self, msg: NavigationResult):
        """
        导航结果回调 - 来自 navigate_to_point_node 的到达通知
        
        当 navigate_to_point_node 判定到达目标后，会发送 NavigationResult。
        注意：平滑切换时也会发送 result（message 包含"已通过"），此时不应停止。
        只有最终到达时（message 包含"成功到达目标"）才停止追踪。
        
        状态转换: ACTIVE → COMPLETED
        """
        goal_id = getattr(msg, 'goal_id', 0)
        success = getattr(msg, 'success', False)
        message = getattr(msg, 'message', '')
        
        self.get_logger().debug(
            f'收到导航结果: goal_id={goal_id}, success={success}, message={message}, '
            f'当前追踪ID={self._current_goal_id}, nav_state={self._navigation_state.name}'
        )
        
        if success:
            # 区分"通过航点"和"最终到达"
            # 平滑切换时 message 包含 "已通过航点"，不应停止
            # 最终到达时 message 包含 "成功到达目标"
            is_pass_through = '已通过' in message or '通过航点' in message
            
            if is_pass_through:
                self.get_logger().debug(
                    f'收到通过通知 [ID={goal_id}], 继续导航')
                return
            
            # 检查是否是最终到达（包含"成功到达"且导航任务激活）
            is_final_arrival = '成功到达' in message
            is_nav_active = self._navigation_state in (NavigationState.ACTIVE, NavigationState.PAUSED)
            
            if is_final_arrival and is_nav_active:
                self.get_logger().info(
                    f'✅ 收到最终到达通知 [ID={goal_id}], 停止追踪 (当前追踪ID={self._current_goal_id})')
                
                # 使用统一的结束方法，设置为 COMPLETED 状态
                self._end_navigation(NavigationState.COMPLETED, "成功到达目标")
                
                # 延迟切换到 HOLD 模式：等渐进减速完成后再切换
                # 延迟时间 = 减速持续时间 + 缓冲时间，确保 HOLD 在减速完成之后
                # 如果在延迟期内收到新目标，取消 HOLD 切换，避免频繁 HOLD/GUIDED 循环
                hold_delay = self._soft_decel_duration + 1.0
                self._delayed_hold_pending = True
                self._delayed_hold_deadline = time.time() + hold_delay
                self.get_logger().debug(
                    f'⏳ 延迟 HOLD 切换 {hold_delay:.1f}s (减速{self._soft_decel_duration:.1f}s + 缓冲1.0s)')
                
            elif self._current_goal_id is not None and goal_id == self._current_goal_id:
                # 兼容旧逻辑：ID 匹配也停止追踪，但不切换 HOLD
                # 中间航点的内部到达检测（如 _on_goal_reached）会走这条路径
                # 保持 GUIDED 模式，等待 navigate_to_point_node 发送下一个目标
                self.get_logger().info(
                    f'✅ 收到到达通知 [ID={goal_id}], 停止追踪 (保持GUIDED等待下一目标)')
                
                self._end_navigation(NavigationState.COMPLETED, "到达当前航点")
            else:
                self.get_logger().debug(
                    f'收到到达通知 [ID={goal_id}], message="{message}", 忽略'
                )
        else:
            # ===== 失败结果处理 (超时/取消/异常) =====
            # navigate_to_point_node 发送 success=False 表示导航失败:
            #   error_code=1: 导航超时
            #   error_code=2: 暂停
            #   error_code=3: 停止
            # 必须清除 _control_active，否则 velocity_controller 会永远停留在
            # "导航中 vx=0.00" 的僵尸状态 (tracker 已 goal_reached 但 _control_active 未清除)
            error_code = getattr(msg, 'error_code', 0)
            is_nav_active = self._navigation_state in (NavigationState.ACTIVE, NavigationState.PAUSED)
            
            if is_nav_active:
                self.get_logger().warn(
                    f'⚠️ 收到导航失败通知 [ID={goal_id}], error_code={error_code}, '
                    f'message="{message}", 停止追踪 (当前追踪ID={self._current_goal_id})')
                self._end_navigation(NavigationState.FAILED, f"导航失败: {message}")
            else:
                self.get_logger().debug(
                    f'收到导航失败通知 [ID={goal_id}], 但当前非导航状态({self._navigation_state.name}), 忽略')
    
    def _cancel_navigation_callback(self, msg: Bool):
        """
        取消导航回调 - 来自地面站的暂停请求
        
        当用户在地面站点击 HOLD 按钮时，会发送此消息。
        
        ===== 连续HOLD强制暂停机制 =====
        导航任务进行中，GUIDED 模式为最高优先级:
        - 单次 HOLD 请求: 不暂停，自动恢复 GUIDED 继续导航
        - 连续 N 次 HOLD 请求(在时间窗口内): 强制暂停任务
        
        这确保了:
        1. 飞控自动切换的 HOLD 不会中断导航
        2. 偶尔的误触不会影响导航
        3. 操作员需要明确连续操作才能强制暂停
        """
        if not msg.data:
            return
        
        import time as _time
        now = _time.time()
        
        # ===== 连续HOLD计数 =====
        if (now - self._last_hold_request_time) <= self._consecutive_hold_window:
            self._consecutive_hold_count += 1
        else:
            # 超出时间窗口，重新计数
            self._consecutive_hold_count = 1
        self._last_hold_request_time = now
        
        self.get_logger().info(
            f'📥 收到HOLD请求 [{self._consecutive_hold_count}/{self._consecutive_hold_threshold}] '
            f'(需连续{self._consecutive_hold_threshold}次才能强制暂停)')
        
        # 检查是否达到强制暂停阈值
        if self._consecutive_hold_count >= self._consecutive_hold_threshold:
            # 达到阈值，强制暂停
            self.get_logger().warn(
                f'🛑 连续{self._consecutive_hold_count}次HOLD请求，强制暂停导航任务')
            self._manual_hold_requested = True
            self._manual_hold_request_time = self.get_clock().now().nanoseconds / 1e9
            self._consecutive_hold_count = 0  # 重置计数
            
            if self._navigation_state in (NavigationState.ACTIVE, NavigationState.PAUSED):
                self._set_navigation_state(NavigationState.PAUSED, "连续HOLD强制暂停")
                self._publish_velocity_command(VelocityCommand.stop())
            
            self.get_logger().info(
                f'✅ 导航任务已强制暂停 (manual_hold_requested={self._manual_hold_requested})')
        else:
            # 未达到阈值，不暂停，GUIDED 将在下一个控制循环自动恢复
            self.get_logger().info(
                f'⏩ HOLD请求次数不足，导航继续 (GUIDED将自动恢复)')
    
    def _stop_navigation_callback(self, msg: Bool):
        """
        停止导航回调 - 来自地面站集群 STOP 按钮
        
        完全停止导航任务，清空所有目标。
        需要发送新任务才能重新开始导航。
        
        注意: STOP 是强制操作，不受连续HOLD计数限制，立即生效。
        
        状态转换: ANY → CANCELLED (完全停止)
        """
        if not msg.data:
            return
        
        self.get_logger().warn('🛑 收到停止导航请求（集群STOP），任务完全停止')
        
        # STOP 是明确的停止意图，直接设置标志
        self._manual_hold_requested = True
        self._manual_hold_request_time = self.get_clock().now().nanoseconds / 1e9
        self._consecutive_hold_count = 0  # 重置计数
        
        # 清空所有导航目标
        self._target_x = None
        self._target_y = None
        self._current_goal_id = None
        
        # 设置为 CANCELLED 状态（完全停止）
        if self._navigation_state != NavigationState.IDLE:
            self._set_navigation_state(NavigationState.CANCELLED, "集群STOP停止")
            # 停止当前运动
            self._publish_velocity_command(VelocityCommand.stop())
        
        self.get_logger().info('✅ 导航任务已停止，等待新任务')
    
    def _nav_goal_callback(self, msg: NavigationGoal):
        """
        导航目标回调
        
        接收来自 navigate_to_point_node 或地面站的导航目标
        处理常规导航和旋转机动
        
        注意：NAV_MODE_TERMINAL (离群单点导航) 由 usv_control_node 的位置模式处理
        
        线程安全：使用 _nav_goal_lock 防止 ReentrantCallbackGroup 下的并发竞态。
        当 navigate_to_point_node 乱序修正后快速连发多个目标时，
        确保 set_waypoint 调用严格按消息到达顺序执行，避免后到的目标覆盖先到的目标。
        """
        target = msg.target_pose.pose.position
        goal_id = getattr(msg, 'goal_id', 0)
        nav_mode = getattr(msg, 'nav_mode', 0)
        
        self.get_logger().info(
            f'📨 velocity_controller 收到目标 [ID={goal_id}]: '
            f'({target.x:.2f}, {target.y:.2f}), nav_mode={nav_mode}, control_mode={self.control_mode}')
        
        # 仅在速度模式下处理
        if self.control_mode != 'velocity':
            self.get_logger().warn(f'⚠️ 非速度模式，忽略目标 [ID={goal_id}]')
            return
        
        NAV_MODE_TERMINAL = 3  # 定义在 NavigationGoal.msg
        
        # NAV_MODE_TERMINAL (离群单点导航) 跳过，让 usv_control_node 位置模式处理
        # 位置模式更适合精确定点停留，而非连续路径跟踪
        # 重要：必须主动让出控制权，停止发布速度指令，避免与 usv_control_node 的
        # GPS 位置指令在 ArduPilot GUIDED 模式下产生 setpoint 竞争
        if nav_mode == NAV_MODE_TERMINAL:
            self.get_logger().info(
                f'⏭️ 离群目标 [ID={goal_id}] ({target.x:.2f}, {target.y:.2f}) '
                f'使用位置模式处理 (NAV_MODE_TERMINAL), 让出速度控制权'
            )
            # 清理控制状态，防止控制循环继续发布零速指令
            # 这些零速指令会覆盖 usv_control_node 的 GPS 位置目标
            self.tracker.clear_waypoints()
            self._control_active = False
            self._current_goal_id = None
            # 取消可能残留的软减速和延迟 HOLD
            # （TERMINAL 模式的到达/HOLD 由 usv_control_node 和 navigate_to_point_node 处理）
            self._soft_decel_active = False
            self._delayed_stop_pending = False
            self._delayed_hold_pending = False
            # 旋转状态也需清理
            if self._rotation_active:
                self._rotation_active = False
                self._rotation_initialized = False
            # 设置导航状态为 IDLE，表示速度控制器不再参与控制
            self._set_navigation_state(NavigationState.IDLE, "TERMINAL模式让出控制权")
            return
        
        # 加锁保护：防止并发回调导致 set_waypoint 执行顺序错乱
        with self._nav_goal_lock:
            self._process_nav_goal(msg, target, goal_id, nav_mode)
    
    def _process_nav_goal(self, msg: NavigationGoal, target, goal_id: int, nav_mode: int):
        """导航目标处理（锁内执行，线程安全）"""
        self._current_goal_nav_mode = nav_mode  # 记录当前导航模式，用于 feedback
        maneuver_type = getattr(msg, 'maneuver_type', 0)
        maneuver_param = getattr(msg, 'maneuver_param', 0.0)
        
        # MANEUVER_TYPE_ROTATE = 1 (定义在 NavigationGoal.msg)
        MANEUVER_TYPE_ROTATE = 1
        
        # ==================== 旋转机动处理 ====================
        if maneuver_type == MANEUVER_TYPE_ROTATE:
            # 检查是否是新的旋转任务
            if goal_id != self._rotation_goal_id:
                self._rotation_goal_id = goal_id
                self._rotation_active = True
                self._rotation_initialized = False
                self._rotation_accumulated = 0.0
                self._rotation_target_yaw = maneuver_param * 2 * math.pi  # 圈数转弧度
                
                # 设置旋转方向
                if maneuver_param >= 0:
                    self._rotation_yaw_rate = 0.5  # 顺时针
                else:
                    self._rotation_yaw_rate = -0.5  # 逆时针
                
                # 同时设置导航目标（先导航到位置再旋转）
                waypoint = Waypoint(
                    x=target.x,
                    y=target.y,
                    speed=self.tracker.cruise_speed,
                    goal_id=goal_id,
                    is_final=True
                )
                self.tracker.set_waypoint(waypoint)
                self._control_active = True
                self._current_goal_id = goal_id
                
                # 新任务开始，重置手动 HOLD 请求标志
                self._manual_hold_requested = False
                
                self._set_navigation_state(NavigationState.ACTIVE, "旋转机动目标")
                
                self.get_logger().info(
                    f'🔄 旋转机动目标 [ID={goal_id}]: '
                    f'位置=({target.x:.2f}, {target.y:.2f}), 圈数={maneuver_param:.1f}'
                )
            return
        
        # ==================== 常规导航处理 ====================
        # 如果之前在旋转，取消旋转
        if self._rotation_active:
            self._rotation_active = False
            self._rotation_initialized = False
        
        speed = self.tracker.cruise_speed
        is_new_goal = (goal_id != self._current_goal_id)
        
        # 根据 nav_mode 判断是否是最终航点
        # NAV_MODE_ASYNC (0) = 异步模式，可能还有后续航点，不是最终点
        # NAV_MODE_TERMINAL (3) 已在上面跳过，不会到这里
        # 默认情况下，由 navigate_to_point_node 统一管理到达判断
        # 这里设置 is_final=False，让 tracker 使用 switch_tolerance 而非 goal_tolerance
        is_final = (nav_mode == 3)  # NAV_MODE_TERMINAL
        
        waypoint = Waypoint(
            x=target.x,
            y=target.y,
            speed=speed,
            goal_id=goal_id,
            is_final=is_final
        )
        
        if is_new_goal:
            self._current_goal_id = goal_id
            self.tracker.set_waypoint(waypoint)
            self._control_active = True
            
            # 收到新目标，取消所有待执行的延迟操作 (减速 + 停止 + HOLD 切换)
            if self._soft_decel_active:
                self._soft_decel_active = False
                self.get_logger().debug(f'✓ 收到新目标 [ID={goal_id}], 已取消渐进减速')
            if self._delayed_stop_pending:
                self._delayed_stop_pending = False
                self.get_logger().debug(f'✓ 收到新目标 [ID={goal_id}], 已取消延迟软停止')
            if self._delayed_hold_pending:
                self._delayed_hold_pending = False
                self.get_logger().debug(f'✓ 收到新目标 [ID={goal_id}], 已取消延迟 HOLD 切换')
            
            # 新任务开始，清除手动暂停状态
            # 无论之前是否处于手动暂停，新任务都会开始执行
            if self._manual_hold_requested:
                self.get_logger().info(
                    f'📥 收到新任务 [ID={goal_id}]，清除手动暂停状态'
                )
            self._manual_hold_requested = False

            # 新目标到达，重置 ORCA 脱困状态
            if self._orca_escape_active:
                self._orca_escape_active = False
                self._orca_escape_phase = 0
                self.get_logger().info(f'✓ 收到新目标 [ID={goal_id}], 已取消ORCA脱困')
            self._orca_stall_start_time = 0.0
            self._orca_stall_best_dist = float('inf')
            self._orca_escape_count = 0
            
            # 新任务开始时，如果当前不是 GUIDED 模式，自动切换
            # 解决 USV 处于 MANUAL/HOLD 模式时收到任务立即被取消的问题
            if self.current_state and self.current_state.mode != 'GUIDED':
                self.get_logger().warn(
                    f'⚠️ 收到新任务但当前模式为 {self.current_state.mode}，自动切换到 GUIDED'
                )
                mode_msg = String()
                mode_msg.data = 'GUIDED'
                self.mode_pub.publish(mode_msg)
                # 记录切换请求时间，防止控制循环在切换生效前就取消导航
                self._guided_switch_request_time = time.time()
            
            self._set_navigation_state(NavigationState.ACTIVE, f"新导航目标[ID={goal_id}]")
            
            self.get_logger().info(
                f'🎯 新导航目标 [ID={goal_id}]: '
                f'({target.x:.2f}, {target.y:.2f}), 速度={speed:.2f} m/s'
            )
        else:
            # 相同目标ID的更新，也要更新 _current_goal_id（确保同步）
            self._current_goal_id = goal_id
            # 确保导航状态为 ACTIVE (从 PAUSED 恢复的情况)
            if self._navigation_state != NavigationState.ACTIVE:
                self._set_navigation_state(NavigationState.ACTIVE, "继续导航")
            # 使用 update_waypoint 原地更新目标位置，保留 MPC 状态
            # 这对编队跟随等高频更新场景至关重要：
            # 避免每次更新都重置 MPC / 角速度滤波器 / 路径历史
            self.tracker.update_waypoint(waypoint)
            self.get_logger().debug(
                f'📥 原地更新航点: ({target.x:.2f}, {target.y:.2f})'
            )
    
    def _avoidance_position_callback(self, msg: PositionTarget):
        """避障目标位置回调"""
        if self.control_mode != 'velocity':
            return
        
        new_pos = Pose2D(
            x=msg.position.x,
            y=msg.position.y,
            yaw=0.0
        )
        
        if new_pos.is_valid():
            old_pos = self._avoidance_position
            self._avoidance_position = new_pos
            
            # 只在位置变化时记录日志
            if old_pos is None or old_pos.distance_to(new_pos) > 0.1:
                self.get_logger().debug(
                    f'避障目标更新: ({new_pos.x:.2f}, {new_pos.y:.2f})'
                )
    
    def _avoidance_flag_callback(self, msg: Bool):
        """避障标志回调"""
        if self.control_mode != 'velocity':
            return
        
        old_state = self._avoidance_active
        self._avoidance_active = msg.data
        
        if old_state != msg.data:
            mode = "避障模式" if msg.data else "常规导航"
            self.get_logger().info(f'⚠️ 切换到: {mode}')
            
            if msg.data:
                # 进入避障模式，初始化停滞检测
                self._avoidance_best_dist = float('inf')
                self._avoidance_no_progress_start = 0.0
            else:
                # 退出避障模式，重置停滞检测和后退状态
                self._avoidance_best_dist = float('inf')
                self._avoidance_no_progress_start = 0.0
                self._retreat_active = False

    
    def _cruise_speed_callback(self, msg: Float32):
        """更新巡航速度"""
        if msg.data > 0:
            old_speed = self.tracker.cruise_speed
            self.tracker.set_cruise_speed(msg.data)
            self.get_logger().info(f'巡航速度更新: {old_speed:.2f} → {msg.data:.2f} m/s')
    
    def _goal_tolerance_callback(self, msg: Float32):
        """更新到达阈值"""
        if msg.data > 0:
            old_tol = self.tracker.goal_tolerance
            self.tracker.set_goal_tolerance(msg.data)
            self.get_logger().info(f'到达阈值更新: {old_tol:.2f} → {msg.data:.2f} m')
    
    def _switch_tolerance_callback(self, msg: Float32):
        """更新切换阈值"""
        if msg.data > 0:
            old_tol = self.tracker.switch_tolerance
            self.tracker.set_switch_tolerance(msg.data)
            self.get_logger().info(f'切换阈值更新: {old_tol:.2f} → {msg.data:.2f} m')
    
    def _max_angular_callback(self, msg: Float32):
        """更新最大角速度"""
        if msg.data > 0:
            old_val = self.tracker.max_angular_velocity
            self.tracker.max_angular_velocity = msg.data
            self.get_logger().info(f'最大角速度更新: {old_val:.2f} → {msg.data:.2f} rad/s')

    def _apf_neighbor_pose_callback(self, msg: PoseStamped, topic_name: str):
        """缓存邻船位姿（用于分布式 APF 本地计算）"""
        now_sec = time.time()
        self._apf_neighbor_states[topic_name] = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            0.0,
            0.0,
            now_sec,
        )

    def _apf_fleet_neighbors_callback(self, msg: FleetNeighborPoses):
        """处理 GS 统一下发的邻船位姿列表"""
        if msg.target_usv_id and msg.target_usv_id != self._usv_id:
            return

        now_sec = time.time()
        updated_states: Dict[str, Tuple[float, float, float, float, float]] = {}
        updated_ids = set()
        for neighbor in msg.neighbors:
            if not neighbor.usv_id or neighbor.usv_id == self._usv_id:
                continue

            # 忽略消息自带的时间戳，直接使用本地接收时间，避免分布式系统时钟不同步导致数据被丢弃
            stamp_sec = now_sec

            updated_states[neighbor.usv_id] = (
                float(neighbor.x),
                float(neighbor.y),
                float(neighbor.vx),
                float(neighbor.vy),
                stamp_sec,
            )
            updated_ids.add(str(neighbor.usv_id))
            self._update_neighbor_freeze_health(
                str(neighbor.usv_id),
                float(neighbor.x),
                float(neighbor.y),
                float(neighbor.vx),
                float(neighbor.vy),
                now_sec,
            )

        stale_neighbor_ids = [nid for nid in self._health_neighbor_freeze.keys() if nid not in updated_ids]
        for nid in stale_neighbor_ids:
            self._health_neighbor_freeze.pop(nid, None)

        self._apf_neighbor_states = updated_states

    def _update_own_freeze_health(self, pose: Pose2D, now_sec: float):
        if not self._apf_health_monitor_enabled:
            return

        if self._health_last_own_pose is None:
            self._health_last_own_pose = (pose.x, pose.y)
            self._health_own_freeze_start = 0.0
            return

        last_x, last_y = self._health_last_own_pose
        moved = math.hypot(pose.x - last_x, pose.y - last_y)
        freeze_eps = max(1e-4, self._apf_health_freeze_pos_epsilon)
        speed_threshold = max(0.0, self._apf_health_min_speed_for_freeze)

        if moved <= freeze_eps and self._current_speed >= speed_threshold:
            if self._health_own_freeze_start <= 0.0:
                self._health_own_freeze_start = now_sec
            freeze_duration = now_sec - self._health_own_freeze_start
            warn_duration = max(0.1, self._apf_health_own_freeze_warn_duration)
            warn_interval = max(0.2, self._apf_health_warn_interval)
            if freeze_duration >= warn_duration and (now_sec - self._health_own_last_warn) >= warn_interval:
                self._health_own_last_warn = now_sec
                # self.get_logger().warn(
                #     f'⚠️ 数据健康告警: 本船位姿冻结 {freeze_duration:.2f}s '
                #     f'(Δpos={moved:.3f}m, speed={self._current_speed:.2f}m/s)'
                # )
        else:
            self._health_own_freeze_start = 0.0

        self._health_last_own_pose = (pose.x, pose.y)

    def _update_neighbor_freeze_health(
        self,
        neighbor_id: str,
        x: float,
        y: float,
        vx: float,
        vy: float,
        now_sec: float,
    ):
        if not self._apf_health_monitor_enabled:
            return

        state = self._health_neighbor_freeze.get(neighbor_id)
        if state is None:
            self._health_neighbor_freeze[neighbor_id] = {
                'x': x,
                'y': y,
                'freeze_start': 0.0,
                'last_warn': 0.0,
            }
            return

        moved = math.hypot(x - float(state.get('x', x)), y - float(state.get('y', y)))
        speed = math.hypot(vx, vy)
        freeze_eps = max(1e-4, self._apf_health_freeze_pos_epsilon)
        speed_threshold = max(0.0, self._apf_health_min_speed_for_freeze)

        freeze_start = float(state.get('freeze_start', 0.0))
        if moved <= freeze_eps and speed >= speed_threshold:
            if freeze_start <= 0.0:
                freeze_start = now_sec
            freeze_duration = now_sec - freeze_start
            warn_duration = max(0.1, self._apf_health_neighbor_freeze_warn_duration)
            warn_interval = max(0.2, self._apf_health_warn_interval)
            last_warn = float(state.get('last_warn', 0.0))
            if freeze_duration >= warn_duration and (now_sec - last_warn) >= warn_interval:
                state['last_warn'] = now_sec
                # self.get_logger().warn(
                #     f'⚠️ 数据健康告警: 邻船[{neighbor_id}]位姿冻结 {freeze_duration:.2f}s '
                #     f'(Δpos={moved:.3f}m, speed={speed:.2f}m/s)'
                # )
        else:
            freeze_start = 0.0

        state['x'] = x
        state['y'] = y
        state['freeze_start'] = freeze_start
        self._health_neighbor_freeze[neighbor_id] = state

    def _get_health_freeze_summary(self, now_sec: float) -> Tuple[float, int]:
        own_freeze_duration = 0.0
        if self._health_own_freeze_start > 0.0:
            own_freeze_duration = max(0.0, now_sec - self._health_own_freeze_start)

        neighbor_freeze_count = 0
        for state in self._health_neighbor_freeze.values():
            freeze_start = float(state.get('freeze_start', 0.0))
            if freeze_start > 0.0 and (now_sec - freeze_start) >= max(0.1, self._apf_health_neighbor_freeze_warn_duration):
                neighbor_freeze_count += 1

        return own_freeze_duration, neighbor_freeze_count

    def _apply_apf_to_command(self, cmd: VelocityCommand) -> VelocityCommand:
        """
        将 APF 排斥修正叠加到导航速度指令。

        保持导航主算法为主，APF 仅作局部避碰修正。
        """
        if (not self._apf_enabled) or (self.current_pose is None):
            return cmd

        if self._apf_orca_enabled:
            return self._apply_orca_to_command(cmd)

        linear_correction, angular_correction, valid_neighbor_count = self._compute_apf_corrections()
        if valid_neighbor_count == 0:
            return cmd

        # 到点阶段削弱 APF，避免目标点附近不可达
        nav_speed_abs = abs(cmd.linear_x)
        if self._apf_goal_slow_speed_threshold > 1e-6:
            relax_scale = nav_speed_abs / self._apf_goal_slow_speed_threshold
            relax_scale = max(self._apf_goal_relax_min_scale, min(1.0, relax_scale))
        else:
            relax_scale = 1.0

        corrected_linear_x = cmd.linear_x + linear_correction * relax_scale
        corrected_angular_z = cmd.angular_z + angular_correction * relax_scale

        # 承诺生效期间强约束最终角速度方向，避免被基础导航项抵消
        if (
            self._apf_orca_enforce_commit_direction
            and commit_side != 0
            and closest_distance <= commit_distance
        ):
            enforce_min = max(0.0, self._apf_orca_enforce_min_angular)
            if corrected_angular_z * commit_side < 0.0:
                corrected_angular_z = commit_side * max(enforce_min, abs(corrected_angular_z))
            elif abs(corrected_angular_z) < enforce_min:
                corrected_angular_z = commit_side * enforce_min

        corrected = VelocityCommand(
            linear_x=corrected_linear_x,
            linear_y=cmd.linear_y,
            angular_z=corrected_angular_z,
        )
        return corrected.sanitize()

    def _compute_apf_corrections(self, distance_limit: Optional[float] = None) -> Tuple[float, float, int]:
        """计算 APF 在当前邻船场景下的线/角速度修正（不含到点 relax）。"""
        if (not self._apf_enabled) or (self.current_pose is None):
            return 0.0, 0.0, 0

        now_sec = time.time()
        fx_global_sum = 0.0
        fy_global_sum = 0.0
        valid_neighbor_count = 0
        active_distance = float(distance_limit) if distance_limit is not None else self._apf_safe_distance
        if active_distance <= 1e-3:
            return 0.0, 0.0, 0

        for _, (neighbor_x, neighbor_y, _, _, stamp_sec) in self._apf_neighbor_states.items():
            if now_sec - stamp_sec > self._apf_neighbor_timeout:
                continue

            dx = self.current_pose.x - neighbor_x
            dy = self.current_pose.y - neighbor_y
            distance = math.hypot(dx, dy)

            if distance < 1e-3 or distance >= active_distance:
                continue

            inv_distance = 1.0 / distance
            inv_safe = 1.0 / active_distance
            repulsion_force = self._apf_repulsion_gain * (inv_distance - inv_safe) * (inv_distance ** 2)

            ux = dx * inv_distance
            uy = dy * inv_distance
            fx_global_sum += repulsion_force * ux
            fy_global_sum += repulsion_force * uy
            valid_neighbor_count += 1

        if valid_neighbor_count == 0:
            return 0.0, 0.0, 0

        cos_yaw = math.cos(self.current_pose.yaw)
        sin_yaw = math.sin(self.current_pose.yaw)

        fx_body = cos_yaw * fx_global_sum + sin_yaw * fy_global_sum
        fy_body = -sin_yaw * fx_global_sum + cos_yaw * fy_global_sum

        linear_correction = max(
            -self._apf_max_linear_correction,
            min(self._apf_max_linear_correction, fx_body)
        )
        angular_correction = self._apf_lateral_to_yaw_gain * fy_body
        angular_correction = max(
            -self._apf_max_angular_correction,
            min(self._apf_max_angular_correction, angular_correction)
        )
        return linear_correction, angular_correction, valid_neighbor_count

    def _default_avoid_side(self) -> int:
        """返回默认避让方向: +1 左转, -1 右转。"""
        policy = self._apf_orca_commit_policy
        if policy in ('starboard', 'right'):
            return -1
        if policy in ('port', 'left'):
            return 1

        digits = ''.join(ch for ch in self._usv_id if ch.isdigit())
        if digits:
            return -1 if (int(digits) % 2 == 0) else 1
        score = sum(ord(ch) for ch in self._usv_id)
        return -1 if (score % 2 == 0) else 1

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _smooth_encounter_type(self, encounter_type: str, now_sec: float) -> str:
        if encounter_type == self._orca_encounter_type_smoothed:
            self._orca_encounter_candidate = None
            self._orca_encounter_candidate_since = 0.0
            return encounter_type

        if encounter_type != self._orca_encounter_candidate:
            self._orca_encounter_candidate = encounter_type
            self._orca_encounter_candidate_since = now_sec
            return self._orca_encounter_type_smoothed

        hold_time = max(0.0, self._apf_orca_encounter_hold_time)
        if now_sec - self._orca_encounter_candidate_since >= hold_time:
            self._orca_encounter_type_smoothed = encounter_type
            self._orca_encounter_candidate = None
            self._orca_encounter_candidate_since = 0.0
        return self._orca_encounter_type_smoothed

    def _compute_encounter_geometry(
        self,
        rx: float,
        ry: float,
        neighbor_vx: float,
        neighbor_vy: float,
        yaw: float,
    ) -> Tuple[float, float, float, float, float]:
        if self.current_pose is None:
            return -1.0, -1.0, -1.0, -1.0, -1.0

        own_to_neighbor_x = -rx
        own_to_neighbor_y = -ry
        rel_body_x = math.cos(yaw) * own_to_neighbor_x + math.sin(yaw) * own_to_neighbor_y
        rel_body_y = -math.sin(yaw) * own_to_neighbor_x + math.cos(yaw) * own_to_neighbor_y
        bearing = math.atan2(rel_body_y, rel_body_x)

        own_speed = math.hypot(self._current_vx, self._current_vy)
        nei_speed = math.hypot(neighbor_vx, neighbor_vy)
        own_course = math.atan2(self._current_vy, self._current_vx) if own_speed > 0.05 else yaw
        if nei_speed > 0.05:
            nei_course = math.atan2(neighbor_vy, neighbor_vx)
        else:
            nei_course = math.atan2(-ry, -rx)

        rel_course = self._wrap_angle(nei_course - own_course)
        rel_speed = math.hypot(self._current_vx - neighbor_vx, self._current_vy - neighbor_vy)

        rvx = self._current_vx - neighbor_vx
        rvy = self._current_vy - neighbor_vy
        v2 = rvx * rvx + rvy * rvy
        if v2 < 1e-6:
            tcpa = -1.0
            dcpa = math.hypot(rx, ry)
        else:
            tcpa_raw = - (rx * rvx + ry * rvy) / v2
            if tcpa_raw < 0.0:
                tcpa = -1.0
                dcpa = math.hypot(rx, ry)
            else:
                cpa_rx = rx + rvx * tcpa_raw
                cpa_ry = ry + rvy * tcpa_raw
                tcpa = tcpa_raw
                dcpa = math.hypot(cpa_rx, cpa_ry)

        return (
            math.degrees(bearing),
            math.degrees(rel_course),
            rel_speed,
            tcpa,
            dcpa,
        )

    def _classify_encounter_and_side(
        self,
        rx: float,
        ry: float,
        neighbor_vx: float,
        neighbor_vy: float,
        yaw: float,
    ) -> Tuple[str, int]:
        if self.current_pose is None:
            return 'unknown', 0

        own_to_neighbor_x = -rx
        own_to_neighbor_y = -ry
        rel_body_x = math.cos(yaw) * own_to_neighbor_x + math.sin(yaw) * own_to_neighbor_y
        rel_body_y = -math.sin(yaw) * own_to_neighbor_x + math.cos(yaw) * own_to_neighbor_y
        bearing = math.atan2(rel_body_y, rel_body_x)

        own_speed = math.hypot(self._current_vx, self._current_vy)
        nei_speed = math.hypot(neighbor_vx, neighbor_vy)

        # P5: COLREGs 仅适用于"在航"船舶 (IMO COLREGs Rule 3)
        # 静止/锚泊/停车船舶不构成 COLREGs 会遇，应作为静态障碍物处理
        stationary_threshold = max(0.01, self._apf_orca_stationary_speed_threshold)
        if nei_speed <= stationary_threshold:
            return 'stationary', 0

        own_course = math.atan2(self._current_vy, self._current_vx) if own_speed > 0.05 else yaw
        nei_course = math.atan2(neighbor_vy, neighbor_vx)
        rel_course = abs(self._wrap_angle(nei_course - own_course))

        head_on_rad = math.radians(max(5.0, self._apf_orca_encounter_head_on_deg))
        head_on_bearing_rad = math.radians(max(5.0, self._apf_orca_encounter_head_on_bearing_deg))
        head_on_course_rad = math.radians(max(120.0, self._apf_orca_encounter_head_on_course_deg))
        crossing_rad = math.radians(max(20.0, self._apf_orca_encounter_crossing_deg))
        overtaking_rad = math.radians(max(5.0, self._apf_orca_encounter_overtake_deg))

        is_ahead = abs(bearing) <= crossing_rad
        # 更严格 head_on 判据: 航向差接近 180° 且在前方扇区
        is_head_on = abs(bearing) <= head_on_bearing_rad and rel_course >= head_on_course_rad
        is_overtaking = abs(bearing) >= (math.pi - overtaking_rad)

        if is_head_on:
            # COLREGS Rule 14: 对遇 → 双方右转 (starboard = -1)
            return 'head_on', -1
        if is_overtaking:
            # COLREGS Rule 13: 追越 → 追越船让路，从右侧通过
            # 判断是否为追越船: 邻船在正前方附近 → 我方是追越船
            if abs(bearing) >= (math.pi - overtaking_rad):
                return 'overtaking', -1  # 我方追越 → 右转避让
            return 'being_overtaken', 0  # 被追越 → 保持航向

        if is_ahead:
            # P6: crossing 死区 — bearing 极接近 0°(正前方) 时, 微小偏航即翻转 give_way↔stand_on
            # 参考 Eriksen et al. (2019): 在 ±dead_zone 内按 head_on 保守处理 (Rule 14(c))
            # 以及 Tam & Bucknall (2010): crossing 起始角 = head_on 边界 = 15° (无间隙)
            dead_zone_rad = math.radians(max(0.0, self._apf_orca_crossing_dead_zone_deg))
            if abs(bearing) <= dead_zone_rad:
                # 正前方极窄扇区: 与 head_on 判据重叠, 按 head_on 保守处理
                return 'head_on', -1
            if bearing < 0.0:
                # COLREGS Rule 15: 交叉相遇，他船在我右舷 → 我方让路 → 右转
                return 'crossing_give_way', -1
            if bearing > 0.0:
                # COLREGS Rule 17: 交叉相遇，他船在我左舷 → 我方直航权 → 保持航向
                return 'crossing_stand_on', 0

        return 'other', 0

    def _compute_cpa_tcpa_risk(self) -> Tuple[Optional[float], Optional[float]]:
        """计算最危险邻船的 CPA/TCPA（预测碰撞风险）。"""
        if self.current_pose is None:
            return None, None

        now_sec = time.time()
        own_vx = self._current_vx
        own_vy = self._current_vy

        min_cpa: Optional[float] = None
        min_tcpa: Optional[float] = None
        tcpa_limit = max(0.1, self._apf_orca_tcpa_threshold)

        for _, (neighbor_x, neighbor_y, neighbor_vx, neighbor_vy, stamp_sec) in self._apf_neighbor_states.items():
            if now_sec - stamp_sec > self._apf_neighbor_timeout:
                continue

            rx = self.current_pose.x - neighbor_x
            ry = self.current_pose.y - neighbor_y
            rvx = own_vx - neighbor_vx
            rvy = own_vy - neighbor_vy

            v2 = rvx * rvx + rvy * rvy
            if v2 < 1e-6:
                cpa = math.hypot(rx, ry)
                tcpa = None
            else:
                tcpa_raw = - (rx * rvx + ry * rvy) / v2
                if tcpa_raw < 0.0:
                    # 远离趋势，不作为前向风险
                    continue
                if tcpa_raw > tcpa_limit:
                    continue
                cpa_rx = rx + rvx * tcpa_raw
                cpa_ry = ry + rvy * tcpa_raw
                cpa = math.hypot(cpa_rx, cpa_ry)
                tcpa = tcpa_raw

            if (min_cpa is None) or (cpa < min_cpa):
                min_cpa = cpa
                min_tcpa = tcpa

        return min_cpa, min_tcpa

    def _select_smart_avoid_side(
        self,
        cmd: VelocityCommand,
        yaw: float,
        rx: float,
        ry: float,
        neighbor_vx: float,
        neighbor_vy: float,
        encounter_side: int,
        min_sep: float,
    ) -> int:
        """根据短时预测分离效果 + 规则偏置，智能选择左/右避让方向。"""
        if not self._apf_orca_smart_side_enabled:
            return 0

        horizon = max(0.5, self._apf_orca_smart_side_horizon)
        yaw_rate_ref = max(0.05, self._apf_orca_smart_side_ref_yaw_rate)
        clearance_w = max(0.0, self._apf_orca_smart_side_clearance_weight)
        colreg_w = max(0.0, self._apf_orca_smart_side_colreg_weight)
        goal_w = max(0.0, self._apf_orca_smart_side_goal_weight)

        cmd_speed = max(0.0, abs(cmd.linear_x))
        own_vx = math.cos(yaw) * cmd_speed
        own_vy = math.sin(yaw) * cmd_speed

        best_side = 0
        best_score = -1e9
        second_score = -1e9
        for side in (-1, 1):
            yaw_future = yaw + side * yaw_rate_ref * horizon
            own_vx_future = math.cos(yaw_future) * cmd_speed
            own_vy_future = math.sin(yaw_future) * cmd_speed

            rvx = own_vx_future - neighbor_vx
            rvy = own_vy_future - neighbor_vy
            rx_future = rx + rvx * horizon
            ry_future = ry + rvy * horizon
            clearance = math.hypot(rx_future, ry_future)
            clearance_score = (clearance - min_sep) / max(0.2, min_sep)

            colreg_score = 1.0 if (encounter_side != 0 and side == encounter_side) else 0.0

            # 倾向减少对基础导航转向的反向对抗
            goal_conflict = 1.0 if (cmd.angular_z * side) < 0.0 else 0.0

            score = clearance_w * clearance_score + colreg_w * colreg_score - goal_w * goal_conflict

            if score > best_score:
                second_score = best_score
                best_score = score
                best_side = side
            elif score > second_score:
                second_score = score

        # 分数过于接近时保持中立，交给既有规则/承诺处理
        if (best_score - second_score) < 0.05:
            return 0
        return best_side

    def _apply_orca_to_command(self, cmd: VelocityCommand) -> VelocityCommand:
        """ORCA 风格速度障碍修正：多船场景下以最小速度改动实现分离。"""
        if self.current_pose is None:
            return cmd

        now_sec = time.time()
        yaw = self.current_pose.yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # 优选速度（导航器给出的速度）转到全局系
        v_pref_x = cos_yaw * cmd.linear_x - sin_yaw * cmd.linear_y
        v_pref_y = sin_yaw * cmd.linear_x + cos_yaw * cmd.linear_y
        v_new_x = v_pref_x
        v_new_y = v_pref_y

        horizon_base = max(0.5, self._apf_orca_time_horizon)
        min_sep = max(0.1, self._apf_orca_min_separation)
        influence_distance = max(min_sep, self._apf_orca_influence_distance)

        # P2: 迟滞 — ORCA 激活状态下使用扩展影响距离，防止快速抖动切换
        hysteresis_margin = max(0.0, self._apf_orca_hysteresis_margin)
        effective_influence = (influence_distance + hysteresis_margin) if self._orca_active else influence_distance

        has_risk_neighbor = False
        has_entry_neighbor = False           # 是否有邻船在标准 influence_distance 内
        closest_distance = float('inf')
        closest_rx = 0.0
        closest_ry = 0.0
        closest_neighbor_vx = 0.0
        closest_neighbor_vy = 0.0
        closest_neighbor_id = ''

        # 第 1 遍: 找到最近邻船距离 (用于距离自适应 horizon)
        for neighbor_id, (neighbor_x, neighbor_y, neighbor_vx, neighbor_vy, stamp_sec) in self._apf_neighbor_states.items():
            if now_sec - stamp_sec > self._apf_neighbor_timeout:
                continue
            rx = self.current_pose.x - neighbor_x
            ry = self.current_pose.y - neighbor_y
            distance = math.hypot(rx, ry)
            if distance < 1e-3 or distance > effective_influence:
                continue
            has_risk_neighbor = True
            if distance <= influence_distance:
                has_entry_neighbor = True
            if distance < closest_distance:
                closest_distance = distance
                closest_rx = rx
                closest_ry = ry
                closest_neighbor_vx = neighbor_vx
                closest_neighbor_vy = neighbor_vy
                closest_neighbor_id = str(neighbor_id)

        # 距离自适应时间地平线: 距离越近 → horizon 越短 → VO 修正越强
        # horizon_eff = clamp(distance * 1.0, 0.3, horizon_base)
        # 示例: d=4m→T=4.0(柔), d=2m→T=2.0(中), d=1m→T=1.0(强), d=0.5m→T=0.5(急), d=0.3m→T=0.3(极急)
        if closest_distance < float('inf'):
            horizon = max(0.3, min(horizon_base, closest_distance * 1.0))
        else:
            horizon = horizon_base
        radius = min_sep / horizon

        # 第 2 遍: 基于自适应 horizon 计算 VO 投影
        for _, (neighbor_x, neighbor_y, neighbor_vx, neighbor_vy, stamp_sec) in self._apf_neighbor_states.items():
            if now_sec - stamp_sec > self._apf_neighbor_timeout:
                continue
            rx = self.current_pose.x - neighbor_x
            ry = self.current_pose.y - neighbor_y
            distance = math.hypot(rx, ry)
            if distance < 1e-3 or distance > effective_influence:
                continue

            # 速度障碍圆盘近似中心（距离自适应时间地平线 T）
            center_x = neighbor_vx - rx / horizon
            center_y = neighbor_vy - ry / horizon

            dx = v_new_x - center_x
            dy = v_new_y - center_y
            norm = math.hypot(dx, dy)

            # 若优选速度落入禁区，投影到边界最近点（最小改动）
            if norm < radius:
                if norm < 1e-6:
                    # 完全重合时，沿相对位置方向推出禁区
                    inv_dist = 1.0 / max(distance, 1e-6)
                    ux = rx * inv_dist
                    uy = ry * inv_dist
                else:
                    ux = dx / norm
                    uy = dy / norm
                v_new_x = center_x + ux * radius
                v_new_y = center_y + uy * radius

        # 更新安全锁距离记录
        if closest_distance < float('inf'):
            self._orca_last_closest_distance = closest_distance

        # P2: ORCA 迟滞进入/退出逻辑
        if not self._orca_active:
            if has_entry_neighbor:
                # 首次进入: 邻船在标准 influence_distance 内 → 激活 ORCA
                self._orca_active = True
                self._orca_activate_time = now_sec
                self._orca_smooth_initialized = False  # 重置平滑器
            else:
                # 未激活且无邻船在标准范围内 → 不做 ORCA 修正
                return cmd
        else:
            # 已激活: 检查退出条件
            hold_elapsed = now_sec - self._orca_activate_time
            min_hold = max(0.0, self._apf_orca_min_hold_time)
            # 安全锁: 上次已知距离<3m时，禁止因数据超时而退出 ORCA
            safety_lock = self._orca_last_closest_distance < 3.0 * min_sep
            if not has_risk_neighbor and hold_elapsed >= min_hold and not safety_lock:
                # 所有邻船已超出扩展范围 且 保持时间已满 且 安全 → 退出 ORCA
                self._orca_active = False
                self._orca_activate_time = 0.0
                self._orca_committed_side = 0
                self._orca_close_enter_time = 0.0
                self._orca_coupling_enter_time = 0.0
                self._orca_coupling_active = False
                self._orca_last_closest_distance = float('inf')
                # P6: ORCA 退出时释放会遇锁定
                self._orca_encounter_locked_type = 'none'
                self._orca_encounter_locked_side = 0
                self._orca_encounter_lock_time = 0.0
                self._orca_encounter_lock_distance = 0.0
                # ORCA退出时重置停滞检测和脱困计数
                self._orca_stall_start_time = 0.0
                self._orca_stall_best_dist = float('inf')
                self._orca_escape_count = 0
                return cmd
            # 最小保持期内或仍有邻船在扩展范围 → 保持 ORCA 激活
            if not has_risk_neighbor:
                # 扩展范围外但保持期未满或安全锁激活 → 应用缓出修正
                if self._orca_committed_side != 0:
                    fadeout_angular = self._orca_committed_side * 0.05
                    corrected_hold = VelocityCommand(
                        linear_x=cmd.linear_x,
                        linear_y=cmd.linear_y,
                        angular_z=cmd.angular_z + fadeout_angular,
                    )
                    self.tracker.debug_info['active_controller'] = 'ORCA'
                    return corrected_hold.sanitize()
                return cmd

        # 将 ORCA 速度修正映射回船体系（线速度+角速度）
        v_body_x = cos_yaw * v_new_x + sin_yaw * v_new_y
        v_body_y = -sin_yaw * v_new_x + cos_yaw * v_new_y

        linear_correction = v_body_x - cmd.linear_x
        angular_correction = self._apf_lateral_to_yaw_gain * v_body_y

        encounter_type_raw = 'other'
        encounter_type = 'other'
        encounter_side = 0
        smart_side = 0
        is_stand_on = False                # P1: 标记直航权状态
        guidance_distance = max(0.1, self._apf_orca_encounter_guidance_distance)
        rel_bearing_deg = -1.0
        rel_course_deg = -1.0
        rel_speed = -1.0
        rel_tcpa = -1.0
        rel_dcpa = -1.0
        if closest_distance < float('inf'):
            rel_bearing_deg, rel_course_deg, rel_speed, rel_tcpa, rel_dcpa = self._compute_encounter_geometry(
                closest_rx,
                closest_ry,
                closest_neighbor_vx,
                closest_neighbor_vy,
                yaw,
            )

        if self._apf_orca_encounter_rule_enabled and closest_distance <= guidance_distance:
            encounter_type_raw, encounter_side = self._classify_encounter_and_side(
                closest_rx,
                closest_ry,
                closest_neighbor_vx,
                closest_neighbor_vy,
                yaw,
            )
            # P6: 会遇类型锁定 — 防止本船转向时 bearing 翻转导致 stand_on↔give_way 振荡
            # 参考 COLREGs: 同一次会遇中, 让路/直航权关系在首次评估后应保持稳定
            # 参考 Tam & Bucknall (2010): "The encounter type is determined at first detection
            #   and maintained until the encounter is resolved"
            # 参考 Eriksen et al. (2019): 使用首次 bearing 锁定 + 距离解锁策略
            if self._apf_orca_encounter_lock_enabled:
                lock_timeout = max(1.0, self._apf_orca_encounter_lock_timeout)
                release_ratio = max(1.1, self._apf_orca_encounter_lock_release_ratio)
                is_crossing_type = encounter_type_raw in ('crossing_give_way', 'crossing_stand_on')
                locked_is_crossing = self._orca_encounter_locked_type in ('crossing_give_way', 'crossing_stand_on')

                # 解锁条件: (1) 距离增大到初始距离 N 倍 (会遇已解除)
                #           (2) 锁定超时 (防止死锁)
                #           (3) 从 crossing 变成非 crossing 且距离增大 (已驶过)
                should_unlock = False
                if self._orca_encounter_locked_type != 'none':
                    elapsed = now_sec - self._orca_encounter_lock_time
                    dist_ratio = closest_distance / max(0.1, self._orca_encounter_lock_distance)
                    if elapsed > lock_timeout:
                        should_unlock = True
                    elif dist_ratio > release_ratio:
                        should_unlock = True
                    elif locked_is_crossing and not is_crossing_type and dist_ratio > 1.0:
                        should_unlock = True

                if should_unlock:
                    self._orca_encounter_locked_type = 'none'
                    self._orca_encounter_locked_side = 0
                    self._orca_encounter_lock_time = 0.0
                    self._orca_encounter_lock_distance = 0.0

                # 锁定逻辑: 首次检测到 crossing 类型时锁定
                if self._orca_encounter_locked_type == 'none' and is_crossing_type:
                    self._orca_encounter_locked_type = encounter_type_raw
                    self._orca_encounter_locked_side = encounter_side
                    self._orca_encounter_lock_time = now_sec
                    self._orca_encounter_lock_distance = closest_distance
                    self.get_logger().info(
                        f'🔒 P6 会遇锁定: {encounter_type_raw} side={encounter_side} '
                        f'd={closest_distance:.2f}m bearing={rel_bearing_deg:.1f}°'
                    )

                # 应用锁定: 如果当前判定是 crossing 类型, 但锁定了另一种 crossing, 使用锁定值
                if locked_is_crossing and is_crossing_type:
                    if encounter_type_raw != self._orca_encounter_locked_type:
                        self.get_logger().debug(
                            f'🔒 P6 锁定覆盖: {encounter_type_raw}→{self._orca_encounter_locked_type} '
                            f'(bearing翻转, 距离={closest_distance:.2f}m)'
                        )
                    encounter_type_raw = self._orca_encounter_locked_type
                    encounter_side = self._orca_encounter_locked_side

            encounter_type = self._smooth_encounter_type(encounter_type_raw, now_sec)
            # P1: COLREGS 强化 — 对遇时加强右转偏置
            if encounter_type == 'head_on':
                head_on_bias = max(0.0, self._apf_orca_colregs_head_on_bias)
                proximity = max(0.0, min(1.0, (guidance_distance - closest_distance) / guidance_distance))
                # 近距时加倍偏置: d<2m时额外增加最多2x
                close_boost = 1.0
                if closest_distance < 2.0 * min_sep:
                    close_boost = 1.0 + max(0.0, min(1.0, (2.0 * min_sep - closest_distance) / (2.0 * min_sep)))
                angular_correction += -1 * head_on_bias * (0.5 + 0.5 * proximity) * close_boost
            elif encounter_side != 0:
                proximity = max(0.0, min(1.0, (guidance_distance - closest_distance) / guidance_distance))
                angular_correction += encounter_side * max(0.0, self._apf_orca_encounter_bias_angular) * (0.4 + 0.6 * proximity)

            # P1: 直航权船标记— crossing_stand_on / being_overtaken 时减弱 ORCA 干预
            if encounter_type in ('crossing_stand_on', 'being_overtaken'):
                is_stand_on = True

        # P5: 静止船舶处理 — 不施加 COLREGs 方向强制，由 smart_side 选择最优方向
        is_stationary_encounter = (encounter_type == 'stationary')

        if closest_distance <= guidance_distance:
            smart_side = self._select_smart_avoid_side(
                cmd=cmd,
                yaw=yaw,
                rx=closest_rx,
                ry=closest_ry,
                neighbor_vx=closest_neighbor_vx,
                neighbor_vy=closest_neighbor_vy,
                encounter_side=encounter_side,
                min_sep=min_sep,
            )

        # 预测性早避让：在进入近距承诺前，根据 CPA/TCPA 提前进行轻微转向与减速
        early_commit_distance = max(0.1, self._apf_orca_side_commit_distance)
        if (
            self._apf_orca_predictive_early_enabled
            and closest_distance > early_commit_distance
            and closest_distance <= max(early_commit_distance, self._apf_orca_predictive_max_distance)
        ):
            rvx = v_pref_x - closest_neighbor_vx
            rvy = v_pref_y - closest_neighbor_vy
            v2 = rvx * rvx + rvy * rvy
            if v2 > 1e-6:
                tcpa_raw = - (closest_rx * rvx + closest_ry * rvy) / v2
                tcpa_limit = max(0.1, self._apf_orca_predictive_tcpa_threshold)
                if 0.0 <= tcpa_raw <= tcpa_limit:
                    cpa_rx = closest_rx + rvx * tcpa_raw
                    cpa_ry = closest_ry + rvy * tcpa_raw
                    cpa_pred = math.hypot(cpa_rx, cpa_ry)
                    cpa_limit = max(0.1, self._apf_orca_predictive_cpa_threshold)
                    if cpa_pred <= cpa_limit:
                        risk_tcpa = max(0.0, min(1.0, (tcpa_limit - tcpa_raw) / tcpa_limit))
                        risk_cpa = max(0.0, min(1.0, (cpa_limit - cpa_pred) / cpa_limit))
                        risk = max(risk_tcpa, risk_cpa)

                        if smart_side != 0:
                            early_side = smart_side
                        elif encounter_side != 0:
                            early_side = encounter_side
                        else:
                            early_side = self._default_avoid_side()

                        angular_max = max(0.0, self._apf_orca_predictive_angular_max)
                        speed_cut_max = max(0.0, self._apf_orca_predictive_speed_cut_max)
                        angular_correction += early_side * angular_max * risk
                        if cmd.linear_x >= 0.0:
                            linear_correction -= speed_cut_max * risk
                        else:
                            linear_correction += speed_cut_max * risk

        # 近距固定左右承诺 + 滞回，打破对称避让
        commit_distance = max(0.1, self._apf_orca_side_commit_distance)
        commit_side = 0
        side_changed = False
        if self._apf_orca_side_commit_enabled and closest_distance <= commit_distance:
            if self._orca_committed_side != 0 and now_sec < self._orca_commit_deadline:
                commit_side = self._orca_committed_side
                # P1: COLREGS 覆盖 — 当 COLREGS 有明确方向且与已承诺方向冲突时，强制修正
                if self._apf_orca_colregs_override and encounter_side != 0 and encounter_side != commit_side:
                    commit_side = encounter_side
                    self._orca_committed_side = encounter_side
                    self._orca_commit_deadline = now_sec + max(0.2, self._apf_orca_side_commit_hold_time)
            else:
                # P1: COLREGS 优先 → smart_side → 默认
                if self._apf_orca_colregs_override and encounter_side != 0:
                    commit_side = encounter_side
                elif smart_side != 0:
                    commit_side = smart_side
                elif encounter_side != 0:
                    commit_side = encounter_side
                else:
                    commit_side = self._default_avoid_side()
                side_changed = (commit_side != self._orca_committed_side)
                self._orca_committed_side = commit_side
                self._orca_commit_deadline = now_sec + max(0.2, self._apf_orca_side_commit_hold_time)

        # P3: 选边有效性判据已移除 — 分析显示自动换边会导致振荡加剧(omega方向翻转12-23次/分钟)
        # COLREGS 方向规则(P1)取代了选边有效性评估的职责

        if commit_side != 0:
            proximity = max(0.0, min(1.0, (commit_distance - closest_distance) / commit_distance))
            angular_correction += commit_side * self._apf_orca_side_bias_lateral * (0.5 + 0.5 * proximity)

        rel_body_x = cos_yaw * closest_rx + sin_yaw * closest_ry
        rel_body_y = -sin_yaw * closest_rx + cos_yaw * closest_ry
        closest_neighbor_speed = math.hypot(closest_neighbor_vx, closest_neighbor_vy)
        own_pref_speed = math.hypot(v_pref_x, v_pref_y)
        own_actual_speed = max(0.0, self._current_speed)
        rel_speed = math.hypot(v_pref_x - closest_neighbor_vx, v_pref_y - closest_neighbor_vy)
        stationary_coupling_context = False

        # 伴行耦合保护：长时间近距并行/推行时，强制解耦（减速+横向分离）
        if self._apf_orca_coupling_guard_enabled:
            coupling_distance = max(0.1, self._apf_orca_coupling_distance)
            release_distance = max(coupling_distance + 0.05, self._apf_orca_coupling_release_distance)
            lateral_limit = max(0.05, self._apf_orca_coupling_lateral_threshold)
            rel_speed_max = max(0.01, self._apf_orca_coupling_rel_speed_max)
            speed_min = max(0.0, self._apf_orca_coupling_speed_min)
            hold_time = max(0.2, self._apf_orca_coupling_hold_time)
            stationary_distance = max(coupling_distance, self._apf_orca_coupling_stationary_distance)
            stationary_speed_max = max(rel_speed_max, self._apf_orca_coupling_stationary_speed_max)

            coupling_candidate = (
                closest_distance <= coupling_distance
                and abs(rel_body_y) <= lateral_limit
                and own_pref_speed >= speed_min
                and closest_neighbor_speed >= speed_min
                and rel_speed <= rel_speed_max
            )
            stationary_coupling_context = (
                is_stationary_encounter
                and closest_distance <= stationary_distance
                and own_pref_speed >= 0.08
                and own_actual_speed <= stationary_speed_max
                and 0.01 <= closest_neighbor_speed <= stationary_speed_max
                and rel_speed <= stationary_speed_max
            )
            coupling_candidate = coupling_candidate or stationary_coupling_context
            candidate_hold_time = min(hold_time, 1.5) if stationary_coupling_context else hold_time

            if coupling_candidate:
                if self._orca_coupling_enter_time <= 0.0:
                    self._orca_coupling_enter_time = now_sec
                elif (now_sec - self._orca_coupling_enter_time) >= candidate_hold_time:
                    self._orca_coupling_active = True
            elif closest_distance >= release_distance:
                self._orca_coupling_enter_time = 0.0
                self._orca_coupling_active = False

            if self._orca_coupling_active:
                # P3: 简化解耦方向 — 优先使用 COLREGS/commit 方向
                if commit_side != 0:
                    decouple_side = commit_side
                elif abs(rel_body_y) > 0.03:
                    decouple_side = 1 if rel_body_y > 0.0 else -1
                else:
                    decouple_side = self._default_avoid_side()

                proximity = max(0.0, min(1.0, (release_distance - closest_distance) / max(0.05, release_distance)))
                turn_boost = max(0.0, self._apf_orca_coupling_turn_boost)
                lane_push = max(0.0, self._apf_orca_coupling_lane_push)
                if stationary_coupling_context:
                    turn_boost += 0.05
                    lane_push += 0.04
                angular_correction += decouple_side * (turn_boost + lane_push * (0.4 + 0.6 * proximity))

                speed_cut = max(0.0, self._apf_orca_coupling_speed_cut)
                if cmd.linear_x >= 0.0:
                    linear_correction -= speed_cut
                else:
                    linear_correction += speed_cut

                if commit_side == 0:
                    commit_side = decouple_side

        # 虚拟通行走廊：通过横向偏置将双方分到不同"车道"，减少轨迹重合
        lane_distance = max(0.1, self._apf_orca_lane_guidance_distance)
        if self._apf_orca_lane_guidance_enabled and commit_side != 0 and closest_distance <= lane_distance:
            # v17-fix2 方向守卫：仅当邻船不在 commit 侧时才施加车道修正
            #   commit_side=+1(左转避让) → 邻船应在右舷(rel_body_y>0) → 乘积>0 → 施加
            #   commit_side=-1(右转避让) → 邻船应在左舷(rel_body_y<0) → 乘积>0 → 施加
            #   若邻船已在 commit 侧(乘积<0)，车道修正会推向邻船，必须跳过
            if commit_side * rel_body_y >= 0:
                desired_neighbor_y = commit_side * max(0.0, self._apf_orca_lane_offset)
                lane_error = desired_neighbor_y - rel_body_y
                lane_proximity = max(0.0, min(1.0, (lane_distance - closest_distance) / lane_distance))
                angular_correction += self._apf_orca_lane_gain * lane_error * (0.4 + 0.6 * lane_proximity)


        # 双边分担：近距时保证双方都有最小转向份额，避免“单船硬躲”
        shared_turn_distance = max(0.1, self._apf_orca_shared_turn_distance)
        if self._apf_orca_shared_turn_enabled and closest_distance <= shared_turn_distance:
            share_side = commit_side if commit_side != 0 else (1 if angular_correction >= 0.0 else -1)
            share_min = max(0.0, self._apf_orca_shared_turn_min_angular)
            if abs(angular_correction) < share_min:
                angular_correction = share_side * share_min

        # ORCA 近距叠加 APF 兜底，避免最近点修正塌缩
        near_apf_distance = max(0.1, self._apf_orca_near_apf_distance)
        if self._apf_orca_near_apf_fallback_enabled and closest_distance <= near_apf_distance:
            apf_linear, apf_angular, apf_count = self._compute_apf_corrections(distance_limit=near_apf_distance)
            if apf_count > 0:
                weight = max(0.0, self._apf_orca_near_apf_weight)
                linear_correction += weight * apf_linear
                angular_correction += weight * apf_angular

        # CPA/TCPA 预测风险加权（让预测指标真正参与控制）
        if self._apf_orca_cpa_enabled:
            min_cpa, min_tcpa = self._compute_cpa_tcpa_risk()
            cpa_threshold = max(0.1, self._apf_orca_cpa_distance_threshold)
            tcpa_threshold = max(0.1, self._apf_orca_tcpa_threshold)
            if min_cpa is not None:
                cpa_risk = max(0.0, min(1.0, (cpa_threshold - min_cpa) / cpa_threshold))
                if min_tcpa is None:
                    tcpa_risk = 0.5 if min_cpa <= cpa_threshold else 0.0
                else:
                    tcpa_risk = max(0.0, min(1.0, (tcpa_threshold - min_tcpa) / tcpa_threshold))
                risk_level = max(cpa_risk, tcpa_risk)

                urgency_max = max(1.0, self._apf_orca_urgency_scale_max)
                urgency = 1.0 + (urgency_max - 1.0) * risk_level
                angular_correction *= urgency

                # 同时适度降低线速度，给分离腾时间
                slowdown = max(0.0, self._apf_orca_linear_slowdown_max) * risk_level
                if cmd.linear_x >= 0.0:
                    linear_correction -= slowdown
                else:
                    linear_correction += slowdown

        # 近距脱困：若长时间贴近不分离，增加转向并适度降速
        deadlock_distance = max(0.1, self._apf_orca_deadlock_distance)
        deadlock_hold = max(0.2, self._apf_orca_deadlock_hold_time)
        deadlock_active = False
        if self._apf_orca_deadlock_enabled and closest_distance <= deadlock_distance:
            if self._orca_close_enter_time <= 0.0:
                self._orca_close_enter_time = now_sec
            elif (now_sec - self._orca_close_enter_time) >= deadlock_hold:
                deadlock_active = True
        else:
            self._orca_close_enter_time = 0.0

        if deadlock_active:
            deadlock_side = commit_side if commit_side != 0 else (1 if angular_correction >= 0.0 else -1)
            angular_correction += deadlock_side * max(0.0, self._apf_orca_deadlock_turn_boost)
            speed_cut = max(0.0, self._apf_orca_deadlock_speed_reduction)
            if cmd.linear_x >= 0.0:
                linear_correction -= speed_cut
            else:
                linear_correction += speed_cut

        # 轨迹重合同向跟随防陷入：邻船在前且横向重合时优先减速让路，避免持续侧推
        if self._apf_orca_overlap_guard_enabled and closest_distance <= max(0.1, self._apf_orca_overlap_ahead_distance):
            is_neighbor_ahead = rel_body_x < 0.0
            near_same_lane = abs(rel_body_y) <= max(0.05, self._apf_orca_overlap_lateral_threshold)
            if is_neighbor_ahead and near_same_lane:
                speed_cut = max(0.0, self._apf_orca_overlap_speed_cut)
                if cmd.linear_x >= 0.0:
                    linear_correction -= speed_cut
                else:
                    linear_correction += speed_cut
                turn_scale = max(0.2, min(1.0, self._apf_orca_overlap_turn_scale))
                angular_correction *= turn_scale

        # 超近距设置最小转向下限，确保动作可见
        min_turn_distance = max(0.1, self._apf_orca_min_angular_distance)
        min_turn = max(0.0, self._apf_orca_min_angular_near)
        if closest_distance <= min_turn_distance and min_turn > 1e-6:
            side = commit_side if commit_side != 0 else (1 if angular_correction >= 0.0 else -1)
            if angular_correction * side <= 0.0 or abs(angular_correction) < min_turn:
                angular_correction = side * min_turn

        # 转向预算保护：当基础控制已接近角速度饱和，抑制同向避让增量，避免单船过激规避
        if self._apf_orca_turn_budget_enabled:
            max_turn = max(1e-3, float(self.tracker.max_angular_velocity))
            base_turn_abs = abs(cmd.angular_z)
            ratio = max(0.5, min(0.99, self._apf_orca_turn_budget_ratio))
            if base_turn_abs >= ratio * max_turn and (cmd.angular_z * angular_correction) > 0.0:
                near_sat = (base_turn_abs / max_turn - ratio) / max(1e-6, (1.0 - ratio))
                near_sat = max(0.0, min(1.0, near_sat))
                same_scale_min = max(0.0, min(1.0, self._apf_orca_turn_budget_same_side_scale))
                scale = 1.0 - (1.0 - same_scale_min) * near_sat
                angular_correction *= scale
                linear_correction *= max(0.4, scale)

        # P1: 直航权船修正缩减 — 减小 ORCA 对保持航向船的干预强度
        # 安全兜底: 当距离 < 2*min_sep 时恢复全力避让 (COLREGS Rule 17(b))
        if is_stand_on and self._apf_orca_colregs_override:
            standon_scale = max(0.0, min(1.0, self._apf_orca_colregs_standon_scale))
            emergency_dist = 2.0 * min_sep  # 紧急距离阈值
            if closest_distance < emergency_dist:
                # 距离过近 → 线性插值恢复到全力避让
                emergency_ratio = max(0.0, min(1.0, closest_distance / emergency_dist))
                standon_scale = standon_scale + (1.0 - standon_scale) * (1.0 - emergency_ratio)
            angular_correction *= standon_scale
            linear_correction *= standon_scale

        # P5: 静止船舶修正缩减 — 静止船不构成运动碰撞风险, 降低修正强度
        # v15 优化: 扩大紧急恢复距离到 2.5*min_sep，更早恢复全力避让
        # 数据分析: stationary类avg=1.58m, p10=0.57m, 34%触发hard_brake, min=0.41m
        # 原2.0*min_sep=3.0m太小，改为2.5*min_sep=3.75m提前介入
        if is_stationary_encounter:
            stationary_scale = max(0.0, min(1.0, self._apf_orca_stationary_correction_scale))
            emergency_dist = 2.5 * min_sep  # v15: 2.0→2.5, 提前恢复全力避让
            if closest_distance < emergency_dist:
                emergency_ratio = max(0.0, min(1.0, closest_distance / emergency_dist))
                stationary_scale = stationary_scale + (1.0 - stationary_scale) * (1.0 - emergency_ratio)
            angular_correction *= stationary_scale
            linear_correction *= stationary_scale

        # P4: NH-ORCA 非完整运动学约束投影
        # 将 ORCA 速度约束到 USV 实际可达集合 (前向速度 + 有限转向率)
        if self._apf_orca_nh_enabled:
            v_orca_speed = math.hypot(v_new_x, v_new_y)
            if v_orca_speed > 1e-6:
                v_orca_angle = math.atan2(v_new_y, v_new_x)
                angle_diff = self._wrap_angle(v_orca_angle - yaw)
                nh_horizon = max(0.5, self._apf_orca_nh_reachable_horizon)
                max_angle_change = self.tracker.max_angular_velocity * nh_horizon
                if abs(angle_diff) > max_angle_change:
                    # ORCA 建议方向超出 USV 可达航向弧 → 钳位到最近可达方向
                    clamped_angle = yaw + math.copysign(max_angle_change, angle_diff)
                    # 大偏角时降速，因为转向需要时间
                    speed_factor = max(0.3, math.cos(max_angle_change))
                    v_new_x_nh = v_orca_speed * speed_factor * math.cos(clamped_angle)
                    v_new_y_nh = v_orca_speed * speed_factor * math.sin(clamped_angle)
                    # 重新计算修正量
                    v_body_x_nh = cos_yaw * v_new_x_nh + sin_yaw * v_new_y_nh
                    v_body_y_nh = -sin_yaw * v_new_x_nh + cos_yaw * v_new_y_nh
                    linear_correction = v_body_x_nh - cmd.linear_x
                    angular_correction = self._apf_lateral_to_yaw_gain * v_body_y_nh

        # 紧急减速 (软): 距离 < 1.8*min_sep 时叠加减速修正
        # v15: 1.5→1.8*min_sep, 提前减速; 释放距离 2.0→2.5*min_sep
        # 数据分析: stationary avg=1.58m, hard_brake触发avg=0.78m, 说明soft brake介入太晚
        # 保留最低速度 0.10m/s，保证 USV 有足够前进速度来执行转向避让
        emergency_brake_dist = max(0.5, 1.8 * min_sep)  # v15: 2.7m (原1.5*1.5=2.25m)
        emergency_brake_release = max(0.8, 2.5 * min_sep)  # v15: 3.75m (原2.0*1.5=3.0m)
        if closest_distance < emergency_brake_dist:
            self._orca_soft_brake_active = True
        elif closest_distance > emergency_brake_release:
            self._orca_soft_brake_active = False
        # head_on 时双方都减速 (Rule 14); 其他情况 stand_on 免减速
        # v15: stationary不豁免减速 (数据显示stationary是最危险的near-miss类型)
        soft_brake_exempt = is_stand_on and encounter_type not in ('head_on', 'stationary')
        if self._orca_soft_brake_active and not soft_brake_exempt:
            # d=2.7m→keep=1.0, d=1.35m→keep=0.50, d=0.5m→keep=0.19
            speed_keep = max(0.05, closest_distance / emergency_brake_dist)
            target_speed = max(0.10, cmd.linear_x * speed_keep)  # v15: 最低保留 0.10m/s
            emergency_speed_cut = max(0.0, cmd.linear_x - target_speed)
            if cmd.linear_x >= 0.0:
                linear_correction -= emergency_speed_cut
            else:
                linear_correction += emergency_speed_cut

        # 近距强制最小转向: 距离 < 2*min_sep 时，确保有足够转向幅值
        # 不再要求 commit_side!=0，缺失时使用 encounter_side 或默认右转(COLREGS)
        # v15: 扩大最小转向触发距离到 2.5*min_sep，静止船和crossing_give_way增强
        min_turn_threshold = 2.5 * min_sep  # v15: 2.0→2.5
        if closest_distance < min_turn_threshold:
            effective_turn_side = commit_side if commit_side != 0 else (encounter_side if encounter_side != 0 else -1)
            # 距离越近，最小omega越大: d=3.75m→0.10, d=1.5m→0.22, d=0.5m→0.30
            dist_ratio = max(0.0, min(1.0, closest_distance / min_turn_threshold))
            min_omega_close = 0.10 + 0.12 * (1.0 - dist_ratio)  # v15: 0.10-0.22
            # v16: stationary和crossing_give_way近距时大幅提高最小转向
            # v15数据: 即使|ang|=0.45在d=0.5m时仍不足以逃脱，需要更强转向
            if encounter_type in ('stationary', 'crossing_give_way') and closest_distance < 1.5 * min_sep:
                min_omega_close = max(min_omega_close, 0.25 + 0.15 * (1.0 - dist_ratio))
            # v16: 极近距(< min_sep)时，所有encounter类型都强制高转向
            if closest_distance < min_sep:
                min_omega_close = max(min_omega_close, 0.30 + 0.10 * (1.0 - dist_ratio))
            if abs(angular_correction) < min_omega_close:
                angular_correction = effective_turn_side * min_omega_close

        linear_correction = max(
            -self._apf_max_linear_correction,
            min(self._apf_max_linear_correction, linear_correction)
        )
        angular_correction = max(
            -self._apf_max_angular_correction,
            min(self._apf_max_angular_correction, angular_correction)
        )

        # ORCA脱困 Phase2: 按比例削弱修正量，允许 MPC 驱动绕行
        escape_scale = self._get_orca_escape_suppression_scale()
        if escape_scale < 1.0:
            linear_correction *= escape_scale
            angular_correction *= escape_scale

        # 到点阶段削弱避让修正，保持终点可达
        nav_speed_abs = abs(cmd.linear_x)
        if self._apf_goal_slow_speed_threshold > 1e-6:
            relax_scale = nav_speed_abs / self._apf_goal_slow_speed_threshold
            relax_scale = max(self._apf_goal_relax_min_scale, min(1.0, relax_scale))
        else:
            relax_scale = 1.0

        # 目标优先: 仅在最终航点(is_final)时削弱ORCA修正，允许错时到达
        # 中间航点不削弱，避免路径交叉时丧失避碰能力
        # v16: 当邻船距离 < 安全阈值时，不再削弱ORCA修正，安全优先于到点
        # 数据分析: Goal 200009(is_final)时 goal_priority=0.10 导致实际角修正仅0.045rad/s
        # 这是 min_d=0.39m 的根因——ORCA输出被削弱90%
        goal_priority_scale = 1.0
        cur_wp = self.tracker.get_current_waypoint()
        if self.current_pose is not None and cur_wp is not None and cur_wp.is_final:
            dtg = self.tracker.get_distance_to_goal(self.current_pose)
            if dtg < 1.5:
                goal_priority_scale = 0.10
            elif dtg < 3.0:
                goal_priority_scale = 0.10 + 0.90 * (dtg - 1.5) / 1.5
        # v16: 安全兜底——当邻船距离 < 3.0m 时，逐步恢复ORCA修正强度
        # d=1.5m→scale≥1.0, d=2.25m→scale≥0.75, d=3.0m→scale≥0.50
        if self._orca_active and closest_distance < 3.0:
            safety_floor = max(0.50, min(1.0, closest_distance / 3.0))
            if closest_distance < 1.5 * min_sep:  # < 2.25m: 完全恢复
                safety_floor = 1.0
            goal_priority_scale = max(goal_priority_scale, safety_floor)
        relax_scale *= goal_priority_scale

        corrected = VelocityCommand(
            linear_x=cmd.linear_x + linear_correction * relax_scale,
            linear_y=cmd.linear_y,
            angular_z=cmd.angular_z + angular_correction * relax_scale,
        )

        # 紧急减速 (硬上限): d < 1.0*min_sep 时激活，真正碰撞风险
        # v16: 1.2→1.0*min_sep, 回退v15扩展(HB从1118→2381翻倍但安全距离未改善)
        # Rule 17(b): 极近距时所有船都必须硬刹，不论 stand_on
        coupling_hard_brake_relaxed = self._orca_coupling_active and stationary_coupling_context
        hard_brake_dist = max(0.3, 1.0 * min_sep)  # v16: 1.5m (回退到v14水平)
        hard_brake_release = max(0.5, 1.5 * min_sep)  # v16: 2.25m
        if coupling_hard_brake_relaxed:
            hard_brake_scale = max(0.3, min(1.0, self._apf_orca_coupling_hard_brake_scale))
            hard_brake_dist = max(0.3, hard_brake_dist * hard_brake_scale)
            hard_brake_release = max(hard_brake_dist + 0.2, hard_brake_release * max(hard_brake_scale, 0.7))
        if closest_distance < hard_brake_dist:
            self._orca_hard_brake_active = True
        elif closest_distance > hard_brake_release:
            self._orca_hard_brake_active = False
        # 硬刹不豁免 stand_on (d<1.0m 是真正碰撞风险)
        # 脱困 Phase2 期间提高硬刹最低速度，保证绕行动力
        if self._orca_hard_brake_active and cmd.linear_x > 0.0:
            # d=1.0m→keep=1.0, d=0.5m→keep=0.25, d=0.3m→keep=0.09
            hard_speed_keep = max(0.03, (closest_distance / hard_brake_dist) ** 2.0)
            hard_max_speed = max(0.02, cmd.linear_x * hard_speed_keep)  # v16: 0.05→0.02m/s, 允许近乎完全停止减少漂移
            if coupling_hard_brake_relaxed:
                hard_max_speed = max(hard_max_speed, self._apf_orca_coupling_speed_floor)
            # 脱困 Phase2: 提高最低速度下限，确保有足够动力绕行
            if escape_scale < 1.0:
                escape_min_speed = self.tracker.cruise_speed * 0.3
                hard_max_speed = max(hard_max_speed, escape_min_speed)
            if corrected.linear_x > hard_max_speed:
                corrected = VelocityCommand(
                    linear_x=hard_max_speed,
                    linear_y=corrected.linear_y,
                    angular_z=corrected.angular_z,
                )

        # --- 更新 ORCA 调试状态 (供 MpcDebug 发布) ---
        self._orca_debug_closest_distance = closest_distance
        self._orca_debug_encounter_type = encounter_type if isinstance(encounter_type, str) else 'none'
        self._orca_debug_encounter_type_raw = encounter_type_raw if isinstance(encounter_type_raw, str) else 'none'
        self._orca_debug_commit_side = commit_side
        self._orca_debug_linear_correction = linear_correction
        self._orca_debug_angular_correction = angular_correction
        self._orca_debug_rel_bearing_deg = rel_bearing_deg
        self._orca_debug_rel_course_deg = rel_course_deg
        self._orca_debug_rel_speed = rel_speed
        self._orca_debug_tcpa = rel_tcpa
        self._orca_debug_dcpa = rel_dcpa
        self._orca_debug_primary_neighbor_id = closest_neighbor_id if closest_distance < float('inf') else ''
        self._orca_debug_escape_active = self._orca_escape_active
        self._orca_debug_escape_phase = int(self._orca_escape_phase)
        self._orca_debug_escape_direction = int(self._orca_escape_direction if self._orca_escape_active else 0)
        self._orca_debug_escape_count = int(self._orca_escape_count)
        # 统计有效邻居数
        valid_count = 0
        for _, (_, _, _, _, stamp_sec) in self._apf_neighbor_states.items():
            if now_sec - stamp_sec <= self._apf_neighbor_timeout:
                valid_count += 1
        self._orca_debug_neighbor_count = valid_count

        # --- 记录 ORCA 触发日志 ---
        if not hasattr(self, '_last_orca_log_time'):
            self._last_orca_log_time = 0.0
        # 只要 ORCA 处于激活状态就标记，而非仅在修正幅度>0.01时
        if self._orca_active:
            self.tracker.debug_info['active_controller'] = 'ORCA'
        if abs(linear_correction) > 0.01 or abs(angular_correction) > 0.01:
            if now_sec - self._last_orca_log_time > 1.0:
                self._last_orca_log_time = now_sec
                self.get_logger().info(
                    f'🛡️ ORCA 避让触发: 距离={closest_distance:.2f}m, '
                    f'遭遇={encounter_type}, 选边={commit_side}, '
                    f'T_eff={horizon:.2f}s, VO_r={radius:.3f}, '
                    f'原指令(v={cmd.linear_x:.2f}, w={cmd.angular_z:.2f}), '
                    f'修正后(v={corrected.linear_x:.3f}, w={corrected.angular_z:.3f})'
                )

        # === 输出平滑: 速率限制器，防止急停急走/角速度振荡 ===
        result = corrected.sanitize()
        if not self._orca_smooth_initialized:
            # ORCA 刚激活时，以当前指令初始化平滑器
            self._orca_smooth_vx = result.linear_x
            self._orca_smooth_omega = result.angular_z
            self._orca_smooth_initialized = True
        else:
            # 速率限制: 每 tick (0.1s) 最大变化量
            max_dvx = 0.03    # 0.3 m/s²
            # v16: 硬刹时放宽角速度平滑约束，加快转向响应
            # v15数据: 常规0.08需5.6tick才达0.45rad/s，紧急时太慢
            max_domega = 0.12 if self._orca_hard_brake_active else 0.08  # v16: HB时1.2rad/s²
            # 线速度平滑
            dvx = result.linear_x - self._orca_smooth_vx
            if abs(dvx) > max_dvx:
                dvx = max_dvx if dvx > 0 else -max_dvx
            smoothed_vx = self._orca_smooth_vx + dvx
            # 角速度平滑
            domega = result.angular_z - self._orca_smooth_omega
            if abs(domega) > max_domega:
                domega = max_domega if domega > 0 else -max_domega
            smoothed_omega = self._orca_smooth_omega + domega
            # 更新状态并输出
            self._orca_smooth_vx = smoothed_vx
            self._orca_smooth_omega = smoothed_omega
            result = VelocityCommand(
                linear_x=smoothed_vx,
                linear_y=result.linear_y,
                angular_z=smoothed_omega,
            )
        return result.sanitize()
    
    # ==================== 控制循环 ====================
    
    def _control_loop(self):
        """
        主控制循环
        
        处理优先级: 避障 > 旋转机动 > 常规导航
        """
        # 仅在速度模式下运行
        if self.control_mode != 'velocity':
            return
        
        # 检查延迟停止和延迟 HOLD 切换
        self._check_delayed_stop()
        self._check_delayed_hold()
        
        # ==================== 渐进减速处理 ====================
        # 导航完成后平滑减速，优先于常规导航控制
        if self._soft_decel_active:
            self._handle_soft_deceleration()
            return
        
        # 检查前置条件
        if not self._check_preconditions():
            return
        
        # 确保 current_pose 不为 None
        if self.current_pose is None:
            return
        
        # ==================== 优先级 0: 后退动作 ====================
        if self._retreat_active:
            self._handle_retreat_control()
            return
        
        # ==================== 优先级 0.5: ORCA 脱困 ====================
        if self._orca_escape_active:
            self._handle_orca_escape()
            return
        
        # ==================== 优先级 1: 避障模式 ====================
        if self._avoidance_active and self._avoidance_position is not None:
            self._handle_avoidance_control()
            return
        
        # ==================== 优先级 2: 旋转机动 ====================
        if self._rotation_active:
            # 检查是否已到达旋转位置
            dist_to_goal = self.tracker.get_distance_to_goal(self.current_pose)
            rotation_start_threshold = 1.0  # 开始旋转的距离阈值
            
            if dist_to_goal <= rotation_start_threshold or self.tracker.is_goal_reached():
                # 已到达位置，开始/继续旋转
                self._handle_rotation_control()
                return
            # 否则继续导航到目标位置
        
        # ==================== 优先级 3: 常规导航 ====================
        # 注意：不在这里判断到达，由 navigate_to_point_node 通过 navigation_result 通知
        # 这样可以避免两个节点判断标准不一致导致的问题
        # tracker.is_goal_reached() 仅用于防止无目标时的空转
        if self.tracker.is_goal_reached() and not self._control_active:
            # 没有活跃目标，不需要控制
            return
        
        # 计算速度指令
        cmd = self.tracker.compute_velocity(self.current_pose)
        cmd = self._apply_apf_to_command(cmd)
        self._last_velocity_cmd = cmd
        
        # ORCA 停滞检测: 若长时间无进展则触发脱困
        if self._check_orca_stall():
            return  # 脱困已触发，跳过本次指令发布
        
        # 发布速度指令
        self._publish_velocity_command(cmd)
        
        # 发布导航反馈 (每 5 个周期一次，约 4Hz)
        if self._log_counter % 5 == 0:
            self._publish_navigation_feedback(cmd)
        
        # 定期日志
        self._log_counter += 1
        if self._log_counter % 40 == 0:  # 约 2 秒一次 (20Hz)
            dist = self.tracker.get_distance_to_goal(self.current_pose)
            queue_len = self.tracker.get_queue_length()
            
            self.get_logger().info(
                f'🚀 导航中: vx={cmd.linear_x:.2f} m/s, ω={cmd.angular_z:.2f} rad/s, '
                f'距离={dist:.2f}m, 队列={queue_len}'
            )
    
    def _handle_avoidance_control(self):
        """
        处理避障控制
        
        使用简单的方向追踪算法追踪避障目标点。
        同时监测是否长时间未靠近导航目标点，若停滞超过 60s 则触发后退动作。
        """
        if self._avoidance_position is None or self.current_pose is None:
            return
        
        # 计算到导航目标点的距离（用于停滞检测）
        dist_to_nav_goal = self.tracker.get_distance_to_goal(self.current_pose)
        current_time = time.time()
        
        # 停滞检测：检查是否在靠近导航目标点
        if dist_to_nav_goal < self._avoidance_best_dist - 0.5:
            # 有进展（距离显著减少），重置计时器
            self._avoidance_best_dist = dist_to_nav_goal
            self._avoidance_no_progress_start = current_time
        elif self._avoidance_no_progress_start == 0.0:
            # 首次进入避障，初始化计时器
            self._avoidance_best_dist = dist_to_nav_goal
            self._avoidance_no_progress_start = current_time
        else:
            # 检查是否超时
            no_progress_duration = current_time - self._avoidance_no_progress_start
            if no_progress_duration >= self._avoidance_stall_timeout:
                self.get_logger().warn(
                    f'⚠️ 避障停滞检测: {no_progress_duration:.0f}s 未靠近目标点 '
                    f'(最优距离={self._avoidance_best_dist:.2f}m, '
                    f'当前距离={dist_to_nav_goal:.2f}m), 触发后退动作'
                )
                self._retreat_active = True
                self._retreat_start_time = current_time
                # 重置停滞检测状态，后退结束后重新计时
                self._avoidance_best_dist = float('inf')
                self._avoidance_no_progress_start = 0.0
                return
        
        # 计算到避障点的距离
        dist = math.hypot(
            self._avoidance_position.x - self.current_pose.x,
            self._avoidance_position.y - self.current_pose.y
        )
        
        # 使用简单方向追踪计算速度指令
        # 直接朝向目标
        dx = self._avoidance_position.x - self.current_pose.x
        dy = self._avoidance_position.y - self.current_pose.y
        target_yaw = math.atan2(dy, dx)
        
        # 计算航向误差
        yaw_error = target_yaw - self.current_pose.yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # P 控制角速度
        angular_z = 2.0 * yaw_error  # 增益 2.0
        angular_z = max(-self.tracker.max_angular_velocity, 
                       min(self.tracker.max_angular_velocity, angular_z))
        
        # 根据航向误差调整线速度（误差大时减速）
        speed_factor = max(0.3, 1.0 - abs(yaw_error) / math.pi)
        linear_x = self.tracker.cruise_speed * speed_factor
        
        cmd = VelocityCommand(linear_x=linear_x, linear_y=0.0, angular_z=angular_z)
        cmd = self._apply_apf_to_command(cmd)
        self._last_velocity_cmd = cmd
        self._publish_velocity_command(cmd)
        
        # 定期日志
        self._log_counter += 1
        if self._log_counter % 40 == 0:
            no_progress_time = current_time - self._avoidance_no_progress_start if self._avoidance_no_progress_start > 0 else 0.0
            self.get_logger().info(
                f'⚠️ 避障中: vx={linear_x:.2f} m/s, ω={angular_z:.2f} rad/s, '
                f'避障距离={dist:.2f}m, 目标距离={dist_to_nav_goal:.2f}m, '
                f'无进展={no_progress_time:.0f}s/{self._avoidance_stall_timeout:.0f}s'
            )
    
    def _handle_retreat_control(self):
        """
        处理后退控制
        
        避障停滞时执行后退动作，持续 _retreat_duration 秒后自动恢复导航。
        """
        current_time = time.time()
        elapsed = current_time - self._retreat_start_time
        
        if elapsed >= self._retreat_duration:
            # 后退结束，恢复导航
            self._retreat_active = False
            self.get_logger().info(
                f'✅ 后退动作完成 ({self._retreat_duration:.0f}s)，恢复导航'
            )
            return
        
        # 后退: 发送负的线速度，不旋转
        retreat_speed = -self.tracker.cruise_speed * 0.5  # 后退速度为巡航速度的50%
        cmd = VelocityCommand(linear_x=retreat_speed, linear_y=0.0, angular_z=0.0)
        self._last_velocity_cmd = cmd
        self._publish_velocity_command(cmd)
        
        # 定期日志
        self._log_counter += 1
        if self._log_counter % 40 == 0:
            remaining = self._retreat_duration - elapsed
            self.get_logger().info(
                f'⏪ 后退中: vx={retreat_speed:.2f} m/s, 剩余={remaining:.1f}s'
            )
    
    # ==================== ORCA 脱困方法 ====================

    def _check_orca_stall(self) -> bool:
        """
        检测 ORCA 导致的长时间导航停滞。

        在正常导航路径中调用，检查 ORCA 激活期间到目标距离是否有实质进展。
        若超过 _orca_stall_timeout 秒无进展（距离未减少超过阈值），
        且位移低于阈值（排除缓慢前进的情况），则触发脱困。

        Returns:
            True 表示已触发脱困，调用方应跳过本次导航指令发布。
        """
        if not self._orca_active or self.current_pose is None:
            # ORCA 未激活，重置停滞检测
            self._orca_stall_start_time = 0.0
            self._orca_stall_best_dist = float('inf')
            self._orca_stall_start_pose = None
            return False

        dist_to_goal = self.tracker.get_distance_to_goal(self.current_pose)
        current_time = time.time()

        if self._orca_stall_start_time == 0.0:
            # 首次检测，初始化
            self._orca_stall_start_time = current_time
            self._orca_stall_best_dist = dist_to_goal
            self._orca_stall_start_pose = (self.current_pose.x, self.current_pose.y)
            return False

        # 有实质进展(距离减少超过阈值)，重置计时器
        if dist_to_goal < self._orca_stall_best_dist - self._orca_stall_progress_threshold:
            self._orca_stall_best_dist = dist_to_goal
            self._orca_stall_start_time = current_time
            self._orca_stall_start_pose = (self.current_pose.x, self.current_pose.y)
            return False

        stall_duration = current_time - self._orca_stall_start_time
        if stall_duration >= self._orca_stall_timeout:
            # 额外检查: 位移是否足够小（真正停滞 vs 缓慢前进）
            if self._orca_stall_start_pose is not None:
                sx, sy = self._orca_stall_start_pose
                displacement = math.hypot(
                    self.current_pose.x - sx, self.current_pose.y - sy
                )
                if displacement > self._orca_stall_displacement_threshold:
                    # USV 仍在移动（位移较大），只是进展缓慢，不触发脱困
                    self.get_logger().info(
                        f'⏳ ORCA停滞检测: {stall_duration:.0f}s无目标进展, '
                        f'但位移={displacement:.2f}m > {self._orca_stall_displacement_threshold}m, '
                        f'判定为缓慢前进, 继续监控'
                    )
                    # 重置计时器起点，继续监控
                    self._orca_stall_start_time = current_time
                    self._orca_stall_start_pose = (self.current_pose.x, self.current_pose.y)
                    return False

            self._trigger_orca_escape(stall_duration, dist_to_goal)
            return True

        return False

    def _trigger_orca_escape(self, stall_duration: float, dist_to_goal: float):
        """
        触发 ORCA 脱困机制。

        根据最近邻船位置计算脱困方向，激活脱困状态机。
        连续脱困次数越多，脱困动作越激进（更长的后退时间、更强的 ORCA 抑制）。
        """
        self._orca_escape_count += 1
        current_time = time.time()

        # 计算脱困方向: 远离最近邻船
        escape_dir = self._compute_orca_escape_direction()

        # 渐进升级: 每次脱困增加 Phase1 时长
        base_phase1 = 8.0
        escalation = min(self._orca_escape_count - 1, 4) * 2.0  # 每次+2s, 最多+8s
        self._orca_escape_phase1_duration = base_phase1 + escalation

        self._orca_escape_active = True
        self._orca_escape_start_time = current_time
        self._orca_escape_phase = 1
        self._orca_escape_direction = escape_dir

        # 重置停滞检测（脱困结束后重新开始计时）
        self._orca_stall_start_time = 0.0
        self._orca_stall_best_dist = float('inf')
        self._orca_stall_start_pose = None

        # 释放 ORCA 状态，防止脱困期间 ORCA 残留影响
        self._orca_committed_side = 0
        self._orca_close_enter_time = 0.0
        self._orca_hard_brake_active = False
        self._orca_soft_brake_active = False

        self.get_logger().warn(
            f'🆘 ORCA脱困触发 (第{self._orca_escape_count}次): '
            f'停滞{stall_duration:.0f}s, 目标距离={dist_to_goal:.1f}m, '
            f'方向={"左" if escape_dir > 0 else "右"}, '
            f'Phase1={self._orca_escape_phase1_duration:.0f}s'
        )

    def _compute_orca_escape_direction(self) -> int:
        """
        计算脱困转向方向: 选择远离最近邻船的方向。

        通过邻船在船体坐标系中的横向位置判断:
        - 邻船在左侧 → 右转 (-1)
        - 邻船在右侧 → 左转 (+1)
        - 无邻船信息 → 默认右转 (COLREGS 惯例)

        Returns:
            +1 表示左转, -1 表示右转。
        """
        if self.current_pose is None:
            return -1  # 默认右转 (COLREGS)

        now_sec = time.time()
        closest_dist = float('inf')
        closest_rel_body_y = 0.0

        yaw = self.current_pose.yaw
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for _, (nx, ny, _, _, stamp_sec) in self._apf_neighbor_states.items():
            if now_sec - stamp_sec > self._apf_neighbor_timeout:
                continue
            dx = nx - self.current_pose.x
            dy = ny - self.current_pose.y
            dist = math.hypot(dx, dy)
            if dist < closest_dist:
                closest_dist = dist
                # 邻船在船体坐标系中的 y 分量 (左正右负)
                closest_rel_body_y = -sin_yaw * dx + cos_yaw * dy

        if closest_dist == float('inf'):
            return -1  # 无邻船信息，默认右转

        # 向邻船反方向转向
        if closest_rel_body_y > 0.1:
            return -1  # 邻船在左边，向右转
        elif closest_rel_body_y < -0.1:
            return 1   # 邻船在右边，向左转
        else:
            return -1  # 邻船正前方，默认右转 (COLREGS)

    def _handle_orca_escape(self):
        """
        执行 ORCA 脱困动作。

        两阶段策略:
        - Phase 1 (后退+转向): 以后退速度 + 强转向远离邻船，增大分离距离。
          ORCA 完全抑制，避免 hard_brake 阻止后退。
        - Phase 2 (削弱绕行): 恢复正常导航，但 ORCA 修正量按比例削弱，
          允许 MPC 驱动 USV 实际绕过障碍区域而非被 ORCA 困住。
        """
        if self.current_pose is None:
            return

        current_time = time.time()
        elapsed = current_time - self._orca_escape_start_time

        if self._orca_escape_phase == 1:
            # Phase 1: 后退 + 强转向
            if elapsed >= self._orca_escape_phase1_duration:
                # Phase 1 结束，进入 Phase 2
                self._orca_escape_phase = 2
                self._orca_escape_start_time = current_time
                self._orca_smooth_initialized = False  # 重置 ORCA 平滑器
                self.get_logger().info(
                    f'🔄 ORCA脱困Phase1完成 ({self._orca_escape_phase1_duration:.0f}s), '
                    f'进入Phase2削弱绕行 ({self._orca_escape_phase2_duration:.0f}s)'
                )
                return

            # 后退速度 + 强转向（远离邻船方向）
            retreat_speed = -self.tracker.cruise_speed * 0.6
            turn_rate = self._orca_escape_direction * 0.5  # 0.5 rad/s
            cmd = VelocityCommand(
                linear_x=retreat_speed,
                linear_y=0.0,
                angular_z=turn_rate,
            )
            self._last_velocity_cmd = cmd
            self._publish_velocity_command(cmd)

            # 定期日志
            self._log_counter += 1
            if self._log_counter % 40 == 0:
                remaining = self._orca_escape_phase1_duration - elapsed
                self.get_logger().info(
                    f'🆘 ORCA脱困Phase1: 后退+转向, '
                    f'vx={retreat_speed:.2f}, ω={turn_rate:.2f}, '
                    f'剩余={remaining:.1f}s'
                )

        elif self._orca_escape_phase == 2:
            # Phase 2: 正常导航 + ORCA 削弱
            if elapsed >= self._orca_escape_phase2_duration:
                # Phase 2 结束，脱困完成
                self._orca_escape_active = False
                self._orca_escape_phase = 0
                self._orca_smooth_initialized = False
                self.get_logger().info(
                    f'✅ ORCA脱困完成 (第{self._orca_escape_count}次), 恢复正常导航'
                )
                return

            # 正常计算导航指令，ORCA 修正在 _apply_orca_to_command 中会自动削弱
            cmd = self.tracker.compute_velocity(self.current_pose)
            cmd = self._apply_apf_to_command(cmd)
            self._last_velocity_cmd = cmd
            self._publish_velocity_command(cmd)

            # 定期日志
            self._log_counter += 1
            if self._log_counter % 40 == 0:
                remaining = self._orca_escape_phase2_duration - elapsed
                dist = self.tracker.get_distance_to_goal(self.current_pose)
                self.get_logger().info(
                    f'🆘 ORCA脱困Phase2: 削弱绕行, '
                    f'vx={cmd.linear_x:.2f}, ω={cmd.angular_z:.2f}, '
                    f'目标距离={dist:.1f}m, 剩余={remaining:.1f}s'
                )

        else:
            # 异常状态，清除脱困标记
            self._orca_escape_active = False
            self._orca_escape_phase = 0

    def _get_orca_escape_suppression_scale(self) -> float:
        """
        获取 ORCA 脱困 Phase 2 期间的修正抑制系数。

        Returns:
            0.0~1.0 之间的缩放系数。
            1.0 = 正常修正, 0.0 = 完全抑制。
            不在脱困 Phase 2 期间返回 1.0。
        """
        if not self._orca_escape_active or self._orca_escape_phase != 2:
            return 1.0
        # 渐进升级: 连续脱困次数越多, ORCA 抑制越强
        # 第1次: 0.35, 第2次: 0.25, 第3次: 0.15, 第4次+: 0.10
        base_scale = 0.35
        reduction = min(self._orca_escape_count - 1, 3) * 0.08
        return max(0.10, base_scale - reduction)

    def _handle_rotation_control(self):
        """
        处理旋转机动控制
        
        原地旋转指定圈数
        """
        if self.current_pose is None:
            return
        
        current_yaw = self.current_pose.yaw
        
        # 初始化旋转
        if not self._rotation_initialized:
            self._rotation_last_yaw = current_yaw
            self._rotation_accumulated = 0.0
            self._rotation_initialized = True
            self.get_logger().info(f'🔄 开始旋转: 目标角度={math.degrees(self._rotation_target_yaw):.1f}°')
            return
        
        # 计算角度变化
        delta_yaw = current_yaw - self._rotation_last_yaw
        # 处理角度跳变 (-π ↔ π)
        if delta_yaw > math.pi:
            delta_yaw -= 2 * math.pi
        elif delta_yaw < -math.pi:
            delta_yaw += 2 * math.pi
        
        self._rotation_accumulated += delta_yaw
        self._rotation_last_yaw = current_yaw
        
        # 检查是否完成旋转
        rotation_done = False
        if self._rotation_target_yaw > 0:
            rotation_done = self._rotation_accumulated >= self._rotation_target_yaw
        else:
            rotation_done = self._rotation_accumulated <= self._rotation_target_yaw
        
        if rotation_done:
            # 旋转完成，发送停止指令
            self._publish_velocity_command(VelocityCommand.stop())
            self._rotation_active = False
            self._rotation_initialized = False
            self.get_logger().info(
                f'✅ 旋转完成: 累计={math.degrees(self._rotation_accumulated):.1f}°'
            )
            
            # 发布完成结果
            result = NavigationResult()
            result.goal_id = self._rotation_goal_id or 0
            result.success = True
            result.message = f'Rotation completed: {self._rotation_accumulated:.2f} rad'
            self.result_pub.publish(result)
            return
        
        # 继续旋转 - 发布 yaw_rate 指令
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        
        # 只使用 yaw_rate，忽略其他
        msg.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
        msg.yaw_rate = self._rotation_yaw_rate
        
        self.velocity_pub.publish(msg)
        
        # 定期日志
        self._log_counter += 1
        if self._log_counter % 20 == 0:
            progress = abs(self._rotation_accumulated / self._rotation_target_yaw) * 100 if self._rotation_target_yaw != 0 else 0
            self.get_logger().info(
                f'🔄 旋转中: {math.degrees(self._rotation_accumulated):.1f}°/'
                f'{math.degrees(self._rotation_target_yaw):.1f}° ({progress:.0f}%)'
            )
    
    def _check_preconditions(self) -> bool:
        """
        检查控制前置条件
        
        包含超时检测和连续异常处理
        """
        import time
        current_time = time.time()
        
        # ==================== 检查位姿 ====================
        if self.current_pose is None:
            self.get_logger().debug('等待位姿数据...')
            return False
        
        # 位姿超时检测
        if self._last_pose_time > 0:
            pose_age = current_time - self._last_pose_time
            if pose_age > self._pose_timeout:
                self._consecutive_timeout_count += 1
                if self._consecutive_timeout_count >= self._max_timeout_before_stop:
                    if not self._was_timed_out:
                        self.get_logger().warn(
                            f'位姿数据超时 {pose_age:.1f}s，连续 {self._consecutive_timeout_count} 次，停止导航'
                        )
                        self.stop_usv()
                        self._was_timed_out = True
                else:
                    self.get_logger().warn(f'位姿数据超时 {pose_age:.1f}s')
                return False
            else:
                # 位姿恢复正常，自动恢复导航
                if self._was_timed_out and self._recovery_enabled:
                    self.get_logger().info('✅ 位姿数据恢复，自动继续导航')
                    self._was_timed_out = False
                    self._consecutive_timeout_count = 0
        
        # ==================== 检查飞控状态 ====================
        if self.current_state is None:
            self.get_logger().debug('等待飞控状态...')
            return False
        
        # 状态超时检测
        if self._last_state_time > 0:
            state_age = current_time - self._last_state_time
            if state_age > self._state_timeout:
                self.get_logger().warn(f'飞控状态超时 {state_age:.1f}s')
                return False
        
        # ==================== 检查连接 ====================
        if not self.current_state.connected:
            self.get_logger().debug('飞控未连接...')
            return False
        
        # ==================== 检查解锁 ====================
        if self.require_armed and not self.current_state.armed:
            self.get_logger().debug('飞控未解锁...')
            return False
        
        # ==================== 检查模式 ====================
        # 已切换到 GUIDED，清除保护期标记
        if self.require_guided_mode and self.current_state.mode == 'GUIDED':
            if self._guided_switch_request_time > 0:
                self._guided_switch_request_time = 0.0
        
        if self.require_guided_mode and self.current_state.mode != 'GUIDED':
            current_mode = self.current_state.mode
            
            import time as _time
            now = _time.time()
            
            # ============================================================
            # GUIDED 模式最高优先级策略:
            # 导航任务 ACTIVE 状态下，任何非 GUIDED 模式都立即恢复
            # 包括 HOLD、MANUAL、LOITER、RTL 等所有模式
            # 只有经过连续HOLD强制暂停(PAUSED+manual_hold)才不恢复
            # ============================================================
            
            # 检查是否在 GUIDED 切换命令生效等待期
            in_guided_switch_grace = (
                self._guided_switch_request_time > 0 and
                (now - self._guided_switch_request_time) < self._guided_switch_grace_period
            )
            
            if in_guided_switch_grace:
                # 刚发送了 GUIDED 切换命令，等待生效
                self.get_logger().debug(
                    f'GUIDED 切换保护期内 ({now - self._guided_switch_request_time:.1f}s)，等待生效')
                return False
            
            # 检查是否是连续HOLD强制暂停状态
            is_force_paused = (
                self._manual_hold_requested and
                self._navigation_state == NavigationState.PAUSED
            )
            
            if is_force_paused:
                # 连续HOLD强制暂停中 - 不恢复 GUIDED，等待用户发新任务或手动GUIDED
                self.get_logger().debug(
                    f'强制暂停中 ({current_mode})，不自动恢复 GUIDED')
                return False
            
            # ===== 核心逻辑: 导航进行中 → 立即恢复 GUIDED =====
            if self._navigation_state == NavigationState.ACTIVE:
                # 导航进行中，任何非 GUIDED 模式都立即恢复
                # 不进入 PAUSED，不等待 cancel_navigation，不停止运动
                self.get_logger().warn(
                    f'⚡ 导航进行中检测到 {current_mode} 模式，立即恢复 GUIDED (零停顿策略)')
                self._restore_guided_mode()
                # 保持 ACTIVE 状态，不发送 stop 指令，无缝继续导航
                return False
            
            if self._navigation_state == NavigationState.PAUSED and not is_force_paused:
                # 非强制暂停的 PAUSED 状态(可能因之前的短暂模式切换遗留)
                # 恢复 GUIDED 并恢复为 ACTIVE
                self.get_logger().info(
                    f'⏩ PAUSED状态检测到 {current_mode}，恢复 GUIDED 继续导航')
                self._restore_guided_mode()
                self._set_navigation_state(NavigationState.ACTIVE, "自动恢复GUIDED模式")
                return False
            
            # IDLE, CANCELLED, COMPLETED, FAILED 状态不恢复
            self.get_logger().debug(f'需要 GUIDED 模式，当前: {current_mode} (状态: {self._navigation_state.name})')
            return False
        
        # 模式正确 (GUIDED)，检查是否应该恢复导航
        if self._navigation_state == NavigationState.PAUSED:
            # 用户明确切换到 GUIDED 模式，清除手动暂停状态并恢复导航
            # 这允许用户通过点击 GUIDED 按钮来恢复被暂停的任务
            if self._manual_hold_requested:
                self.get_logger().info(
                    '▶️ 用户切换到 GUIDED 模式，清除手动暂停状态，恢复导航'
                )
                self._manual_hold_requested = False
            
            self._set_navigation_state(NavigationState.ACTIVE, "GUIDED模式已恢复")
        
        return True
    
    def _publish_velocity_command(self, cmd: VelocityCommand):
        """
        发布速度指令到 MAVROS
        
        包含指令安全校验，确保不会发送无效值
        """
        # ==================== 指令校验 ====================
        # 确保指令有效 (已在 tracker 中 sanitize，这里再次确认)
        if not cmd.is_valid():
            self.get_logger().warn('检测到无效速度指令，使用停止指令')
            cmd = VelocityCommand.stop()
        
        # 限幅保护
        cmd = cmd.sanitize()
        
        # ==================== 构建消息 ====================
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        
        # 忽略位置和加速度，只使用速度
        msg.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW  # 使用 yaw_rate 而非 yaw
        )
        
        msg.velocity.x = cmd.linear_x
        msg.velocity.y = cmd.linear_y
        msg.velocity.z = 0.0
        msg.yaw_rate = cmd.angular_z
        
        self.velocity_pub.publish(msg)
        
        # 保存最后一次指令用于平滑处理
        self._last_velocity_cmd = cmd
        
        # --- 发布调试信息 ---
        try:
            debug_info = self.tracker.debug_info
            
            # 使用更健壮的获取方式，因为 tracker.debug_info 可能是部分更新的
            mpc_info = debug_info.get('mpc', {})
            
            debug_msg = MpcDebug()
            debug_msg.timestamp = self.get_clock().now().to_msg()
            
            debug_msg.solve_time_ms = float(mpc_info.get('solve_time_ms', 0.0))
            debug_msg.cost = float(mpc_info.get('cost', 0.0))
            debug_msg.mpc_pred_x = float(mpc_info.get('pred_x', 0.0))
            debug_msg.mpc_pred_y = float(mpc_info.get('pred_y', 0.0))
            debug_msg.mpc_pred_theta = float(mpc_info.get('pred_theta', 0.0))
            
            debug_msg.solver_status = int(mpc_info.get('status', -1))
            debug_msg.active_controller = str(debug_info.get('active_controller', 'none'))
            debug_msg.ref_curvature = 0.0 # 暂未实现
            
            # v5 新增: 一阶惯性模型状态
            debug_msg.omega_actual = float(mpc_info.get('omega_actual_est', 0.0))
            debug_msg.omega_cmd = float(mpc_info.get('pred_omega', 0.0))  # 使用预测的下一步角速度
            debug_msg.cross_track_error = float(mpc_info.get('cte', 0.0))
            debug_msg.path_theta = float(mpc_info.get('path_theta_deg', 0.0) * 3.14159 / 180.0)  # 转回弧度
            
            # 添加 MPC 参数信息，用于日志记录和调试
            debug_msg.param_q_pos = float(self._mpc_params['q_pos'])
            debug_msg.param_q_theta = float(self._mpc_params['q_theta'])
            debug_msg.param_r_w = float(self._mpc_params['r_w'])
            debug_msg.param_r_dw = float(self._mpc_params['r_dw'])
            debug_msg.param_w_max = float(self._mpc_params['w_max'])
            debug_msg.param_n_steps = int(self._mpc_params['n_steps'])
            # v5 新增参数
            debug_msg.param_tau_omega = float(self._mpc_params['tau_omega'])
            debug_msg.param_q_cte = float(self._mpc_params['q_cte'])
            
            # v6 新增: USV ID 和速度自适应 tau_omega 参数
            debug_msg.usv_id = self._usv_id
            debug_msg.adaptive_tau_enabled = self._adaptive_tau_enabled
            debug_msg.tau_omega_low_speed = float(self._tau_omega_low)
            debug_msg.tau_omega_high_speed = float(self._tau_omega_high)
            debug_msg.tau_speed_threshold_low = float(self._tau_speed_low)
            debug_msg.tau_speed_threshold_high = float(self._tau_speed_high)
            
            # v8: AMPC 自适应信息
            if self._ampc_enabled and hasattr(self.tracker, 'mpc_tracker'):
                ampc_info = mpc_info.get('ampc', {})
                # 使用 AMPC 在线估计的 tau 替代 v6 速度自适应的 tau
                debug_msg.current_tau_omega = float(ampc_info.get('tau_estimated', self._current_tau_omega))
                # 将 AMPC 特有信息写入已有字段
                debug_msg.ampc_enabled = True
                debug_msg.ampc_tau_estimated = float(ampc_info.get('tau_estimated', 0.0))
                debug_msg.ampc_tau_confidence = float(ampc_info.get('tau_confidence', 0.0))
                debug_msg.ampc_omega_measured = float(ampc_info.get('omega_measured', 0.0))
                debug_msg.ampc_saturation_ratio = float(ampc_info.get('saturation_ratio', 0.0))
                debug_msg.ampc_heading_noise = float(ampc_info.get('heading_noise_std', 0.0))
                debug_msg.ampc_rebuild_count = int(ampc_info.get('rebuild_count', 0))
                debug_msg.ampc_converged = bool(ampc_info.get('is_converged', False))
            else:
                debug_msg.current_tau_omega = float(self._current_tau_omega)
                debug_msg.ampc_enabled = False
            
            # v14 新增: ORCA/APF 避障状态
            debug_msg.orca_active = self._orca_active
            debug_msg.orca_closest_distance = self._orca_debug_closest_distance
            debug_msg.orca_encounter_type = self._orca_debug_encounter_type
            debug_msg.orca_encounter_type_raw = self._orca_debug_encounter_type_raw
            debug_msg.orca_commit_side = int(self._orca_debug_commit_side)
            debug_msg.orca_linear_correction = self._orca_debug_linear_correction
            debug_msg.orca_angular_correction = self._orca_debug_angular_correction
            debug_msg.orca_hard_brake_active = self._orca_hard_brake_active
            debug_msg.apf_neighbor_count = int(self._orca_debug_neighbor_count)
            debug_msg.orca_rel_bearing_deg = float(self._orca_debug_rel_bearing_deg)
            debug_msg.orca_rel_course_deg = float(self._orca_debug_rel_course_deg)
            debug_msg.orca_rel_speed = float(self._orca_debug_rel_speed)
            debug_msg.orca_tcpa = float(self._orca_debug_tcpa)
            debug_msg.orca_dcpa = float(self._orca_debug_dcpa)
            debug_msg.orca_primary_neighbor_id = self._orca_debug_primary_neighbor_id
            debug_msg.orca_escape_active = bool(self._orca_escape_active)
            debug_msg.orca_escape_phase = int(self._orca_escape_phase)
            debug_msg.orca_escape_direction = int(self._orca_escape_direction if self._orca_escape_active else 0)
            debug_msg.orca_escape_count = int(self._orca_escape_count)

            self.debug_pub.publish(debug_msg)
        except Exception as e:
            # 调试信息发布失败不影响主循环
            pass
    
    def _publish_navigation_feedback(self, cmd: VelocityCommand):
        """
        发布导航反馈
        
        提供实时导航状态，便于地面站或其他节点监控
        """
        if self.current_pose is None:
            return
        
        feedback = NavigationFeedback()
        feedback.goal_id = self._current_goal_id or 0
        feedback.distance_to_goal = self.tracker.get_distance_to_goal(self.current_pose)
        feedback.timestamp = self.get_clock().now().to_msg()
        
        # 航向误差 (弧度)
        wp = self.tracker.get_current_waypoint()
        if wp:
            dx = wp.x - self.current_pose.x
            dy = wp.y - self.current_pose.y
            target_yaw = math.atan2(dy, dx)
            heading_error = target_yaw - self.current_pose.yaw
            # 归一化到 [-π, π]
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
            feedback.heading_error = heading_error  # 弧度
        
        # 预计剩余时间
        if cmd.linear_x > 0.01:
            feedback.estimated_time = feedback.distance_to_goal / cmd.linear_x
        else:
            feedback.estimated_time = 0.0
        
        # 导航模式
        feedback.nav_mode = getattr(self, '_current_goal_nav_mode', 0)
        
        # 队列状态
        feedback.queue_length = self.tracker.get_queue_length()
        feedback.queue_capacity = 10  # 默认队列大小
        feedback.smooth_navigation = True  # 速度模式本身就是平滑导航
        
        self.feedback_pub.publish(feedback)
    
    def _on_goal_reached(self):
        """目标到达处理"""
        self._control_active = False
        
        # 发送停止指令
        self._publish_velocity_command(VelocityCommand.stop())
        
        self.get_logger().info(f'✅ 目标到达 [ID={self._current_goal_id}]')
        
        # 发布导航结果
        result = NavigationResult()
        result.goal_id = self._current_goal_id or 0
        result.success = True
        result.message = 'Goal reached'
        self.result_pub.publish(result)
    
    # ==================== 导航状态管理 ====================
    
    def _set_navigation_state(self, new_state: NavigationState, reason: str = ""):
        """
        设置导航状态并同步更新兼容性变量
        
        Args:
            new_state: 新的导航状态
            reason: 状态变更原因 (用于日志)
        """
        old_state = self._navigation_state
        if old_state == new_state:
            return  # 状态未变化
        
        self._navigation_state = new_state
        
        # 同步更新兼容性变量
        self._navigation_active = new_state == NavigationState.ACTIVE
        
        # 记录进入 PAUSED 状态的时间（用于保护期检查）
        if new_state == NavigationState.PAUSED:
            self._paused_state_enter_time = self.get_clock().now().nanoseconds / 1e9
        
        # 记录状态变化
        reason_str = f" ({reason})" if reason else ""
        self.get_logger().info(
            f'📊 导航状态: {old_state.name} → {new_state.name}{reason_str}'
        )
    
    def _is_navigation_resumable(self) -> bool:
        """
        检查当前导航状态是否可恢复
        
        只有 ACTIVE 或 PAUSED 状态才需要自动恢复 GUIDED 模式。
        CANCELLED, COMPLETED, FAILED 状态不应自动恢复。
        
        Returns:
            bool: 是否应该尝试恢复导航
        """
        return self._navigation_state in (NavigationState.ACTIVE, NavigationState.PAUSED)
    
    def _should_protect_navigation(self) -> bool:
        """
        检查是否应该保护导航（自动恢复 GUIDED）
        
        GUIDED 最高优先级策略:
        - ACTIVE 状态: 始终保护
        - PAUSED 且非强制暂停: 保护（尝试恢复）
        - PAUSED 且强制暂停(连续HOLD): 不保护（尊重用户意图）
        
        Returns:
            bool: 是否应该自动恢复 GUIDED 模式
        """
        if not self._mode_protection_enabled:
            return False
        
        # ACTIVE 状态始终保护
        if self._navigation_state == NavigationState.ACTIVE:
            return True
        
        # PAUSED 状态: 非强制暂停时保护
        if self._navigation_state == NavigationState.PAUSED:
            is_force_paused = self._manual_hold_requested
            return not is_force_paused
        
        return False
    
    def _end_navigation(self, end_state: NavigationState, reason: str = ""):
        """
        结束当前导航任务
        
        统一处理导航结束的所有清理工作。
        根据结束状态决定停止策略:
        - COMPLETED: 延迟软停止，新航点可能很快到达，避免不必要的停-起循环
        - CANCELLED/FAILED: 立即紧急停止
        
        Args:
            end_state: 结束状态 (COMPLETED, CANCELLED, FAILED)
            reason: 结束原因 (用于日志)
        """
        # 清除航点和控制状态
        self.tracker.clear_waypoints()
        self._current_goal_id = None
        self._control_active = False
        self._rotation_active = False
        self._rotation_goal_id = None
        
        # 更新导航状态
        self._set_navigation_state(end_state, reason)
        
        # 根据结束状态决定停止策略
        if end_state in (NavigationState.CANCELLED, NavigationState.FAILED):
            # 用户取消或异常: 立即紧急停止
            self.stop_usv()
        elif end_state == NavigationState.COMPLETED:
            # 正常完成: 启动渐进减速，平滑地将速度降至零
            # 替代之前的延迟急停方案，避免同步等待时的突然刹车
            # 如果新航点在减速期内到达，自动取消减速并恢复导航
            self._start_soft_deceleration()
    
    def _publish_status(self):
        """
        发布控制器状态
        
        提供详细的诊断信息，便于监控和调试
        """
        import time
        
        if self.control_mode != 'velocity':
            return
        
        status_parts = []
        current_time = time.time()
        
        # 位姿状态
        if self.current_pose:
            pose_age = current_time - self._last_pose_time if self._last_pose_time > 0 else 0
            if pose_age < self._pose_timeout:
                status_parts.append(f'pose:ok({pose_age:.1f}s)')
            else:
                status_parts.append(f'pose:stale({pose_age:.1f}s)')
        else:
            status_parts.append('pose:waiting')
        
        # 飞控状态
        if self.current_state:
            status_parts.append(f'mode:{self.current_state.mode}')
            armed_str = 'armed' if self.current_state.armed else 'disarmed'
            status_parts.append(armed_str)
        else:
            status_parts.append('fcu:waiting')
        
        # 导航状态
        if self._control_active:
            dist = self.tracker.get_distance_to_goal(self.current_pose) if self.current_pose else 0
            queue_len = self.tracker.get_queue_length()
            status_parts.append(f'nav:active,dist:{dist:.2f}m,queue:{queue_len}')
            
            # 速度信息
            if self._last_velocity_cmd:
                status_parts.append(
                    f'v:{self._last_velocity_cmd.linear_x:.2f}m/s,'
                    f'ω:{self._last_velocity_cmd.angular_z:.2f}rad/s'
                )
        else:
            status_parts.append('nav:idle')
        
        # 健康状态
        if self._consecutive_timeout_count > 0:
            status_parts.append(f'timeouts:{self._consecutive_timeout_count}')
        if self._apf_health_monitor_enabled:
            own_freeze_s, neighbor_freeze_n = self._get_health_freeze_summary(current_time)
            if own_freeze_s > 0.0:
                status_parts.append(f'freeze_own:{own_freeze_s:.1f}s')
            if neighbor_freeze_n > 0:
                status_parts.append(f'freeze_neighbor:{neighbor_freeze_n}')
        if self._orca_active:
            orca_hold = max(0.0, current_time - self._orca_activate_time)
            status_parts.append(f'orca_active:{orca_hold:.1f}s')
        
        status_msg = String()
        status_msg.data = ','.join(status_parts)
        self.status_pub.publish(status_msg)
    
    # ==================== 模式切换 ====================
    
    def _check_delayed_stop(self):
        """
        检查是否应执行延迟软停止 (作为渐进减速的备用机制)
        
        正常情况下渐进减速已在 _end_navigation(COMPLETED) 时启动，
        此方法仅在渐进减速未激活时作为安全兜底。
        """
        if not self._delayed_stop_pending:
            return
        
        import time
        if time.time() >= self._delayed_stop_deadline:
            self._delayed_stop_pending = False
            if self._navigation_state == NavigationState.COMPLETED:
                if not self._soft_decel_active:
                    # 渐进减速未激活（异常情况），启动减速作为兜底
                    self.get_logger().info('🛑 延迟软停止: 渐进减速未激活，启动兜底减速')
                    self._start_soft_deceleration()
            else:
                self.get_logger().debug(
                    f'延迟软停止取消: 当前状态={self._navigation_state.name}')
    
    def _check_delayed_hold(self):
        """
        检查是否应执行延迟 HOLD 切换
        
        在控制循环中调用，当延迟超时且仍处于 COMPLETED 状态时切换到 HOLD。
        如果在延迟期间收到了新目标，_delayed_hold_pending 已被清除，不会切换。
        """
        if not self._delayed_hold_pending:
            return
        
        import time
        if time.time() >= self._delayed_hold_deadline:
            self._delayed_hold_pending = False
            if self._navigation_state == NavigationState.COMPLETED:
                self.get_logger().info('🛑 延迟期满，无后续航点，自动切换到 HOLD 模式')
                self._switch_to_hold_mode()
            else:
                self.get_logger().debug(
                    f'延迟 HOLD 取消: 当前状态={self._navigation_state.name}')
    
    def _start_soft_deceleration(self):
        """
        启动渐进减速
        
        从当前速度线性减速到零，替代急刹车。
        在同步等待等场景下提供更平滑的停止体验。
        如果减速期间收到新目标，减速自动取消。
        """
        import time
        self._soft_decel_active = True
        self._soft_decel_start_time = time.time()
        # 使用最后的速度指令作为起始速度
        if self._last_velocity_cmd is not None:
            self._soft_decel_start_vx = self._last_velocity_cmd.linear_x
            self._soft_decel_start_omega = self._last_velocity_cmd.angular_z
        else:
            self._soft_decel_start_vx = 0.0
            self._soft_decel_start_omega = 0.0
        self.get_logger().info(
            f'🔽 开始渐进减速: vx={self._soft_decel_start_vx:.2f} m/s, '
            f'ω={self._soft_decel_start_omega:.2f} rad/s, '
            f'持续时间={self._soft_decel_duration:.1f}s')
    
    def _handle_soft_deceleration(self):
        """
        处理渐进减速控制
        
        在控制循环中调用，线性递减速度直到归零。
        减速完成后发送零速指令确保 USV 完全停止。
        """
        import time
        elapsed = time.time() - self._soft_decel_start_time
        
        if elapsed >= self._soft_decel_duration:
            # 减速完成，发送零速指令
            self._soft_decel_active = False
            self._publish_velocity_command(VelocityCommand.stop())
            self.get_logger().info('🔽 渐进减速完成，速度已归零')
            return
        
        # 线性减速: ratio 从 1.0 线性递减到 0.0
        ratio = 1.0 - (elapsed / self._soft_decel_duration)
        vx = self._soft_decel_start_vx * ratio
        omega = self._soft_decel_start_omega * ratio
        cmd = VelocityCommand(vx, 0.0, omega)
        self._publish_velocity_command(cmd)
    
    def _switch_to_hold_mode(self):
        """
        导航任务完成后切换到 HOLD 模式
        """
        if self.current_state is None:
            return
        
        if self.current_state.mode == 'HOLD':
            return  # 已经是 HOLD 模式
        
        self.get_logger().info('🛑 导航完成，自动切换到 HOLD 模式')
        mode_msg = String()
        mode_msg.data = 'HOLD'
        self.mode_pub.publish(mode_msg)
    
    def _restore_guided_mode(self):
        """
        导航进行中检测到非 GUIDED 模式时，立即恢复到 GUIDED 模式
        
        零停顿策略: 以最短冷却时间(0.2s)发送 GUIDED 切换命令,
        并设置切换保护期避免在 GUIDED 生效前重复发送。
        """
        import time
        current_time = time.time()
        
        # 冷却时间检查，避免频繁切换 (0.2s 极短冷却)
        if current_time - self._last_mode_restore_time < self._mode_restore_cooldown:
            return
        
        self._last_mode_restore_time = current_time
        # 设置切换保护期，避免在 GUIDED 生效前重复触发
        self._guided_switch_request_time = current_time
        
        current_mode = self.current_state.mode if self.current_state else 'UNKNOWN'
        self.get_logger().warn(
            f'⚡ 立即恢复 GUIDED 模式 (当前: {current_mode})'
        )
        mode_msg = String()
        mode_msg.data = 'GUIDED'
        self.mode_pub.publish(mode_msg)
    
    # ==================== 安全关闭 ====================
    
    def stop_usv(self):
        """紧急停止 USV"""
        self.get_logger().warn('发送紧急停止指令')
        self._publish_velocity_command(VelocityCommand.stop())
        self.tracker.clear_waypoints()
        self._control_active = False
    
    def destroy_node(self):
        """节点销毁时确保停止"""
        self.stop_usv()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = VelocityControllerNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_usv()  # 确保退出时停止
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
