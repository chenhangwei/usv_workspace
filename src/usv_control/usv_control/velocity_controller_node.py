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

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import String, Float32, Bool
from common_interfaces.msg import NavigationGoal, NavigationFeedback, NavigationResult, MpcDebug

import math
from typing import Optional
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
        
        # 回调组
        self.callback_group = ReentrantCallbackGroup()
        
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
        self.declare_parameter('mpc_tau_omega', 0.4)              # 转向时间常数 (秒)
        self.declare_parameter('mpc_weight_cte', 15.0)            # Cross Track Error 权重
        
        # v6 新增: 速度自适应 tau_omega 参数 (解决低速S形振荡)
        self.declare_parameter('adaptive_tau_enabled', True)      # 是否启用速度自适应 tau_omega
        self.declare_parameter('tau_omega_low_speed', 0.8)        # 低速时的 tau_omega (秒)
        self.declare_parameter('tau_omega_high_speed', 0.4)       # 高速时的 tau_omega (秒)
        self.declare_parameter('tau_speed_threshold_low', 0.15)   # 低速阈值 (m/s)
        self.declare_parameter('tau_speed_threshold_high', 0.35)  # 高速阈值 (m/s)
        
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
        
        # ==================== 获取参数 ====================
        self.control_mode = str(self.get_parameter('control_mode').value or 'velocity')
        self.get_logger().info(f'🔍 正在初始化速度控制器... 模式: MPC')
        
        # 控制器类型固定为 MPC
        controller_type = ControllerType.MPC
        
        self.require_guided_mode = bool(self.get_parameter('require_guided_mode').value)
        self.require_armed = bool(self.get_parameter('require_armed').value)
        
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
                
                min_speed=float(self.get_parameter('min_speed').value or 0.05),
                goal_tolerance=float(self.get_parameter('goal_tolerance').value or 0.5),
                switch_tolerance=float(self.get_parameter('switch_tolerance').value or 1.5),
                angular_velocity_filter=float(self.get_parameter('angular_velocity_filter').value or 0.3),
            )
            self.get_logger().info('✅ VelocityPathTracker 初始化成功')
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
        self._pose_timeout: float = 2.0   # 位姿超时 (秒)
        self._state_timeout: float = 3.0  # 飞控状态超时 (秒)
        self._consecutive_timeout_count: int = 0
        self._max_timeout_before_stop: int = 5  # 连续超时次数阈值
        
        # ==================== 模式保护 ====================
        self._mode_protection_enabled: bool = True  # 导航中自动恢复 GUIDED 模式
        self._last_mode_restore_time: float = 0.0   # 上次恢复模式的时间
        self._mode_restore_cooldown: float = 2.0    # 恢复模式冷却时间 (秒)
        
        # 手动HOLD/取消请求标志 - 用于区分手动切换和飞控自动切换
        # 当收到 cancel_navigation 消息时设置为 True
        # 防止模式检测在 cancel_navigation 消息到达前就尝试恢复 GUIDED
        self._manual_hold_requested: bool = False
        self._manual_hold_request_time: float = 0.0  # 请求时间戳
        self._manual_hold_timeout: float = 3600.0    # 手动请求有效期 (1小时，实际由新任务清除)
        
        # PAUSED 状态保护 - 刚进入暂停状态时不立即尝试恢复
        self._paused_state_enter_time: float = 0.0   # 进入 PAUSED 状态的时间
        self._paused_state_grace_period: float = 5.0  # 暂停状态保护期 (秒)，等待cancel_navigation消息
        
        # 导航状态管理 (使用枚举替代简单布尔值)
        self._navigation_state: NavigationState = NavigationState.IDLE
        self._navigation_active: bool = False       # 兼容性：是否有活跃的导航任务
        
        self._last_valid_pose: Optional[Pose2D] = None  # 用于跳变检测
        self._pose_jump_threshold: float = 3.0  # 位姿跳变阈值 (m) - 降低以检测小幅漂移
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
            if self._use_velocity_heading:
                self.get_logger().info('  航向估计: L1风格（飞控EKF速度向量优先，低速回退磁力计）')
            else:
                self.get_logger().info('  航向估计: 磁力计')
        else:
            self.get_logger().info('  功能: 待机 (由 usv_control_node 处理)')
        self.get_logger().info('  控制器类型: MPC')
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
        # 低速时舵效差，转向动力学变化，需要更大的 tau_omega
        if self._adaptive_tau_enabled:
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
                
                # 导航完成后自动切换到 HOLD 模式
                self._switch_to_hold_mode()
                
            elif self._current_goal_id is not None and goal_id == self._current_goal_id:
                # 兼容旧逻辑：ID 匹配也停止
                self.get_logger().info(
                    f'✅ 收到到达通知 [ID={goal_id}], 停止追踪')
                
                self._end_navigation(NavigationState.COMPLETED, "成功到达目标")
                self._switch_to_hold_mode()
            else:
                self.get_logger().debug(
                    f'收到到达通知 [ID={goal_id}], message="{message}", 忽略'
                )
    
    def _cancel_navigation_callback(self, msg: Bool):
        """
        取消导航回调 - 来自地面站的强制取消请求
        
        当用户在地面站点击 HOLD 或 MANUAL 按钮时，会发送此消息。
        导航任务进入 PAUSED 状态（暂停），不会自动恢复 GUIDED。
        
        状态转换: ANY → PAUSED (手动暂停)
        - 手动暂停状态不会触发自动恢复 GUIDED
        - 设置 _manual_hold_requested 标志，区分手动暂停和飞控自动切换
        - 当用户发送新导航任务时，任务会自动恢复
        """
        if not msg.data:
            return
        
        self.get_logger().warn('🛑 收到暂停导航请求（来自地面站），任务进入暂停状态')
        
        # 设置手动HOLD请求标志 - 防止模式检测在状态更新前就尝试恢复 GUIDED
        # 这解决了时序竞争问题：地面站同时发送 cancel_navigation 和 set_mode HOLD，
        # 但模式检测可能在 cancel_navigation 消息处理前就检测到 HOLD 模式
        self._manual_hold_requested = True
        self._manual_hold_request_time = self.get_clock().now().nanoseconds / 1e9
        
        # 无论当前状态如何，都设置为 PAUSED 状态
        # 即使已经是 PAUSED（由模式检测触发），也要确保是"手动暂停"
        if self._navigation_state in (NavigationState.ACTIVE, NavigationState.PAUSED):
            self._set_navigation_state(NavigationState.PAUSED, "用户手动暂停")
            # 停止当前运动但保留航点信息
            self._publish_velocity_command(VelocityCommand.stop())
        
        self.get_logger().info(
            f'✅ 导航任务已暂停，手动暂停标志已设置 (manual_hold_requested={self._manual_hold_requested})'
        )
    
    def _stop_navigation_callback(self, msg: Bool):
        """
        停止导航回调 - 来自地面站集群 STOP 按钮
        
        完全停止导航任务，清空所有目标。
        需要发送新任务才能重新开始导航。
        
        状态转换: ANY → CANCELLED (完全停止)
        """
        if not msg.data:
            return
        
        self.get_logger().warn('🛑 收到停止导航请求（集群STOP），任务完全停止')
        
        # 设置手动HOLD请求标志
        self._manual_hold_requested = True
        self._manual_hold_request_time = self.get_clock().now().nanoseconds / 1e9
        
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
        if nav_mode == NAV_MODE_TERMINAL:
            self.get_logger().info(
                f'⏭️ 离群目标 [ID={goal_id}] ({target.x:.2f}, {target.y:.2f}) '
                f'使用位置模式处理 (NAV_MODE_TERMINAL)'
            )
            return
        
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
            
            # 新任务开始，清除手动暂停状态
            # 无论之前是否处于手动暂停，新任务都会开始执行
            if self._manual_hold_requested:
                self.get_logger().info(
                    f'📥 收到新任务 [ID={goal_id}]，清除手动暂停状态'
                )
            self._manual_hold_requested = False
            
            # 新任务开始时，如果当前不是 GUIDED 模式，自动切换
            # 解决 USV 处于 MANUAL/HOLD 模式时收到任务立即被取消的问题
            if self.current_state and self.current_state.mode != 'GUIDED':
                self.get_logger().warn(
                    f'⚠️ 收到新任务但当前模式为 {self.current_state.mode}，自动切换到 GUIDED'
                )
                mode_msg = String()
                mode_msg.data = 'GUIDED'
                self.mode_pub.publish(mode_msg)
            
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
            self.tracker.add_waypoint(waypoint)
            self.get_logger().debug(
                f'📥 添加航点到队列: ({target.x:.2f}, {target.y:.2f})'
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
    
    # ==================== 控制循环 ====================
    
    def _control_loop(self):
        """
        主控制循环
        
        处理优先级: 避障 > 旋转机动 > 常规导航
        """
        # 仅在速度模式下运行
        if self.control_mode != 'velocity':
            return
        
        # 检查前置条件
        if not self._check_preconditions():
            return
        
        # 确保 current_pose 不为 None
        if self.current_pose is None:
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
        self._last_velocity_cmd = cmd
        
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
        
        使用简单的方向追踪算法追踪避障目标点
        """
        if self._avoidance_position is None or self.current_pose is None:
            return
        
        # 创建临时航点追踪避障目标
        avoidance_waypoint = Waypoint(
            x=self._avoidance_position.x,
            y=self._avoidance_position.y,
            speed=self.tracker.cruise_speed,
            goal_id=0,
            is_final=True
        )
        
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
        self._last_velocity_cmd = cmd
        self._publish_velocity_command(cmd)
        
        # 定期日志
        self._log_counter += 1
        if self._log_counter % 40 == 0:
            self.get_logger().info(
                f'⚠️ 避障中: vx={linear_x:.2f} m/s, ω={angular_z:.2f} rad/s, 距离={dist:.2f}m'
            )
    
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
        if self.require_guided_mode and self.current_state.mode != 'GUIDED':
            current_mode = self.current_state.mode
            
            # MANUAL 模式: 用户明确要求手动控制，尊重用户意图，结束导航
            if current_mode == 'MANUAL':
                if self._navigation_state == NavigationState.ACTIVE:
                    self.get_logger().warn(
                        f'⚠️ 检测到切换为 MANUAL 模式，尊重用户意图，结束导航任务'
                    )
                    # 强制结束导航，设置为 CANCELLED 状态，不自动恢复 GUIDED
                    self._end_navigation(NavigationState.CANCELLED, "用户切换到MANUAL模式")
                else:
                    self.get_logger().debug(f'需要 GUIDED 模式，当前: {current_mode}')
                return False
            
            # HOLD/LOITER 模式: 根据导航状态和手动请求标志决定是否恢复 GUIDED
            if current_mode in ['HOLD', 'LOITER']:
                # 检查是否是手动请求的 HOLD（来自地面站的暂停请求）
                current_time = self.get_clock().now().nanoseconds / 1e9
                is_manual_hold = (
                    self._manual_hold_requested and 
                    (current_time - self._manual_hold_request_time) < self._manual_hold_timeout
                )
                
                # 检查是否在 PAUSED 状态的保护期内
                # 保护期内不尝试恢复 GUIDED，等待 cancel_navigation 消息到达
                in_paused_grace_period = (
                    self._navigation_state == NavigationState.PAUSED and
                    (current_time - self._paused_state_enter_time) < self._paused_state_grace_period
                )
                
                if is_manual_hold:
                    # 手动请求的 HOLD，尊重用户意图，进入暂停状态，不恢复 GUIDED
                    if self._navigation_state == NavigationState.ACTIVE:
                        self.get_logger().warn(
                            f'⚠️ 检测到手动切换 {current_mode} 模式（来自地面站），任务进入暂停状态'
                        )
                        self._set_navigation_state(NavigationState.PAUSED, f"用户手动切换{current_mode}模式")
                        # 停止当前运动
                        self._publish_velocity_command(VelocityCommand.stop())
                    # 手动暂停状态下不恢复 GUIDED，等待用户发送新任务或手动切换 GUIDED
                    self.get_logger().debug(f'手动 {current_mode} 模式（暂停中），不自动恢复 GUIDED')
                elif in_paused_grace_period:
                    # 在 PAUSED 状态保护期内，不尝试恢复 GUIDED
                    # 等待 cancel_navigation 消息到达，以确定是手动暂停还是飞控自动切换
                    self.get_logger().debug(
                        f'PAUSED 状态保护期内 ({current_time - self._paused_state_enter_time:.1f}s < {self._paused_state_grace_period}s)，'
                        f'等待 cancel_navigation 消息'
                    )
                elif self._navigation_state == NavigationState.ACTIVE:
                    # 导航进行中被切换到 HOLD（可能是飞控自动切换或手动切换）
                    # 先进入 PAUSED 状态并等待保护期，让 cancel_navigation 消息有时间到达
                    self._set_navigation_state(NavigationState.PAUSED, f"被{current_mode}模式打断，等待确认")
                    self._publish_velocity_command(VelocityCommand.stop())
                    self.get_logger().info(
                        f'⏸️ 检测到 {current_mode} 模式，进入保护期等待 ({self._paused_state_grace_period}s)，'
                        f'以确定是手动暂停还是飞控自动切换'
                    )
                    # 不立即恢复 GUIDED，等待下一个循环检查是否有 cancel_navigation 消息
                elif self._navigation_state == NavigationState.PAUSED:
                    # 已经是 PAUSED 状态，且已过保护期
                    # 保护期已过但没有收到 cancel_navigation 消息，说明是飞控自动切换
                    # 应该自动恢复 GUIDED 继续任务
                    if not is_manual_hold:
                        self.get_logger().info(
                            f'⏱️ 保护期已过，未收到手动暂停请求，判定为飞控自动切换 {current_mode}，'
                            f'自动恢复 GUIDED 模式'
                        )
                        self._restore_guided_mode()
                        # 恢复导航状态
                        self._set_navigation_state(NavigationState.ACTIVE, "自动恢复GUIDED模式")
                    else:
                        self.get_logger().debug(
                            f'PAUSED 状态保持 {current_mode}，手动暂停标志仍有效，不恢复 GUIDED'
                        )
                else:
                    # IDLE, CANCELLED, COMPLETED, FAILED 状态不恢复
                    self.get_logger().debug(f'需要 GUIDED 模式，当前: {current_mode}')
                return False
            
            # 其他模式 (RTL, AUTO 等): 导航进行中尝试恢复 GUIDED
            if self._should_protect_navigation():
                self._set_navigation_state(NavigationState.PAUSED, f"被{current_mode}模式打断")
                self._restore_guided_mode()
            else:
                self.get_logger().debug(f'需要 GUIDED 模式，当前: {current_mode}')
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
        
        Returns:
            bool: 是否应该自动恢复 GUIDED 模式
        """
        if not self._mode_protection_enabled:
            return False
        
        # 只有 ACTIVE 状态才保护（被意外切换模式时恢复）
        # PAUSED 状态表示已经被打断一次，如果再次被打断可能是用户意图
        return self._navigation_state == NavigationState.ACTIVE
    
    def _end_navigation(self, end_state: NavigationState, reason: str = ""):
        """
        结束当前导航任务
        
        统一处理导航结束的所有清理工作。
        
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
        
        # 停止 USV
        self.stop_usv()
    
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
        
        status_msg = String()
        status_msg.data = ','.join(status_parts)
        self.status_pub.publish(status_msg)
    
    # ==================== 模式切换 ====================
    
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
        导航进行中检测到非 GUIDED 模式时，自动恢复到 GUIDED 模式
        """
        import time
        current_time = time.time()
        
        # 冷却时间检查，避免频繁切换
        if current_time - self._last_mode_restore_time < self._mode_restore_cooldown:
            return
        
        self._last_mode_restore_time = current_time
        
        current_mode = self.current_state.mode if self.current_state else 'UNKNOWN'
        self.get_logger().warn(
            f'⚠️ 导航进行中检测到模式切换为 {current_mode}，自动恢复 GUIDED 模式'
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
