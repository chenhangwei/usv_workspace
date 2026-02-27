#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# ROS 2 Node implementation: Navigate To Point Node.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
NavigateToPoint 导航节点 (基于话题通信) - 支持平滑航点切换

该节点替代 Action Server,使用普通 ROS 2 话题实现导航功能。
更适合跨 Domain 通信场景,避免了 Domain Bridge 对 Action 转发的复杂性。

新增功能 (航点队列 + 平滑切换):
- 航点队列管理: 支持连续接收多个目标点
- 平滑切换: 中间航点提前切换，避免减速停止
- 精确到达: 仅最终航点才真正到达判定

话题接口:
- 订阅: navigation_goal (NavigationGoal) - 接收导航目标
- 发布: navigation_feedback (NavigationFeedback) - 发送导航反馈
- 发布: navigation_result (NavigationResult) - 发送导航结果
- 发布: set_usv_nav_goal (NavigationGoal) - 转发到控制节点

作者: Auto-generated
日期: 2025-11-19
更新: 2026-01-21 - 添加平滑导航支持 (航点队列 + 提前切换)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from common_interfaces.msg import NavigationGoal, NavigationFeedback, NavigationResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
from std_msgs.msg import Float32
from collections import deque
from typing import Optional


class NavigateToPointNode(Node):
    """
    基于话题通信的导航节点 - 支持平滑航点切换
    
    功能:
    1. 接收导航目标(NavigationGoal) - 加入航点队列
    2. 定期发送反馈(NavigationFeedback)
    3. 中间航点: 提前切换，不减速
    4. 最终航点: 到达阈值内才停止
    5. 转发目标点到控制节点
    
    平滑导航原理:
    - 中间航点: 距离 < switch_threshold 时立即切换到下一航点
    - 最终航点: 距离 < nav_arrival_threshold 时才视为到达
    - 这样可以避免在中间航点处减速停止
    """

    def __init__(self):
        """初始化导航节点"""
        super().__init__('navigate_to_point_node')
        
        # 创建回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # ==================== 平滑导航参数 ====================
        self.declare_parameter('smooth_navigation', True)
        self.declare_parameter('switch_threshold', 1.5)       # 中间航点切换阈值(米)
        self.declare_parameter('waypoint_queue_size', 10)     # 航点队列大小
        self.declare_parameter('lookahead_distance', 5.0)     # 前视距离(备用)
        
        self.smooth_navigation = self.get_parameter('smooth_navigation').value
        self.switch_threshold = self.get_parameter('switch_threshold').value
        self.waypoint_queue_size = self.get_parameter('waypoint_queue_size').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # ==================== 导航模式参数 ====================
        # 导航模式常量 (与 NavigationGoal.msg 中的定义对应)
        self.NAV_MODE_ASYNC = 0      # 异步模式: 到达后立即前往下一个目标点
        self.NAV_MODE_SYNC = 1       # 同步模式: 等待到达质量确认
        self.NAV_MODE_ROTATE = 2     # 旋转模式: 到达后原地旋转
        self.NAV_MODE_TERMINAL = 3   # 终止模式: 到达后停止
        
        self.declare_parameter('default_sync_timeout', 10.0)           # 同步模式默认超时(秒)
        self.declare_parameter('default_arrival_quality_threshold', 0.8)  # 默认到达质量阈值
        self.declare_parameter('arrival_check_window', 5.0)            # 到达质量检测窗口(秒)
        
        self.default_sync_timeout = self.get_parameter('default_sync_timeout').value
        self.default_arrival_quality_threshold = self.get_parameter('default_arrival_quality_threshold').value
        self.arrival_check_window = self.get_parameter('arrival_check_window').value
        
        # 同步模式状态跟踪
        self._sync_mode_start_time = None           # 同步模式开始时间
        self._arrival_check_samples: list = []     # 到达质量检测样本 (distance, timestamp)
        
        # 旋转模式状态跟踪
        self._rotate_start_yaw = None              # 旋转开始时的航向
        self._rotate_total_angle = 0.0             # 已累计旋转角度
        self._rotate_last_yaw = None               # 上一次检测的航向
        self._rotate_in_progress = False           # 是否正在执行旋转
        
        # 延迟最终到达判定 - ASYNC 模式下队列为空时不立即判定为“最终航点”
        # 给后续 batch 航点到达的时间窗口，避免中间航点被误判为最终目标
        self._pending_final_arrival: bool = False          # 是否有待确认的最终到达
        self._pending_final_arrival_time: float = 0.0      # 开始等待的时间
        self._pending_final_arrival_delay: float = 2.0     # 等待时间(秒)，覆盖典型 batch 间隔(~1s)
        self._pending_final_distance: float = 0.0          # 待确认时的距离
        
        # ==================== 偏离检测 (Divergence Detection) ====================
        # 补救机制: 当 USV 进入监控半径后如果距离持续增大，
        # 说明无法到达目标点，强制判定为"偏离到达"并切换到下一个航点
        self._divergence_monitor_radius: float = 3.0    # 开始监控的半径 (m)
        self._divergence_min_distance: float = float('inf')  # 监控期间的最小距离
        self._divergence_increase_count: int = 0         # 距离连续增大的计数
        self._divergence_increase_threshold: int = 8     # 连续增大多少次判定为偏离
        self._divergence_monitoring: bool = False         # 是否处于偏离监控状态
        self._divergence_last_distance: float = float('inf')  # 上次检测的距离
        
        # ==================== 进度感知超时 (Progress-Aware Timeout) ====================
        # 如果 USV 在持续接近目标点，超时计时器会重置，避免远距离航点因固定超时而失败
        self.declare_parameter('nav_timeout_progress_check_interval', 30.0)  # 进度检查间隔(秒)
        self.declare_parameter('nav_timeout_progress_min_decrease', 0.5)    # 判定为有进度的最小距离减少(米)
        self.declare_parameter('nav_timeout_max_multiplier', 5.0)           # 硬性最大超时倍率
        
        self._timeout_progress_check_interval: float = self.get_parameter(
            'nav_timeout_progress_check_interval').value
        self._timeout_progress_min_decrease: float = self.get_parameter(
            'nav_timeout_progress_min_decrease').value
        self._timeout_max_multiplier: float = self.get_parameter(
            'nav_timeout_max_multiplier').value
        
        # 进度追踪状态
        self._timeout_original_start_time = None       # 绝对起始时间 (不会因进度重置)
        self._timeout_last_progress_time = None        # 上次进度检查时间
        self._timeout_progress_last_distance = None    # 上次进度检查时的距离
        
        # ==================== 航点队列 ====================
        self.waypoint_queue: deque = deque(maxlen=self.waypoint_queue_size)
        
        # 订阅导航目标
        self.goal_sub = self.create_subscription(
            NavigationGoal,
            'navigation_goal',
            self.goal_callback,
            qos_reliable,
            callback_group=self.callback_group)
        
        # 发布导航反馈
        self.feedback_pub = self.create_publisher(
            NavigationFeedback,
            'navigation_feedback',
            qos_reliable)
        
        # 发布导航结果
        self.result_pub = self.create_publisher(
            NavigationResult,
            'navigation_result',
            qos_reliable)
        
        # 发布目标位置到控制节点
        self.target_pub = self.create_publisher(
            NavigationGoal,
            'set_usv_nav_goal',
            qos_reliable)

        # 订阅导航参数下发：到达阈值（米）
        self.nav_threshold_sub = self.create_subscription(
            Float32,
            'set_nav_arrival_threshold',
            self._nav_arrival_threshold_callback,
            qos_reliable,
            callback_group=self.callback_group)
        
        # 订阅切换阈值更新
        self.switch_threshold_sub = self.create_subscription(
            Float32,
            'set_nav_switch_threshold',
            self._switch_threshold_callback,
            qos_reliable,
            callback_group=self.callback_group)
        
        # 订阅平滑导航开关更新
        from std_msgs.msg import Bool
        self.smooth_nav_sub = self.create_subscription(
            Bool,
            'set_nav_smooth_navigation',
            self._smooth_navigation_callback,
            qos_reliable,
            callback_group=self.callback_group)
        
        # 订阅暂停导航请求 (来自地面站 HOLD 按钮)
        # 暂停：保留当前任务和队列，点击 GUIDED 可恢复
        self.cancel_nav_sub = self.create_subscription(
            Bool,
            'cancel_navigation',
            self._cancel_navigation_callback,
            qos_reliable,
            callback_group=self.callback_group)
        
        # 订阅停止导航请求 (来自地面站 STOP 按钮)
        # 停止：清空当前任务和队列，需要重新发送新任务
        self.stop_nav_sub = self.create_subscription(
            Bool,
            'stop_navigation',
            self._stop_navigation_callback,
            qos_reliable,
            callback_group=self.callback_group)
        
        # 订阅飞控模式变化 (用于检测 GUIDED 恢复)
        from mavros_msgs.msg import State
        self._current_mode = None
        self.state_sub = self.create_subscription(
            State,
            'state',
            self._state_callback,
            qos_best_effort,
            callback_group=self.callback_group)
        
        # 订阅当前位置 (使用 GPS 转换的统一坐标系)
        self.current_pose: Optional[PoseStamped] = None
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose_from_gps',
            self.pose_callback,
            qos_best_effort,
            callback_group=self.callback_group)
        
        # 使用ParamLoader统一加载导航参数
        from common_utils import ParamLoader
        loader = ParamLoader(self)
        self.nav_arrival_threshold = loader.load_param('nav_arrival_threshold', 1.0)
        self.nav_feedback_period = loader.load_param('nav_feedback_period', 0.5)
        self.distance_mode = loader.load_param(
            'distance_mode', '2d',
            validator=lambda x: x in ['2d', '3d'])

        # 重复目标去重参数
        self.declare_parameter('dedup_goal_enabled', True)
        self.declare_parameter('dedup_goal_pos_epsilon', 0.01)
        self.declare_parameter('dedup_goal_yaw_epsilon_deg', 2.0)
        self.declare_parameter('dedup_goal_maneuver_param_epsilon', 1e-3)
        
        # ==================== 通信保护参数 ====================
        self.declare_parameter('idle_timeout_sec', 30.0)  # 空闲超时时间(秒)
        self.declare_parameter('idle_protection_enabled', True)  # 是否启用空闲保护
        self.idle_timeout_sec = self.get_parameter('idle_timeout_sec').value
        self.idle_protection_enabled = self.get_parameter('idle_protection_enabled').value
        self._last_goal_received_time = None  # 上次收到航点的时间
        self._idle_warning_logged = False     # 是否已记录空闲警告
        
        # 当前任务状态
        self.current_goal: Optional[NavigationGoal] = None
        self.current_goal_id: Optional[int] = None
        self.goal_start_time = None
        self._last_dedup_goal_id = None
        self._is_paused = False  # 暂停标志（HOLD时暂停，GUIDED时恢复）
        self._manual_pause_requested = False  # 手动暂停标志（来自地面站的暂停请求）
        self._manual_pause_time = 0.0  # 手动暂停请求时间戳
        
        # 创建导航循环定时器
        self.nav_timer = self.create_timer(
            self.nav_feedback_period,
            self.navigation_loop,
            callback_group=self.callback_group)
        
        self.get_logger().info('NavigateToPoint 节点已启动 (平滑导航模式)')
        self.get_logger().info(f'  到达阈值: {self.nav_arrival_threshold}m')
        self.get_logger().info(f'  切换阈值: {self.switch_threshold}m')
        self.get_logger().info(f'  距离模式: {self.distance_mode.upper()}')
        self.get_logger().info(f'  反馈周期: {self.nav_feedback_period}s')
        self.get_logger().info(f'  平滑导航: {"启用" if self.smooth_navigation else "禁用"}')
        self.get_logger().info(f'  空闲保护: {"启用" if self.idle_protection_enabled else "禁用"} ({self.idle_timeout_sec}s)')
        self.get_logger().info(f'  同步超时: {self.default_sync_timeout}s')
        self.get_logger().info(f'  到达质量: {self.default_arrival_quality_threshold*100:.0f}%')

    def _nav_arrival_threshold_callback(self, msg: Float32):
        """运行时更新到达阈值（米）。"""
        try:
            value = float(msg.data)
        except Exception:
            self.get_logger().warn(f"收到非法 nav_arrival_threshold: {msg.data}")
            return

        if value <= 0.0:
            self.get_logger().warn(f"忽略 nav_arrival_threshold<=0: {value}")
            return

        old = getattr(self, 'nav_arrival_threshold', None)
        self.nav_arrival_threshold = value
        try:
            if old is None:
                self.get_logger().info(f"nav_arrival_threshold 已设置为 {value:.2f}m")
            else:
                self.get_logger().info(f"nav_arrival_threshold 更新: {float(old):.2f}m -> {value:.2f}m")
        except Exception:
            self.get_logger().info(f"nav_arrival_threshold 更新为 {value:.2f}m")

    def _switch_threshold_callback(self, msg: Float32):
        """运行时更新切换阈值（米）。"""
        try:
            value = float(msg.data)
        except Exception:
            self.get_logger().warn(f"收到非法 switch_threshold: {msg.data}")
            return

        if value <= 0.0:
            self.get_logger().warn(f"忽略 switch_threshold<=0: {value}")
            return

        old = self.switch_threshold
        self.switch_threshold = value
        self.get_logger().info(f"switch_threshold 更新: {old:.2f}m -> {value:.2f}m")

    def _smooth_navigation_callback(self, msg):
        """运行时更新平滑导航开关。"""
        try:
            value = bool(msg.data)
        except Exception:
            self.get_logger().warn(f"收到非法 smooth_navigation: {msg.data}")
            return

        old = self.smooth_navigation
        self.smooth_navigation = value
        old_str = "启用" if old else "禁用"
        new_str = "启用" if value else "禁用"
        self.get_logger().info(f"smooth_navigation 更新: {old_str} -> {new_str}")
    
    def _cancel_navigation_callback(self, msg):
        """
        取消导航回调 - 来自地面站的暂停/取消请求
        
        当用户在地面站点击 HOLD 按钮时，会发送此消息。
        任务进入暂停状态，但**保留当前任务和队列**。
        需要用户发送新任务才能恢复（不会因为 GUIDED 模式自动恢复）。
        
        行为：
        - 暂停导航（设置 _is_paused 标志）
        - 设置手动暂停标志（防止 GUIDED 模式时自动恢复）
        - 保留当前目标和航点队列（不清空）
        - 只有收到新任务时才恢复，GUIDED 模式不再自动恢复
        """
        from std_msgs.msg import Bool
        import time
        if not isinstance(msg, Bool) or not msg.data:
            return
        
        self.get_logger().warn('🛑 收到暂停导航请求（手动暂停），任务进入暂停状态')
        
        # 设置暂停标志，而不是清空任务
        self._is_paused = True
        
        # 设置手动暂停标志 - 防止 GUIDED 模式时自动恢复
        self._manual_pause_requested = True
        self._manual_pause_time = time.time()
        
        # 发布暂停结果（但不是取消结果）
        result = NavigationResult()
        # goal_id 是 uint32，不能为负数，用 0 表示无有效目标
        result.goal_id = self.current_goal_id if self.current_goal_id is not None else 0
        result.success = False
        result.error_code = 2  # 2 = 暂停（区别于 0=成功, 1=超时）
        result.message = '导航任务已暂停（等待恢复）'
        self.result_pub.publish(result)
        
        queue_len = len(self.waypoint_queue)
        current_info = f'当前目标ID={self.current_goal_id}' if self.current_goal else '无当前目标'
        self.get_logger().info(f'✅ 导航已暂停 ({current_info}, 队列={queue_len}), 手动暂停不会自动恢复')
    
    def _stop_navigation_callback(self, msg):
        """
        停止导航回调 - 来自地面站的完全停止请求
        
        当用户在地面站点击集群 STOP 按钮时，会发送此消息。
        任务完全停止，**清空当前任务和队列**。
        需要发送新任务才能重新开始导航。
        
        行为：
        - 清空当前目标和航点队列
        - 重置所有导航状态
        - 发布停止结果
        """
        from std_msgs.msg import Bool
        if not isinstance(msg, Bool) or not msg.data:
            return
        
        self.get_logger().warn('🛑 收到停止导航请求，清空所有任务')
        
        # 清空航点队列
        queue_len = len(self.waypoint_queue)
        self.waypoint_queue.clear()
        
        # 清除当前任务
        old_goal_id = self.current_goal_id
        self.current_goal = None
        self.current_goal_id = None
        self.goal_start_time = None
        
        # 重置暂停状态
        self._is_paused = False
        
        # 重置同步模式和旋转模式状态
        self._sync_mode_start_time = None
        self._arrival_check_samples = []
        self._rotate_in_progress = False
        self._rotate_start_yaw = None
        
        # 发布停止结果
        result = NavigationResult()
        # goal_id 是 uint32，不能为负数，用 0 表示无有效目标
        result.goal_id = old_goal_id if old_goal_id is not None else 0
        result.success = False
        result.error_code = 3  # 3 = 停止（区别于 0=成功, 1=超时, 2=暂停）
        result.message = '导航任务已停止（需重新发送任务）'
        self.result_pub.publish(result)
        
        self.get_logger().info(f'✅ 导航已停止 (清空 {queue_len} 个排队航点), 等待新任务')
    
    def _state_callback(self, msg):
        """
        飞控模式状态回调 - 检测 GUIDED 模式恢复
        
        当暂停状态下检测到 GUIDED 模式时：
        - 用户明确点击 GUIDED 按钮，清除手动暂停状态并恢复导航
        - 这允许用户通过点击 GUIDED 按钮来恢复被暂停的任务
        """
        new_mode = msg.mode
        old_mode = self._current_mode
        self._current_mode = new_mode
        
        # 如果模式变化为 GUIDED 且当前处于暂停状态，恢复导航
        if self._is_paused and new_mode == 'GUIDED' and old_mode != 'GUIDED':
            # 用户明确切换到 GUIDED 模式，清除手动暂停状态并恢复导航
            if self._manual_pause_requested:
                self.get_logger().info(
                    '▶️ 用户切换到 GUIDED 模式，清除手动暂停状态'
                )
                self._manual_pause_requested = False
            
            self._is_paused = False
            current_info = f'目标ID={self.current_goal_id}' if self.current_goal else '无目标'
            queue_len = len(self.waypoint_queue)
            self.get_logger().info(
                f'▶️ 检测到 GUIDED 模式，导航已恢复 ({current_info}, 队列={queue_len})')
            
            # 如果有当前目标，重新转发到控制节点
            if self.current_goal is not None:
                self.target_pub.publish(self.current_goal)
                self.get_logger().info(f'🔄 重新转发目标 [ID={self.current_goal_id}] 到控制节点')
    
    def pose_callback(self, msg):
        """更新当前位置"""
        self.current_pose = msg
    
    def goal_callback(self, msg: NavigationGoal):
        """
        接收新的导航目标
        
        平滑导航模式:
        - 如果当前没有任务，立即开始执行
        - 如果当前有任务，新目标加入队列等待
        
        Args:
            msg (NavigationGoal): 导航目标消息
        """
        # 记录收到航点的时间 (用于空闲保护)
        self._last_goal_received_time = self.get_clock().now()
        self._idle_warning_logged = False  # 重置警告标志
        
        # 收到新任务时，自动解除暂停状态和手动暂停标志
        if self._is_paused or self._manual_pause_requested:
            self._is_paused = False
            self._manual_pause_requested = False
            self.get_logger().info('▶️ 收到新目标，自动解除暂停状态（包括手动暂停）')
        
        # 收到新航点，取消延迟最终到达判定（新航点证明这不是最终目标）
        if self._pending_final_arrival:
            self._pending_final_arrival = False
            self.get_logger().debug(
                f'✓ 收到新目标 [ID={msg.goal_id}], 已取消延迟最终到达判定')
        
        # 详细诊断日志
        current_goal_info = f"current_goal={'有' if self.current_goal else '无'}"
        if self.current_goal:
            current_goal_info += f"(ID={self.current_goal_id})"
        
        self.get_logger().info(
            f'📥 收到目标 [ID={msg.goal_id}]: '
            f'({msg.target_pose.pose.position.x:.2f}, '
            f'{msg.target_pose.pose.position.y:.2f}) '
            f'| 状态: {current_goal_info}, 队列={len(self.waypoint_queue)}')

        # ========== 过时步骤拒绝 + 新任务检测 + 消息乱序修正 ==========
        # 从确定性 goal_id 解析步骤号: goal_id = usv编号 × 10000 + step
        incoming_step = msg.goal_id % 10000
        current_step = (self.current_goal_id % 10000) if self.current_goal_id else 0
        
        # 处理消息乱序: step N+1 可能先于 step N 到达 (ROS2 不保证 topic 消息顺序)
        # 场景: GS 几乎同时发送 step N 和 step N+1 (lookahead)，N+1 先到达 USV
        if incoming_step > 0 and incoming_step < current_step:
            # 检查当前目标是否刚设置 (< 1秒) → 可能是消息乱序而非过时航点
            if self.goal_start_time:
                elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
                if elapsed < 1.0 and incoming_step == current_step - 1:
                    # 极可能是乱序: step N+1 先到达成为 current，现在 step N 到达
                    # 修正: 将 step N 设为 current，step N+1 推入队列前端
                    old_current = self.current_goal
                    old_current_id = self.current_goal_id
                    
                    # 推入队列前端 (保持顺序)
                    self.waypoint_queue.appendleft(old_current)
                    
                    # 设置低步骤号为当前目标
                    self.current_goal = msg
                    self.current_goal_id = msg.goal_id
                    self.goal_start_time = self.get_clock().now()
                    self._last_dedup_goal_id = None
                    
                    self.get_logger().warn(
                        f'🔄 消息乱序修正: [ID={msg.goal_id}](step={incoming_step}) '
                        f'应在 [ID={old_current_id}](step={current_step}) 之前, '
                        f'已交换执行顺序, 队列={len(self.waypoint_queue)}')
                    
                    # 转发修正后的当前目标
                    self.target_pub.publish(msg)
                    return
            
            # 非乱序场景: 真正的过时航点 (重试时USV已走过)
            self.get_logger().info(
                f'⏭️ 跳过过时航点 [ID={msg.goal_id}] step={incoming_step} < current={current_step}')
            return
        
        # 新任务检测：step 回到 1 而当前在更高步骤 → 新任务开始
        # （主要作为 stop_navigation 消息的备份机制，防止消息丢失时残留活动）
        if incoming_step == 1 and current_step > 1:
            old_goal_id = self.current_goal_id
            old_queue_len = len(self.waypoint_queue)
            
            # 清除当前任务
            self.current_goal = None
            self.current_goal_id = None
            self.goal_start_time = None
            self._last_dedup_goal_id = None
            
            # 清空队列
            self.waypoint_queue.clear()
            
            # 重置状态
            self._sync_mode_start_time = None
            self._arrival_check_samples = []
            self._rotate_in_progress = False
            
            self.get_logger().info(
                f'🗑️ 新任务检测 [ID={msg.goal_id}, step={incoming_step}], 清空残留: '
                f'旧任务ID={old_goal_id}, 队列长度={old_queue_len}')

        # 检查是否与当前目标重复
        is_dup = self._is_duplicate_goal(msg)
        if is_dup:
            if self._last_dedup_goal_id != msg.goal_id:
                self.get_logger().warn(
                    f'♻️ 重复目标已合并: new_id={msg.goal_id} -> keep_id={self.current_goal_id}'
                )
                self._last_dedup_goal_id = msg.goal_id
            self.current_goal_id = msg.goal_id
            return

        # 检查是否与队列尾部重复
        if self.waypoint_queue:
            last = self.waypoint_queue[-1]
            if self._is_same_position(msg, last):
                self.get_logger().debug(f'忽略队列重复: ID={msg.goal_id}')
                return

        # ========== 平滑导航模式 ==========
        if self.smooth_navigation:
            if self.current_goal is None:
                # 没有当前任务，立即执行
                self._set_current_goal(msg)
            else:
                # 有当前任务，加入队列等待
                self.waypoint_queue.append(msg)
                self.get_logger().info(
                    f'📋 航点入队 [ID={msg.goal_id}], 队列长度={len(self.waypoint_queue)}')
        else:
            # 非平滑模式：直接覆盖当前目标
            self._set_current_goal(msg)

    def _set_current_goal(self, msg: NavigationGoal):
        """设置当前目标并转发到控制节点"""
        self.current_goal = msg
        self.current_goal_id = msg.goal_id
        self.goal_start_time = self.get_clock().now()
        self._last_dedup_goal_id = None
        
        # 重置进度感知超时状态
        self._timeout_original_start_time = None
        self._timeout_last_progress_time = None
        self._timeout_progress_last_distance = None
        
        # 转发目标到控制节点
        self.target_pub.publish(msg)
        self.get_logger().info(f'✓ 目标已转发 [ID={msg.goal_id}]')

    def _is_same_position(self, msg1: NavigationGoal, msg2: NavigationGoal) -> bool:
        """检查两个目标位置是否相同（用于队列去重）"""
        p1 = msg1.target_pose.pose.position
        p2 = msg2.target_pose.pose.position
        dist = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
        return dist < 0.5

    def _is_duplicate_goal(self, msg: NavigationGoal) -> bool:
        """判断新 goal 是否与当前 goal 等价（位置/航向/机动一致）。"""
        try:
            enabled_val = self.get_parameter('dedup_goal_enabled').value
            if enabled_val is None:
                return False
            if not bool(enabled_val):
                return False
        except Exception:
            return False

        if self.current_goal is None:
            return False

        def _param_float(name: str, default: float) -> float:
            try:
                v = self.get_parameter(name).value
                if v is None:
                    return default
                return float(v)
            except Exception:
                return default

        pos_eps = _param_float('dedup_goal_pos_epsilon', 0.01)
        yaw_eps_deg = _param_float('dedup_goal_yaw_epsilon_deg', 2.0)
        man_eps = _param_float('dedup_goal_maneuver_param_epsilon', 1e-3)

        pos_eps = max(0.0, pos_eps)
        yaw_eps = math.radians(max(0.0, yaw_eps_deg))
        man_eps = max(0.0, man_eps)

        a = self.current_goal
        b = msg

        ap = a.target_pose.pose.position
        bp = b.target_pose.pose.position
        dx = float(ap.x) - float(bp.x)
        dy = float(ap.y) - float(bp.y)
        dz = float(ap.z) - float(bp.z)
        if math.sqrt(dx * dx + dy * dy + dz * dz) > pos_eps:
            return False

        if bool(getattr(a, 'enable_yaw', False)) != bool(getattr(b, 'enable_yaw', False)):
            return False

        if bool(getattr(b, 'enable_yaw', False)):
            ay = self._yaw_from_quat(a.target_pose.pose.orientation)
            by = self._yaw_from_quat(b.target_pose.pose.orientation)
            if abs(self._wrap_pi(ay - by)) > yaw_eps:
                return False

        if int(getattr(a, 'maneuver_type', 0)) != int(getattr(b, 'maneuver_type', 0)):
            return False
        if abs(float(getattr(a, 'maneuver_param', 0.0)) - float(getattr(b, 'maneuver_param', 0.0))) > man_eps:
            return False

        return True

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _yaw_from_quat(q) -> float:
        """从四元数计算 yaw (rad)，与常见 ENU 公式一致。"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def navigation_loop(self):
        """
        导航主循环 - 支持平滑航点切换
        
        该函数每隔固定周期执行一次,负责:
        1. 计算到目标点的距离和航向误差
        2. 发送导航反馈
        3. 平滑导航模式: 中间航点提前切换，最终航点精确到达
        4. 标准模式: 每个航点都精确到达
        5. 空闲保护: 检测通信断开
        """
        # ========== 空闲保护检测 ==========
        self._check_idle_protection()
        
        # 如果处于暂停状态，跳过导航逻辑
        if self._is_paused:
            return
        
        # 如果没有活动任务,跳过
        if self.current_goal is None:
            return
        
        # 如果还没收到当前位置,跳过
        if self.current_pose is None:
            return
        
        # 计算到目标点的距离
        distance = self._calculate_distance(
            self.current_goal.target_pose,
            self.current_pose)
        
        # 计算航向误差
        heading_error = self._calculate_heading_error(
            self.current_goal.target_pose,
            self.current_pose)
        
        # 估算剩余时间 (假设平均速度 0.5 m/s)
        estimated_time = distance / 0.5
        
        # 发布反馈
        self._publish_feedback(distance, heading_error, estimated_time)
        
        # 简化日志 - 只在距离变化较大时输出
        if not hasattr(self, '_last_distance') or abs(distance - self._last_distance) > 0.5:
            queue_info = f', 队列={len(self.waypoint_queue)}' if self.waypoint_queue else ''
            self.get_logger().info(
                f'导航中 [ID={self.current_goal_id}]: '
                f'距离={distance:.2f}m, 航向误差={math.degrees(heading_error):.1f}°{queue_info}')
            self._last_distance = distance
        
        # ========== 关键：平滑航点切换逻辑 ==========
        if self.smooth_navigation:
            self._handle_smooth_navigation(distance)
        else:
            self._handle_standard_navigation(distance)

    def _handle_smooth_navigation(self, distance: float):
        """
        平滑导航模式 - 支持四种导航模式
        
        导航模式:
        - NAV_MODE_ASYNC (0): 异步模式 - 到达后立即前往下一个目标点
        - NAV_MODE_SYNC (1): 同步模式 - 等待到达质量确认(80%到达判断，超时自动切换异步)
        - NAV_MODE_ROTATE (2): 旋转模式 - 到达后原地旋转指定圈数
        - NAV_MODE_TERMINAL (3): 终止模式 - 到达后停止，无后续任务
        """
        has_next = len(self.waypoint_queue) > 0
        nav_mode = getattr(self.current_goal, 'nav_mode', self.NAV_MODE_ASYNC)
        
        # ===== 检查延迟最终到达判定 =====
        # 如果在等待期间队列中来了新航点，取消延迟并平滑切换
        if self._pending_final_arrival and has_next:
            self._pending_final_arrival = False
            self.get_logger().info(
                f'✓ 延迟判定取消: 队列收到新航点, 平滑切换 [ID={self.current_goal_id}]')
            self._switch_to_next_waypoint(distance, "延迟后平滑切换")
            return
        
        # ===== 检查延迟最终到达超时 =====
        if self._pending_final_arrival:
            import time
            elapsed = time.time() - self._pending_final_arrival_time
            if elapsed >= self._pending_final_arrival_delay:
                # 等待超时，确认是真正的最终航点
                self._pending_final_arrival = False
                self.get_logger().info(
                    f'🎯 到达最终目标! [ID={self.current_goal_id}], '
                    f'距离={self._pending_final_distance:.3f}m (延迟{elapsed:.1f}s确认)')
                self._publish_result(
                    self.current_goal_id, success=True,
                    message=f'成功到达目标(距离={self._pending_final_distance:.3f}m, 模式={self._nav_mode_name(nav_mode)})')
                self._clear_current_goal()
            # 在等待期间不做其他判断
            return
        
        # ===== 中间航点且为异步模式：提前切换 =====
        if has_next and nav_mode == self.NAV_MODE_ASYNC and distance < self.switch_threshold:
            self._switch_to_next_waypoint(distance, "平滑切换")
            return
        
        # ===== 检查是否到达目标点（所有模式共用的到达判断） =====
        if distance < self.nav_arrival_threshold:
            self._reset_divergence_monitor()  # 已到达，重置偏离检测
            self._handle_arrival(distance, nav_mode, has_next)
            return
        
        # ===== 偏离检测补救: 进入3m后距离持续增大则强制切换 =====
        if self._check_divergence(distance):
            return
        
        # 超时检查
        self._check_timeout(distance)
    
    def _handle_arrival(self, distance: float, nav_mode: int, has_next: bool):
        """
        处理到达目标点后的行为，根据导航模式执行不同操作
        
        Args:
            distance: 当前距离目标点的距离
            nav_mode: 导航模式
            has_next: 是否有下一个航点
        """
        # ===== 旋转模式：先完成旋转再处理后续 =====
        if nav_mode == self.NAV_MODE_ROTATE:
            if self._handle_rotate_mode():
                return  # 旋转未完成，继续等待
        
        # ===== 同步模式：等待到达质量确认 =====
        if nav_mode == self.NAV_MODE_SYNC:
            sync_result = self._handle_sync_mode(distance)
            if sync_result == 'waiting':
                return  # 继续等待确认
            elif sync_result == 'timeout':
                # 超时，自动切换为异步模式
                self.get_logger().warn(
                    f'⏱️ 同步模式超时 [ID={self.current_goal_id}], 自动切换为异步模式')
                nav_mode = self.NAV_MODE_ASYNC  # 降级为异步模式
        
        # ===== 终止模式：立即停止 =====
        if nav_mode == self.NAV_MODE_TERMINAL:
            self.get_logger().info(
                f'🎯 到达终止点! [ID={self.current_goal_id}], 距离={distance:.3f}m')
            self._publish_result(
                self.current_goal_id, success=True,
                message=f'成功到达目标(距离={distance:.3f}m, 模式={self._nav_mode_name(nav_mode)})')
            self._clear_current_goal()
            return
        
        # ===== 最后一个航点（队列为空） =====
        if not has_next:
            if nav_mode == self.NAV_MODE_ASYNC and not self._pending_final_arrival:
                # ASYNC 模式 + 队列为空: 可能是 batch 间隙，延迟判定
                # 给后续航点 2s 时间窗口，避免中间航点被误判为最终目标
                import time
                self._pending_final_arrival = True
                self._pending_final_arrival_time = time.time()
                self._pending_final_distance = distance
                self.get_logger().debug(
                    f'⏳ ASYNC 模式队列为空 [ID={self.current_goal_id}], '
                    f'延迟{self._pending_final_arrival_delay}s确认是否为最终航点')
                return
            elif not self._pending_final_arrival:
                # SYNC/ROTATE 模式: 立即判定为最终航点
                self.get_logger().info(
                    f'🎯 到达最终目标! [ID={self.current_goal_id}], 距离={distance:.3f}m')
                self._publish_result(
                    self.current_goal_id, success=True,
                    message=f'成功到达目标(距离={distance:.3f}m, 模式={self._nav_mode_name(nav_mode)})')
                self._clear_current_goal()
                return
            # _pending_final_arrival=True 的情况在上方 _handle_smooth_navigation 中处理
            return
        
        # ===== 异步模式或同步确认完成：切换到下一航点 =====
        self._switch_to_next_waypoint(distance, "到达切换")
    
    def _handle_sync_mode(self, distance: float) -> str:
        """
        处理同步模式 - 等待到达质量确认
        
        同步模式会在到达阈值内持续采样，判断是否稳定到达（80%样本在阈值内）
        
        Args:
            distance: 当前距离
            
        Returns:
            'confirmed': 到达质量确认通过
            'waiting': 继续等待
            'timeout': 同步模式超时
        """
        now = self.get_clock().now()
        
        # 初始化同步模式
        if self._sync_mode_start_time is None:
            self._sync_mode_start_time = now
            self._arrival_check_samples = []
            self.get_logger().info(
                f'🔄 进入同步模式 [ID={self.current_goal_id}], 开始到达质量检测')
        
        # 添加样本
        sample_time = now.nanoseconds / 1e9
        self._arrival_check_samples.append((distance, sample_time))
        
        # 清理过期样本（保留检测窗口内的样本）
        window_start = sample_time - self.arrival_check_window
        self._arrival_check_samples = [
            (d, t) for d, t in self._arrival_check_samples if t >= window_start
        ]
        
        # 检查是否有足够样本（至少5个）
        if len(self._arrival_check_samples) < 5:
            return 'waiting'
        
        # 计算到达质量（在阈值内的样本比例）
        threshold = getattr(self.current_goal, 'arrival_quality_threshold', 
                           self.default_arrival_quality_threshold)
        samples_in_threshold = sum(1 for d, _ in self._arrival_check_samples 
                                   if d < self.nav_arrival_threshold)
        quality = samples_in_threshold / len(self._arrival_check_samples)
        
        self.get_logger().debug(
            f'同步模式: 到达质量={quality*100:.1f}%, 样本数={len(self._arrival_check_samples)}')
        
        # 检查到达质量是否达标
        if quality >= threshold:
            self.get_logger().info(
                f'✅ 到达质量确认通过 [ID={self.current_goal_id}]: '
                f'{quality*100:.1f}% >= {threshold*100:.0f}%')
            return 'confirmed'
        
        # 检查是否超时
        sync_timeout = getattr(self.current_goal, 'sync_timeout', self.default_sync_timeout)
        elapsed = (now - self._sync_mode_start_time).nanoseconds / 1e9
        
        if elapsed > sync_timeout:
            self.get_logger().warn(
                f'⏱️ 同步模式超时 [ID={self.current_goal_id}]: '
                f'{elapsed:.1f}s > {sync_timeout:.0f}s, 质量={quality*100:.1f}%')
            return 'timeout'
        
        return 'waiting'
    
    def _handle_rotate_mode(self) -> bool:
        """
        处理旋转模式 - 在目标点原地旋转指定圈数
        
        Returns:
            True: 旋转未完成，需要继续
            False: 旋转已完成
        """
        if self.current_pose is None:
            return True
        
        # 获取当前航向
        current_yaw = self._yaw_from_quat(self.current_pose.pose.orientation)
        
        # 初始化旋转模式
        if not self._rotate_in_progress:
            self._rotate_in_progress = True
            self._rotate_start_yaw = current_yaw
            self._rotate_last_yaw = current_yaw
            self._rotate_total_angle = 0.0
            
            rotate_circles = getattr(self.current_goal, 'maneuver_param', 1.0)
            self.get_logger().info(
                f'🔄 开始旋转 [ID={self.current_goal_id}]: 目标={rotate_circles:.1f}圈')
            return True
        
        # 计算角度增量
        delta_yaw = self._wrap_pi(current_yaw - self._rotate_last_yaw)
        self._rotate_total_angle += delta_yaw
        self._rotate_last_yaw = current_yaw
        
        # 获取目标旋转角度
        rotate_circles = getattr(self.current_goal, 'maneuver_param', 1.0)
        target_angle = rotate_circles * 2 * math.pi
        
        # 检查是否完成旋转
        if abs(self._rotate_total_angle) >= abs(target_angle):
            completed_circles = self._rotate_total_angle / (2 * math.pi)
            self.get_logger().info(
                f'✅ 旋转完成 [ID={self.current_goal_id}]: '
                f'完成={completed_circles:.2f}圈')
            return False
        
        # 每隔一定时间输出旋转进度
        if not hasattr(self, '_last_rotate_log_time'):
            self._last_rotate_log_time = 0.0
        
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_rotate_log_time > 2.0:
            current_circles = self._rotate_total_angle / (2 * math.pi)
            progress = abs(self._rotate_total_angle / target_angle) * 100
            self.get_logger().info(
                f'🔄 旋转中 [ID={self.current_goal_id}]: '
                f'{current_circles:.2f}/{rotate_circles:.1f}圈 ({progress:.0f}%)')
            self._last_rotate_log_time = now_sec
        
        return True
    
    def _switch_to_next_waypoint(self, distance: float, reason: str):
        """
        切换到下一个航点
        
        Args:
            distance: 当前距离
            reason: 切换原因描述
        """
        old_id = self.current_goal_id
        
        # 发布"通过"结果
        self._publish_result(
            old_id, 
            success=True,
            message=f'已通过航点(距离={distance:.2f}m, {reason})')
        
        # 从队列取出下一个航点
        next_goal = self.waypoint_queue.popleft()
        
        self.get_logger().info(
            f'🔄 {reason}: [ID={old_id}] → [ID={next_goal.goal_id}], '
            f'距离={distance:.2f}m, 剩余队列={len(self.waypoint_queue)}')
        
        # 重置模式相关状态
        self._sync_mode_start_time = None
        self._arrival_check_samples = []
        self._rotate_in_progress = False
        
        # 重置偏离检测状态
        self._reset_divergence_monitor()
        
        # 重置进度感知超时状态
        self._timeout_original_start_time = None
        self._timeout_last_progress_time = None
        self._timeout_progress_last_distance = None
        
        # 设置新目标
        self.current_goal = next_goal
        self.current_goal_id = next_goal.goal_id
        self.goal_start_time = self.get_clock().now()
        
        # 立即转发新目标到控制节点
        self.target_pub.publish(next_goal)
    
    def _nav_mode_name(self, nav_mode: int) -> str:
        """获取导航模式的名称"""
        mode_names = {
            self.NAV_MODE_ASYNC: '异步',
            self.NAV_MODE_SYNC: '同步',
            self.NAV_MODE_ROTATE: '旋转',
            self.NAV_MODE_TERMINAL: '终止'
        }
        return mode_names.get(nav_mode, f'未知({nav_mode})')

    def _handle_standard_navigation(self, distance: float):
        """
        标准导航模式 - 每个航点都精确到达，支持四种导航模式
        """
        if distance < self.nav_arrival_threshold:
            self._reset_divergence_monitor()  # 已到达，重置偏离检测
            nav_mode = getattr(self.current_goal, 'nav_mode', self.NAV_MODE_ASYNC)
            has_next = len(self.waypoint_queue) > 0
            
            # 处理到达后的行为
            self._handle_arrival(distance, nav_mode, has_next)
            
            # 如果当前目标已清除且队列中还有航点，开始执行下一个
            if self.current_goal is None and self.waypoint_queue:
                next_goal = self.waypoint_queue.popleft()
                self.get_logger().info(f'📋 执行队列下一航点 [ID={next_goal.goal_id}]')
                self._set_current_goal(next_goal)
            return
        
        # 偏离检测补救: 进入3m后距离持续增大则强制切换
        if self._check_divergence(distance):
            # 如果当前目标已清除且队列中还有航点，开始执行下一个
            if self.current_goal is None and self.waypoint_queue:
                next_goal = self.waypoint_queue.popleft()
                self.get_logger().info(f'📋 执行队列下一航点 [ID={next_goal.goal_id}]')
                self._set_current_goal(next_goal)
            return
        
        self._check_timeout(distance)

    def _reset_divergence_monitor(self):
        """重置偏离检测状态"""
        self._divergence_monitoring = False
        self._divergence_min_distance = float('inf')
        self._divergence_increase_count = 0
        self._divergence_last_distance = float('inf')
    
    def _check_divergence(self, distance: float) -> bool:
        """
        偏离检测补救机制
        
        当 USV 进入监控半径 (默认3m) 后，持续追踪距离变化趋势:
        - 记录最小距离 (最近接近点)
        - 如果距离连续增大超过阈值次数，说明 USV 正在偏离目标
        - 此时强制判定为"偏离到达"，切换到下一个航点，避免绕圈或卡死
        
        Args:
            distance: 当前到目标的距离
            
        Returns:
            True 如果检测到偏离并已执行切换
        """
        if distance >= self._divergence_monitor_radius:
            # 还没进入监控区域
            if self._divergence_monitoring:
                # 曾经进入过监控区域后又离开了 → 立即判定为偏离
                self.get_logger().warn(
                    f'⚠️ 偏离检测: USV 已离开监控区域 [ID={self.current_goal_id}], '
                    f'当前距离={distance:.2f}m, 最近距离={self._divergence_min_distance:.2f}m')
                self._handle_divergence_arrival(distance)
                return True
            return False
        
        # 进入监控区域
        if not self._divergence_monitoring:
            self._divergence_monitoring = True
            self._divergence_min_distance = distance
            self._divergence_last_distance = distance
            self._divergence_increase_count = 0
            return False
        
        # 更新最小距离
        if distance < self._divergence_min_distance:
            self._divergence_min_distance = distance
        
        # 检查距离是否在增大 (加0.05m容差，过滤GPS抖动)
        if distance > self._divergence_last_distance + 0.05:
            self._divergence_increase_count += 1
        else:
            # 距离没有增大，重置计数 (但不重置最小距离)
            self._divergence_increase_count = 0
        
        self._divergence_last_distance = distance
        
        # 判定: 连续增大次数超过阈值，且最小距离未能进入到达阈值
        if (self._divergence_increase_count >= self._divergence_increase_threshold
                and self._divergence_min_distance > self.nav_arrival_threshold):
            self.get_logger().warn(
                f'⚠️ 偏离检测触发 [ID={self.current_goal_id}]: '
                f'距离连续增大{self._divergence_increase_count}次, '
                f'最近距离={self._divergence_min_distance:.2f}m > 到达阈值={self.nav_arrival_threshold:.2f}m, '
                f'当前距离={distance:.2f}m')
            self._handle_divergence_arrival(distance)
            return True
        
        return False
    
    def _handle_divergence_arrival(self, distance: float):
        """
        处理偏离到达 - 强制切换到下一航点或判定到达
        
        Args:
            distance: 当前距离
        """
        has_next = len(self.waypoint_queue) > 0
        
        self._reset_divergence_monitor()
        
        if has_next:
            # 还有后续航点，切换过去
            self._switch_to_next_waypoint(distance, "偏离补救切换")
        else:
            # 没有后续航点，判定为偏离到达最终目标
            self.get_logger().warn(
                f'⚠️ 偏离到达最终目标 [ID={self.current_goal_id}], '
                f'距离={distance:.2f}m (未达到{self.nav_arrival_threshold:.2f}m阈值)')
            self._publish_result(
                self.current_goal_id, success=True,
                message=f'偏离到达目标(距离={distance:.2f}m, 最近={self._divergence_min_distance:.2f}m, '
                        f'未达阈值={self.nav_arrival_threshold:.2f}m)')
            self._clear_current_goal()

    def _check_timeout(self, distance: float):
        """检查是否超时 - 支持进度感知的智能超时
        
        如果 USV 在持续接近目标点(有进度)，超时计时器会重置，
        避免远距离航点因固定超时而失败。
        同时设置硬性最大时间限制(timeout * max_multiplier)，防止无限导航。
        """
        now = self.get_clock().now()
        
        if self.goal_start_time is None:
            self.goal_start_time = now
        
        # 初始化进度追踪
        if self._timeout_original_start_time is None:
            self._timeout_original_start_time = now
            self._timeout_last_progress_time = now
            self._timeout_progress_last_distance = distance
        
        elapsed = (now - self.goal_start_time).nanoseconds / 1e9
        total_elapsed = (now - self._timeout_original_start_time).nanoseconds / 1e9
        
        # 定期检查进度
        progress_check_elapsed = (now - self._timeout_last_progress_time).nanoseconds / 1e9
        if progress_check_elapsed >= self._timeout_progress_check_interval:
            distance_decrease = self._timeout_progress_last_distance - distance
            if distance_decrease >= self._timeout_progress_min_decrease:
                # USV 正在接近目标 → 重置超时计时器
                self.get_logger().info(
                    f'🔄 导航有进度: 距离减少 {distance_decrease:.2f}m '
                    f'({self._timeout_progress_last_distance:.1f}→{distance:.1f}m), '
                    f'超时计时器已重置 [ID={self.current_goal_id}], '
                    f'总耗时={total_elapsed:.0f}s')
                self.goal_start_time = now
                elapsed = 0.0
            else:
                self.get_logger().info(
                    f'⚠️ 导航无明显进度: 距离变化={distance_decrease:.2f}m '
                    f'(需>{self._timeout_progress_min_decrease:.1f}m), '
                    f'剩余超时={self.current_goal.timeout - elapsed:.0f}s '
                    f'[ID={self.current_goal_id}]')
            # 更新检查点
            self._timeout_last_progress_time = now
            self._timeout_progress_last_distance = distance
        
        # 硬性最大时间限制
        hard_max = self.current_goal.timeout * self._timeout_max_multiplier
        
        if elapsed > self.current_goal.timeout:
            # 超时且无进度
            self.get_logger().warn(
                f'⏱️ 导航超时(无进度)! [ID={self.current_goal_id}] '
                f'无进度耗时={elapsed:.1f}s, 总耗时={total_elapsed:.1f}s, '
                f'剩余距离={distance:.2f}m')
            self._publish_result(
                self.current_goal_id,
                success=False,
                error_code=1,
                message=f'导航超时-无进度(无进度耗时={elapsed:.1f}s, '
                        f'总耗时={total_elapsed:.1f}s, 距离={distance:.2f}m)')
            self._clear_current_goal()
        elif total_elapsed > hard_max:
            # 绝对最大时间限制
            self.get_logger().warn(
                f'⏱️ 导航硬性超时! [ID={self.current_goal_id}] '
                f'总耗时={total_elapsed:.1f}s > 最大限制={hard_max:.0f}s, '
                f'剩余距离={distance:.2f}m')
            self._publish_result(
                self.current_goal_id,
                success=False,
                error_code=1,
                message=f'导航硬性超时(总耗时={total_elapsed:.1f}s, '
                        f'最大限制={hard_max:.0f}s, 距离={distance:.2f}m)')
            self._clear_current_goal()

    def _publish_feedback(self, distance: float, heading_error: float, estimated_time: float):
        """发布导航反馈 (包含航点队列状态和导航模式状态)"""
        feedback = NavigationFeedback()
        # goal_id 是 uint32，确保不为负数或 None
        feedback.goal_id = self.current_goal_id if self.current_goal_id is not None and self.current_goal_id >= 0 else 0
        feedback.distance_to_goal = distance
        feedback.heading_error = heading_error
        feedback.estimated_time = estimated_time
        feedback.timestamp = self.get_clock().now().to_msg()
        
        # 添加队列状态信息 (帮助 GS 决定是否预发送航点)
        feedback.queue_length = len(self.waypoint_queue)
        feedback.queue_capacity = self.waypoint_queue_size
        feedback.smooth_navigation = self.smooth_navigation
        
        # 添加导航模式状态
        nav_mode = getattr(self.current_goal, 'nav_mode', self.NAV_MODE_ASYNC)
        feedback.nav_mode = nav_mode
        
        # 同步模式: 计算当前到达质量
        if nav_mode == self.NAV_MODE_SYNC and len(self._arrival_check_samples) > 0:
            samples_in_threshold = sum(1 for d, _ in self._arrival_check_samples 
                                       if d < self.nav_arrival_threshold)
            feedback.arrival_quality = samples_in_threshold / len(self._arrival_check_samples)
        else:
            feedback.arrival_quality = 0.0
        
        # 旋转模式: 计算旋转进度
        if nav_mode == self.NAV_MODE_ROTATE and self._rotate_in_progress:
            rotate_circles = getattr(self.current_goal, 'maneuver_param', 1.0)
            target_angle = abs(rotate_circles * 2 * math.pi)
            if target_angle > 0:
                feedback.rotate_progress = min(1.0, abs(self._rotate_total_angle) / target_angle)
            else:
                feedback.rotate_progress = 1.0
        else:
            feedback.rotate_progress = 0.0
        
        self.feedback_pub.publish(feedback)

    def _publish_result(self, goal_id: int, success: bool, 
                        message: str, error_code: int = 0):
        """发布导航结果"""
        result = NavigationResult()
        # goal_id 是 uint32，确保不为负数或 None
        result.goal_id = goal_id if goal_id is not None and goal_id >= 0 else 0
        result.success = success
        # error_code 是 uint8，确保在有效范围内
        result.error_code = max(0, min(255, error_code))
        result.message = message
        result.timestamp = self.get_clock().now().to_msg()
        self.result_pub.publish(result)
    
    def _clear_current_goal(self):
        """清除当前导航任务"""
        self.current_goal = None
        self.current_goal_id = None
        self.goal_start_time = None
        if hasattr(self, '_last_distance'):
            delattr(self, '_last_distance')
        
        # 重置同步模式状态
        self._sync_mode_start_time = None
        self._arrival_check_samples = []
        
        # 重置旋转模式状态
        self._rotate_start_yaw = None
        self._rotate_total_angle = 0.0
        self._rotate_last_yaw = None
        self._rotate_in_progress = False
        
        # 重置延迟最终到达状态
        self._pending_final_arrival = False
        
        # 重置偏离检测状态
        self._reset_divergence_monitor()
        
        # 重置进度感知超时状态
        self._timeout_original_start_time = None
        self._timeout_last_progress_time = None
        self._timeout_progress_last_distance = None
    
    def _check_idle_protection(self):
        """
        检查空闲保护 - 当队列为空且长时间无新航点时发出警告
        
        这是一种通信断开保护机制:
        - 如果 USV 完成所有航点后长时间没有收到新航点
        - 可能意味着与 GS 的通信已断开
        - 发出警告日志提醒运维人员检查
        """
        if not self.idle_protection_enabled:
            return
        
        # 如果当前有任务或队列有待执行航点，不检查
        if self.current_goal is not None or len(self.waypoint_queue) > 0:
            return
        
        # 如果从未收到过航点，不检查
        if self._last_goal_received_time is None:
            return
        
        # 计算空闲时间
        now = self.get_clock().now()
        idle_sec = (now - self._last_goal_received_time).nanoseconds / 1e9
        
        # 超过空闲超时时间，发出警告
        if idle_sec > self.idle_timeout_sec:
            if not self._idle_warning_logged:
                self.get_logger().warn(
                    f'⚠️ 空闲保护: 已 {idle_sec:.1f}s 未收到新航点，'
                    f'队列为空，请检查与地面站的通信状态')
                self._idle_warning_logged = True
    
    def _calculate_distance(self, target_pose, current_pose):
        """
        计算到目标点的距离
        
        Args:
            target_pose (PoseStamped): 目标位置
            current_pose (PoseStamped): 当前位置
        
        Returns:
            float: 距离(米)
        """
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        
        if self.distance_mode == '3d':
            dz = target_pose.pose.position.z - current_pose.pose.position.z
            return math.sqrt(dx*dx + dy*dy + dz*dz)
        else:
            return math.sqrt(dx*dx + dy*dy)
    
    def _calculate_heading_error(self, target_pose, current_pose):
        """
        计算航向误差
        
        Args:
            target_pose (PoseStamped): 目标位置
            current_pose (PoseStamped): 当前位置
        
        Returns:
            float: 航向误差(弧度, 范围 -π 到 π)
        """
        # 计算期望航向
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        desired_yaw = math.atan2(dy, dx)
        
        # 获取当前航向 (从四元数转换)
        from tf_transformations import euler_from_quaternion
        q = current_pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # 计算误差并归一化到 [-π, π]
        error = desired_yaw - current_yaw
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        return error

    def destroy_node(self):
        """节点销毁时的资源清理"""
        if hasattr(self, 'nav_timer'):
            self.nav_timer.cancel()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = NavigateToPointNode()
    
    # 使用多线程执行器支持并发
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
