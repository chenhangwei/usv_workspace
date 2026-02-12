#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of log collector.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
USV 导航日志收集节点

收集所有导航相关数据，保存为 CSV 文件，便于事后分析。

收集内容:
- 当前位姿 (pose_from_gps)
- 飞控速度向量 (velocity_local)
- 磁力计航向 (local_position/pose)
- 导航目标 (set_usv_nav_goal)
- 控制指令 (setpoint_raw/local)

作者: Auto-generated
日期: 2026-01-25
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import Bool
from common_interfaces.msg import NavigationGoal, NavigationFeedback, NavigationResult, MpcDebug

import math
import csv
import os
from datetime import datetime
from pathlib import Path


class LogCollectorNode(Node):
    """导航日志收集节点"""
    
    def __init__(self):
        super().__init__('log_collector')
        
        # QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # ==================== 状态变量 ====================
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        
        self._velocity_vx = 0.0
        self._velocity_vy = 0.0
        self._velocity_speed = 0.0
        self._velocity_yaw = 0.0
        
        self._magnetometer_yaw = 0.0
        
        self._target_x = 0.0
        self._target_y = 0.0
        self._goal_id = 0
        
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_omega = 0.0
        
        self._distance_to_goal = 0.0
        self._heading_error_rad = 0.0  # 弧度
        self._nav_mode = 0               # 导航模式: 0=async, 1=sync, 2=rotate, 3=terminal
        
        # MPC Debug Info
        self._mpc_solve_time = 0.0
        self._mpc_cost = 0.0
        self._mpc_pred_theta = 0.0
        self._active_controller = ''
        
        # MPC 参数配置 (用于日志记录)
        self._mpc_param_q_pos = 0.0
        self._mpc_param_q_theta = 0.0
        self._mpc_param_r_w = 0.0
        self._mpc_param_r_dw = 0.0
        self._mpc_param_w_max = 0.0
        self._mpc_param_n_steps = 0
        # v5 新增参数
        self._mpc_param_tau_omega = 0.0
        self._mpc_param_q_cte = 0.0
        self._mpc_params_received = False  # 标记是否已收到参数
        
        # v6 新增: USV ID 和速度自适应 tau_omega 参数
        self._usv_id = ''
        self._adaptive_tau_enabled = False
        self._tau_omega_low_speed = 0.0
        self._tau_omega_high_speed = 0.0
        self._tau_speed_threshold_low = 0.0
        self._tau_speed_threshold_high = 0.0
        self._current_tau_omega = 0.0
        
        # v5 新增: 一阶惯性模型状态
        self._omega_actual = 0.0
        self._omega_cmd = 0.0
        self._cross_track_error = 0.0
        self._path_theta = 0.0
        
        # v8 新增: AMPC 在线辨识状态
        self._ampc_enabled = False
        self._ampc_tau_estimated = 0.0
        self._ampc_tau_confidence = 0.0
        self._ampc_omega_measured = 0.0
        self._ampc_saturation_ratio = 0.0
        self._ampc_heading_noise = 0.0
        self._ampc_rebuild_count = 0
        self._ampc_converged = False
        
        # 飞控状态 (用于记录模式切换)
        self._flight_mode = ''
        self._is_armed = False
        self._last_flight_mode = ''  # 用于检测模式切换
        self._mode_change_events = []  # 模式切换事件列表
        
        # ==================== 任务状态 ====================
        self._is_navigating = False       # 是否正在导航
        self._is_paused = False           # 是否处于暂停状态 (HOLD)
        self._last_goal_time = 0.0        # 上次收到目标的时间
        self._idle_timeout = 30.0         # 空闲超时（秒），只在非暂停状态下检查
        self._record_count = 0            # 本次任务记录条数
        self._goal_count = 0              # 本次任务处理的目标数
        

        # ==================== 日志文件句柄 ====================
        self._log_dir = Path.home() / 'usv_logs'
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._csv_file = None
        self._csv_writer = None
        self._current_log_path = None
        
        # ==================== 订阅者 ====================
        self.create_subscription(
            PoseStamped, 'local_position/pose_from_gps',
            self._pose_callback, qos_best_effort)
            
        self.create_subscription(
            TwistStamped, 'local_position/velocity_local',
            self._velocity_callback, qos_best_effort)
            
        self.create_subscription(
            PoseStamped, 'local_position/pose',
            self._mavros_pose_callback, qos_best_effort)
            
        self.create_subscription(
            NavigationGoal, 'set_usv_nav_goal',
            self._nav_goal_callback, qos_reliable)
            
        self.create_subscription(
            PositionTarget, 'setpoint_raw/local',
            self._cmd_callback, qos_best_effort)
            
        self.create_subscription(
            NavigationFeedback, 'navigation_feedback',
            self._feedback_callback, qos_reliable)
            
        self.create_subscription(
            NavigationResult, 'navigation_result',
            self._result_callback, qos_reliable)
            
        self.create_subscription(
            MpcDebug, 'velocity_controller/debug',
            self._debug_callback, qos_best_effort)
        
        # 飞控状态订阅 (用于记录模式切换)
        self.create_subscription(
            State, 'state',
            self._state_callback, qos_best_effort)
        
        # 导航控制订阅 (用于检测暂停/停止)
        self.create_subscription(
            Bool, 'cancel_navigation',
            self._cancel_navigation_callback, qos_reliable)
        self.create_subscription(
            Bool, 'stop_navigation',
            self._stop_navigation_callback, qos_reliable)
            
        # ==================== 定时器 ====================
        self.create_timer(0.1, self._log_data)
        
        self.get_logger().info('='*50)
        self.get_logger().info('📊 日志收集节点已启动')
        self.get_logger().info(f'   日志目录: {self._log_dir}')
        self.get_logger().info(f'   采样频率: 10 Hz')
        self.get_logger().info(f'   模式: 整个集群任务记为一个文件')
        self.get_logger().info(f'   结束条件: stop_navigation 或空闲超时 {self._idle_timeout:.0f}s')
        self.get_logger().info('='*50)
    
    def _start_new_log(self, goal_id, task_name=None):
        """开始新的日志文件"""
        self._close_current_log()
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 构建文件名：nav_log_{时间}_{任务名或ID}.csv
        if task_name:
            # 清理文件名中的非法字符
            safe_name = "".join(c for c in task_name if c.isalnum() or c in (' ', '_', '-')).strip()
            safe_name = safe_name.replace(' ', '_')
            filename = f'nav_log_{timestamp}_{safe_name}.csv'
        else:
            filename = f'nav_log_{timestamp}_goal_{goal_id}.csv'
            
        self._current_log_path = self._log_dir / filename
        
        try:
            self._csv_file = open(self._current_log_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            
            # 写入 MPC 参数信息作为注释行 (便于后续分析时追溯参数配置)
            if self._mpc_params_received:
                self._csv_file.write(f'# MPC Parameters Configuration (v14)\n')
                self._csv_file.write(f'# USV ID: {self._usv_id}\n')
                self._csv_file.write(f'# Q_pos (position weight): {self._mpc_param_q_pos:.2f}\n')
                self._csv_file.write(f'# Q_theta (heading weight): {self._mpc_param_q_theta:.2f}\n')
                self._csv_file.write(f'# R_w (steering weight): {self._mpc_param_r_w:.2f}\n')
                self._csv_file.write(f'# R_dw (steering rate weight): {self._mpc_param_r_dw:.2f}\n')
                self._csv_file.write(f'# w_max (max angular velocity): {self._mpc_param_w_max:.3f} rad/s\n')
                self._csv_file.write(f'# N_steps (prediction horizon): {self._mpc_param_n_steps}\n')
                self._csv_file.write(f'# tau_omega (base steering time constant): {self._mpc_param_tau_omega:.2f} s\n')
                self._csv_file.write(f'# Q_cte (cross-track error weight): {self._mpc_param_q_cte:.2f}\n')
                self._csv_file.write(f'# --- v6 Adaptive Tau Parameters ---\n')
                self._csv_file.write(f'# adaptive_tau_enabled: {self._adaptive_tau_enabled}\n')
                self._csv_file.write(f'# tau_omega_low_speed: {self._tau_omega_low_speed:.2f} s\n')
                self._csv_file.write(f'# tau_omega_high_speed: {self._tau_omega_high_speed:.2f} s\n')
                self._csv_file.write(f'# tau_speed_threshold_low: {self._tau_speed_threshold_low:.2f} m/s\n')
                self._csv_file.write(f'# tau_speed_threshold_high: {self._tau_speed_threshold_high:.2f} m/s\n')
                self._csv_file.write(f'# --- v8 AMPC Parameters ---\n')
                self._csv_file.write(f'# ampc_enabled: {self._ampc_enabled}\n')
                self._csv_file.write(f'#\n')
            
            # 写入表头
            self._csv_writer.writerow([
                'timestamp',
                'pose_x', 'pose_y', 'pose_yaw_deg',
                'velocity_vx', 'velocity_vy', 'velocity_speed', 'velocity_yaw_deg',
                'magnetometer_yaw_deg',
                'target_x', 'target_y', 'goal_id',
                'cmd_vx', 'cmd_vy', 'cmd_omega',
                'distance_to_goal', 'heading_error_deg',
                'yaw_diff_deg',
                'mpc_solve_time_ms', 'mpc_cost', 'mpc_pred_theta_deg', 'active_ctrl',
                # v5 新增字段
                'omega_actual', 'omega_cmd', 'cross_track_error', 'path_theta_deg',
                'flight_mode', 'armed',
                # v6 新增字段
                'current_tau_omega',
                # v8 新增: AMPC 在线辨识字段
                'ampc_enabled', 'ampc_tau_estimated', 'ampc_tau_confidence',
                'ampc_omega_measured', 'ampc_saturation_ratio',
                'ampc_heading_noise', 'ampc_rebuild_count', 'ampc_converged',
                # v14 新增字段
                'nav_mode'
            ])
            
            # 清空模式切换事件列表
            self._mode_change_events = []
            
            self._record_count = 0
            self.get_logger().info(f'📝 新建日志文件: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'创建日志文件失败: {e}')
            self._csv_file = None
            
    def _close_current_log(self):
        """关闭当前日志文件"""
        if self._csv_file:
            try:
                # 在文件末尾写入模式切换摘要
                if self._mode_change_events:
                    self._csv_file.write(f'\n# ==================== 模式切换事件 ====================\n')
                    for event in self._mode_change_events:
                        self._csv_file.write(
                            f'# {event["timestamp"]:.3f}: {event["from_mode"]} → {event["to_mode"]} '
                            f'(armed={event["armed"]})\n'
                        )
                    self._csv_file.write(f'# 共 {len(self._mode_change_events)} 次模式切换\n')
                
                self._csv_file.flush()
                self._csv_file.close()
                
                # 日志统计
                mode_changes = len(self._mode_change_events)
                self.get_logger().info(
                    f'📁 日志已保存: {self._current_log_path.name} '
                    f'({self._record_count} 条, {mode_changes} 次模式切换)')
            except Exception as e:
                self.get_logger().error(f'关闭日志文件失败: {e}')
            finally:
                self._csv_file = None
                self._csv_writer = None
                self._current_log_path = None
                self._mode_change_events = []

    def _pose_callback(self, msg: PoseStamped):

        """位姿回调"""
        self._pose_x = msg.pose.position.x
        self._pose_y = msg.pose.position.y
        
        # 从四元数提取 yaw
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self._pose_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def _velocity_callback(self, msg: TwistStamped):
        """速度向量回调"""
        self._velocity_vx = msg.twist.linear.x
        self._velocity_vy = msg.twist.linear.y
        self._velocity_speed = math.sqrt(
            self._velocity_vx ** 2 + self._velocity_vy ** 2
        )
        
        if self._velocity_speed > 0.05:
            self._velocity_yaw = math.atan2(
                self._velocity_vy, self._velocity_vx
            )
    
    def _mavros_pose_callback(self, msg: PoseStamped):
        """MAVROS 原始位姿回调 (获取磁力计航向)"""
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self._magnetometer_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def _nav_goal_callback(self, msg: NavigationGoal):
        """导航目标回调
        
        整个集群任务使用同一个日志文件，不因中间航点切换而分割。
        日志只在首次收到目标时创建，在 stop_navigation 或空闲超时时关闭。
        """
        self._target_x = msg.target_pose.pose.position.x
        self._target_y = msg.target_pose.pose.position.y
        self._goal_id = getattr(msg, 'goal_id', 0)
        self._nav_mode = getattr(msg, 'nav_mode', 0)
        task_name = getattr(msg, 'task_name', None)
        
        # 收到导航目标，开始/继续记录
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 收到新目标时，重置暂停状态
        if self._is_paused:
            self._is_paused = False
            self.get_logger().info('▶️ 导航恢复，继续记录日志')
        
        if not self._is_navigating:
            # 首次收到目标：开始新日志文件
            self._is_navigating = True
            self._is_paused = False
            self._record_count = 0
            self._goal_count = 1
            self.get_logger().info(f'🔴 开始记录导航日志 [目标 ID={self._goal_id}]')
            self._start_new_log(self._goal_id, task_name)
        else:
            # 后续目标：继续使用同一个日志文件，不切分
            self._goal_count = getattr(self, '_goal_count', 0) + 1
            self.get_logger().info(
                f'📍 新目标 [ID={self._goal_id}], '
                f'继续在同一日志文件中记录 (第{self._goal_count}个目标)')
             
        self._current_task_name = task_name 
        self._last_goal_time = current_time
    
    def _cmd_callback(self, msg: PositionTarget):
        """控制指令回调"""
        self._cmd_vx = msg.velocity.x
        self._cmd_vy = msg.velocity.y
        self._cmd_omega = msg.yaw_rate
    
    def _feedback_callback(self, msg: NavigationFeedback):
        """导航反馈回调"""
        self._distance_to_goal = msg.distance_to_goal
        self._heading_error_rad = getattr(msg, 'heading_error', 0.0)  # 弧度
        
        # 收到反馈也说明正在导航
        if self._is_navigating:
            self._last_goal_time = self.get_clock().now().nanoseconds / 1e9
    
    def _debug_callback(self, msg: MpcDebug):
        """调试信息回调"""
        self._mpc_solve_time = msg.solve_time_ms
        self._mpc_cost = msg.cost
        self._mpc_pred_theta = msg.mpc_pred_theta
        self._active_controller = msg.active_controller
        
        # 接收 MPC 参数配置
        self._mpc_param_q_pos = getattr(msg, 'param_q_pos', 0.0)
        self._mpc_param_q_theta = getattr(msg, 'param_q_theta', 0.0)
        self._mpc_param_r_w = getattr(msg, 'param_r_w', 0.0)
        self._mpc_param_r_dw = getattr(msg, 'param_r_dw', 0.0)
        self._mpc_param_w_max = getattr(msg, 'param_w_max', 0.0)
        self._mpc_param_n_steps = getattr(msg, 'param_n_steps', 0)
        # v5 新增参数
        self._mpc_param_tau_omega = getattr(msg, 'param_tau_omega', 0.0)
        self._mpc_param_q_cte = getattr(msg, 'param_q_cte', 0.0)
        
        # v6 新增: USV ID 和速度自适应 tau_omega 参数
        self._usv_id = getattr(msg, 'usv_id', '')
        self._adaptive_tau_enabled = getattr(msg, 'adaptive_tau_enabled', False)
        self._tau_omega_low_speed = getattr(msg, 'tau_omega_low_speed', 0.0)
        self._tau_omega_high_speed = getattr(msg, 'tau_omega_high_speed', 0.0)
        self._tau_speed_threshold_low = getattr(msg, 'tau_speed_threshold_low', 0.0)
        self._tau_speed_threshold_high = getattr(msg, 'tau_speed_threshold_high', 0.0)
        self._current_tau_omega = getattr(msg, 'current_tau_omega', 0.0)
        
        # v5 新增: 一阶惯性模型状态
        self._omega_actual = getattr(msg, 'omega_actual', 0.0)
        self._omega_cmd = getattr(msg, 'omega_cmd', 0.0)
        self._cross_track_error = getattr(msg, 'cross_track_error', 0.0)
        self._path_theta = getattr(msg, 'path_theta', 0.0)
        
        # v8 新增: AMPC 在线辨识状态
        self._ampc_enabled = getattr(msg, 'ampc_enabled', False)
        self._ampc_tau_estimated = getattr(msg, 'ampc_tau_estimated', 0.0)
        self._ampc_tau_confidence = getattr(msg, 'ampc_tau_confidence', 0.0)
        self._ampc_omega_measured = getattr(msg, 'ampc_omega_measured', 0.0)
        self._ampc_saturation_ratio = getattr(msg, 'ampc_saturation_ratio', 0.0)
        self._ampc_heading_noise = getattr(msg, 'ampc_heading_noise', 0.0)
        self._ampc_rebuild_count = getattr(msg, 'ampc_rebuild_count', 0)
        self._ampc_converged = getattr(msg, 'ampc_converged', False)
        
        self._mpc_params_received = True

    def _state_callback(self, msg: State):
        """飞控状态回调 - 记录模式切换"""
        new_mode = msg.mode
        self._is_armed = msg.armed
        
        # 检测模式切换
        if self._flight_mode != '' and new_mode != self._flight_mode:
            current_time = self.get_clock().now().nanoseconds / 1e9
            event = {
                'timestamp': current_time,
                'from_mode': self._flight_mode,
                'to_mode': new_mode,
                'armed': self._is_armed
            }
            self._mode_change_events.append(event)
            self.get_logger().info(
                f'🔄 模式切换: {self._flight_mode} → {new_mode} (armed={self._is_armed})'
            )
        
        self._last_flight_mode = self._flight_mode
        self._flight_mode = new_mode

    def _cancel_navigation_callback(self, msg: Bool):
        """暂停导航回调 (HOLD 按钮)
        
        暂停任务但不结束日志记录，任务可以恢复
        """
        if msg.data and self._is_navigating:
            self._is_paused = True
            self.get_logger().info('⏸️ 导航暂停，日志记录继续...')

    def _stop_navigation_callback(self, msg: Bool):
        """停止导航回调 (集群 STOP 按钮)
        
        任务完全结束，关闭日志文件
        """
        if msg.data and self._is_navigating:
            self._is_navigating = False
            self._is_paused = False
            self.get_logger().info(
                f'⏹️ 任务停止 (手动终止), 停止记录, '
                f'本次记录 {self._record_count} 条, {self._goal_count} 个目标')
            self._close_current_log()

    def _result_callback(self, msg: NavigationResult):
        """导航结果回调
        
        整个集群任务（多航点）使用同一个日志文件。
        中间航点到达只记录事件，不关闭日志。
        日志关闭只由 stop_navigation 或空闲超时触发。
        """
        message = getattr(msg, 'message', '')
        goal_id = getattr(msg, 'goal_id', 0)
        
        # 检测暂停（不结束日志）
        is_paused = '已暂停' in message or '等待恢复' in message
        
        # 检测任务停止（手动终止）
        is_stopped = '已停止' in message or '任务停止' in message
        
        # 检测航点到达（中间或最终）
        is_arrival = '成功到达' in message and '已通过' not in message
        is_waypoint_passed = '已通过' in message
        
        if is_paused and self._is_navigating:
            # 暂停状态：不关闭日志
            self._is_paused = True
            self.get_logger().info(f'⏸️ 任务暂停 [ID={goal_id}], 日志记录继续...')
        elif is_stopped and self._is_navigating:
            # 手动停止：关闭日志
            self._is_navigating = False
            self._is_paused = False
            self.get_logger().info(
                f'⏹️ 任务停止 [ID={goal_id}], 停止记录, '
                f'本次记录 {self._record_count} 条')
            self._close_current_log()
        elif is_arrival and self._is_navigating:
            # 航点到达：只记录事件，不关闭日志
            # 后续可能还有更多航点，由空闲超时自动关闭
            self._last_goal_time = self.get_clock().now().nanoseconds / 1e9
            goal_count = getattr(self, '_goal_count', 0)
            self.get_logger().info(
                f'🏁 目标到达 [ID={goal_id}], 日志继续记录 '
                f'(已处理 {goal_count} 个目标, {self._record_count} 条记录)')
        elif is_waypoint_passed and self._is_navigating:
            # 中间航点通过：刷新时间戳，继续记录
            self._last_goal_time = self.get_clock().now().nanoseconds / 1e9
    
    def _log_data(self):
        """记录数据到 CSV（仅在导航任务进行时）"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 检查是否应该停止记录
        if self._is_navigating:
            # 暂停状态下不检查超时，只有正常导航时才检查
            if not self._is_paused:
                idle_time = current_time - self._last_goal_time
                if idle_time > self._idle_timeout:
                    self._is_navigating = False
                    self._is_paused = False
                    self.get_logger().info(
                        f'⏹️ 停止记录导航日志 (空闲 {idle_time:.1f}s), '
                        f'本次记录 {self._record_count} 条, {self._goal_count} 个目标')
                    self._close_current_log()
                    return
        else:
            # 未在导航中，不记录
            return
            
        if self._csv_writer is None:
            return
        
        timestamp = current_time
        
        # 计算距离 (始终使用当前目标与位姿的几何距离，避免切点时序不一致)
        if self._goal_id > 0:
            dx = self._target_x - self._pose_x
            dy = self._target_y - self._pose_y
            self._distance_to_goal = math.sqrt(dx * dx + dy * dy)
        
        # 计算航向差异
        yaw_diff = self._velocity_yaw - self._magnetometer_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        self._csv_writer.writerow([
            f'{timestamp:.3f}',
            f'{self._pose_x:.4f}',
            f'{self._pose_y:.4f}',
            f'{math.degrees(self._pose_yaw):.2f}',
            f'{self._velocity_vx:.4f}',
            f'{self._velocity_vy:.4f}',
            f'{self._velocity_speed:.4f}',
            f'{math.degrees(self._velocity_yaw):.2f}',
            f'{math.degrees(self._magnetometer_yaw):.2f}',
            f'{self._target_x:.4f}',
            f'{self._target_y:.4f}',
            f'{self._goal_id}',
            f'{self._cmd_vx:.4f}',
            f'{self._cmd_vy:.4f}',
            f'{self._cmd_omega:.4f}',
            f'{self._distance_to_goal:.4f}',
            f'{math.degrees(self._heading_error_rad):.2f}',  # 弧度转度数
            f'{math.degrees(yaw_diff):.2f}',
            f'{self._mpc_solve_time:.2f}',
            f'{self._mpc_cost:.4f}',
            f'{math.degrees(self._mpc_pred_theta):.2f}',
            f'{self._active_controller}',
            # v5 新增字段
            f'{self._omega_actual:.4f}',
            f'{self._omega_cmd:.4f}',
            f'{self._cross_track_error:.4f}',
            f'{math.degrees(self._path_theta):.2f}',
            f'{self._flight_mode}',
            f'{1 if self._is_armed else 0}',
            # v6 新增字段
            f'{self._current_tau_omega:.3f}',
            # v8 新增: AMPC 在线辨识字段
            f'{1 if self._ampc_enabled else 0}',
            f'{self._ampc_tau_estimated:.4f}',
            f'{self._ampc_tau_confidence:.3f}',
            f'{self._ampc_omega_measured:.4f}',
            f'{self._ampc_saturation_ratio:.3f}',
            f'{self._ampc_heading_noise:.4f}',
            f'{self._ampc_rebuild_count}',
            f'{1 if self._ampc_converged else 0}',
            # v14 新增字段
            f'{self._nav_mode}'
        ])
        self._record_count += 1
    
    def _flush_file(self):
        """刷新文件到磁盘"""
        if self._csv_file:
            self._csv_file.flush()
    
    def destroy_node(self):
        """清理资源"""
        self._close_current_log()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LogCollectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
