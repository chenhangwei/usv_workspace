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
from mavros_msgs.msg import PositionTarget
from common_interfaces.msg import NavigationGoal, NavigationFeedback, NavigationResult

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
        
        # ==================== 任务状态 ====================
        self._is_navigating = False       # 是否正在导航
        self._last_goal_time = 0.0        # 上次收到目标的时间
        self._idle_timeout = 5.0          # 空闲超时（秒），超时后停止记录
        self._record_count = 0            # 本次任务记录条数
        
        # ==================== 创建日志文件 ====================
        log_dir = Path.home() / 'usv_logs'
        log_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_file = log_dir / f'nav_log_{timestamp}.csv'
        
        # 创建 CSV 文件并写入表头
        self._csv_file = open(self._log_file, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp',
            'pose_x', 'pose_y', 'pose_yaw_deg',
            'velocity_vx', 'velocity_vy', 'velocity_speed', 'velocity_yaw_deg',
            'magnetometer_yaw_deg',
            'target_x', 'target_y', 'goal_id',
            'cmd_vx', 'cmd_vy', 'cmd_omega',
            'distance_to_goal', 'heading_error_deg',
            'yaw_diff_deg'  # 速度航向 - 磁力计航向
        ])
        
        self.get_logger().info(f'📝 日志文件: {self._log_file}')
        
        # ==================== 订阅者 ====================
        # 位姿 (来自 GPS 转换)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose_from_gps',
            self._pose_callback,
            qos_best_effort
        )
        
        # 飞控速度向量
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            'local_position/velocity_local',
            self._velocity_callback,
            qos_best_effort
        )
        
        # 磁力计位姿 (获取原始航向)
        self.mavros_pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose',
            self._mavros_pose_callback,
            qos_best_effort
        )
        
        # 导航目标
        self.nav_goal_sub = self.create_subscription(
            NavigationGoal,
            'set_usv_nav_goal',
            self._nav_goal_callback,
            qos_reliable
        )
        
        # 控制指令
        self.cmd_sub = self.create_subscription(
            PositionTarget,
            'setpoint_raw/local',
            self._cmd_callback,
            qos_best_effort
        )
        
        # 导航反馈
        self.feedback_sub = self.create_subscription(
            NavigationFeedback,
            'velocity_controller/feedback',
            self._feedback_callback,
            qos_best_effort
        )
        
        # 导航结果（用于检测任务完成）
        self.result_sub = self.create_subscription(
            NavigationResult,
            'navigation_result',
            self._result_callback,
            qos_reliable
        )
        
        # ==================== 定时器 ====================
        # 每 0.1 秒记录一次 (10Hz)
        self._log_timer = self.create_timer(0.1, self._log_data)
        
        # 每 5 秒刷新文件
        self._flush_timer = self.create_timer(5.0, self._flush_file)
        
        self.get_logger().info('='*50)
        self.get_logger().info('📊 日志收集节点已启动')
        self.get_logger().info(f'   日志路径: {self._log_file}')
        self.get_logger().info(f'   采样频率: 10 Hz')
        self.get_logger().info(f'   模式: 仅在导航任务执行时记录')
        self.get_logger().info(f'   空闲超时: {self._idle_timeout} 秒')
        self.get_logger().info('='*50)
    
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
        """导航目标回调"""
        self._target_x = msg.target_pose.pose.position.x
        self._target_y = msg.target_pose.pose.position.y
        self._goal_id = getattr(msg, 'goal_id', 0)
        
        # 收到导航目标，开始/继续记录
        current_time = self.get_clock().now().nanoseconds / 1e9
        if not self._is_navigating:
            self._is_navigating = True
            self._record_count = 0
            self.get_logger().info(f'🔴 开始记录导航日志 [目标 ID={self._goal_id}]')
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
    
    def _result_callback(self, msg: NavigationResult):
        """导航结果回调"""
        message = getattr(msg, 'message', '')
        goal_id = getattr(msg, 'goal_id', 0)
        
        # 检测最终到达（不是平滑切换）
        is_final_arrival = '成功到达' in message and '已通过' not in message
        
        if is_final_arrival and self._is_navigating:
            self._is_navigating = False
            self._csv_file.flush()
            self.get_logger().info(
                f'✅ 任务完成 [ID={goal_id}], 停止记录, '
                f'本次记录 {self._record_count} 条')
    
    def _log_data(self):
        """记录数据到 CSV（仅在导航任务进行时）"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 检查是否应该停止记录
        if self._is_navigating:
            idle_time = current_time - self._last_goal_time
            if idle_time > self._idle_timeout:
                self._is_navigating = False
                self.get_logger().info(
                    f'⏹️ 停止记录导航日志 (空闲 {idle_time:.1f}s), '
                    f'本次记录 {self._record_count} 条')
                self._csv_file.flush()  # 立即刷新到磁盘
                return
        else:
            # 未在导航中，不记录
            return
        
        timestamp = current_time
        
        # 计算距离 (如果没有从反馈获取)
        if self._distance_to_goal == 0.0 and self._goal_id > 0:
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
            f'{math.degrees(yaw_diff):.2f}'
        ])
        self._record_count += 1
    
    def _flush_file(self):
        """刷新文件到磁盘"""
        self._csv_file.flush()
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info(f'📁 日志已保存: {self._log_file}')
        self._csv_file.close()
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
