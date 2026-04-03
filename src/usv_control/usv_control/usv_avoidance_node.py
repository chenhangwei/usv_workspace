#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# ROS 2 Node implementation: Usv Avoidance Node.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
无人船避障节点

该节点负责处理无人船的避障逻辑。通过订阅雷达数据、飞控状态、当前位置和目标位置，
当检测到障碍物时，自动调整目标点以避开障碍物，确保无人船安全航行。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import PoseStamped, Point
import math
from std_msgs.msg import Bool
from mavros_msgs.msg import State, PositionTarget
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from common_interfaces.msg import VisionObstacleArray


class UsvAvoidanceNode(Node):
    """
    无人船避障节点类
    
    该节点实现基于超声波雷达和双目视觉融合的避障功能，当检测到障碍物时，
    自动调整无人船的目标位置以避开障碍物。
    视觉数据提供障碍物方位信息，使绕障方向更精准。
    """

    def __init__(self):
        """初始化无人船避障节点"""
        super().__init__('usv_avoidance_node')

        # 创建 QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        qos_reliable = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # 从参数服务器加载避障距离阈值，默认值为 1.2 米
        self.declare_parameter('in_distance_value', 1.2)
        self.in_distance_value = self.get_parameter('in_distance_value').get_parameter_value().double_value

        # 视觉避障触发距离 (米)
        self.declare_parameter('vision_distance_threshold', 5.0)
        # 避障偏移距离 (米)
        self.declare_parameter('avoidance_offset', 2.0)

        # 订阅飞控状态信息
        self.state_sub = self.create_subscription(
            State, 'state', self.state_callback, qos_best_effort)
        
        # 订阅超声波雷达数据
        self.radar_sub = self.create_subscription(
            Range, 'ultrasonic_radar_range', self.radar_callback, qos_best_effort)
        
        # 订阅视觉障碍物数据
        self.vision_sub = self.create_subscription(
            VisionObstacleArray, 'vision_obstacles', self.vision_callback, qos_best_effort)
        
        # 订阅当前目标点
        self.target_sub = self.create_subscription(
            PositionTarget, 'setpoint_raw/local', self.target_callback, qos_best_effort)
        
        # 订阅当前位置（使用 GPS 转换的统一坐标系）
        self.position_sub = self.create_subscription(
            PoseStamped, 'local_position/pose_from_gps', self.position_callback, qos_best_effort)

        # 发布调整后的目标点
        self.target_pub = self.create_publisher(PositionTarget, 'avoidance_position', 10)

        # 发布避障状态（true/false）
        self.avoidance_flag = self.create_publisher(Bool, 'avoidance_flag', qos_reliable)
    
        # 定时运行避障程序
        self.avoidance_timer = self.create_timer(0.2, self.avoidance_run)

        # 初始化状态变量
        self.current_laserscan = Range()     # 当前雷达数据
        self.current_state = State()         # 当前飞控设备状态
        self.current_position = Point()      # 当前位置坐标
        self.current_target = Point()        # 目标位置坐标
        self.obstacle_detected = False       # 避障动作标志
        self.vision_obstacles = []           # 最近一帧视觉检测结果

        self.get_logger().info('USV 避障节点已启动 (雷达+视觉融合)')
        self.get_logger().info(f'雷达避障距离阈值: {self.in_distance_value} 米')
        self.get_logger().info(
            f'视觉避障距离阈值: '
            f'{self.get_parameter("vision_distance_threshold").value} 米')

    def radar_callback(self, msg):
        """
        雷达数据回调函数
        
        Args:
            msg (Range): 包含雷达距离信息的消息
        """
        if isinstance(msg, Range):
            self.current_laserscan = msg         

    def state_callback(self, msg):
        """
        飞控状态回调函数
        
        Args:
            msg (State): 包含飞控状态信息的消息
        """
        if isinstance(msg, State):
            self.current_state = msg

    def position_callback(self, msg):
        """
        位置信息回调函数
        
        Args:
            msg (PoseStamped): 包含当前位置信息的消息
        """
        if isinstance(msg, PoseStamped):
            self.current_position = msg.pose.position

    def vision_callback(self, msg):
        """
        视觉障碍物回调函数
        
        Args:
            msg (VisionObstacleArray): 包含视觉检测到的障碍物列表
        """
        if isinstance(msg, VisionObstacleArray):
            self.vision_obstacles = msg.obstacles

    def target_callback(self, msg):
        """
        目标点回调函数
        
        Args:
            msg (PositionTarget): 包含目标点信息的消息
        """
        if isinstance(msg, PositionTarget):
            self.current_target = msg.position

    def avoidance_run(self):
        """
        避障主逻辑函数 (雷达+视觉融合)
        
        融合策略: 保守模式 — 雷达或视觉任一检测到障碍即触发避障。
        视觉提供方位信息，使绕障方向更精准 (朝障碍物反方向偏移)。
        雷达无方位信息时沿用原始右偏策略。
        """
        try:
            # 获取最新参数
            self.in_distance_value = self.get_parameter(
                "in_distance_value").get_parameter_value().double_value
            vision_threshold = self.get_parameter(
                "vision_distance_threshold").get_parameter_value().double_value
            avoidance_offset = self.get_parameter(
                "avoidance_offset").get_parameter_value().double_value
            
            # 检查飞控是否已连接、已解锁且处于GUIDED模式
            if (not self.current_state.connected
                    or not self.current_state.armed
                    or self.current_state.mode != "GUIDED"):
                return

            # ---- 雷达检测 ----
            radar_obstacle = self.current_laserscan.range < self.in_distance_value
            radar_distance = (self.current_laserscan.range
                              if radar_obstacle else float('inf'))

            # ---- 视觉检测 — 取最近的障碍物 ----
            vision_distance = float('inf')
            vision_bearing = 0.0
            for obs in self.vision_obstacles:
                if obs.distance < vision_threshold and obs.distance < vision_distance:
                    vision_distance = obs.distance
                    vision_bearing = obs.bearing

            vision_obstacle = vision_distance < vision_threshold

            # ---- 融合判定: 任一源触发即避障 ----
            self.obstacle_detected = radar_obstacle or vision_obstacle

            # 如果检测到障碍物且位置信息有效，则计算避障目标点
            if self.obstacle_detected and self.current_position and self.current_target:
                # 计算当前到目标的方向
                dx = self.current_target.x - self.current_position.x
                dy = self.current_target.y - self.current_position.y
                
                # 避免除零错误
                if dx == 0 and dy == 0:
                    heading = 0.0
                else:
                    heading = math.atan2(dy, dx)
                
                if vision_obstacle and vision_distance < radar_distance:
                    # 视觉检测到更近的障碍 → 利用方位角信息精准绕障
                    # 向障碍物反方向偏移
                    avoid_direction = heading - vision_bearing
                    avoid_x = (self.current_position.x
                               + avoidance_offset * math.cos(avoid_direction))
                    avoid_y = (self.current_position.y
                               + avoidance_offset * math.sin(avoid_direction))
                    self.get_logger().info(
                        f'视觉避障: 障碍距离={vision_distance:.1f}m '
                        f'方位={math.degrees(vision_bearing):.0f}° → '
                        f'目标点({avoid_x:.2f}, {avoid_y:.2f})')
                else:
                    # 仅雷达触发 → 沿用原始右偏策略
                    avoid_x = (self.current_position.x
                               + avoidance_offset * math.sin(heading))
                    avoid_y = (self.current_position.y
                               - avoidance_offset * math.cos(heading))
                    self.get_logger().info(
                        f'雷达避障: 障碍距离={radar_distance:.2f}m → '
                        f'目标点({avoid_x:.2f}, {avoid_y:.2f})')

                # 构造并发布避障目标点消息
                msg = PositionTarget()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                msg.type_mask = (
                    PositionTarget.IGNORE_VX |
                    PositionTarget.IGNORE_VY |
                    PositionTarget.IGNORE_VZ |
                    PositionTarget.IGNORE_AFX |
                    PositionTarget.IGNORE_AFY |
                    PositionTarget.IGNORE_AFZ |
                    PositionTarget.FORCE |
                    PositionTarget.IGNORE_YAW |
                    PositionTarget.IGNORE_YAW_RATE
                )
                msg.position.x = avoid_x
                msg.position.y = avoid_y 
                msg.position.z = 0.0

                self.target_pub.publish(msg)

            # 发布避障状态
            temp = Bool()
            temp.data = self.obstacle_detected
            self.avoidance_flag.publish(temp)

        except Exception as e:
            self.get_logger().error(f'避障程序运行异常: {str(e)}')

    def destroy_node(self):
        """节点销毁时的资源清理"""
        if hasattr(self, 'avoidance_timer'):
            self.avoidance_timer.cancel()
        super().destroy_node()


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = UsvAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()