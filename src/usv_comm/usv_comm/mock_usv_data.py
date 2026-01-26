#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of mock usv data.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
虚拟数据发布节点 - 模拟实际USV运行数据

功能：
1. 模拟 GPS 位置数据 (global_position/global)
2. 模拟本地位置数据 (local_position/pose)
3. 模拟 MAVROS 状态 (state)
4. 模拟 Home Position (home_position/home)
5. 接收导航目标点并模拟移动

使用方法：
    ros2 run usv_comm mock_usv_data --ros-args -p namespace:=usv_01
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, HomePosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import math

# 导入common_utils工具
from common_utils import ParamLoader


class MockUSVData(Node):
    """虚拟USV数据发布节点"""

    def __init__(self):
        super().__init__('mock_usv_data')
        
        # 创建参数加载器
        param_loader = ParamLoader(self)
        
        # 参数
        self.declare_parameter('namespace', 'usv_01')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('move_speed', 1.0)  # m/s
        
        # GPS 原点配置 - 使用统一加载方法
        gps_origin = param_loader.load_gps_origin(
            lat_param='gps_origin_lat',
            lon_param='gps_origin_lon',
            alt_param='gps_origin_alt'
        )
        self.origin_lat = gps_origin['lat']
        self.origin_lon = gps_origin['lon']
        self.origin_alt = gps_origin['alt']
        
        # 获取参数
        self.namespace = self.get_parameter('namespace').value
        publish_rate = self.get_parameter('publish_rate').value
        self.move_speed = self.get_parameter('move_speed').value
        
        # 当前状态
        self.current_x = self.get_parameter('initial_x').value
        self.current_y = self.get_parameter('initial_y').value
        self.current_z = 0.0
        
        # 目标状态
        self.target_x = self.current_x
        self.target_y = self.current_y
        self.target_z = self.current_z
        
        # MAVROS 状态
        self.mavros_state = State()
        self.mavros_state.connected = True
        self.mavros_state.armed = True
        self.mavros_state.guided = True
        self.mavros_state.mode = "GUIDED"
        
        # QoS 配置
        qos_sensor = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        qos_state = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 发布器
        self.gps_pub = self.create_publisher(
            NavSatFix,
            f'/{self.namespace}/global_position/global',
            qos_sensor
        )
        
        self.local_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{self.namespace}/local_position/pose',
            qos_sensor
        )
        
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            f'/{self.namespace}/local_position/velocity_local',
            qos_sensor
        )
        
        self.state_pub = self.create_publisher(
            State,
            f'/{self.namespace}/state',
            qos_state
        )
        
        self.home_pub = self.create_publisher(
            HomePosition,
            f'/{self.namespace}/home_position/home',
            qos_state
        )
        
        # 订阅目标点
        self.target_sub = self.create_subscription(
            PoseStamped,
            f'/{self.namespace}/set_usv_target_position',
            self.target_callback,
            10
        )
        
        # 定时器
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_data)
        
        # 发布 Home Position（只发一次）
        self.create_timer(1.0, self.publish_home_once)
        self.home_published = False
        
        self.get_logger().info(
            f"🎮 [虚拟USV数据] 已启动\n"
            f"  ├─ 命名空间: {self.namespace}\n"
            f"  ├─ GPS原点: ({self.origin_lat:.7f}°, {self.origin_lon:.7f}°)\n"
            f"  ├─ 初始位置: ({self.current_x:.1f}, {self.current_y:.1f}) m\n"
            f"  └─ 移动速度: {self.move_speed:.1f} m/s"
        )
    
    def target_callback(self, msg: PoseStamped):
        """接收目标点回调"""
        old_target = (self.target_x, self.target_y, self.target_z)
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_z = msg.pose.position.z
        
        distance = math.sqrt(
            (self.target_x - self.current_x)**2 +
            (self.target_y - self.current_y)**2
        )
        
        self.get_logger().info(
            f"🎯 [虚拟USV] 收到新目标点\n"
            f"  ├─ 目标: ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}) m\n"
            f"  ├─ 当前: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) m\n"
            f"  └─ 距离: {distance:.2f} m"
        )
    
    def publish_home_once(self):
        """发布 Home Position（只发一次）"""
        if not self.home_published:
            home_msg = HomePosition()
            home_msg.header.stamp = self.get_clock().now().to_msg()
            home_msg.header.frame_id = 'map'
            home_msg.geo.latitude = self.origin_lat
            home_msg.geo.longitude = self.origin_lon
            home_msg.geo.altitude = self.origin_alt
            home_msg.position.x = 0.0
            home_msg.position.y = 0.0
            home_msg.position.z = 0.0
            
            self.home_pub.publish(home_msg)
            self.home_published = True
            
            self.get_logger().info(
                f"🏠 [虚拟USV] 发布 Home Position\n"
                f"  ├─ GPS: ({self.origin_lat:.7f}°, {self.origin_lon:.7f}°)\n"
                f"  └─ Local: (0.0, 0.0, 0.0) m"
            )
    
    def publish_data(self):
        """发布虚拟数据"""
        # 更新位置（模拟移动）
        self._update_position()
        
        # 发布 GPS 位置
        self._publish_gps()
        
        # 发布本地位置
        self._publish_local_pose()
        
        # 发布速度
        self._publish_velocity()
        
        # 发布状态
        self._publish_state()
    
    def _update_position(self):
        """更新位置（模拟移动到目标点）"""
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dz = self.target_z - self.current_z
        
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if distance > 0.1:  # 距离大于10cm才移动
            # 计算移动步长
            dt = 1.0 / 10.0  # 10Hz
            max_step = self.move_speed * dt
            
            if distance > max_step:
                # 按速度移动
                ratio = max_step / distance
                self.current_x += dx * ratio
                self.current_y += dy * ratio
                self.current_z += dz * ratio
            else:
                # 直接到达目标
                self.current_x = self.target_x
                self.current_y = self.target_y
                self.current_z = self.target_z
    
    def _publish_gps(self):
        """发布 GPS 位置"""
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'map'
        gps_msg.status.status = 0  # GPS Fix
        gps_msg.status.service = 1  # GPS Service
        
        # XYZ → GPS
        lat, lon, alt = self._xyz_to_gps(self.current_x, self.current_y, self.current_z)
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        
        gps_msg.position_covariance_type = 1  # COVARIANCE_TYPE_APPROXIMATED
        
        self.gps_pub.publish(gps_msg)
    
    def _publish_local_pose(self):
        """发布本地位置"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.current_x
        pose_msg.pose.position.y = self.current_y
        pose_msg.pose.position.z = self.current_z
        pose_msg.pose.orientation.w = 1.0
        
        self.local_pose_pub.publish(pose_msg)
    
    def _publish_velocity(self):
        """发布速度"""
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'map'
        
        # 计算速度向量
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance > 0.1:
            vel_msg.twist.linear.x = (dx / distance) * self.move_speed
            vel_msg.twist.linear.y = (dy / distance) * self.move_speed
        
        self.velocity_pub.publish(vel_msg)
    
    def _publish_state(self):
        """发布 MAVROS 状态"""
        self.mavros_state.header.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(self.mavros_state)
    
    def _xyz_to_gps(self, x, y, z):
        """XYZ → GPS 转换"""
        dlat = y / 111320.0
        lat = self.origin_lat + dlat
        
        dlon = x / (111320.0 * math.cos(math.radians(self.origin_lat)))
        lon = self.origin_lon + dlon
        
        alt = z + self.origin_alt
        
        return lat, lon, alt

    def destroy_node(self):
        """节点销毁时的资源清理"""
        if hasattr(self, 'timer'):
            self.timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MockUSVData()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
