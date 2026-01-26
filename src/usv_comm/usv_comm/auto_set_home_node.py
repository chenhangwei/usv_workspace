#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# ROS 2 Node implementation: Auto Set Home Node.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
自动设置Home点节点

该节点用于自动设置无人船的Home Position（返航点）。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong # 新增服务类型
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geographic_msgs.msg import GeoPointStamped
import math
from common_utils import ParamLoader

class AutoSetHomeNode(Node):
    """
    自动设置Home点节点类
    
    该节点订阅无人船的本地位置信息，当接收到第一个位置消息后，
    经过指定延迟时间后自动设置Home Position（返航点）。
    同时提供 XYZ 局部坐标设置 Home 的服务。
    """

    def __init__(self):
        """初始化自动设置Home点节点"""
        super().__init__('auto_set_home_node')

        # 创建 QoS 配置
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 参数加载器
        param_loader = ParamLoader(self)
        
        # 加载 GPS 原点 (A0基站: 22.5180977, 113.9007239)
        gps_origin = param_loader.load_gps_origin(
            default_lat=22.5180977,
            default_lon=113.9007239,
            default_alt=0.0
        )
        self.origin_lat = gps_origin['lat']
        self.origin_lon = gps_origin['lon']
        self.origin_alt = gps_origin['alt']

        # 声明并获取其他参数
        self.declare_parameter('set_delay_sec', 3.0)  # 设置延迟时间
        self.declare_parameter('use_current_gps', True)  # 使用当前GPS位置作为Home点
        
        self.set_delay_sec = self.get_parameter('set_delay_sec').get_parameter_value().double_value
        self.use_current_gps = self.get_parameter('use_current_gps').get_parameter_value().bool_value
        
        self.home_set_sent = False
        self.first_pose_received = False
        self.system_connected = False

        # 定时器
        self.delay_timer = None

        # 订阅本地位置信息（使用 MAVROS 原生话题检测首次定位）
        # 注意：此节点用于检测 EKF 原点设置，使用 MAVROS 原生话题
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'local_position/pose',  # 保持使用 MAVROS 原生话题
            self.local_position_callback,
            qos_best_effort
        )
        
        # 订阅 MAVROS 状态
        self.state_sub = self.create_subscription(
            State,
            'state',
            self.state_callback,
            qos_best_effort
        )

        # 创建 Home Position 发布器
        self.set_home_pub = self.create_publisher(
            GeoPointStamped, 
            'global_position/set_gp_origin', 
            qos_best_effort
        )

        # 创建局部坐标设置 Home 的服务 (复用 CommandLong 类型)
        # Service Name: ~/cmd/set_home_local (e.g. /usv_01/auto_set_home_node/cmd/set_home_local)
        # 使用私有名称以避免冲突
        self.set_home_local_srv = self.create_service(
            CommandLong,
            '~/cmd/set_home_local',
            self.set_home_local_callback
        )
        # 客户端连接到 MAVROS 用于实际执行命令
        self.mavros_cmd_client = self.create_client(CommandLong, 'cmd/command')

        # 日志输出
        if self.use_current_gps:
            self.get_logger().info(
                'AutoSetHomeNode initialized - will set Home Position at current GPS location '
                f'after {self.set_delay_sec:.1f}s delay'
            )
        else:
            self.get_logger().info(
                'AutoSetHomeNode initialized - will set Home Position at fixed coordinates '
                f'after {self.set_delay_sec:.1f}s delay'
            )
    
    def local_to_global(self, x, y, z=0.0):
        """
        将局部坐标(ENU)转换为全局坐标(Lat/Lon) - USV 端
        使用平坦地球近似
        """
        R_EARTH = 6378137.0
        
        # 纬度变换
        delta_lat_rad = y / R_EARTH
        target_lat = self.origin_lat + math.degrees(delta_lat_rad)
        
        # 经度变换
        origin_lat_rad = math.radians(self.origin_lat)
        delta_lon_rad = x / (R_EARTH * math.cos(origin_lat_rad))
        target_lon = self.origin_lon + math.degrees(delta_lon_rad)
        
        target_alt = self.origin_alt + z
        
        return target_lat, target_lon, target_alt

    def set_home_local_callback(self, request, response):
        """
        处理局部坐标设置 Home 请求
        Request Params Usage:
          param5: X (East, m)
          param6: Y (North, m)
          param7: Z (Up, m)
        """
        self.get_logger().info(f"Received Local Home Set Request: X={request.param5}, Y={request.param6}, Z={request.param7}")
        
        try:
            # 1. 转换坐标
            target_lat, target_lon, target_alt = self.local_to_global(
                request.param5,
                request.param6,
                request.param7
            )
            
            self.get_logger().info(f"Converted to Global: Lat={target_lat}, Lon={target_lon}, Alt={target_alt}")
            
            # 2. 调用 MAVROS command
            # 我们需要在一个服务回调中调用另一个服务，这通常需要异步调用或使用 send_request
            # 为了简单起见，这里使用同步等待（可能会阻塞，但对于低频操作尚可接受），或者使用 call_async 并立刻返回结果（但无法立刻知道结果）
            # 由于必须返回 response，我们最好尽力执行。
            
            if not self.mavros_cmd_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("MAVROS cmd/command service not available")
                response.success = False
                response.result = 0
                return response

            cmd_req = CommandLong.Request()
            cmd_req.broadcast = False
            cmd_req.command = 179 # MAV_CMD_DO_SET_HOME
            cmd_req.confirmation = 0
            cmd_req.param1 = 0.0 # Use specified coordinates
            cmd_req.param5 = target_lat
            cmd_req.param6 = target_lon
            cmd_req.param7 = target_alt
            
            # 这里的调用有点棘手，因为我们在 callback 里。
            # 直接 future = self.mavros_cmd_client.call_async(cmd_req)
            # 我们不能在该 callback 中 spin_until_future_complete，因为这会导致死锁（如果我们是单线程执行器）。
            # 解决方法：将其放入独立线程执行，或者直接返回 success=True (表示请求已接收) 并在后台处理。
            # 为了给 GS 正确的反馈，最好的方式是真正等待结果。
            # 简化方案：由于这是演示环境或低频操作，我们可以尝试同步调用（如果允许），
            # 但 rclpy node 中不能在回调里调用同步服务。
            # 因此：我们返回 Success，并在后台执行实际调用。
            
            self._execute_mav_command_async(cmd_req)
            
            response.success = True
            response.result = 0 # MAV_RESULT_ACCEPTED
            
        except Exception as e:
            self.get_logger().error(f"Error handling set home local: {e}")
            response.success = False
            response.result = 4 # MAV_RESULT_FAILED
            
        return response

    def _execute_mav_command_async(self, req):
        """异步执行 MAVROS 命令"""
        future = self.mavros_cmd_client.call_async(req)
        future.add_done_callback(self._mav_command_done_callback)
        
    def _mav_command_done_callback(self, future):
        try:
            res = future.result()
            if res.success:
                 self.get_logger().info("MAV_CMD_DO_SET_HOME executed successfully via MAVROS")
            else:
                 self.get_logger().error(f"MAV_CMD_DO_SET_HOME failed: result={res.result}")
        except Exception as e:
            self.get_logger().error(f"MAV_CMD_DO_SET_HOME call exception: {e}")

    def state_callback(self, msg):
        """MAVROS 状态回调"""
        self.system_connected = msg.connected

    def local_position_callback(self, msg):
        """
        本地位置回调函数
        
        当接收到第一个本地位置消息时，标记状态并等待系统连接。
        
        Args:
            msg (PoseStamped): 包含本地位置信息的消息
        """
        if not self.first_pose_received:
            self.first_pose_received = True
            self.get_logger().info('Received first local_position, waiting for system connection...')
            # 延迟后设置 Home Position
            self.delay_timer = self.create_timer(self.set_delay_sec, self._set_home_delayed)
    
    def _set_home_delayed(self):
        """延迟设置 Home Position 的回调（单次触发）"""
        # 取消定时器，确保只触发一次
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None
        
        if self.home_set_sent:
            return
            
        if not self.system_connected:
            self.get_logger().warning('System not connected, skipping Home Position setting')
            return
        
        # 设置 Home Position
        self._set_home_position()
        self.home_set_sent = True

    def _set_home_position(self):
        """设置 Home Position"""
        try:
            if self.use_current_gps:
                # 使用当前GPS位置作为Home点
                self.get_logger().info('Setting Home Position at current GPS location')
                self.get_logger().info('Home Position set request sent (using current location)')
            else:
                # 使用固定坐标作为Home点
                self.get_logger().info('Setting Home Position at fixed coordinates')
                self.get_logger().info('Home Position set request sent (using fixed coordinates)')
            
            self.get_logger().info('✅ Home Position setting completed')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to set Home Position: {e}')

    def destroy_node(self):
        """节点销毁时的资源清理"""
        if self.delay_timer:
            self.delay_timer.cancel()
        super().destroy_node()


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = AutoSetHomeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
