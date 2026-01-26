#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# ROS 2 Node implementation: Usv Ultrasonic Radar Node.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
无人船超声波雷达节点

该节点负责读取超声波雷达数据并通过ROS 2 Range消息发布。
支持多种超声波雷达设备，通过串口通信获取数据。
"""

from blinker import Namespace
import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Range

# 导入common_utils工具
from common_utils import SerialResourceManager, ParamLoader, ParamValidator


class UltrasonicRadarNode(Node):
    """
    无人船超声波雷达节点类
    
    该节点实现超声波雷达数据读取和发布功能，通过串口与超声波雷达设备通信，
    将读取到的数据转换为Range消息格式并发布。
    """

    def __init__(self):
        """初始化无人船超声波雷达节点"""
        super().__init__('usv_ultrasonic_radar_node')

        namespace = self.get_namespace()
        
        # 创建参数加载器
        param_loader = ParamLoader(self)
        
        # 加载串口参数
        port = param_loader.load_param(
            'serial_port',
            '/dev/ttyUSB0',
            ParamValidator.non_empty_string,
            '超声波雷达串口路径'
        )
        baudrate = param_loader.load_param(
            'baud_rate',
            9600,
            lambda x: x in [9600, 19200, 38400, 57600, 115200],
            '超声波雷达波特率'
        )
        timeout = param_loader.load_param(
            'timeout',
            1.0,
            ParamValidator.positive,
            '串口超时(秒)'
        )
        
        # 创建串口资源管理器
        self.serial_manager = SerialResourceManager(self.get_logger())
        
        # 打开串口
        if not self.serial_manager.open(port, baudrate, timeout):
            self.get_logger().error('初始化超声波雷达串口失败，节点退出')
            raise RuntimeError(f'Failed to open serial port {port}')
        
        # 创建发布者
        self.publisher_ = self.create_publisher(Range, 'ultrasonic_radar_range', 10)
        
        # 设置定时器
        self.timer = self.create_timer(0.3, self.timer_callback)
            
        # 初始化 Range 消息
        self.range_msg = Range()
        self.range_msg.radiation_type = Range.ULTRASOUND
        self.range_msg.field_of_view = 0.5  # 视场角（弧度）
        self.range_msg.min_range = 0.2      # 最小检测距离（米）
        self.range_msg.max_range = 3.0      # 最大检测距离（米）
        self.range_msg.header.frame_id = f'laser_frame_{namespace.strip("/")}'

        self.get_logger().info('超声波雷达节点启动，监听 /dev/ttyUSB0 串口数据')

    def timer_callback(self):
        """
        定时器回调函数
        
        定期从串口读取超声波雷达数据并发布。
        """
        if not self.serial_manager.is_open:
            self.get_logger().error('串口未初始化或已关闭')
            return

        try:
            # 读取所有可用数据（至少 3 字节为一帧）
            if self.serial_manager.serial_port:
                in_waiting = self.serial_manager.serial_port.in_waiting or 3
                data = self.serial_manager.read(in_waiting)
            else:
                return
            if data:
                # self.get_logger().info(f'原始数据: {data.hex()} (hex)')
                # 按帧解析（每帧 3 字节）
                frame_length = 3
                for i in range(0, len(data), frame_length):
                    frame = data[i:i+frame_length]
                    if len(frame) == frame_length:
                        # 提取距离（第 1-2 字节，毫米，大端序）
                        distance_mm = int.from_bytes(frame[0:2], 'big')
                        distance_m = distance_mm / 1000.0  # 转换为米
                        
                        # 验证校验和
                        expected_checksum = (frame[0] + frame[1]) & 0xFF
                        if frame[2] == expected_checksum:
                            if self.range_msg.min_range <= distance_m <= self.range_msg.max_range:
                                self.range_msg.header.stamp = self.get_clock().now().to_msg()
                                self.range_msg.range = distance_m
                                self.publisher_.publish(self.range_msg)
                                self.get_logger().debug(f'发布距离: {distance_m} m (十进制: {distance_mm} mm)')
                            else:
                                pass
                                # self.get_logger().warn(f'距离 {distance_m} m 超出范围 [{self.range_msg.min_range}, {self.range_msg.max_range}]')
                        else:
                            pass
                            # self.get_logger().warn(f'校验和错误: 帧 {frame.hex()} (hex), 计算校验和: {hex(expected_checksum)}, 实际: {hex(frame[2])}')
                    else:
                        pass
                        # self.get_logger().warn(f'帧长度不足: {frame.hex()} (hex), 长度: {len(frame)}')
            else:
                self.get_logger().debug('串口无数据返回')
        except Exception as e:
            self.get_logger().error(f'处理数据时发生错误: {str(e)}')
    
    def destroy_node(self):
        """节点销毁时关闭串口"""
        self.serial_manager.close()
        super().destroy_node()


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    usv_ultrasonic_radar_node = UltrasonicRadarNode()
    rclpy.spin(usv_ultrasonic_radar_node)
    usv_ultrasonic_radar_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()