#!/usr/bin/env python3
"""
无人船SU04超声波传感器节点

该节点负责读取SU04超声波传感器数据并通过ROS 2 Range消息发布。
支持SU04超声波传感器，通过串口通信获取数据。
"""

import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Range


class SU04Node(Node):
    """
    无人船SU04超声波传感器节点类
    
    该节点实现SU04超声波传感器数据读取和发布功能，通过串口与传感器通信，
    将读取到的数据转换为Range消息格式并发布。
    """

    def __init__(self):
        """初始化无人船SU04超声波传感器节点"""
        super().__init__('su04_node')
        
        # 创建发布者，发布到 /su04_range 话题
        self.publisher_ = self.create_publisher(Range, 'su04_range', 10)
        
        # 设置定时器，每 0.1 秒读取一次数据
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 初始化串口
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyS0',  # 树莓派 UART 端口（可能为 /dev/ttyAMA0）
                baudrate=9600,      # SU04 默认波特率，需参考说明书
                timeout=1
            )
            self.get_logger().info('SU04串口打开成功')
        except serial.SerialException as e:
            self.get_logger().error(f'打开SU04串口失败: {e}')
            return
            
        # 初始化 Range 消息
        self.range_msg = Range()
        self.range_msg.radiation_type = Range.ULTRASOUND
        self.range_msg.field_of_view = 0.5  # 视场角（弧度），参考 SU04 说明书
        self.range_msg.min_range = 0.2      # 最小检测距离（米）
        self.range_msg.max_range = 4.0      # 最大检测距离（米）
        self.range_msg.header.frame_id = 'su04_frame'

    def timer_callback(self):
        """
        定时器回调函数
        
        定期从串口读取SU04传感器数据并发布。
        """
        try:
            if self.serial_port.in_waiting > 0:
                # 读取串口数据
                data = self.serial_port.readline().decode('utf-8').strip()
                if data:
                    # 假设 SU04 返回纯数字距离（单位厘米），需根据实际数据格式调整
                    distance = float(data) / 100.0  # 转换为米
                    # 填充 Range 消息
                    self.range_msg.header.stamp = self.get_clock().now().to_msg()
                    self.range_msg.range = distance
                    # 发布消息
                    self.publisher_.publish(self.range_msg)
                    self.get_logger().debug(f'Distance: {distance} m')
        except ValueError as e:
            self.get_logger().warn(f'数据格式错误: {str(e)}')
        except serial.SerialException as e:
            self.get_logger().error(f'读取SU04串口错误: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'处理SU04数据时发生错误: {str(e)}')

    def __del__(self):
        """关闭串口"""
        if hasattr(self, 'serial_port') and self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('SU04串口已关闭')


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    su04_node = SU04Node()
    rclpy.spin(su04_node)
    su04_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()