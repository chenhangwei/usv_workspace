#!/usr/bin/env python3
"""
无人球SU04超声波传感器节点

该节点负责读取SU04超声波传感器数据并通过ROS 2 Range消息发布。
支持SU04超声波传感器，通过串口通信获取数据。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# 导入common_utils工具
from common_utils import SerialResourceManager, ParamLoader, ParamValidator


class SU04Node(Node):
    """
    无人球SU04超声波传感器节点类
    
    该节点实现SU04超声波传感器数据读取和发布功能，通过串口与传感器通信，
    将读取到的数据转换为Range消息格式并发布。
    """

    def __init__(self):
        """初始化无人球SU04超声波传感器节点"""
        super().__init__('su04_node')
        
        # 创建参数加载器
        param_loader = ParamLoader(self)
        
        # 加载串口参数
        port = param_loader.load_param(
            'serial_port',
            '/dev/ttyS0',
            ParamValidator.non_empty_string,
            'SU04串口路径'
        )
        baudrate = param_loader.load_param(
            'baud_rate',
            9600,
            lambda x: x in [9600, 19200, 38400, 57600, 115200],
            'SU04波特率'
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
            self.get_logger().error('初始化SU04串口失败，节点退出')
            raise RuntimeError(f'Failed to open serial port {port}')
        
        # 创建发布者
        self.publisher_ = self.create_publisher(Range, 'su04_range', 10)
        
        # 设置定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
            
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
            if not self.serial_manager.is_open:
                return
            
            if self.serial_manager.serial_port and self.serial_manager.serial_port.in_waiting > 0:
                # 读取串口数据
                data = self.serial_manager.readline()
                if data:
                    data_str = data.decode('utf-8').strip()
                    if data_str:
                        # 假设 SU04 返回纯数字距离（单位厘米）
                        distance = float(data_str) / 100.0  # 转换为米
                        # 填充 Range 消息
                        self.range_msg.header.stamp = self.get_clock().now().to_msg()
                        self.range_msg.range = distance
                        # 发布消息
                        self.publisher_.publish(self.range_msg)
                        self.get_logger().debug(f'Distance: {distance} m')
        except ValueError as e:
            self.get_logger().warn(f'数据格式错误: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'处理SU04数据时发生错误: {str(e)}')
    
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
    node = SU04Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()