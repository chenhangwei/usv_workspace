"""
无人船UWB定位节点

该节点负责读取UWB定位数据并通过ROS 2 PoseStamped消息发布。
支持多种UWB定位设备，通过串口通信获取数据。
"""

import serial 
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
import re

# 导入common_utils工具
from common_utils import SerialResourceManager, ParamLoader, ParamValidator


class UsvUwbNode(Node):
    """
    无人船UWB定位节点类
    
    该节点实现UWB定位数据读取和发布功能，通过串口与UWB定位设备通信，
    将读取到的数据转换为PoseStamped消息格式并发布。
    """

    def __init__(self):
        """初始化无人船UWB定位节点"""
        super().__init__('usv_uwb_node')
        
        # 创建参数加载器
        param_loader = ParamLoader(self)
        
        # 加载串口参数
        port = param_loader.load_param(
            'uwb_port',
            '/dev/ttyUSB0',
            ParamValidator.non_empty_string,
            'UWB串口路径'
        )
        baudrate = param_loader.load_param(
            'uwb_baudrate',
            115200,
            lambda x: x in [9600, 19200, 38400, 57600, 115200],
            'UWB波特率'
        )
        timeout = param_loader.load_param(
            'uwb_timeout',
            1.0,
            ParamValidator.positive,
            '串口超时(秒)'
        )
        
        # 创建串口资源管理器
        self.serial_manager = SerialResourceManager(self.get_logger())
        
        # 打开串口
        if not self.serial_manager.open(port, baudrate, timeout):
            self.get_logger().error('初始化UWB串口失败，节点退出')
            raise RuntimeError(f'Failed to open serial port {port}')
        
        # 创建发布器
        self.uwb_pub = self.create_publisher(PoseStamped, 'vision_pose/pose', 10)
         
        # 100 Hz 定时器
        self.timer = self.create_timer(0.01, self.read_and_publish)
        
        self.uwb_msg = PoseStamped()

    def read_and_publish(self):
        """
        读取串口数据并发布 PoseStamped 消息
        
        从串口读取UWB定位数据，转换为PoseStamped消息格式并发布。
        """
        # 检查串口是否打开
        if not self.serial_manager.is_open:
            return
        
        try:  
            # 读取一行数据
            line = self.serial_manager.readline()
            if not line:
                self.get_logger().debug("没有接收到数据")
                return
            data = line.decode('ASCII').strip()
            # self.get_logger().info(f"Raw data: {data}")

            # 查找 LO=[...]
            ma = re.search(r'LO=\[([^]]+)\]', data)

            if ma:
                coords_str = ma.group(1)  # 提取括号内的内容，例如 "1.41,4.13,0.26" 或 "no solution"
                # 检查是否为 "no solution"
                if coords_str == 'no solution':
                    self.get_logger().debug("No valid position solution from $KT0")
                    return
                try:
                    values = list(map(float, ma.group(1).split(',')))
                    if len(values) == 3:
                        x, y, z = values
                        # self.get_logger().info(f'x:{x},y:{y},z:{z}')
                        self.uwb_msg.header.stamp = self.get_clock().now().to_msg()
                        self.uwb_msg.header.frame_id = 'map'
                        self.uwb_msg.pose.position.x = x
                        self.uwb_msg.pose.position.y = y
                        self.uwb_msg.pose.position.z = 0.0  # 保持在水面高度
                        self.uwb_pub.publish(self.uwb_msg)
                        self.get_logger().debug(f'发布UWB位置: x={x:.2f}, y={y:.2f}')
                    else:
                        pass
                        # self.get_logger().warning(f'括号内数据不符合三个数值的要求')  
                except ValueError as e:
                    self.get_logger().warning(f'数据转换错误: {e}')
            else:
                pass
                # self.get_logger().warning(f'括号内数据不符合三个数值的要求')  
        except Exception as e:
            self.get_logger().error(f'读取UWB数据出错: {e}')
    
    def destroy_node(self):
        """节点销毁时关闭串口"""
        self.serial_manager.close()
        super().destroy_node()


def main():
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    """
    rclpy.init() 
    node = UsvUwbNode() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 


if __name__ == '__main__': 
    main()