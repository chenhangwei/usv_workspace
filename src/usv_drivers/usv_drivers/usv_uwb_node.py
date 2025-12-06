"""
无人船UWB定位节点

该节点负责读取UWB定位数据并通过ROS 2 PoseStamped消息发布。
支持多种UWB定位设备，通过串口通信获取数据。
"""

import serial 
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
import struct

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
            '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5787006321-if00',
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
        # MAVROS vision_pose 插件默认订阅 vision_pose/pose
        # 注意：MAVROS 话题已扁平化，直接发布到 vision_pose/pose
        self.uwb_pub = self.create_publisher(PoseStamped, 'vision_pose/pose', 10)
         
        # 20 Hz 定时器
        self.timer = self.create_timer(0.05, self.read_and_publish)
        
        self.uwb_msg = PoseStamped()
        self.buffer = bytearray()

    def read_and_publish(self):
        """
        读取串口数据并发布 PoseStamped 消息
        
        解析 NLink_LinkTrack_Tag_Frame0 协议 (128 bytes)
        """
        # 检查串口是否打开
        if not self.serial_manager.is_open:
            return
        
        try:
            # 读取所有可用数据
            if self.serial_manager.serial_port.in_waiting > 0:
                data = self.serial_manager.serial_port.read(self.serial_manager.serial_port.in_waiting)
                self.buffer.extend(data)
            
            # 处理缓冲区
            while len(self.buffer) >= 128:
                # 查找帧头 0x55
                header_idx = self.buffer.find(b'\x55')
                
                if header_idx == -1:
                    # 没有找到帧头，清空缓冲区
                    self.buffer.clear()
                    break
                
                # 丢弃帧头之前的数据
                if header_idx > 0:
                    del self.buffer[:header_idx]
                
                # 检查是否有足够的数据
                if len(self.buffer) < 128:
                    break
                
                # 检查功能字 (0x01)
                if self.buffer[1] != 0x01:
                    # 帧头错误，丢弃第一个字节继续查找
                    del self.buffer[0]
                    continue
                
                # 校验和检查 (Sum of bytes 0-126)
                frame = self.buffer[:128]
                checksum = sum(frame[:-1]) & 0xFF
                if checksum != frame[127]:
                    self.get_logger().warn(f"校验和错误: calc {checksum} != recv {frame[127]}")
                    del self.buffer[0]
                    continue
                
                # 解析并发布
                self.parse_and_publish_frame(frame)
                
                # 移除已处理的帧
                del self.buffer[:128]
                
        except Exception as e:
            self.get_logger().error(f'读取UWB数据出错: {e}')

    def parse_and_publish_frame(self, frame):
        """解析一帧数据并发布"""
        try:
            # 解析位置 (int24 * 1000) -> meters
            # x: bytes 4-6, y: bytes 7-9, z: bytes 10-12
            pos_x = self.parse_int24(frame[4:7]) / 1000.0
            pos_y = self.parse_int24(frame[7:10]) / 1000.0
            pos_z = self.parse_int24(frame[10:13]) / 1000.0
            
            # 解析四元数 (float * 4)
            # q0-q3: bytes 88-103
            q0, q1, q2, q3 = struct.unpack('<ffff', frame[88:104])
            
            # 填充消息
            self.uwb_msg.header.stamp = self.get_clock().now().to_msg()
            self.uwb_msg.header.frame_id = 'map'
            
            self.uwb_msg.pose.position.x = pos_x
            self.uwb_msg.pose.position.y = pos_y
            self.uwb_msg.pose.position.z = pos_z
            
            self.uwb_msg.pose.orientation.w = 1.0
            self.uwb_msg.pose.orientation.x = 0.0
            self.uwb_msg.pose.orientation.y = 0.0
            self.uwb_msg.pose.orientation.z = 0.0
            
            self.uwb_pub.publish(self.uwb_msg)
            # self.get_logger().info(f'发布UWB位置: x={pos_x:.3f}, y={pos_y:.3f}, z={pos_z:.3f}')
            
        except Exception as e:
            self.get_logger().error(f"解析帧错误: {e}")

    def parse_int24(self, data):
        """解析3字节整数 (Little-endian)"""
        val = data[0] | (data[1] << 8) | (data[2] << 16)
        # 符号扩展
        if val & 0x800000:
            val -= 0x1000000
        return val
    
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