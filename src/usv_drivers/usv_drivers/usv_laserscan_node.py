"""
无人船激光雷达节点

该节点负责读取激光雷达数据并通过ROS 2 LaserScan消息发布。
支持多种激光雷达设备，通过串口通信获取数据。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import serial
import time


class UsvLaserScanNode(Node):
    """
    无人船激光雷达节点类
    
    该节点实现激光雷达数据读取和发布功能，通过串口与激光雷达设备通信，
    将读取到的数据转换为LaserScan消息格式并发布。
    """

    def __init__(self):
        """初始化无人船激光雷达节点"""
        super().__init__('usv_laserscan_node')
        
        # 创建 LaserScan 消息发布者，发布到 'laser_scan' 话题，队列大小为 10
        self.radar_pub = self.create_publisher(LaserScan, 'laser_scan', 10)
        
        # 串口参数（根据实际雷达设备调整）
        self.serial_port = '/dev/ttyUSB0'  # 串口设备路径
        self.baud_rate = 115200            # 波特率
        self.timeout = 1                   # 超时时间（秒）
        
        # 尝试打开串口
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            self.get_logger().info(f'串口 {self.serial_port} 打开成功')
        except serial.SerialException as e:
            self.get_logger().error(f'打开串口失败: {e}')
            raise
        
        # LaserScan 参数（根据雷达规格调整）
        self.angle_min = 0.0              # 最小角度（弧度）
        self.angle_max = 3.14159          # 最大角度（180度，弧度）
        self.angle_increment = 0.0174533  # 角度增量（1度，弧度）
        self.range_min = 0.1              # 最小距离（米）
        self.range_max = 10.0             # 最大距离（米）
        # 根据角度范围和增量计算采样点数
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        
        # 创建定时器，10Hz 定时发布雷达数据（每 0.1 秒调用一次）
        self.timer = self.create_timer(0.1, self.publish_radar_data)

    def publish_radar_data(self):
        """
        读取串口数据并发布 LaserScan 消息
        
        从串口读取激光雷达数据，转换为LaserScan消息格式并发布。
        """
        try:
            # 调用读取雷达数据函数，获取一帧数据（列表格式）
            ranges = self.read_laser_frame()
            if not ranges:
                return

            # 创建 LaserScan 消息对象
            scan = LaserScan()
            # 设置消息头，使用当前时间戳和帧 ID 'radar_link'
            scan.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='radar_link')
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_increment
            scan.range_min = self.range_min
            scan.range_max = self.range_max  # 设置最大距离
            scan.ranges = ranges           # 设置距离数组

            # 计算每个测量点的采样时间（假设采样均匀）
            scan.time_increment = 0.1 / self.num_readings
            scan.scan_time = 0.1          # 整个扫描周期

            # 发布 LaserScan 消息
            self.radar_pub.publish(scan)
            self.get_logger().debug('已发布雷达扫描数据')
        except Exception as e:
            self.get_logger().error(f'发布雷达数据出错: {e}')

    def read_laser_frame(self):
        """
        从串口读取一帧雷达数据
        假设每行数据的格式为 "angle:distance"，角度单位为度，距离单位为米
        
        Returns:
            list: 包含距离数据的列表，如果读取失败则返回None
        """
        # 初始化距离列表，初始值为无穷大（表示无数据）
        ranges = [float('inf')] * self.num_readings
        try:
            # 清空输入缓冲区，避免读取到旧数据
            self.ser.reset_input_buffer()
            # 循环读取一帧数据，每次读取一行
            for _ in range(self.num_readings):
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    # 假设数据格式为 "angle:distance"
                    angle_str, distance_str = line.split(':')
                    angle = float(angle_str)
                    distance = float(distance_str)
                    # 将角度转换为索引，假设角度范围为 0 到 180 度
                    index = int(angle / (180.0 / (self.num_readings - 1)))
                    if 0 <= index < self.num_readings:
                        # 如果距离在有效范围内则赋值，否则置为无穷大
                        ranges[index] = distance if self.range_min <= distance <= self.range_max else float('inf')
            return ranges
        except Exception as e:
            self.get_logger().error(f'读取雷达数据出错: {e}')
            return None

    def __del__(self):
        """关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('串口已关闭')


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    node = UsvLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()