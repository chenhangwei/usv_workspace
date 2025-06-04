#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Range

class SU04Node(Node):
    def __init__(self):
        super().__init__('su04_node')
        # 创建发布者，发布到 /su04_range 话题
        self.publisher_ = self.create_publisher(Range, 'su04_range', 10)
        # 设置定时器，每 0.1 秒读取一次数据
        self.timer = self.create_timer(0.1, self.timer_callback)
        # 初始化串口
        self.serial_port = serial.Serial(
            port='/dev/ttyS0',  # 树莓派 UART 端口（可能为 /dev/ttyAMA0）
            baudrate=9600,      # SU04 默认波特率，需参考说明书
            timeout=1
        )
        # 初始化 Range 消息
        self.range_msg = Range()
        self.range_msg.radiation_type = Range.ULTRASOUND
        self.range_msg.field_of_view = 0.5  # 视场角（弧度），参考 SU04 说明书
        self.range_msg.min_range = 0.2      # 最小检测距离（米）
        self.range_msg.max_range = 4.0      # 最大检测距离（米）
        self.range_msg.header.frame_id = 'su04_frame'

    def timer_callback(self):
        try:
            if self.serial_port.in_waiting > 0:
                # 读取串口数据
                data = self.serial_port.readline().decode('utf-8').strip()
                # 假设 SU04 返回纯数字距离（单位厘米），需根据实际数据格式调整
                distance = float(data) / 100.0  # 转换为米
                # 填充 Range 消息
                self.range_msg.header.stamp = self.get_clock().now().to_msg()
                self.range_msg.range = distance
                # 发布消息
                self.publisher_.publish(self.range_msg)
                self.get_logger().info(f'Distance: {distance} m')
        except Exception as e:
            self.get_logger().error(f'Error reading SU04: {str(e)}')

    def __del__(self):
        self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    su04_node = SU04Node()
    rclpy.spin(su04_node)
    su04_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()