#!/usr/bin/env python3
from blinker import Namespace
import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Range


class UltrasonicRadarNode(Node):
    def __init__(self):
        super().__init__('usv_ultrasonic_radar_node')

        namespace=self.get_namespace()

        # 创建发布者，发布 话题
        self.publisher_ = self.create_publisher(Range, 'ultrasonic_radar_range', 10)
        # 设置定时器，每 0.3秒读取一次数据
        self.timer = self.create_timer(0.3, self.timer_callback)
          # 初始化串口
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=9600,
                timeout=1
            )

        except serial.SerialException as e:
            self.get_logger().error(f"打开串口 /dev/ttyUSB0 失败: {str(e)}")
            return  # 避免继续初始化
        # 初始化 Range 消息
        self.range_msg = Range()
        self.range_msg.radiation_type = Range.ULTRASOUND
        self.range_msg.field_of_view = 0.5  # 视场角（弧度）
        self.range_msg.min_range = 0.2      # 最小检测距离（米）
        self.range_msg.max_range = 3.0      # 最大检测距离（米）
        self.range_msg.header.frame_id = f'laser_frame_{namespace.strip("/")}'

        self.get_logger().info('超声波雷达节点启动，监听 /dev/ttyUSB0 串口数据')
    def timer_callback(self):
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error('串口未初始化或已关闭')
            return

        # self.get_logger().info('正在读取串口数据...')
        try:
            # 读取所有可用数据（至少 3 字节为一帧）
            data = self.serial_port.read(self.serial_port.in_waiting or 3)
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
                                # self.get_logger().info(f'发布距离: {distance_m} m (十进制: {distance_mm} mm)')
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
                self.get_logger().warn('串口无数据返回')
        except serial.SerialException as e:
            self.get_logger().error(f'读取串口失败: {str(e)}')


    def __del__(self):
        if hasattr(self, 'serial_port') and self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('串口已关闭')


def main(args=None):
    rclpy.init(args=args)
    usv_ultrasonic_radar_node = UltrasonicRadarNode()
    rclpy.spin(usv_ultrasonic_radar_node )
    usv_ultrasonic_radar_node .destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()