#!/usr/bin/env python3
"""
测试 FailsafeFlags 消息接收
验证 px4_msgs FailsafeFlags 与飞控固件版本是否匹配
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from px4_msgs.msg import FailsafeFlags


class FailsafeFlagsTest(Node):
    def __init__(self):
        super().__init__('failsafe_flags_test')
        
        qos_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.sub = self.create_subscription(
            FailsafeFlags,
            'fmu/out/failsafe_flags',
            self.callback,
            qos_px4
        )
        
        self.get_logger().info('等待 failsafe_flags 消息...')
        self.msg_count = 0
    
    def callback(self, msg: FailsafeFlags):
        self.msg_count += 1
        if self.msg_count <= 3:  # 只打印前3条
            self.get_logger().info(f'收到 FailsafeFlags #{self.msg_count}:')
            self.get_logger().info(f'  timestamp: {msg.timestamp}')
            self.get_logger().info(f'  angular_velocity_invalid: {msg.angular_velocity_invalid}')
            self.get_logger().info(f'  attitude_invalid: {msg.attitude_invalid}')
            self.get_logger().info(f'  local_position_invalid: {msg.local_position_invalid}')
            self.get_logger().info(f'  manual_control_signal_lost: {msg.manual_control_signal_lost}')
            self.get_logger().info(f'  gcs_connection_lost: {msg.gcs_connection_lost}')
            self.get_logger().info(f'  battery_warning: {msg.battery_warning}')
            self.get_logger().info(f'  fd_critical_failure: {msg.fd_critical_failure}')
            self.get_logger().info('---')
        elif self.msg_count == 4:
            self.get_logger().info('继续接收... (后续消息不再打印详情)')
        
        if self.msg_count % 10 == 0:
            self.get_logger().info(f'已接收 {self.msg_count} 条消息')


def main():
    rclpy.init()
    node = FailsafeFlagsTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
