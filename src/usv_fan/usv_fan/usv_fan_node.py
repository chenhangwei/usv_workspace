#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gpiod as gpiod # 使用 pgpio 

class UsvFanNode(Node):
    def __init__(self):
        super().__init__('usv_fan_node')
        # 订阅 usv_temperature 话题
        self.subscription = self.create_subscription(
            Int32,
            'usv_temperature',
            self.temperature_callback,
            10
        )
        # 设置 GPIO
        self.fan_pin = 17  # BCM编号，GPIO17
        self.chip = gpiod.Chip('gpiochip4')  # 树莓派5推荐gpiochip4
        self.line = self.chip.get_line(self.fan_pin)
        self.line.request(consumer='keep_high', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])  # 初始为低电平
        self.fan_state = False  # 风扇状态
        # 温度阈值（单位：毫摄氏度）
        self.temp_threshold_on = 50000  # 50°C
        self.temp_threshold_off = 45000  # 45°C
        self.get_logger().info('散热风扇控制器节点已启动。')

    def temperature_callback(self, msg):
        temp = msg.data  # 温度（毫摄氏度）
        temp_celsius = temp / 1000.0  # 转换为摄氏度，仅用于日志
        # self.get_logger().info(f'Received temperature: {temp_celsius:.1f}°C')

        # 控制风扇
        if temp >= self.temp_threshold_on and not self.fan_state:
            self.line.set_value(1)  # 开启风扇
            self.fan_state = True
            self.get_logger().info('Fan turned ON')
        elif temp <= self.temp_threshold_off and self.fan_state:
            self.line.set_value(0)  # 关闭风扇（低电平）
            self.fan_state = False
            # self.get_logger().info('Fan turned OFF')

    def __del__(self):
        # 清理 GPIO
        try:
            if hasattr(self, 'line'):
                self.line.set_value(0)
                self.line.release()
            if hasattr(self, 'chip'):
                self.chip.close()
            self.get_logger().info('GPIO cleaned up.')
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    fan_controller_node = UsvFanNode()
    rclpy.spin(fan_controller_node)
    fan_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()