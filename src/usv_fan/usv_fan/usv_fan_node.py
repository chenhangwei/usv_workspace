#!/usr/bin/env python3
"""
无人球散热风扇控制节点

该节点负责根据温度传感器数据控制散热风扇的开关。
当温度超过设定阈值时开启风扇，温度降低到另一个阈值时关闭风扇。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import gpiod
from common_utils import ParamLoader


class UsvFanNode(Node):
    """
    无人球散热风扇控制节点类
    
    该节点订阅温度传感器数据，根据设定的温度阈值自动控制散热风扇的开关。
    使用GPIO控制风扇的电源状态。
    """

    def __init__(self):
        """初始化无人球散热风扇控制节点"""
        super().__init__('usv_fan_node')
        
        # 使用ParamLoader统一加载参数
        loader = ParamLoader(self)
        self.fan_pin = loader.load_param('fan_pin', 17)
        self.gpio_chip_name = loader.load_param('gpio_chip', 'gpiochip4')
        self.temp_threshold_on = loader.load_param('temp_threshold_on', 50000)    # 50°C (毫摄氏度)
        self.temp_threshold_off = loader.load_param('temp_threshold_off', 45000)  # 45°C (毫摄氏度)
        
        self.get_logger().info(f'风扇控制引脚: {self.fan_pin}')
        self.get_logger().info(f'GPIO芯片: {self.gpio_chip_name}')
        self.get_logger().info(f'开启温度阈值: {self.temp_threshold_on/1000.0}°C')
        self.get_logger().info(f'关闭温度阈值: {self.temp_threshold_off/1000.0}°C')

        # 订阅温度话题
        self.subscription = self.create_subscription(
            Int32,
            'usv_temperature',
            self.temperature_callback,
            10
        )
        
        # 初始化GPIO
        self.chip = None
        self.line = None
        self.fan_state = False  # 风扇状态
        
        try:
            self.chip = gpiod.Chip(self.gpio_chip_name)
            self.line = self.chip.get_line(self.fan_pin)
            self.line.request(consumer='usv_fan', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
            self.get_logger().info('GPIO初始化成功')
        except Exception as e:
            self.get_logger().error(f'GPIO初始化失败: {e}')
            raise

        self.get_logger().info('散热风扇控制器节点已启动')

    def temperature_callback(self, msg):
        """
        温度回调函数
        
        根据接收到的温度数据控制风扇开关。
        
        Args:
            msg (Int32): 包含温度数据的消息（单位：毫摄氏度）
        """
        try:
            temp = msg.data  # 温度（毫摄氏度）
            temp_celsius = temp / 1000.0  # 转换为摄氏度，仅用于日志
            
            # 控制风扇
            if temp >= self.temp_threshold_on and not self.fan_state:
                self.line.set_value(1)  # 开启风扇
                self.fan_state = True
                self.get_logger().info(f'风扇已开启 (温度: {temp_celsius:.1f}°C)')
            elif temp <= self.temp_threshold_off and self.fan_state:
                self.line.set_value(0)  # 关闭风扇（低电平）
                self.fan_state = False
                self.get_logger().info(f'风扇已关闭 (温度: {temp_celsius:.1f}°C)')
        except Exception as e:
            self.get_logger().error(f'处理温度数据时发生错误: {e}')

    def destroy_node(self):
        """节点销毁时的资源清理"""
        try:
            if hasattr(self, 'line') and self.line:
                self.line.set_value(0)
                self.line.release()
            if hasattr(self, 'chip') and self.chip:
                self.chip.close()
            self.get_logger().info('GPIO资源已清理')
        except Exception as e:
            self.get_logger().warn(f'清理GPIO资源时发生错误: {e}')
        super().destroy_node()

    def __del__(self):
        """清理GPIO资源"""
        try:
            if hasattr(self, 'line') and self.line:
                self.line.set_value(0)
                self.line.release()
            if hasattr(self, 'chip') and self.chip:
                self.chip.close()
        except Exception as e:
            pass


def main(args=None):
    """
    主函数
    
    初始化ROS 2节点并开始处理消息。
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    try:
        fan_controller_node = UsvFanNode()
        rclpy.spin(fan_controller_node)
    except Exception as e:
        rclpy.logging.get_logger('usv_fan_node').error(f'节点运行时发生错误: {e}')
    finally:
        if 'fan_controller_node' in locals():
            fan_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()