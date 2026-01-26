#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of test low battery sound.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
低电量声音播放修复验证脚本

用途：
1. 模拟发送低电量模式消息
2. 验证 Sound 节点是否自动启动声音播放
3. 检查日志输出

使用方法：
    # 在地面站或任意有 ROS 2 环境的终端
    cd ~/usv_workspace
    source install/setup.bash
    python3 src/usv_sound/scripts/test_low_battery_sound.py usv_02
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import time


class LowBatterySoundTester(Node):
    """低电量声音播放测试节点"""
    
    def __init__(self, usv_namespace):
        """
        初始化测试节点
        
        Args:
            usv_namespace: USV 命名空间（如 usv_02）
        """
        super().__init__('low_battery_sound_tester')
        
        self.usv_namespace = usv_namespace
        
        # 创建发布器
        self.low_voltage_pub = self.create_publisher(
            Bool,
            f'/{usv_namespace}/low_voltage_mode',
            10
        )
        
        self.get_logger().info(f'低电量声音测试节点已启动')
        self.get_logger().info(f'目标 USV: {usv_namespace}')
        
    def test_low_battery_trigger(self):
        """测试低电量触发"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('测试场景 1: 触发低电量模式')
        self.get_logger().info('=' * 60)
        
        # 等待订阅者连接
        self.get_logger().info('等待订阅者连接...')
        time.sleep(2)
        
        # 发送低电量触发消息
        msg = Bool()
        msg.data = True
        
        self.get_logger().warn('🔴 发送低电量触发消息: True')
        self.low_voltage_pub.publish(msg)
        
        self.get_logger().info('')
        self.get_logger().info('期望结果:')
        self.get_logger().info('  1. ✅ LED 节点显示红色闪烁')
        self.get_logger().info('  2. ✅ Sound 节点输出: [!][!][!] 低电压模式触发！')
        self.get_logger().info('  3. ✅ Sound 节点输出: [!] 自动启动低电量警告声音播放')
        self.get_logger().info('  4. ✅ Sound 节点开始播放 moon101.wav')
        self.get_logger().info('')
        self.get_logger().info('请检查以下话题的日志输出:')
        self.get_logger().info(f'  ros2 topic echo /rosout | grep "{self.usv_namespace}.*sound"')
        self.get_logger().info('')
        
    def test_low_battery_recovery(self):
        """测试退出低电量模式"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('测试场景 2: 退出低电量模式')
        self.get_logger().info('=' * 60)
        
        # 发送恢复消息
        msg = Bool()
        msg.data = False
        
        self.get_logger().info('🟢 发送退出低电量消息: False')
        self.low_voltage_pub.publish(msg)
        
        self.get_logger().info('')
        self.get_logger().info('期望结果:')
        self.get_logger().info('  1. ✅ LED 节点恢复正常显示')
        self.get_logger().info('  2. ✅ Sound 节点输出: [OK] 退出低电压模式')
        self.get_logger().info('  3. ✅ Sound 节点切换回正常音效（gaga）')
        self.get_logger().info('')
    
    def run_test(self):
        """运行完整测试"""
        try:
            # 测试 1: 触发低电量
            self.test_low_battery_trigger()
            time.sleep(8)
            
            # 测试 2: 退出低电量
            self.test_low_battery_recovery()
            time.sleep(3)
            
            self.get_logger().info('=' * 60)
            self.get_logger().info('✅ 测试完成！')
            self.get_logger().info('=' * 60)
            
        except Exception as e:
            self.get_logger().error(f'测试失败: {e}')


def main(args=None):
    """主函数"""
    if len(sys.argv) < 2:
        print('用法: python3 test_low_battery_sound.py <usv_namespace>')
        print('示例: python3 test_low_battery_sound.py usv_02')
        sys.exit(1)
    
    usv_namespace = sys.argv[1]
    
    rclpy.init(args=args)
    
    try:
        node = LowBatterySoundTester(usv_namespace)
        node.run_test()
        
        # 短暂 spin 以确保消息发送完成
        rclpy.spin_once(node, timeout_sec=1)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
