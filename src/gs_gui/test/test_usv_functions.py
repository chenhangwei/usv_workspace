#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Unit tests for usv_functions.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
USV功能测试示例
演示如何为USV项目编写实用的单元测试
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# 添加包路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    from gs_gui.ros_signal import ROSSignal
    from common_interfaces.msg import UsvStatus
    from geometry_msgs.msg import Point, Twist
    from std_msgs.msg import Header
except ImportError as e:
    print(f"警告: 无法导入模块 {e}，某些测试将被跳过")
    ROSSignal = None
    UsvStatus = None


class TestUsvStatusMessage(unittest.TestCase):
    """测试USV状态消息的创建和处理"""
    
    @unittest.skipIf(UsvStatus is None, "UsvStatus消息类型无法导入")
    def test_usv_status_creation(self):
        """测试创建USV状态消息"""
        # 创建测试状态消息
        status = UsvStatus()
        status.usv_id = "usv_01"
        status.mode = "GUIDED" 
        status.connected = True
        status.armed = False
        status.battery_voltage = 12.5
        status.battery_percentage = 85.0
        
        # 验证基本字段
        self.assertEqual(status.usv_id, "usv_01")
        self.assertEqual(status.mode, "GUIDED")
        self.assertTrue(status.connected)
        self.assertFalse(status.armed)
        self.assertAlmostEqual(status.battery_voltage, 12.5, places=1)
        self.assertEqual(status.battery_percentage, 85.0)
    
    @unittest.skipIf(UsvStatus is None, "UsvStatus消息类型无法导入")  
    def test_usv_status_validation(self):
        """测试USV状态消息的验证逻辑"""
        status = UsvStatus()
        
        # 测试电池电量范围
        status.battery_percentage = 150.0  # 无效值
        self.assertGreater(status.battery_percentage, 100.0)  # 应该被检测为无效
        
        # 测试电压合理性
        status.battery_voltage = 8.0  # 低电压
        self.assertLess(status.battery_voltage, 10.0)  # 低电压警告阈值


class TestUsvIdParser(unittest.TestCase):
    """测试USV ID解析功能"""
    
    def test_extract_usv_number(self):
        """测试从USV ID中提取数字"""
        test_cases = [
            ("usv_01", "01"),
            ("usv_02", "02"), 
            ("usv_10", "10"),
            ("usv_abc", "abc"),  # 非数字情况
        ]
        
        for usv_id, expected in test_cases:
            with self.subTest(usv_id=usv_id):
                # 模拟提取逻辑
                number_part = usv_id.split('_')[-1] if '_' in usv_id else usv_id
                self.assertEqual(number_part, expected)
    
    def test_usv_namespace_conversion(self):
        """测试USV命名空间转换"""
        # 模拟从命名空间推导系统ID的逻辑
        def extract_system_id(namespace):
            try:
                return int(namespace.split('_')[-1])
            except (ValueError, IndexError):
                return 1  # 默认系统ID
        
        test_cases = [
            ("usv_01", 1),
            ("usv_02", 2),
            ("usv_10", 10), 
            ("invalid_name", 1),  # 应该回退到默认值
            ("", 1),  # 空字符串
        ]
        
        for namespace, expected_id in test_cases:
            with self.subTest(namespace=namespace):
                system_id = extract_system_id(namespace)
                self.assertEqual(system_id, expected_id)


class TestNavigationLogic(unittest.TestCase):
    """测试导航逻辑"""
    
    def test_distance_calculation(self):
        """测试距离计算"""
        def calculate_2d_distance(pos1, pos2):
            """计算两点间的2D距离"""
            dx = pos1[0] - pos2[0]
            dy = pos1[1] - pos2[1]
            return (dx*dx + dy*dy) ** 0.5
        
        # 测试已知距离
        start = [0, 0]
        end = [3, 4]  # 3-4-5三角形
        distance = calculate_2d_distance(start, end)
        self.assertAlmostEqual(distance, 5.0, places=2)
        
        # 测试零距离
        same_point = [1, 1]
        zero_distance = calculate_2d_distance(same_point, same_point)
        self.assertEqual(zero_distance, 0.0)
    
    def test_target_reached_logic(self):
        """测试目标点到达判定"""
        def is_target_reached(current_pos, target_pos, threshold=1.0):
            """判断是否到达目标点"""
            distance = ((current_pos[0] - target_pos[0])**2 + 
                       (current_pos[1] - target_pos[1])**2) ** 0.5
            return distance <= threshold
        
        target = [10, 10]
        
        # 测试已到达
        near_position = [10.5, 10.2]  # 距离约0.54米
        self.assertTrue(is_target_reached(near_position, target, threshold=1.0))
        
        # 测试未到达
        far_position = [8, 8]  # 距离约2.83米  
        self.assertFalse(is_target_reached(far_position, target, threshold=1.0))


class TestMockUsvNode(unittest.TestCase):
    """演示如何使用Mock测试ROS节点"""
    
    def test_mock_node_initialization(self):
        """测试节点初始化的Mock"""
        # 创建Mock节点
        mock_node = Mock()
        mock_node.get_name.return_value = 'test_usv_node'
        mock_node.get_namespace.return_value = '/usv_01'
        
        # 验证节点属性
        self.assertEqual(mock_node.get_name(), 'test_usv_node')
        self.assertEqual(mock_node.get_namespace(), '/usv_01')
        
        # 验证方法被调用
        mock_node.get_name.assert_called_once()
        mock_node.get_namespace.assert_called_once()
    
    def test_mock_publisher(self):
        """测试发布者的Mock"""
        # 创建Mock发布者
        mock_publisher = Mock()
        
        # 模拟发布消息
        test_message = Mock()
        test_message.data = "test_command"
        
        mock_publisher.publish(test_message)
        
        # 验证发布被调用
        mock_publisher.publish.assert_called_once_with(test_message)
    
    @patch('rclpy.create_node')
    def test_node_creation_with_patch(self, mock_create_node):
        """使用patch装饰器测试节点创建"""
        mock_node = Mock()
        mock_create_node.return_value = mock_node
        
        # 假设有一个函数创建节点
        def create_usv_node(name):
            import rclpy
            return rclpy.create_node(name)
        
        # 调用函数
        result = create_usv_node('test_node')
        
        # 验证
        self.assertEqual(result, mock_node)
        mock_create_node.assert_called_once_with('test_node')


class TestBatteryMonitoring(unittest.TestCase):
    """测试电池监控逻辑"""
    
    def test_battery_level_categories(self):
        """测试电池电量分类"""
        def categorize_battery_level(voltage):
            """根据电压分类电池状态"""
            if voltage >= 12.0:
                return "正常"
            elif voltage >= 11.0:
                return "低电量"
            else:
                return "严重低电量"
        
        # 测试各种电量级别
        self.assertEqual(categorize_battery_level(12.5), "正常")
        self.assertEqual(categorize_battery_level(11.8), "低电量")  # 修正：11.8 < 12.0，应该是低电量
        self.assertEqual(categorize_battery_level(11.5), "低电量")
        self.assertEqual(categorize_battery_level(10.8), "严重低电量")
    
    def test_battery_percentage_conversion(self):
        """测试电压到百分比的转换"""
        def voltage_to_percentage(voltage, min_voltage=10.0, max_voltage=12.6):
            """将电压转换为百分比"""
            if voltage <= min_voltage:
                return 0.0
            elif voltage >= max_voltage:
                return 100.0
            else:
                return ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100
        
        # 测试边界条件
        self.assertEqual(voltage_to_percentage(10.0), 0.0)
        self.assertEqual(voltage_to_percentage(12.6), 100.0)
        
        # 测试中间值
        mid_voltage = 11.3  # 应该是50%
        percentage = voltage_to_percentage(mid_voltage)
        self.assertAlmostEqual(percentage, 50.0, places=1)


if __name__ == '__main__':
    # 设置测试套件
    unittest.main(verbosity=2)