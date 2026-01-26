#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Unit tests for demo.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
演示测试文件 - 教学用例
这个文件展示了如何为ROS2包编写单元测试
"""

import unittest
from unittest.mock import Mock, patch
import sys
import os

# 添加包路径以便导入
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    # 尝试导入项目模块（如果导入失败也不要紧，这只是演示）
    from gs_gui.ros_signal import ROSSignal
except ImportError:
    ROSSignal = None


class TestROSSignal(unittest.TestCase):
    """测试ROSSignal类的功能"""
    
    def setUp(self):
        """在每个测试方法之前运行"""
        if ROSSignal is not None:
            self.signal = ROSSignal()
    
    @unittest.skipIf(ROSSignal is None, "ROSSignal无法导入")
    def test_signal_creation(self):
        """测试信号对象是否能正确创建"""
        self.assertIsNotNone(self.signal)
        # 检查是否有预期的信号
        self.assertTrue(hasattr(self.signal, 'update_status'))
        self.assertTrue(hasattr(self.signal, 'update_nav_status'))
    
    def test_simple_math(self):
        """简单的数学测试 - 演示基础测试写法"""
        self.assertEqual(2 + 2, 4)
        self.assertNotEqual(2 + 2, 5)
        self.assertTrue(10 > 5)
        self.assertFalse(10 < 5)
    
    def test_string_operations(self):
        """字符串操作测试"""
        usv_id = "usv_01" 
        self.assertTrue(usv_id.startswith("usv_"))
        self.assertEqual(usv_id.split("_")[1], "01")


class TestMockExample(unittest.TestCase):
    """展示如何使用Mock进行测试"""
    
    @patch('builtins.print')
    def test_mock_print(self, mock_print):
        """测试如何Mock内置函数"""
        print("Hello, Test!")
        mock_print.assert_called_once_with("Hello, Test!")
    
    def test_mock_object(self):
        """测试如何Mock对象"""
        # 创建一个Mock对象
        mock_node = Mock()
        mock_node.get_logger.return_value.info = Mock()
        
        # 调用Mock对象的方法
        logger = mock_node.get_logger()
        logger.info("Test message")
        
        # 验证调用
        mock_node.get_logger.assert_called_once()
        logger.info.assert_called_once_with("Test message")


if __name__ == '__main__':
    # 可以直接运行这个文件进行测试
    unittest.main()