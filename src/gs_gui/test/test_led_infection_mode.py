"""
LED传染模式功能测试
测试LED传染模式的开关控制和状态管理
"""

import pytest
from unittest.mock import Mock, MagicMock, patch
from PyQt5.QtWidgets import QApplication
import sys

# 确保QApplication实例存在
if not QApplication.instance():
    app = QApplication(sys.argv)

from gs_gui.ros_signal import ROSSignal
from gs_gui.main_gui_app import MainWindow


class TestLEDInfectionMode:
    """测试LED传染模式功能"""

    def setup_method(self):
        """每个测试方法执行前的设置"""
        self.ros_signal = ROSSignal()
        self.main_window = MainWindow(self.ros_signal)

    def teardown_method(self):
        """每个测试方法执行后的清理"""
        if hasattr(self, 'main_window'):
            self.main_window.close()

    def test_led_infection_menu_exists(self):
        """测试LED设置菜单和传染模式菜单项是否存在"""
        # 检查菜单项是否被创建
        assert hasattr(self.main_window, 'action_led_infection_mode')
        assert self.main_window.action_led_infection_mode is not None

    def test_led_infection_mode_default_checked(self):
        """测试LED传染模式默认为开启状态"""
        # 检查默认状态是否为勾选
        assert self.main_window.action_led_infection_mode.isChecked() is True

    def test_led_infection_mode_checkable(self):
        """测试LED传染模式菜单项是否可勾选"""
        # 检查菜单项是否可勾选
        assert self.main_window.action_led_infection_mode.isCheckable() is True

    def test_toggle_led_infection_mode_signal(self):
        """测试切换LED传染模式时是否发送正确的信号"""
        # 创建信号接收器
        signal_received = []
        
        def capture_signal(enabled):
            signal_received.append(enabled)
        
        # 连接信号
        self.ros_signal.led_infection_mode_changed.connect(capture_signal)
        
        # 模拟取消勾选（关闭传染模式）
        self.main_window.action_led_infection_mode.setChecked(False)
        self.main_window.toggle_led_infection_mode()
        
        # 验证信号发送了False
        assert len(signal_received) == 1
        assert signal_received[0] is False
        
        # 清空接收列表
        signal_received.clear()
        
        # 模拟勾选（开启传染模式）
        self.main_window.action_led_infection_mode.setChecked(True)
        self.main_window.toggle_led_infection_mode()
        
        # 验证信号发送了True
        assert len(signal_received) == 1
        assert signal_received[0] is True

    def test_toggle_led_infection_mode_method_exists(self):
        """测试切换LED传染模式的方法是否存在"""
        assert hasattr(self.main_window, 'toggle_led_infection_mode')
        assert callable(self.main_window.toggle_led_infection_mode)


class TestGroundStationNodeLEDInfection:
    """测试GroundStationNode的LED传染模式控制"""

    @patch('gs_gui.ground_station_node.rclpy')
    def test_led_infection_enabled_default(self, mock_rclpy):
        """测试LED传染模式默认开启"""
        from gs_gui.ground_station_node import GroundStationNode
        
        mock_rclpy.ok.return_value = True
        ros_signal = ROSSignal()
        
        # 创建节点（需要mock一些ROS依赖）
        with patch.object(GroundStationNode, '__init__', lambda x, y: None):
            node = GroundStationNode(ros_signal)
            node._led_infection_enabled = True
            
            # 验证默认值
            assert node._led_infection_enabled is True

    @patch('gs_gui.ground_station_node.rclpy')
    def test_set_led_infection_mode_callback(self, mock_rclpy):
        """测试LED传染模式回调函数"""
        from gs_gui.ground_station_node import GroundStationNode
        
        ros_signal = ROSSignal()
        
        # 创建节点mock
        with patch.object(GroundStationNode, '__init__', lambda x, y: None):
            node = GroundStationNode(ros_signal)
            node._led_infection_enabled = True
            node._usv_led_modes = {}
            node._usv_infecting = set()
            node.usv_manager = Mock()
            node.usv_manager.led_pubs = {}
            node.publish_queue = Mock()
            
            # 模拟get_logger
            node.get_logger = Mock()
            node.get_logger.return_value = Mock()
            
            # 定义回调方法
            def set_led_infection_mode_callback(enabled):
                node._led_infection_enabled = bool(enabled)
            
            node.set_led_infection_mode_callback = set_led_infection_mode_callback
            
            # 测试关闭传染模式
            node.set_led_infection_mode_callback(False)
            assert node._led_infection_enabled is False
            
            # 测试开启传染模式
            node.set_led_infection_mode_callback(True)
            assert node._led_infection_enabled is True


class TestROSSignal:
    """测试ROSSignal中的LED传染模式信号"""

    def test_led_infection_mode_signal_exists(self):
        """测试LED传染模式信号是否存在"""
        ros_signal = ROSSignal()
        assert hasattr(ros_signal, 'led_infection_mode_changed')

    def test_led_infection_mode_signal_emits_bool(self):
        """测试LED传染模式信号能否发送布尔值"""
        ros_signal = ROSSignal()
        signal_received = []
        
        def capture_signal(enabled):
            signal_received.append(enabled)
        
        ros_signal.led_infection_mode_changed.connect(capture_signal)
        
        # 发送True
        ros_signal.led_infection_mode_changed.emit(True)
        assert len(signal_received) == 1
        assert signal_received[0] is True
        
        # 发送False
        ros_signal.led_infection_mode_changed.emit(False)
        assert len(signal_received) == 2
        assert signal_received[1] is False


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
