#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Unit tests for usv_info_panel.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
USV 信息面板测试
测试新的美化信息显示面板
"""
import sys
import pytest
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
from gs_gui.usv_info_panel import UsvInfoPanel


@pytest.fixture(scope="module")
def qapp():
    """创建 QApplication 实例"""
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    yield app
    # 不要在这里调用 app.quit()，因为可能有其他测试需要它


class TestUsvInfoPanel:
    """测试 USV 信息面板"""
    
    def test_panel_creation(self, qapp):
        """测试面板创建"""
        panel = UsvInfoPanel()
        assert panel is not None
        assert panel.id_label is not None
        assert panel.battery_bar is not None
    
    def test_update_full_state(self, qapp):
        """测试完整状态更新"""
        panel = UsvInfoPanel()
        
        test_state = {
            'namespace': 'usv_01',
            'mode': 'GUIDED',
            'connected': True,
            'armed': True,
            'position': {'x': 10.5, 'y': -5.2, 'z': 0.3},
            'yaw': 0.8,
            'battery_percentage': 75.0,
            'battery_voltage': 12.6,
            'battery_current': 2.3,
            'temperature': 42.5,
            'gps_satellites_visible': 12,
            'gps_eph': 0.8,
            'velocity': {'linear': {'x': 1.1, 'y': 1.0}},
            'vehicle_messages': [],
            'prearm_ready': True,
            'prearm_warnings': [],
            'sensor_status': [{'name': 'GPS Fix', 'status': '3D Fix', 'detail': '', 'level': 'ok'}]
        }
        
        panel.update_state(test_state)
        
        # 验证基本信息
        assert panel.id_label.text() == 'usv_01'
        assert panel.mode_label.text() == 'GUIDED'
        assert panel.status_label.text() == '在线'
        assert '解锁' in panel.armed_label.text()
        
        # 验证位置信息
        assert '10.50' in panel.x_label.text()
        assert '-5.20' in panel.y_label.text()
        assert '0.30' in panel.z_label.text()
        assert panel.yaw_label.text() != '--'
        
        # 验证电池信息
        assert panel.battery_bar.value() == 75
        assert '12.60' in panel.voltage_label.text()
        assert panel.current_label.text().startswith('2.3')
        
        # 验证 GPS 信息
        assert panel.satellite_label.text() == '12'
        assert '0.80' in panel.gps_accuracy_label.text()
        
        # 验证速度信息
        assert panel.ground_speed_label.text() != '--'
        assert panel.heading_speed_label.text() != '--'

        # 验证 Ready 按钮（armed=True 时应显示 'Armed'）
        assert panel.ready_button.text() == 'Armed'
        assert '预检' not in panel.ready_summary_label.text()
        assert panel.sensor_list.count() == 1
    
    def test_update_partial_state(self, qapp):
        """测试部分状态更新（缺少某些字段）"""
        panel = UsvInfoPanel()
        
        # 只包含部分字段的状态
        partial_state = {
            'namespace': 'usv_02',
            'mode': 'MANUAL',
            'position': {'x': 5.0, 'y': 3.0}
            # 缺少其他字段
        }
        
        panel.update_state(partial_state)
        
        # 应该显示有的字段
        assert panel.id_label.text() == 'usv_02'
        assert panel.mode_label.text() == 'MANUAL'
        assert '5.00' in panel.x_label.text()
        assert '3.00' in panel.y_label.text()
        
        # 缺少的字段应该显示 "--"
        assert panel.z_label.text() == '--'
        assert panel.voltage_label.text() == '--'
    
    def test_clear_display(self, qapp):
        """测试清空显示"""
        panel = UsvInfoPanel()
        
        # 先填充数据
        test_state = {
            'namespace': 'usv_01',
            'mode': 'GUIDED',
            'position': {'x': 10.0, 'y': 5.0, 'z': 0.0}
        }
        panel.update_state(test_state)
        assert panel.id_label.text() == 'usv_01'
        
        # 清空
        panel.update_state(None)
        
        # 验证已清空
        assert panel.id_label.text() == '--'
        assert panel.mode_label.text() == '--'
        assert panel.x_label.text() == '--'
        assert panel.battery_bar.value() == 0
    
    def test_battery_style_updates(self, qapp):
        """测试电池样式根据百分比变化"""
        panel = UsvInfoPanel()
        
        # 测试高电量（绿色）
        panel.update_state({'battery_percentage': 80})
        assert panel.battery_bar.value() == 80
        
        # 测试中等电量（橙色）
        panel.update_state({'battery_percentage': 45})
        assert panel.battery_bar.value() == 45
        
        # 测试低电量（红色）
        panel.update_state({'battery_percentage': 15})
        assert panel.battery_bar.value() == 15
    
    def test_mode_style_updates(self, qapp):
        """测试模式样式变化"""
        panel = UsvInfoPanel()
        
        # 测试不同模式
        modes = ['GUIDED', 'MANUAL', 'AUTO', 'UNKNOWN']
        for mode in modes:
            panel.update_state({'mode': mode})
            assert panel.mode_label.text() == mode
    
    def test_gps_satellite_style(self, qapp):
        """测试 GPS 卫星数量样式"""
        panel = UsvInfoPanel()
        
        # 测试良好信号（>=10）
        panel.update_state({'gps_satellites_visible': 12})
        assert panel.satellite_label.text() == '12'
        
        # 测试中等信号（6-9）
        panel.update_state({'gps_satellites_visible': 7})
        assert panel.satellite_label.text() == '7'
        
        # 测试弱信号（<6）
        panel.update_state({'gps_satellites_visible': 4})
        assert panel.satellite_label.text() == '4'

    def test_ready_view_updates(self, qapp):
        """测试 Ready 状态按钮和警告列表"""
        panel = UsvInfoPanel()

        # 所有预检通过
        panel.update_state({
            'namespace': 'usv_01',
            'connected': True,
            'prearm_ready': True,
            'prearm_warnings': [],
            'sensor_status': []
        })
        assert panel.ready_button.text() == 'Ready to Sail'
        assert panel.warning_list.count() == 1

        # 存在预检警告
        panel.update_state({
            'namespace': 'usv_01',
            'connected': True,
            'prearm_ready': False,
            'prearm_warnings': ['PreArm: MAG orientation inconsistent'],
            'sensor_status': []
        })
        assert 'PreArm' in panel.ready_button.text()
        assert panel.warning_list.count() == 1

        # 离线状态
        panel.update_state({
            'namespace': 'usv_01',
            'connected': False,
            'sensor_status': []
        })
        assert panel.ready_button.text() == 'USV 离线'
    
    def test_format_float(self, qapp):
        """测试浮点数格式化"""
        panel = UsvInfoPanel()
        
        # 正常数值
        assert panel._format_float(1.234, 2) == '1.23'
        assert panel._format_float(10.0, 1) == '10.0'
        
        # None 和 '--'
        assert panel._format_float(None) == '--'
        assert panel._format_float('--') == '--'
        
        # 异常值
        assert panel._format_float('invalid') == '--'
    
    def test_armed_state_display(self, qapp):
        """测试解锁状态显示"""
        panel = UsvInfoPanel()
        
        # 测试已解锁
        panel.update_state({'armed': True})
        assert '解锁' in panel.armed_label.text()
        
        panel.update_state({'armed': 'ARMED'})
        assert '解锁' in panel.armed_label.text()
        
        # 测试已锁定
        panel.update_state({'armed': False})
        assert '锁定' in panel.armed_label.text()
        
        panel.update_state({'armed': 'DISARMED'})
        assert '锁定' in panel.armed_label.text()


def test_visual_display():
    """可视化测试（手动运行）"""
    app = QApplication(sys.argv)
    
    panel = UsvInfoPanel()
    panel.setWindowTitle("USV 信息面板测试")
    panel.setMinimumSize(400, 750)
    panel.show()
    
    # 创建测试数据序列
    test_states = [
        {
            'namespace': 'usv_01',
            'connected': True,
            'mode': 'GUIDED',
            'status': 'ACTIVE',
            'armed': True,
            'position': {'x': 10.5, 'y': -5.2, 'z': 0.3},
            'yaw': 45.6,
            'battery_percentage': 85.0,
            'battery_voltage': 12.8,
            'battery_current': 2.1,
            'temperature': 41.0,
            'gps_satellites_visible': 14,
            'gps_eph': 0.5,
            'velocity': {'linear': {'x': 1.2, 'y': 1.4}},
            'prearm_ready': True,
            'prearm_warnings': [],
            'sensor_status': [{'name': 'Battery', 'status': 'OK', 'detail': '85%', 'level': 'ok'}]
        },
        {
            'namespace': 'usv_02',
            'connected': True,
            'mode': 'MANUAL',
            'status': 'STANDBY',
            'armed': False,
            'position': {'x': 5.0, 'y': 3.0, 'z': 0.1},
            'yaw': 120.0,
            'battery_percentage': 45.0,
            'battery_voltage': 11.5,
            'battery_current': 1.5,
            'temperature': 48.0,
            'gps_satellites_visible': 8,
            'gps_eph': 1.2,
            'velocity': {'linear': {'x': 0.2, 'y': 0.4}},
            'prearm_ready': False,
            'prearm_warnings': ['PreArm: waiting for GPS lock'],
            'sensor_status': [{'name': 'GPS Fix', 'status': '2D Fix', 'detail': '8 sats', 'level': 'warn'}]
        },
        {
            'namespace': 'usv_03',
            'connected': True,
            'mode': 'AUTO',
            'status': 'CRITICAL',
            'armed': True,
            'position': {'x': -2.5, 'y': 8.0, 'z': 0.0},
            'yaw': 270.0,
            'battery_percentage': 25.0,
            'battery_voltage': 10.8,
            'battery_current': 3.2,
            'temperature': 55.0,
            'gps_satellites_visible': 5,
            'gps_eph': 2.5,
            'velocity': {'linear': {'x': 0.1, 'y': 0.1}},
            'prearm_ready': False,
            'prearm_warnings': ['PreArm: battery too low'],
            'sensor_status': [{'name': 'Battery', 'status': 'Low', 'detail': '25%', 'level': 'error'}]
        }
    ]
    
    # 使用定时器切换状态
    current_index = [0]
    
    def update_test_data():
        panel.update_state(test_states[current_index[0]])
        current_index[0] = (current_index[0] + 1) % len(test_states)
    
    # 初始显示
    update_test_data()
    
    # 每3秒切换一次状态
    timer = QTimer()
    timer.timeout.connect(update_test_data)
    timer.start(3000)
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    # 运行可视化测试
    # python3 test_usv_info_panel.py
    test_visual_display()
