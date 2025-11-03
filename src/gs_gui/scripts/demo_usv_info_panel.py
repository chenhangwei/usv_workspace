#!/usr/bin/env python3
"""
USV 信息面板快速演示
独立运行，无需完整 ROS 环境
"""
import sys
from pathlib import Path
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer

# 添加模块路径
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from gs_gui.usv_info_panel import UsvInfoPanel


class DemoWindow(QMainWindow):
    """演示窗口"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("USV 信息面板演示")
        self.setMinimumSize(450, 800)
        
        # 中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 布局
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # 创建信息面板
        self.info_panel = UsvInfoPanel()
        layout.addWidget(self.info_panel)
        
        # 测试数据集
        self.test_states = [
            {
                'namespace': 'usv_01',
                'connected': True,
                'mode': 'GUIDED',
                'armed': True,
                'position': {'x': 10.52, 'y': -5.23, 'z': 0.35},
                'yaw': 0.9,
                'battery_percentage': 85.0,
                'battery_voltage': 12.8,
                'battery_current': 2.1,
                'temperature': 40.0,
                'gps_satellites_visible': 14,
                'gps_eph': 0.5,
                'velocity': {'linear': {'x': 1.5, 'y': 0.6}},
                'prearm_ready': True,
                'prearm_warnings': [],
                'sensor_status': [
                    {'name': 'Battery', 'status': 'OK', 'detail': '85%', 'level': 'ok'},
                    {'name': 'GPS Fix', 'status': '3D Fix', 'detail': '14 sats', 'level': 'ok'}
                ],
                'vehicle_messages': [
                    {'severity': 6, 'severity_label': 'INFO', 'text': 'Mission uploaded', 'time': '12:01', 'timestamp': 0.0}
                ]
            },
            {
                'namespace': 'usv_02',
                'connected': True,
                'mode': 'MANUAL',
                'armed': False,
                'position': {'x': 5.00, 'y': 3.00, 'z': 0.10},
                'yaw': 2.1,
                'battery_percentage': 45.0,
                'battery_voltage': 11.5,
                'battery_current': 1.5,
                'temperature': 47.0,
                'gps_satellites_visible': 8,
                'gps_eph': 1.2,
                'velocity': {'linear': {'x': 0.2, 'y': 0.5}},
                'prearm_ready': False,
                'prearm_warnings': ['PreArm: waiting for GPS lock'],
                'sensor_status': [
                    {'name': 'GPS Fix', 'status': '2D Fix', 'detail': '8 sats', 'level': 'warn'}
                ],
                'vehicle_messages': [
                    {'severity': 4, 'severity_label': 'WARNING', 'text': 'GPS accuracy poor', 'time': '12:04', 'timestamp': 0.0}
                ]
            },
            {
                'namespace': 'usv_03',
                'connected': True,
                'mode': 'AUTO',
                'armed': True,
                'position': {'x': -2.55, 'y': 8.03, 'z': 0.02},
                'yaw': 3.5,
                'battery_percentage': 25.0,
                'battery_voltage': 10.8,
                'battery_current': 3.2,
                'temperature': 55.0,
                'gps_satellites_visible': 5,
                'gps_eph': 2.5,
                'velocity': {'linear': {'x': 0.1, 'y': 0.1}},
                'prearm_ready': False,
                'prearm_warnings': ['PreArm: battery too low'],
                'sensor_status': [
                    {'name': 'Battery', 'status': 'Low', 'detail': '25%', 'level': 'error'},
                    {'name': 'GPS Fix', 'status': '2D Fix', 'detail': '5 sats', 'level': 'warn'}
                ],
                'vehicle_messages': [
                    {'severity': 3, 'severity_label': 'ERROR', 'text': 'Battery critically low', 'time': '12:07', 'timestamp': 0.0}
                ]
            },
            {
                'namespace': 'usv_04',
                'connected': False,
                'mode': 'HOLD',
                'armed': True,
                'position': {'x': 0.00, 'y': 0.00, 'z': 0.00},
                'yaw': 0.0,
                'battery_percentage': 5.0,
                'battery_voltage': 10.2,
                'battery_current': 0.5,
                'temperature': 60.0,
                'gps_satellites_visible': 3,
                'gps_eph': 5.0,
                'velocity': {'linear': {'x': 0.0, 'y': 0.0}},
                'prearm_ready': False,
                'prearm_warnings': ['PreArm: vehicle offline'],
                'sensor_status': [],
                'vehicle_messages': [
                    {'severity': 2, 'severity_label': 'CRITICAL', 'text': 'Vehicle lost link', 'time': '12:09', 'timestamp': 0.0}
                ]
            }
        ]
        
        self.current_index = 0
        
        # 初始显示
        self.update_display()
        
        # 设置定时器，每3秒切换一次状态
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(3000)
        
        # 状态栏提示
        status_bar = self.statusBar()
        if status_bar is not None:
            status_bar.showMessage("演示模式：每3秒自动切换 USV 状态")
    
    def update_display(self):
        """更新显示"""
        state = self.test_states[self.current_index]
        self.info_panel.update_state(state)
        
        # 更新窗口标题
        self.setWindowTitle(f"USV 信息面板演示 - {state['namespace']} ({self.current_index + 1}/{len(self.test_states)})")
        
        # 更新状态栏
        status_bar = self.statusBar()
        if status_bar is not None:
            status_msg = f"当前显示: {state['namespace']} | 模式: {state.get('mode', '--')} | 电池: {state.get('battery_percentage', '--')}%"
            status_bar.showMessage(status_msg)
        
        # 切换到下一个状态
        self.current_index = (self.current_index + 1) % len(self.test_states)


def main():
    """主函数"""
    print("=" * 70)
    print("USV 信息面板演示")
    print("=" * 70)
    print()
    print("演示功能:")
    print("  ✅ 自动切换 4 种不同的 USV 状态")
    print("  ✅ 展示不同的模式、电池电量、GPS 信号")
    print("  ✅ 智能颜色编码")
    print()
    print("演示状态:")
    print("  1. USV_01: GUIDED 模式, 电池 85%, GPS 信号强 (14 颗卫星)")
    print("  2. USV_02: MANUAL 模式, 电池 45%, GPS 信号中等 (8 颗卫星)")
    print("  3. USV_03: AUTO 模式, 电池 15% (低电量), GPS 信号弱 (5 颗卫星)")
    print("  4. USV_04: HOLD 模式, 电池 5% (危急), GPS 信号很弱 (3 颗卫星)")
    print()
    print("每 3 秒自动切换一次状态，观察颜色变化。")
    print("=" * 70)
    print()
    
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle('Fusion')
    
    window = DemoWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
