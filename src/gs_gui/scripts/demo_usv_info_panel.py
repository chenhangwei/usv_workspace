#!/usr/bin/env python3
"""
USV 信息面板快速演示
独立运行，无需完整 ROS 环境
"""
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
from pathlib import Path

# 添加模块路径
sys.path.insert(0, str(Path(__file__).parent.parent / 'gs_gui'))

from usv_info_panel import UsvInfoPanel


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
                'mode': 'GUIDED',
                'status': 'ACTIVE',
                'armed': True,
                'position': {'x': 10.52, 'y': -5.23, 'z': 0.35},
                'yaw': 45.6,
                'battery_percentage': 85.0,
                'voltage': 12.8,
                'current': 2.1,
                'gps_satellite_count': 14,
                'gps_accuracy': 0.5,
                'ground_speed': 1.8,
                'heading': 48.2
            },
            {
                'namespace': 'usv_02',
                'mode': 'MANUAL',
                'status': 'STANDBY',
                'armed': False,
                'position': {'x': 5.00, 'y': 3.00, 'z': 0.10},
                'yaw': 120.0,
                'battery_percentage': 45.0,
                'voltage': 11.5,
                'current': 1.5,
                'gps_satellite_count': 8,
                'gps_accuracy': 1.2,
                'ground_speed': 0.5,
                'heading': 125.0
            },
            {
                'namespace': 'usv_03',
                'mode': 'AUTO',
                'status': 'CRITICAL',
                'armed': True,
                'position': {'x': -2.55, 'y': 8.03, 'z': 0.02},
                'yaw': 270.0,
                'battery_percentage': 15.0,
                'voltage': 10.8,
                'current': 3.2,
                'gps_satellite_count': 5,
                'gps_accuracy': 2.5,
                'ground_speed': 2.5,
                'heading': 268.0
            },
            {
                'namespace': 'usv_04',
                'mode': 'HOLD',
                'status': 'EMERGENCY',
                'armed': True,
                'position': {'x': 0.00, 'y': 0.00, 'z': 0.00},
                'yaw': 0.0,
                'battery_percentage': 5.0,
                'voltage': 10.2,
                'current': 0.5,
                'gps_satellite_count': 3,
                'gps_accuracy': 5.0,
                'ground_speed': 0.0,
                'heading': 0.0
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
        self.statusBar().showMessage("演示模式：每3秒自动切换 USV 状态")
    
    def update_display(self):
        """更新显示"""
        state = self.test_states[self.current_index]
        self.info_panel.update_state(state)
        
        # 更新窗口标题
        self.setWindowTitle(f"USV 信息面板演示 - {state['namespace']} ({self.current_index + 1}/{len(self.test_states)})")
        
        # 更新状态栏
        status_msg = f"当前显示: {state['namespace']} | 模式: {state['mode']} | 电池: {state['battery_percentage']}%"
        self.statusBar().showMessage(status_msg)
        
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
