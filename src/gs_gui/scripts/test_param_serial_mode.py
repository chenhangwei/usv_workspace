#!/usr/bin/env python3
"""
参数配置串口模式测试脚本

直接启动参数窗口（无需地面站主程序）
"""

import sys
from PyQt5.QtWidgets import QApplication

# 导入串口参数窗口
from gs_gui.param_window_serial import ParamWindowSerial


def main():
    """主函数"""
    print("=" * 60)
    print("参数配置窗口 - 串口模式测试")
    print("=" * 60)
    print()
    print("功能测试：")
    print("  1. 菜单栏 → 连接 → 连接飞控")
    print("  2. 选择串口设备和波特率")
    print("  3. 连接后自动加载参数")
    print("  4. 编辑参数值")
    print("  5. 菜单栏 → 参数 → 保存修改")
    print()
    print("按 Ctrl+C 退出")
    print("=" * 60)
    print()
    
    app = QApplication(sys.argv)
    
    # 创建参数窗口
    window = ParamWindowSerial()
    window.show()
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
