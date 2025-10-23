"""
测试 Area Offset Dialog 功能
"""

import sys
from PyQt5.QtWidgets import QApplication
from gs_gui.area_offset_dialog import AreaOffsetDialog


def test_area_offset_dialog():
    """测试Area偏移量对话框"""
    app = QApplication(sys.argv)
    
    # 测试1: 默认值（0,0,0）
    dialog1 = AreaOffsetDialog()
    assert dialog1.get_offset() == {'x': 0.0, 'y': 0.0, 'z': 0.0}
    print("✓ 测试1通过: 默认值为(0,0,0)")
    
    # 测试2: 自定义初始值
    initial_offset = {'x': 10.5, 'y': -20.3, 'z': 5.0}
    dialog2 = AreaOffsetDialog(current_offset=initial_offset)
    result = dialog2.get_offset()
    assert result['x'] == 10.5
    assert result['y'] == -20.3
    assert result['z'] == 5.0
    print("✓ 测试2通过: 自定义初始值正确")
    
    # 测试3: 验证spinbox范围
    dialog3 = AreaOffsetDialog()
    assert dialog3.x_spinbox.minimum() == -10000.0
    assert dialog3.x_spinbox.maximum() == 10000.0
    assert dialog3.z_spinbox.minimum() == -1000.0
    assert dialog3.z_spinbox.maximum() == 1000.0
    print("✓ 测试3通过: spinbox范围正确")
    
    print("\n所有测试通过！")


if __name__ == '__main__':
    test_area_offset_dialog()
