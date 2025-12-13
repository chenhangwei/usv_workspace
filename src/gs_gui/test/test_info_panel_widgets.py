"""
info_panel_widgets 模块单元测试
"""

import pytest
from unittest.mock import MagicMock, patch

# 模拟 PyQt5
with patch.dict('sys.modules', {
    'PyQt5': MagicMock(),
    'PyQt5.QtWidgets': MagicMock(),
    'PyQt5.QtCore': MagicMock(),
    'PyQt5.QtGui': MagicMock(),
}):
    from gs_gui.info_panel_widgets import format_float


class TestFormatFloat:
    """测试 format_float 函数"""
    
    def test_format_float_normal(self):
        """测试正常浮点数格式化"""
        assert format_float(3.14159, precision=2) == "3.14"
        assert format_float(10.0, precision=2) == "10.00"
        assert format_float(0.123456, precision=4) == "0.1235"
    
    def test_format_float_none(self):
        """测试 None 值"""
        assert format_float(None) == "--"
    
    def test_format_float_nan(self):
        """测试 NaN 值"""
        import math
        result = format_float(math.nan)
        # NaN 可能返回 "nan" 或 "--"
        assert result in ("nan", "--", "N/A")
    
    def test_format_float_inf(self):
        """测试无穷大"""
        import math
        result = format_float(math.inf)
        # inf 可能返回 "inf" 或 "--"
        assert result in ("inf", "--", "N/A")
        result_neg = format_float(-math.inf)
        assert result_neg in ("-inf", "--", "N/A")
    
    def test_format_float_zero(self):
        """测试零值"""
        assert format_float(0.0, precision=2) == "0.00"
    
    def test_format_float_negative(self):
        """测试负数"""
        assert format_float(-3.14159, precision=2) == "-3.14"


class TestGroupboxStyle:
    """测试 GROUPBOX_STYLE 常量"""
    
    def test_groupbox_style_exists(self):
        """测试样式常量存在"""
        from gs_gui.info_panel_widgets import GROUPBOX_STYLE
        assert isinstance(GROUPBOX_STYLE, str)
        assert "QGroupBox" in GROUPBOX_STYLE
