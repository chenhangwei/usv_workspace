"""
info_panel_styles 模块单元测试
"""

import pytest


class TestGetTemperatureStyle:
    """测试 get_temperature_style 函数"""
    
    def test_normal_temperature(self):
        """测试正常温度"""
        from gs_gui.info_panel_styles import get_temperature_style
        fg, bg, is_high = get_temperature_style(35.0, is_high_temperature=False)
        assert isinstance(fg, str)
        assert isinstance(bg, str)
        assert is_high is False
    
    def test_high_temperature(self):
        """测试高温"""
        from gs_gui.info_panel_styles import get_temperature_style
        fg, bg, is_high = get_temperature_style(70.0, is_high_temperature=False)
        assert isinstance(fg, str)
        assert isinstance(bg, str)
        assert is_high is True  # 70°C > 65°C 触发高温
    
    def test_temperature_hysteresis(self):
        """测试温度滞后效果"""
        from gs_gui.info_panel_styles import get_temperature_style
        # 已处于高温状态，58°C 仍保持高温
        fg, bg, is_high = get_temperature_style(58.0, is_high_temperature=True)
        assert is_high is True  # 58°C > 55°C，仍保持高温
        
        # 降到 50°C 恢复正常
        fg, bg, is_high = get_temperature_style(50.0, is_high_temperature=True)
        assert is_high is False  # 50°C < 55°C，恢复正常
    
    def test_invalid_temperature(self):
        """测试无效温度值"""
        from gs_gui.info_panel_styles import get_temperature_style
        fg, bg, is_high = get_temperature_style(None, is_high_temperature=False)
        assert isinstance(fg, str)
        assert isinstance(bg, str)
        assert is_high is False
