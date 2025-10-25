#!/usr/bin/env python3
"""
温度滞后逻辑测试

测试温度颜色切换的滞后效果，确保：
1. 温度 >= 50°C 时切换到红色
2. 温度 < 48°C 时切换到绿色
3. 在 48-50°C 之间保持当前颜色不变
"""

import unittest
from PyQt5.QtWidgets import QApplication
import sys


class MockTemperatureLabel:
    """模拟 QLabel 对象"""
    def __init__(self):
        self.style = ""
    
    def setStyleSheet(self, style):
        self.style = style


class TemperatureHysteresisLogic:
    """
    温度滞后逻辑类（从 usv_info_panel.py 提取）
    用于独立测试
    """
    def __init__(self):
        self._is_high_temperature = False
        self.temperature_label = MockTemperatureLabel()
        self.color_history = []  # 记录颜色变化历史
    
    def _update_temperature_style(self, temp_celsius):
        """
        更新温度显示的颜色（带滞后效果）
        """
        try:
            temp = float(temp_celsius)
            
            # 滞后逻辑实现
            if self._is_high_temperature:
                # 当前是高温状态（红色）
                if temp < 48:  # 温度降到48°C以下才切换到绿色
                    color = "#27ae60"  # 绿色
                    self._is_high_temperature = False
                else:
                    color = "#e74c3c"  # 保持红色
            else:
                # 当前是低温状态（绿色）
                if temp >= 50:  # 温度升到50°C及以上才切换到红色
                    color = "#e74c3c"  # 红色
                    self._is_high_temperature = True
                else:
                    color = "#27ae60"  # 保持绿色
            
            self.temperature_label.setStyleSheet(f"""
                QLabel {{
                    color: {color};
                    font-weight: bold;
                    font-size: 14px;
                }}
            """)
            
            # 记录颜色变化
            self.color_history.append({
                'temp': temp,
                'color': color,
                'state': self._is_high_temperature
            })
            
        except (ValueError, TypeError):
            self.temperature_label.setStyleSheet("")
    
    def get_current_color(self):
        """提取当前颜色"""
        style = self.temperature_label.style
        if "#27ae60" in style:
            return "green"
        elif "#e74c3c" in style:
            return "red"
        return "unknown"
    
    def reset(self):
        """重置状态"""
        self._is_high_temperature = False
        self.color_history = []


class TestTemperatureHysteresis(unittest.TestCase):
    """温度滞后逻辑测试类"""
    
    def setUp(self):
        """每个测试前初始化"""
        self.logic = TemperatureHysteresisLogic()
    
    def test_initial_state(self):
        """测试初始状态"""
        self.assertFalse(self.logic._is_high_temperature)
    
    def test_normal_temperature_green(self):
        """测试正常温度显示绿色"""
        temps = [20, 30, 40, 45, 49]
        for temp in temps:
            self.logic._update_temperature_style(temp)
            self.assertEqual(self.logic.get_current_color(), "green",
                           f"温度 {temp}°C 应该显示绿色")
    
    def test_rising_through_threshold(self):
        """测试温度上升穿过阈值"""
        temps = [45, 48, 49, 50, 51]
        expected_colors = ["green", "green", "green", "red", "red"]
        
        for temp, expected in zip(temps, expected_colors):
            self.logic._update_temperature_style(temp)
            actual = self.logic.get_current_color()
            self.assertEqual(actual, expected,
                           f"温度 {temp}°C 应该是 {expected}，实际是 {actual}")
    
    def test_falling_through_threshold(self):
        """测试温度下降穿过阈值"""
        # 先升温到红色状态
        self.logic._update_temperature_style(55)
        self.assertEqual(self.logic.get_current_color(), "red")
        
        # 然后降温
        temps = [52, 50, 49, 48, 47]
        expected_colors = ["red", "red", "red", "red", "green"]  # 48°C 还是红色，47°C 才切换
        
        for temp, expected in zip(temps, expected_colors):
            self.logic._update_temperature_style(temp)
            actual = self.logic.get_current_color()
            self.assertEqual(actual, expected,
                           f"温度 {temp}°C 应该是 {expected}，实际是 {actual}")
    
    def test_dead_zone_from_green(self):
        """测试从绿色进入死区（48-50°C）"""
        # 初始为绿色
        self.logic._update_temperature_style(45)
        self.assertEqual(self.logic.get_current_color(), "green")
        
        # 在死区内波动，应保持绿色
        dead_zone_temps = [48.0, 48.5, 49.0, 49.5, 49.9]
        for temp in dead_zone_temps:
            self.logic._update_temperature_style(temp)
            self.assertEqual(self.logic.get_current_color(), "green",
                           f"从绿色进入死区，温度 {temp}°C 应保持绿色")
    
    def test_dead_zone_from_red(self):
        """测试从红色进入死区（48-50°C）"""
        # 先升到红色
        self.logic._update_temperature_style(52)
        self.assertEqual(self.logic.get_current_color(), "red")
        
        # 在死区内波动，应保持红色
        dead_zone_temps = [50.0, 49.5, 49.0, 48.5, 48.1]
        for temp in dead_zone_temps:
            self.logic._update_temperature_style(temp)
            self.assertEqual(self.logic.get_current_color(), "red",
                           f"从红色进入死区，温度 {temp}°C 应保持红色")
    
    def test_oscillation_prevention(self):
        """测试防止振荡（关键测试）"""
        # 场景：温度在 49-51°C 之间快速波动
        oscillating_temps = [49, 50, 49, 51, 49, 50, 48, 49, 50, 47]
        expected_colors = [
            "green",  # 49: 初始绿色
            "red",    # 50: 触发切换到红色
            "red",    # 49: 死区内保持红色
            "red",    # 51: 保持红色
            "red",    # 49: 死区内保持红色
            "red",    # 50: 保持红色
            "red",    # 48: 死区内保持红色（48°C 还不够低）
            "red",    # 49: 死区内保持红色
            "red",    # 50: 保持红色
            "green"   # 47: < 48°C，触发切换到绿色
        ]
        
        for temp, expected in zip(oscillating_temps, expected_colors):
            self.logic._update_temperature_style(temp)
            actual = self.logic.get_current_color()
            self.assertEqual(actual, expected,
                           f"振荡测试：温度 {temp}°C 应该是 {expected}，实际是 {actual}")
    
    def test_state_transitions(self):
        """测试状态转换计数"""
        # 温度序列
        temps = [30, 40, 49, 50, 51, 50, 49, 48, 47, 48, 49, 50]
        
        self.logic.reset()
        for temp in temps:
            self.logic._update_temperature_style(temp)
        
        # 统计状态转换次数
        transitions = 0
        for i in range(1, len(self.logic.color_history)):
            if self.logic.color_history[i]['state'] != self.logic.color_history[i-1]['state']:
                transitions += 1
        
        # 预期转换：绿→红（在50）, 红→绿（在47）, 绿→红（在50）
        # 总共 3 次转换
        self.assertEqual(transitions, 3,
                        f"应该有 3 次状态转换，实际有 {transitions} 次")
    
    def test_extreme_temperatures(self):
        """测试极端温度"""
        # 极低温度
        self.logic._update_temperature_style(-10)
        self.assertEqual(self.logic.get_current_color(), "green")
        
        # 极高温度
        self.logic._update_temperature_style(100)
        self.assertEqual(self.logic.get_current_color(), "red")
        
        # 从极高温降到极低温
        self.logic._update_temperature_style(0)
        self.assertEqual(self.logic.get_current_color(), "green")
    
    def test_exact_threshold_values(self):
        """测试精确阈值"""
        # 测试 50.0°C（上限阈值）
        self.logic._update_temperature_style(49.99)
        self.assertEqual(self.logic.get_current_color(), "green")
        
        self.logic._update_temperature_style(50.0)
        self.assertEqual(self.logic.get_current_color(), "red")
        
        # 重置后测试 48.0°C（下限阈值）
        self.logic.reset()
        self.logic._update_temperature_style(55)  # 先到红色
        self.logic._update_temperature_style(48.01)
        self.assertEqual(self.logic.get_current_color(), "red")
        
        self.logic._update_temperature_style(48.0)
        self.assertEqual(self.logic.get_current_color(), "red")  # 48.0 还是红色
        
        self.logic._update_temperature_style(47.99)
        self.assertEqual(self.logic.get_current_color(), "green")  # < 48 才切换
    
    def test_invalid_temperatures(self):
        """测试无效温度输入"""
        # 先设置一个有效温度（建立初始状态）
        self.logic._update_temperature_style(40)
        
        invalid_temps = [None, "abc", ""]
        
        for invalid in invalid_temps:
            try:
                self.logic._update_temperature_style(invalid)
                # 应该不抛出异常，样式被清空
                self.assertEqual(self.logic.temperature_label.style, "")
            except Exception as e:
                self.fail(f"处理无效温度 {invalid} 时抛出异常: {e}")
        
        # 特殊数值测试（inf 和 nan 可以转 float，但逻辑上应该处理）
        # 注意：float('inf') 和 float('nan') 可以成功转换，所以会触发逻辑
        # 这里我们只测试真正会导致 ValueError 的情况


class TestColorTransitionPath(unittest.TestCase):
    """测试颜色转换路径"""
    
    def setUp(self):
        self.logic = TemperatureHysteresisLogic()
    
    def test_slow_heating_cooling_cycle(self):
        """测试慢速加热-冷却循环"""
        # 模拟真实场景：温度逐渐上升再下降
        temps = list(range(30, 61, 1)) + list(range(60, 29, -1))
        
        self.logic.reset()
        for temp in temps:
            self.logic._update_temperature_style(temp)
        
        # 验证关键转换点
        history = self.logic.color_history
        
        # 找到第一次变红（应该在 50°C）
        first_red_idx = next(i for i, h in enumerate(history) if h['color'] == '#e74c3c')
        self.assertEqual(history[first_red_idx]['temp'], 50,
                        "第一次变红应该在 50°C")
        
        # 找到第一次变回绿（应该在 47°C）
        red_indices = [i for i, h in enumerate(history) if h['color'] == '#e74c3c']
        last_red_idx = red_indices[-1]
        first_green_after_red_idx = last_red_idx + 1
        
        if first_green_after_red_idx < len(history):
            first_green_after_red = history[first_green_after_red_idx]['temp']
            self.assertLess(first_green_after_red, 48,
                           "从红色切回绿色应该在 < 48°C")


def run_visual_demo():
    """运行可视化演示（在终端显示）"""
    print("\n" + "="*60)
    print("温度滞后逻辑可视化演示")
    print("="*60)
    
    logic = TemperatureHysteresisLogic()
    
    # 场景 1: 逐渐升温
    print("\n【场景 1】温度逐渐上升:")
    print(f"{'温度(°C)':<10} {'状态标志':<10} {'显示颜色':<10}")
    print("-" * 30)
    
    for temp in [30, 40, 45, 48, 49, 50, 51, 55]:
        logic._update_temperature_style(temp)
        state_str = "高温(T)" if logic._is_high_temperature else "低温(F)"
        color_str = "🔴 红色" if logic.get_current_color() == "red" else "🟢 绿色"
        print(f"{temp:<10} {state_str:<10} {color_str:<10}")
    
    # 场景 2: 逐渐降温
    print("\n【场景 2】温度逐渐下降:")
    print(f"{'温度(°C)':<10} {'状态标志':<10} {'显示颜色':<10}")
    print("-" * 30)
    
    for temp in [55, 52, 50, 49, 48, 47, 45, 40]:
        logic._update_temperature_style(temp)
        state_str = "高温(T)" if logic._is_high_temperature else "低温(F)"
        color_str = "🔴 红色" if logic.get_current_color() == "red" else "🟢 绿色"
        print(f"{temp:<10} {state_str:<10} {color_str:<10}")
    
    # 场景 3: 在死区波动
    print("\n【场景 3】在死区(48-50°C)内波动:")
    print(f"{'温度(°C)':<10} {'状态标志':<10} {'显示颜色':<10} {'备注':<20}")
    print("-" * 60)
    
    logic.reset()
    logic._update_temperature_style(45)  # 初始绿色
    print(f"{45:<10} {'低温(F)':<10} {'🟢 绿色':<10} {'初始状态':<20}")
    
    oscillation = [49, 50, 49.5, 49, 48.5, 49, 50, 48, 47.5]
    for temp in oscillation:
        logic._update_temperature_style(temp)
        state_str = "高温(T)" if logic._is_high_temperature else "低温(F)"
        color_str = "🔴 红色" if logic.get_current_color() == "red" else "🟢 绿色"
        
        # 添加备注
        remark = ""
        if temp == 50 and state_str == "高温(T)":
            remark = "⚠️ 触发阈值，切换到红色"
        elif temp < 48 and state_str == "低温(F)":
            remark = "✅ 触发阈值，切换到绿色"
        elif 48 <= temp < 50:
            remark = "🔒 死区，保持当前颜色"
        
        print(f"{temp:<10} {state_str:<10} {color_str:<10} {remark:<20}")
    
    print("\n" + "="*60)


if __name__ == '__main__':
    # 创建 QApplication（PyQt5 需要）
    app = QApplication(sys.argv)
    
    # 运行可视化演示
    run_visual_demo()
    
    # 运行单元测试
    print("\n运行单元测试...\n")
    unittest.main(argv=[''], verbosity=2, exit=False)
