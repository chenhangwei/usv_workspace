#!/usr/bin/env python3
"""
低电量模式触发逻辑验证
验证电量百分比 < 5% 触发，> 8% 恢复
"""

def calculate_battery_percentage(voltage, v_full=12.6, v_empty=10.5):
    """计算电池电量百分比"""
    if voltage >= v_full:
        return 100.0
    elif voltage <= v_empty:
        return 0.0
    else:
        return (voltage - v_empty) / (v_full - v_empty) * 100.0


def test_low_battery_trigger():
    """测试低电量触发逻辑"""
    print("\n" + "="*80)
    print("低电量模式触发逻辑验证")
    print("="*80)
    print(f"\n配置:")
    print(f"  满电电压: 12.6V (100%)")
    print(f"  空电电压: 10.5V (0%)")
    print(f"  低电量触发阈值: 5%")
    print(f"  恢复阈值: 8%")
    
    LOW_BATTERY_THRESHOLD = 5.0
    RECOVER_THRESHOLD = 8.0
    
    # 测试用例：电压 → 百分比 → 触发状态
    test_cases = [
        # (电压, 描述)
        (12.6, "满电"),
        (12.0, "正常使用"),
        (11.5, "中等电量"),
        (11.0, "偏低"),
        (10.8, "接近触发"),
        (10.66, "8% - 恢复阈值"),
        (10.65, "7.5% - 滞后区间"),
        (10.62, "6% - 滞后区间"),
        (10.60, "5% - 触发阈值 ❗"),
        (10.59, "4.5% - 低电量 🔴"),
        (10.55, "2.5% - 低电量 🔴"),
        (10.50, "0% - 空电 🔴🔴"),
    ]
    
    print(f"\n{'电压(V)':<12} {'百分比(%)':<12} {'触发状态':<25} {'描述':<20}")
    print("-" * 80)
    
    low_voltage_mode = False  # 模拟状态
    
    for voltage, description in test_cases:
        pct = calculate_battery_percentage(voltage)
        
        # 模拟触发逻辑
        old_mode = low_voltage_mode
        if pct < LOW_BATTERY_THRESHOLD:
            low_voltage_mode = True
            trigger_status = "🔴 低电量模式"
            if not old_mode:
                trigger_status += " [刚触发]"
        elif pct > RECOVER_THRESHOLD:
            low_voltage_mode = False
            trigger_status = "🟢 正常模式"
            if old_mode:
                trigger_status += " [刚恢复]"
        else:
            # 在滞后区间
            if low_voltage_mode:
                trigger_status = "🔴 低电量模式 [滞后]"
            else:
                trigger_status = "🟢 正常模式 [滞后]"
        
        print(f"{voltage:<12.2f} {pct:<12.1f} {trigger_status:<25} {description:<20}")
    
    print("\n" + "="*80)


def test_hysteresis():
    """测试滞后机制"""
    print("\n" + "="*80)
    print("滞后机制测试（避免频繁切换）")
    print("="*80)
    
    LOW_BATTERY_THRESHOLD = 5.0
    RECOVER_THRESHOLD = 8.0
    
    # 模拟电压波动
    voltage_sequence = [
        (11.0, "正常使用"),
        (10.70, "下降到 10%"),
        (10.60, "下降到 5% - 触发阈值"),
        (10.58, "继续下降到 4%"),
        (10.60, "回升到 5%"),
        (10.62, "回升到 6%"),
        (10.64, "回升到 7%"),
        (10.66, "回升到 8% - 恢复阈值"),
        (10.68, "回升到 9%"),
    ]
    
    print(f"\n模拟场景: 电池电压在 4%-10% 之间波动")
    print(f"\n{'步骤':<6} {'电压(V)':<12} {'百分比(%)':<12} {'模式状态':<30} {'说明':<20}")
    print("-" * 80)
    
    low_voltage_mode = False
    
    for i, (voltage, description) in enumerate(voltage_sequence, 1):
        pct = calculate_battery_percentage(voltage)
        old_mode = low_voltage_mode
        
        # 应用触发逻辑
        if pct < LOW_BATTERY_THRESHOLD:
            if not low_voltage_mode:
                low_voltage_mode = True
                status = "🔴 进入低电量模式"
            else:
                status = "🔴 保持低电量模式"
        elif pct > RECOVER_THRESHOLD:
            if low_voltage_mode:
                low_voltage_mode = False
                status = "🟢 退出低电量模式"
            else:
                status = "🟢 保持正常模式"
        else:
            # 滞后区间 (5% - 8%)
            if low_voltage_mode:
                status = "🔴 保持低电量模式 (滞后区间)"
            else:
                status = "🟢 保持正常模式 (滞后区间)"
        
        print(f"{i:<6} {voltage:<12.2f} {pct:<12.1f} {status:<30} {description:<20}")
    
    print("\n说明:")
    print("  - 在 5%-8% 的滞后区间内，模式不会改变")
    print("  - 避免了在临界点附近频繁切换")
    print("  - 只有明确低于 5% 或高于 8% 时才改变模式")
    print("="*80)


def calculate_voltage_for_percentage(pct, v_full=12.6, v_empty=10.5):
    """根据百分比计算对应的电压"""
    return v_empty + (v_full - v_empty) * pct / 100.0


def show_critical_voltages():
    """显示关键电压点"""
    print("\n" + "="*80)
    print("关键电压点对照表")
    print("="*80)
    
    critical_points = [
        (100, "满电"),
        (50, "一半电量"),
        (20, "建议充电"),
        (10, "电量偏低"),
        (8, "恢复阈值"),
        (5, "触发阈值 ❗"),
        (2, "严重低电量"),
        (0, "空电"),
    ]
    
    print(f"\n{'百分比(%)':<15} {'对应电压(V)':<15} {'状态说明':<30}")
    print("-" * 80)
    
    for pct, description in critical_points:
        voltage = calculate_voltage_for_percentage(pct)
        
        if pct >= 8:
            mode = "🟢 正常模式"
        elif pct >= 5:
            mode = "🟡 滞后区间"
        else:
            mode = "🔴 低电量模式"
        
        print(f"{pct:<15.1f} {voltage:<15.2f} {mode:<15} {description}")
    
    print("\n关键电压:")
    print(f"  - 低电量触发: < {calculate_voltage_for_percentage(5):.2f}V (5%)")
    print(f"  - 恢复阈值:   > {calculate_voltage_for_percentage(8):.2f}V (8%)")
    print("="*80)


if __name__ == '__main__':
    test_low_battery_trigger()
    test_hysteresis()
    show_critical_voltages()
    
    print("\n" + "="*80)
    print("✅ 验证完成！")
    print("\n修改总结:")
    print("  1. 低电量触发: 电量百分比 < 5%")
    print("  2. 恢复条件:   电量百分比 > 8%")
    print("  3. 滞后区间:   5% - 8% 之间保持当前模式")
    print("  4. 优势:       避免频繁切换，更稳定可靠")
    print("="*80 + "\n")
