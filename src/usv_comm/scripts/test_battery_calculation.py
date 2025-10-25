#!/usr/bin/env python3
"""
电池电量计算验证脚本

测试不同电压下的电量百分比计算是否正确
"""


def calculate_battery_percentage(voltage, v_full=12.6, v_empty=11.1):
    """
    计算电池电量百分比
    
    Args:
        voltage: 当前电压（V）
        v_full: 满电电压（V），默认 12.6
        v_empty: 空电电压（V），默认 11.1
    
    Returns:
        电量百分比 (0-100)
    """
    if voltage >= v_full:
        return 100.0
    elif voltage <= v_empty:
        return 0.0
    else:
        voltage_range = v_full - v_empty
        voltage_above_empty = voltage - v_empty
        battery_pct = (voltage_above_empty / voltage_range) * 100.0
        return max(0.0, min(100.0, battery_pct))


def test_battery_calculation():
    """测试电池电量计算"""
    print("\n" + "="*70)
    print("电池电量计算验证")
    print("="*70)
    print(f"\n配置参数:")
    print(f"  满电电压: 12.6V (100%)")
    print(f"  空电电压: 11.1V (0%)")
    print(f"  电压范围: 1.5V")
    
    # 测试用例
    test_cases = [
        # (电压, 预期电量, 状态描述)
        (13.0, 100, "充电中/刚充满"),
        (12.6, 100, "满电"),
        (12.45, 90, "接近满电"),
        (12.3, 80, "良好"),
        (12.15, 70, "良好"),
        (12.0, 60, "正常"),
        (11.85, 50, "中等"),
        (11.7, 40, "中等"),
        (11.55, 30, "偏低"),
        (11.4, 20, "低电量 ⚠️"),
        (11.25, 10, "低电量 ⚠️"),
        (11.1, 0, "空电 🔴 需充电"),
        (11.0, 0, "过放 ⚠️ 风险"),
        (10.5, 0, "严重过放 ⚠️"),
    ]
    
    print(f"\n{'电压(V)':<10} {'实际电量(%)':<15} {'预期电量(%)':<15} {'误差':<10} {'状态':<20}")
    print("-" * 70)
    
    total_error = 0.0
    max_error = 0.0
    
    for voltage, expected_pct, status in test_cases:
        actual_pct = calculate_battery_percentage(voltage)
        error = abs(actual_pct - expected_pct)
        total_error += error
        max_error = max(max_error, error)
        
        error_str = f"{error:.1f}%" if error > 0.1 else "✓"
        status_color = ""
        if actual_pct <= 20:
            status_color = "🔴"
        elif actual_pct <= 40:
            status_color = "🟡"
        else:
            status_color = "🟢"
        
        print(f"{voltage:<10.2f} {actual_pct:<15.1f} {expected_pct:<15.1f} {error_str:<10} {status_color} {status:<18}")
    
    avg_error = total_error / len(test_cases)
    
    print("\n" + "="*70)
    print(f"测试统计:")
    print(f"  测试用例数: {len(test_cases)}")
    print(f"  平均误差: {avg_error:.2f}%")
    print(f"  最大误差: {max_error:.2f}%")
    
    if max_error < 1.0:
        print(f"  结果: ✅ 所有测试通过（误差 < 1%）")
    else:
        print(f"  结果: ⚠️ 存在误差 > 1% 的测试用例")
    
    print("="*70)


def test_user_reported_case():
    """测试用户报告的实际情况"""
    print("\n" + "="*70)
    print("用户报告案例验证")
    print("="*70)
    
    voltage = 11.4
    old_percentage = 96.0  # 错误的显示值
    new_percentage = calculate_battery_percentage(voltage)
    
    print(f"\nUSV 实际情况:")
    print(f"  电压: {voltage}V")
    print(f"  修改前显示: {old_percentage}% ❌ 错误！")
    print(f"  修改后显示: {new_percentage:.1f}% ✅ 正确")
    
    print(f"\n分析:")
    print(f"  1. 电压 {voltage}V 在满电(12.6V)和空电(11.1V)之间")
    print(f"  2. 距离空电仅 {voltage - 11.1:.1f}V")
    print(f"  3. 占总范围的 {new_percentage:.1f}%")
    print(f"  4. 状态: {'🔴 低电量，建议充电' if new_percentage <= 20 else '正常'}")
    
    print("="*70)


def test_different_battery_types():
    """测试不同类型电池"""
    print("\n" + "="*70)
    print("不同电池类型测试")
    print("="*70)
    
    battery_types = [
        ("3S 锂电池", 12.6, 11.1),
        ("4S 锂电池", 16.8, 14.0),
        ("2S 锂电池", 8.4, 7.0),
        ("铅酸电池 12V", 13.2, 10.5),
    ]
    
    test_voltages = [0.0, 0.2, 0.5, 0.8, 1.0, 1.2]  # 相对于空电的电压差
    
    for battery_name, v_full, v_empty in battery_types:
        print(f"\n{battery_name}:")
        print(f"  满电: {v_full}V, 空电: {v_empty}V, 范围: {v_full - v_empty:.1f}V")
        print(f"\n  {'电压(V)':<12} {'电量(%)':<12} {'状态':<15}")
        print(f"  {'-'*40}")
        
        for delta in test_voltages:
            voltage = v_empty + delta * (v_full - v_empty)
            pct = calculate_battery_percentage(voltage, v_full, v_empty)
            
            if pct <= 20:
                status = "🔴 低电量"
            elif pct <= 40:
                status = "🟡 偏低"
            elif pct <= 80:
                status = "🟢 正常"
            else:
                status = "🟢 良好"
            
            print(f"  {voltage:<12.2f} {pct:<12.1f} {status:<15}")
    
    print("\n" + "="*70)


def generate_voltage_table():
    """生成电压-电量对照表"""
    print("\n" + "="*70)
    print("电压-电量对照表（3S 锂电池）")
    print("="*70)
    print("\n可以打印此表格贴在设备上作为快速参考\n")
    
    print(f"{'电压(V)':<10} {'电量(%)':<12} {'状态指示':<15} {'建议操作':<20}")
    print("-" * 70)
    
    # 生成从 10.5V 到 13.0V 的对照表，步进 0.1V
    for voltage_int in range(105, 131):
        voltage = voltage_int / 10.0
        pct = calculate_battery_percentage(voltage)
        
        # 状态指示
        if pct >= 80:
            status = "🟢🟢🟢 满电"
            action = "可正常使用"
        elif pct >= 60:
            status = "🟢🟢 良好"
            action = "可正常使用"
        elif pct >= 40:
            status = "🟢 正常"
            action = "可继续使用"
        elif pct >= 20:
            status = "🟡 偏低"
            action = "准备充电"
        elif pct > 0:
            status = "🔴 低电量"
            action = "⚠️ 立即充电"
        else:
            status = "🔴🔴 空电"
            action = "⚠️⚠️ 禁止使用"
        
        print(f"{voltage:<10.1f} {pct:<12.1f} {status:<15} {action:<20}")
    
    print("\n" + "="*70)
    print("注意事项:")
    print("  1. 电压测量应在低负载或静止状态下进行")
    print("  2. 温度会影响电压读数（低温电压偏低）")
    print("  3. 电量低于 20% 时应尽快充电")
    print("  4. 切勿将电池放电至 11.0V 以下（会损坏电池）")
    print("="*70)


if __name__ == '__main__':
    # 运行所有测试
    test_battery_calculation()
    test_user_reported_case()
    test_different_battery_types()
    generate_voltage_table()
    
    print("\n✅ 所有验证完成！\n")
