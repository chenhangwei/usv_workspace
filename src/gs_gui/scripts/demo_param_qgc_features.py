#!/usr/bin/env python3
"""
参数元数据和验证功能演示脚本

展示新增的 QGC 同级功能
"""

from gs_gui.param_metadata import get_param_metadata, load_all_metadata
from gs_gui.param_manager import ParamInfo, ParamType
from gs_gui.param_validator import ParamValidator


def demo_metadata():
    """演示参数元数据功能"""
    print("=" * 60)
    print("参数元数据演示")
    print("=" * 60)
    
    # 加载元数据
    load_all_metadata()
    print("✓ 元数据已加载\n")
    
    # 测试几个常用参数
    test_params = [
        "GPS_TYPE",
        "ARMING_CHECK",
        "BATT_CAPACITY",
        "FRAME_TYPE"
    ]
    
    for param_name in test_params:
        print(f"\n{'─' * 60}")
        print(f"参数: {param_name}")
        print(f"{'─' * 60}")
        
        meta = get_param_metadata(param_name)
        if meta:
            print(f"显示名称: {meta.display_name}")
            print(f"描述: {meta.description}")
            print(f"详细说明: {meta.user_description}")
            print(f"单位: {meta.unit or '(无)'}")
            print(f"范围: {meta.min_value} ~ {meta.max_value}")
            print(f"默认值: {meta.default_value}")
            print(f"步进值: {meta.increment or '(无)'}")
            print(f"需要重启: {'是' if meta.reboot_required else '否'}")
            
            if meta.values:
                print(f"枚举值:")
                for val, desc in sorted(meta.values.items()):
                    print(f"  {val}: {desc}")
            
            if meta.bitmask:
                print(f"位掩码:")
                for bit, desc in sorted(meta.bitmask.items()):
                    print(f"  Bit {bit}: {desc}")
        else:
            print("❌ 无元数据")


def demo_validation():
    """演示参数验证功能"""
    print("\n" + "=" * 60)
    print("参数验证演示")
    print("=" * 60)
    
    # 创建测试参数
    test_cases = [
        # (参数名, 类型, 最小值, 最大值, 步进值, 测试值, 预期结果)
        ("GPS_TYPE", ParamType.INTEGER, 0, 19, None, 1.5, False),  # 整数参数不接受小数
        ("GPS_TYPE", ParamType.INTEGER, 0, 19, None, 1.0, True),
        ("GPS_TYPE", ParamType.INTEGER, 0, 19, None, 25.0, False),  # 超出范围
        ("ARMING_VOLT_MIN", ParamType.REAL, 0.0, 30.0, 0.1, 10.5, True),
        ("ARMING_VOLT_MIN", ParamType.REAL, 0.0, 30.0, 0.1, 35.0, False),  # 超出范围
    ]
    
    for param_name, ptype, min_val, max_val, inc, test_val, expected in test_cases:
        print(f"\n测试: {param_name} = {test_val}")
        
        # 创建参数对象
        param = ParamInfo(
            name=param_name,
            value=0.0,
            original_value=0.0,
            param_type=ptype,
            min_value=min_val,
            max_value=max_val,
            increment=inc
        )
        
        # 验证
        valid, error_msg = ParamValidator.validate(param, test_val)
        
        result_symbol = "✓" if valid == expected else "✗"
        print(f"  {result_symbol} 验证结果: {'有效' if valid else '无效'}")
        
        if not valid:
            print(f"  错误: {error_msg}")
        
        # 建议值
        suggested = ParamValidator.suggest_valid_value(param, test_val)
        if suggested != test_val:
            print(f"  💡 建议值: {suggested}")
        
        # 值描述
        desc = ParamValidator.get_value_description(param, test_val)
        if desc:
            print(f"  📝 值描述: {desc}")


def demo_warnings():
    """演示警告系统"""
    print("\n" + "=" * 60)
    print("警告系统演示")
    print("=" * 60)
    
    test_cases = [
        ("ARMING_CHECK", 0.0, "禁用所有检查"),
        ("FRAME_TYPE", 2.0, "修改机架类型"),
        ("SYSID_THISMAV", 2.0, "修改 MAVLink ID"),
        ("BATT_CAPACITY", 5000.0, "修改电池容量"),
    ]
    
    for param_name, value, action in test_cases:
        print(f"\n{action}: {param_name} = {value}")
        
        # 创建参数
        param = ParamInfo(
            name=param_name,
            value=value,
            original_value=0.0,
            param_type=ParamType.INTEGER
        )
        
        # 获取警告级别
        level = ParamValidator.get_warning_level(param, value)
        level_names = ["无警告", "提示", "警告", "严重警告"]
        print(f"  警告级别: {level} ({level_names[level]})")
        
        # 获取警告消息
        msg = ParamValidator.get_warning_message(param, value)
        if msg:
            print(f"  {msg}")


def demo_value_descriptions():
    """演示值描述功能"""
    print("\n" + "=" * 60)
    print("值描述演示")
    print("=" * 60)
    
    # GPS_TYPE 枚举值
    print("\nGPS_TYPE 枚举值:")
    param = ParamInfo(
        name="GPS_TYPE",
        value=0.0,
        original_value=0.0,
        param_type=ParamType.INTEGER
    )
    
    for val in [0, 1, 2, 5, 9]:
        param.value = float(val)
        desc = ParamValidator.get_value_description(param, float(val))
        print(f"  {val}: {desc}")
    
    # ARMING_CHECK 位掩码
    print("\nARMING_CHECK 位掩码示例:")
    param = ParamInfo(
        name="ARMING_CHECK",
        value=0.0,
        original_value=0.0,
        param_type=ParamType.INTEGER
    )
    
    test_values = [
        (1, "启用所有检查"),
        (7, "GPS + Compass + Barometer"),
        (15, "前4项检查"),
    ]
    
    for val, note in test_values:
        desc = ParamValidator.get_value_description(param, float(val))
        print(f"  {val} ({note}): {desc}")


def main():
    """主函数"""
    print("\n🚀 USV 参数管理 QGC 同级功能演示\n")
    
    try:
        demo_metadata()
        demo_validation()
        demo_warnings()
        demo_value_descriptions()
        
        print("\n" + "=" * 60)
        print("✅ 演示完成！")
        print("=" * 60)
        print("\n下一步：")
        print("1. 启动地面站: ros2 launch gs_bringup gs_launch.py")
        print("2. 打开参数配置窗口，体验新功能")
        print("3. 查看 PARAM_QGC_UPGRADE_GUIDE.md 了解更多")
        
    except Exception as e:
        print(f"\n❌ 演示过程中出错: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
