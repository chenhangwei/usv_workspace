#!/usr/bin/env python3
"""
Phase 3.3-3.5 综合功能演示

演示：
- Phase 3.3: 参数对比（默认值对比、差异统计、TOP 差异）
- Phase 3.4: 高级搜索（名称、描述、正则、组合搜索）
- Phase 3.5: 实时监控（变化追踪、历史记录、日志导出）
"""

import os
import sys
import tempfile
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from gs_gui.param_manager import ParamInfo, ParamType
from gs_gui.param_compare import ParamCompare, DiffType
from gs_gui.param_search import AdvancedSearch
from gs_gui.param_monitor import ParamMonitor, ParamChangeEvent


def create_sample_params():
    """创建示例参数"""
    params = {
        "GPS_TYPE": ParamInfo(
            name="GPS_TYPE", value=5.0, original_value=1.0,
            param_type=ParamType.INTEGER, description="GPS接收器类型",
            unit="", min_value=0.0, max_value=19.0
        ),
        "BATT_CAPACITY": ParamInfo(
            name="BATT_CAPACITY", value=8000.0, original_value=5000.0,
            param_type=ParamType.REAL, description="电池容量",
            unit="mAh", min_value=0.0, max_value=100000.0
        ),
        "ARMING_CHECK": ParamInfo(
            name="ARMING_CHECK", value=1.0, original_value=1.0,
            param_type=ParamType.INTEGER, description="解锁检查位掩码",
            unit="", min_value=0.0, max_value=255.0
        ),
        "ARMING_VOLT_MIN": ParamInfo(
            name="ARMING_VOLT_MIN", value=12.0, original_value=11.5,
            param_type=ParamType.REAL, description="最低解锁电压",
            unit="V", min_value=0.0, max_value=50.0
        ),
        "COMPASS_USE": ParamInfo(
            name="COMPASS_USE", value=1.0, original_value=1.0,
            param_type=ParamType.INTEGER, description="使用罗盘",
            unit="", min_value=0.0, max_value=1.0
        ),
        "WP_SPEED": ParamInfo(
            name="WP_SPEED", value=2.5, original_value=2.0,
            param_type=ParamType.REAL, description="航点速度",
            unit="m/s", min_value=0.0, max_value=10.0
        ),
    }
    return params


def demo_phase33_compare():
    """演示 Phase 3.3: 参数对比"""
    print("=" * 70)
    print("Phase 3.3: 参数对比功能演示")
    print("=" * 70)
    
    params = create_sample_params()
    
    # 1. 默认值对比
    print("\n1. 对比当前值和原始值")
    print("-" * 70)
    
    # 创建对比数据（模拟默认值对比）
    default_params = {}
    for name, param in params.items():
        default_param = ParamInfo(
            name=name, value=param.original_value, original_value=param.original_value,
            param_type=param.param_type, description=param.description,
            unit=param.unit, min_value=param.min_value, max_value=param.max_value
        )
        default_params[name] = default_param
    
    diffs = ParamCompare.compare_two_param_sets(
        params, default_params,
        left_label="当前值", right_label="原始值"
    )
    
    # 统计
    stats = ParamCompare.get_statistics(diffs)
    print(f"总参数: {stats['total']}")
    print(f"  • 相同: {stats['same']}")
    print(f"  • 不同: {stats['different']}")
    print()
    
    # 显示差异
    different_diffs = [d for d in diffs if d.diff_type == DiffType.DIFFERENT]
    if different_diffs:
        print("差异参数:")
        for diff in different_diffs:
            percent = f"{diff.diff_percent:.1f}%" if diff.diff_percent else "N/A"
            print(f"  • {diff.param_name}: {diff.left_value:.6g} → {diff.right_value:.6g} ({percent})")
    
    # 2. TOP 差异
    print("\n2. TOP 差异参数（按百分比）")
    print("-" * 70)
    
    top_diffs = ParamCompare.get_top_diffs(different_diffs, top_n=3, by="percent")
    for i, diff in enumerate(top_diffs, 1):
        percent = f"{diff.diff_percent:.1f}%" if diff.diff_percent else "N/A"
        print(f"{i}. {diff.param_name}: 变化 {percent}")
    
    # 3. 同步脚本
    print("\n3. 生成同步脚本")
    print("-" * 70)
    
    sync_list = ParamCompare.generate_sync_script(diffs, direction="right_to_left")
    print(f"需要同步 {len(sync_list)} 个参数:")
    for param_name, new_value in sync_list[:3]:
        print(f"  • {param_name} = {new_value:.6g}")
    
    print()


def demo_phase34_search():
    """演示 Phase 3.4: 高级搜索"""
    print("=" * 70)
    print("Phase 3.4: 高级搜索功能演示")
    print("=" * 70)
    
    params = create_sample_params()
    
    # 1. 按名称搜索
    print("\n1. 按名称搜索 'ARMING'")
    print("-" * 70)
    
    results = AdvancedSearch.search_by_name(params, "ARMING")
    print(f"找到 {len(results)} 个参数:")
    for name in results:
        print(f"  • {name}")
    
    # 2. 按描述搜索
    print("\n2. 按描述搜索 '电池'")
    print("-" * 70)
    
    results = AdvancedSearch.search_by_description(params, "电池")
    print(f"找到 {len(results)} 个参数:")
    for name in results:
        print(f"  • {name}: {params[name].description}")
    
    # 3. 正则表达式搜索
    print("\n3. 正则表达式搜索 '^GPS.*|^COMPASS.*'")
    print("-" * 70)
    
    results = AdvancedSearch.search_by_regex(params, r"^GPS.*|^COMPASS.*", search_in="name")
    print(f"找到 {len(results)} 个参数:")
    for name in results:
        print(f"  • {name}")
    
    # 4. 按分组过滤
    print("\n4. 按分组过滤 'ARMING'")
    print("-" * 70)
    
    results = AdvancedSearch.filter_by_group(params, "ARMING")
    print(f"找到 {len(results)} 个参数:")
    for name in results:
        print(f"  • {name}")
    
    # 5. 过滤修改的参数
    print("\n5. 过滤修改的参数")
    print("-" * 70)
    
    results = AdvancedSearch.filter_by_modified(params, modified_only=True)
    print(f"找到 {len(results)} 个已修改参数:")
    for name in results:
        param = params[name]
        print(f"  • {name}: {param.original_value:.6g} → {param.value:.6g}")
    
    # 6. 按值范围过滤
    print("\n6. 按值范围过滤 (0 < value < 5)")
    print("-" * 70)
    
    results = AdvancedSearch.filter_by_value_range(params, min_value=0, max_value=5)
    print(f"找到 {len(results)} 个参数:")
    for name in results:
        print(f"  • {name}: {params[name].value:.6g}")
    
    # 7. 组合搜索
    print("\n7. 组合搜索（名称包含'ARMING' AND 已修改）")
    print("-" * 70)
    
    results = AdvancedSearch.combined_search(
        params,
        name_keyword="ARMING",
        modified_only=True
    )
    print(f"找到 {len(results)} 个参数:")
    for name in results:
        print(f"  • {name}")
    
    print()


def demo_phase35_monitor():
    """演示 Phase 3.5: 实时监控"""
    print("=" * 70)
    print("Phase 3.5: 实时监控功能演示")
    print("=" * 70)
    
    params = create_sample_params()
    
    # 1. 创建监控器
    print("\n1. 创建监控器并注册回调")
    print("-" * 70)
    
    monitor = ParamMonitor(max_history=100)
    
    # 注册回调
    def on_param_changed(event: ParamChangeEvent):
        percent = f"{event.change_percent:.1f}%" if event.change_percent else "N/A"
        print(f"  [回调] {event.param_name}: {event.old_value:.6g} → {event.new_value:.6g} ({percent})")
    
    monitor.register_callback(on_param_changed)
    print("✅ 监控器已创建，回调已注册")
    
    # 2. 开始监控参数
    print("\n2. 开始监控参数")
    print("-" * 70)
    
    for name, param in params.items():
        monitor.watch_param(name, param.original_value)
    print(f"✅ 正在监控 {len(params)} 个参数")
    
    # 3. 模拟参数变化
    print("\n3. 模拟参数变化")
    print("-" * 70)
    
    print("变化 1: GPS_TYPE 1 → 5")
    params["GPS_TYPE"].value = 5.0
    monitor.check_changes(params, source="manual")
    
    print("变化 2: BATT_CAPACITY 5000 → 8000")
    params["BATT_CAPACITY"].value = 8000.0
    monitor.check_changes(params, source="manual")
    
    print("变化 3: ARMING_VOLT_MIN 11.5 → 12.0")
    params["ARMING_VOLT_MIN"].value = 12.0
    monitor.check_changes(params, source="import")
    
    # 4. 查看历史
    print("\n4. 查看变化历史")
    print("-" * 70)
    
    recent_changes = monitor.get_recent_changes(limit=10)
    print(f"最近 {len(recent_changes)} 次变化:")
    for event in recent_changes:
        print(f"  • [{event.timestamp.strftime('%H:%M:%S')}] {event.param_name}: "
              f"{event.old_value:.6g} → {event.new_value:.6g} (来源: {event.source})")
    
    # 5. 参数统计
    print("\n5. 参数变化统计")
    print("-" * 70)
    
    for name in ["GPS_TYPE", "BATT_CAPACITY", "ARMING_VOLT_MIN"]:
        count = monitor.get_change_count(name)
        print(f"  • {name}: {count} 次变化")
    
    total_changes = monitor.get_change_count()
    print(f"\n总变化次数: {total_changes}")
    
    # 6. 导出日志
    print("\n6. 导出变化日志")
    print("-" * 70)
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
        log_file = f.name
    
    success = monitor.export_log(log_file)
    if success:
        print(f"✅ 日志已导出到: {log_file}")
        print("\n日志内容预览:")
        with open(log_file, 'r') as f:
            lines = f.readlines()
            for line in lines[:10]:
                print(f"  {line.rstrip()}")
        
        # 清理
        os.remove(log_file)
    
    print()


def main():
    """主函数"""
    print("\n" + "=" * 70)
    print("Phase 3.3-3.5 综合功能演示")
    print("=" * 70)
    print(f"⏰ 演示时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    demo_phase33_compare()
    demo_phase34_search()
    demo_phase35_monitor()
    
    print("=" * 70)
    print("✅ 所有演示完成！")
    print("=" * 70)
    print("\n💡 功能特性总结：")
    print("\n📊 Phase 3.3 - 参数对比：")
    print("   1. ✅ 默认值对比")
    print("   2. ✅ USV 间对比")
    print("   3. ✅ 差异统计")
    print("   4. ✅ TOP 差异分析")
    print("   5. ✅ 同步脚本生成")
    print("\n🔍 Phase 3.4 - 高级搜索：")
    print("   1. ✅ 名称搜索")
    print("   2. ✅ 描述搜索")
    print("   3. ✅ 正则表达式")
    print("   4. ✅ 分组过滤")
    print("   5. ✅ 修改状态过滤")
    print("   6. ✅ 值范围过滤")
    print("   7. ✅ 组合搜索（AND）")
    print("\n📈 Phase 3.5 - 实时监控：")
    print("   1. ✅ 变化检测")
    print("   2. ✅ 回调通知")
    print("   3. ✅ 历史记录")
    print("   4. ✅ 变化统计")
    print("   5. ✅ 日志导出")
    print()


if __name__ == '__main__':
    main()
