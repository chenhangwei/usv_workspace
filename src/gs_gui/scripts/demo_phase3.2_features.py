#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of demo phase3.2 features.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
Phase 3.2 参数导入/导出功能演示

演示：
1. .param 格式导出
2. JSON 格式导出（带/不带元数据）
3. .param 格式导入
4. JSON 格式导入
5. 文件信息读取
6. 导入验证和冲突检测
"""

import os
import sys
import tempfile
from datetime import datetime

# 添加 gs_gui 到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from gs_gui.param_manager import ParamInfo, ParamType
from gs_gui.param_import_export import ParamImportExport


def create_sample_params():
    """创建示例参数字典"""
    params = {
        "GPS_TYPE": ParamInfo(
            name="GPS_TYPE",
            value=1.0,
            original_value=1.0,
            param_type=ParamType.INTEGER,
            description="GPS接收器类型",
            unit="",
            min_value=0.0,
            max_value=19.0
        ),
        "BATT_CAPACITY": ParamInfo(
            name="BATT_CAPACITY",
            value=5000.0,
            original_value=5000.0,
            param_type=ParamType.REAL,
            description="电池容量",
            unit="mAh",
            min_value=0.0,
            max_value=100000.0
        ),
        "ARMING_CHECK": ParamInfo(
            name="ARMING_CHECK",
            value=1.0,
            original_value=1.0,
            param_type=ParamType.INTEGER,
            description="解锁检查位掩码",
            unit="",
            min_value=0.0,
            max_value=255.0
        ),
        "ARMING_VOLT_MIN": ParamInfo(
            name="ARMING_VOLT_MIN",
            value=11.5,
            original_value=11.5,
            param_type=ParamType.REAL,
            description="最低解锁电压",
            unit="V",
            min_value=0.0,
            max_value=50.0
        ),
    }
    return params


def demo_param_export():
    """演示 .param 格式导出"""
    print("=" * 70)
    print("1. 演示 .param 格式导出")
    print("=" * 70)
    
    params = create_sample_params()
    
    # 创建临时文件
    with tempfile.NamedTemporaryFile(mode='w', suffix='.param', delete=False) as f:
        temp_file = f.name
    
    try:
        # 导出
        success = ParamImportExport.export_to_param_file(
            params,
            temp_file,
            vehicle_type="USV Rover",
            firmware_version="ArduPilot 4.5.0"
        )
        
        print(f"[OK] 导出状态: {'成功' if success else '失败'}")
        print(f"📄 文件路径: {temp_file}")
        print(f"▪ 参数数量: {len(params)}")
        print()
        
        # 显示文件内容
        print("▪ 文件内容预览：")
        print("-" * 70)
        with open(temp_file, 'r') as f:
            content = f.read()
            print(content)
        print("-" * 70)
        
    finally:
        # 清理
        if os.path.exists(temp_file):
            os.remove(temp_file)
    
    print()


def demo_json_export():
    """演示 JSON 格式导出"""
    print("=" * 70)
    print("2. 演示 JSON 格式导出")
    print("=" * 70)
    
    params = create_sample_params()
    
    # 2.1 带元数据导出
    print("\n2.1 带完整元数据导出")
    print("-" * 70)
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='_with_meta.json', delete=False) as f:
        temp_file1 = f.name
    
    success = ParamImportExport.export_to_json_file(
        params,
        temp_file1,
        include_metadata=True,
        vehicle_type="USV Rover",
        firmware_version="ArduPilot 4.5.0"
    )
    
    print(f"[OK] 导出状态: {'成功' if success else '失败'}")
    print(f"📄 文件路径: {temp_file1}")
    
    # 显示部分内容
    with open(temp_file1, 'r') as f:
        import json
        data = json.load(f)
        print(f"▪ 参数数量: {data['header']['total_params']}")
        print(f"▪ 文件头:")
        for key, value in data['header'].items():
            print(f"   • {key}: {value}")
        print(f"\n▪ 第一个参数 (GPS_TYPE):")
        gps_param = data['parameters'].get('GPS_TYPE', {})
        for key, value in list(gps_param.items())[:5]:
            print(f"   • {key}: {value}")
    
    # 保存文件大小
    size1 = os.path.getsize(temp_file1)
    
    # 2.2 不带元数据导出
    print("\n2.2 不带元数据导出（仅参数值）")
    print("-" * 70)
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='_no_meta.json', delete=False) as f:
        temp_file2 = f.name
    
    try:
        success = ParamImportExport.export_to_json_file(
            params,
            temp_file2,
            include_metadata=False,
            vehicle_type="USV Rover",
            firmware_version="ArduPilot 4.5.0"
        )
        
        print(f"[OK] 导出状态: {'成功' if success else '失败'}")
        print(f"📄 文件路径: {temp_file2}")
        
        # 显示文件大小对比
        size2 = os.path.getsize(temp_file2)
        print(f"📏 文件大小对比:")
        print(f"   • 带元数据: {size1} bytes")
        print(f"   • 不带元数据: {size2} bytes")
        if size1 > 0 and size2 < size1:
            print(f"   • 减少: {size1 - size2} bytes ({(1 - size2/size1)*100:.1f}%)")
        
    finally:
        if os.path.exists(temp_file1):
            os.remove(temp_file1)
        if os.path.exists(temp_file2):
            os.remove(temp_file2)
    
    print()


def demo_file_info():
    """演示文件信息读取"""
    print("=" * 70)
    print("3. 演示文件信息读取")
    print("=" * 70)
    
    params = create_sample_params()
    
    # 创建 .param 文件
    with tempfile.NamedTemporaryFile(mode='w', suffix='.param', delete=False) as f:
        temp_param = f.name
    
    # 创建 .json 文件
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        temp_json = f.name
    
    try:
        # 导出文件
        ParamImportExport.export_to_param_file(
            params, temp_param,
            vehicle_type="USV Test",
            firmware_version="ArduPilot 4.5.0"
        )
        ParamImportExport.export_to_json_file(
            params, temp_json,
            include_metadata=True,
            vehicle_type="USV Test",
            firmware_version="ArduPilot 4.5.0"
        )
        
        # 读取 .param 文件信息
        print("\n3.1 .param 文件信息")
        print("-" * 70)
        info = ParamImportExport.get_file_info(temp_param)
        if info:
            for key, value in info.items():
                print(f"   • {key}: {value}")
        
        # 读取 .json 文件信息
        print("\n3.2 .json 文件信息")
        print("-" * 70)
        info = ParamImportExport.get_file_info(temp_json)
        if info:
            for key, value in info.items():
                print(f"   • {key}: {value}")
        
    finally:
        if os.path.exists(temp_param):
            os.remove(temp_param)
        if os.path.exists(temp_json):
            os.remove(temp_json)
    
    print()


def demo_param_import():
    """演示 .param 格式导入"""
    print("=" * 70)
    print("4. 演示 .param 格式导入")
    print("=" * 70)
    
    # 创建原始参数
    original_params = create_sample_params()
    
    # 修改一些值以便导入
    modified_params = create_sample_params()
    modified_params["GPS_TYPE"].value = 5.0
    modified_params["BATT_CAPACITY"].value = 8000.0
    
    # 导出修改后的参数
    with tempfile.NamedTemporaryFile(mode='w', suffix='.param', delete=False) as f:
        temp_file = f.name
    
    try:
        ParamImportExport.export_to_param_file(modified_params, temp_file)
        
        # 导入到原始参数
        print(f"📥 从文件导入: {temp_file}")
        print(f"▪ 原始参数值:")
        print(f"   • GPS_TYPE: {original_params['GPS_TYPE'].value}")
        print(f"   • BATT_CAPACITY: {original_params['BATT_CAPACITY'].value}")
        print()
        
        result = ParamImportExport.import_from_param_file(
            temp_file,
            original_params,
            validate=True
        )
        
        print(f"[OK] 导入状态: {'成功' if result.success else '失败'}")
        print(f"▪ 导入结果:")
        print(f"   • 导入参数: {result.imported_count}")
        print(f"   • 跳过参数: {result.skipped_count}")
        print(f"   • 错误参数: {result.error_count}")
        print(f"   • 冲突参数: {len(result.conflicts)}")
        print()
        
        if result.conflicts:
            print(f"[!] 冲突详情:")
            for param_name, file_value, current_value in result.conflicts:
                print(f"   • {param_name}: {current_value} → {file_value}")
            print()
        
        print(f"▪ 导入后参数值:")
        print(f"   • GPS_TYPE: {original_params['GPS_TYPE'].value}")
        print(f"   • BATT_CAPACITY: {original_params['BATT_CAPACITY'].value}")
        
    finally:
        if os.path.exists(temp_file):
            os.remove(temp_file)
    
    print()


def demo_import_validation():
    """演示导入验证"""
    print("=" * 70)
    print("5. 演示导入验证和错误处理")
    print("=" * 70)
    
    params = create_sample_params()
    
    # 创建包含错误的 .param 文件
    with tempfile.NamedTemporaryFile(mode='w', suffix='.param', delete=False) as f:
        temp_file = f.name
        f.write("# Test param file\n")
        f.write("GPS_TYPE,1\n")
        f.write("BATT_CAPACITY,5000\n")
        f.write("INVALID_PARAM,123\n")  # 不存在的参数
        f.write("GPS_TYPE,999\n")  # 超出范围
        f.write("MALFORMED_LINE\n")  # 格式错误
    
    try:
        print(f"📥 导入包含错误的文件: {temp_file}")
        print()
        
        result = ParamImportExport.import_from_param_file(
            temp_file,
            params,
            validate=True
        )
        
        print(f"[OK] 导入状态: {'成功' if result.success else '失败'}")
        print(f"▪ 导入结果:")
        print(f"   • 导入参数: {result.imported_count}")
        print(f"   • 跳过参数: {result.skipped_count}")
        print(f"   • 错误参数: {result.error_count}")
        print()
        
        if result.messages:
            print(f"▪ 详细消息:")
            for msg in result.messages:
                print(f"   • {msg}")
        
    finally:
        if os.path.exists(temp_file):
            os.remove(temp_file)
    
    print()


def demo_json_import():
    """演示 JSON 格式导入"""
    print("=" * 70)
    print("6. 演示 JSON 格式导入")
    print("=" * 70)
    
    # 创建原始参数
    original_params = create_sample_params()
    
    # 修改一些值
    modified_params = create_sample_params()
    modified_params["ARMING_VOLT_MIN"].value = 12.0
    modified_params["ARMING_CHECK"].value = 255.0
    
    # 导出 JSON
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        temp_file = f.name
    
    try:
        ParamImportExport.export_to_json_file(
            modified_params,
            temp_file,
            include_metadata=True
        )
        
        print(f"📥 从 JSON 文件导入: {temp_file}")
        print(f"▪ 原始参数值:")
        print(f"   • ARMING_VOLT_MIN: {original_params['ARMING_VOLT_MIN'].value}")
        print(f"   • ARMING_CHECK: {original_params['ARMING_CHECK'].value}")
        print()
        
        result = ParamImportExport.import_from_json_file(
            temp_file,
            original_params,
            validate=True
        )
        
        print(f"[OK] 导入状态: {'成功' if result.success else '失败'}")
        print(f"▪ 导入结果:")
        print(f"   • 导入参数: {result.imported_count}")
        print(f"   • 冲突参数: {len(result.conflicts)}")
        print()
        
        print(f"▪ 导入后参数值:")
        print(f"   • ARMING_VOLT_MIN: {original_params['ARMING_VOLT_MIN'].value}")
        print(f"   • ARMING_CHECK: {original_params['ARMING_CHECK'].value}")
        
    finally:
        if os.path.exists(temp_file):
            os.remove(temp_file)
    
    print()


def main():
    """主函数"""
    print("\n" + "=" * 70)
    print("Phase 3.2 参数导入/导出功能演示")
    print("=" * 70)
    print(f"⏰ 演示时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    demo_param_export()
    demo_json_export()
    demo_file_info()
    demo_param_import()
    demo_import_validation()
    demo_json_import()
    
    print("=" * 70)
    print("[OK] 所有演示完成！")
    print("=" * 70)
    print("\n[*] 功能特性：")
    print("   1. [OK] .param 格式：兼容 QGroundControl")
    print("   2. [OK] JSON 格式：支持完整元数据")
    print("   3. [OK] 文件信息：自动识别格式和版本")
    print("   4. [OK] 导入验证：参数范围和类型检查")
    print("   5. [OK] 冲突检测：显示值变化")
    print("   6. [OK] 错误处理：跳过无效参数，报告详细信息")
    print()


if __name__ == '__main__':
    main()
