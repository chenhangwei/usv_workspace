#!/usr/bin/env python3
"""
从 ArduPilot 官方下载参数元数据并转换为我们的格式

使用方法：
    python3 download_ardupilot_metadata.py [vehicle_type]
    
参数：
    vehicle_type: 可选，默认为 Rover
                  可选值: Rover, Copter, Plane, Sub, Antenna
"""

import sys
import json
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Any


def download_metadata(vehicle_type: str = "Rover") -> str:
    """
    从 ArduPilot 官方下载参数定义 XML
    
    Args:
        vehicle_type: 飞行器类型（Rover, Copter, Plane, Sub, Antenna）
        
    Returns:
        XML 文件内容
    """
    url = f"https://autotest.ardupilot.org/Parameters/{vehicle_type}/apm.pdef.xml"
    
    print(f"📥 正在下载 {vehicle_type} 参数定义...")
    print(f"   URL: {url}")
    
    try:
        with urllib.request.urlopen(url, timeout=30) as response:
            xml_content = response.read().decode('utf-8')
        
        print(f"✅ 下载成功！")
        return xml_content
        
    except Exception as e:
        print(f"❌ 下载失败: {e}")
        raise


def parse_bitmask(bitmask_str: str) -> Dict[int, str]:
    """
    解析位掩码字符串
    
    Args:
        bitmask_str: 格式如 "0:All,1:Barometer,2:Compass"
        
    Returns:
        {0: "All", 1: "Barometer", 2: "Compass"}
    """
    if not bitmask_str:
        return {}
    
    result = {}
    for item in bitmask_str.split(','):
        if ':' in item:
            bit_str, name = item.split(':', 1)
            try:
                result[int(bit_str.strip())] = name.strip()
            except ValueError:
                continue
    
    return result


def parse_values(values_str: str) -> Dict[int, str]:
    """
    解析枚举值字符串
    
    Args:
        values_str: 格式如 "0:None,1:AUTO,2:uBlox"
        
    Returns:
        {0: "None", 1: "AUTO", 2: "uBlox"}
    """
    if not values_str:
        return {}
    
    result = {}
    for item in values_str.split(','):
        if ':' in item:
            val_str, name = item.split(':', 1)
            try:
                result[int(val_str.strip())] = name.strip()
            except ValueError:
                continue
    
    return result


def convert_xml_to_json(xml_content: str) -> Dict[str, Dict[str, Any]]:
    """
    将 ArduPilot XML 元数据转换为我们的 JSON 格式
    
    Args:
        xml_content: XML 文件内容
        
    Returns:
        参数元数据字典
    """
    print("🔄 解析 XML 并转换格式...")
    
    root = ET.fromstring(xml_content)
    metadata = {}
    
    # 遍历所有参数
    for param in root.findall('.//param'):
        name = param.get('name')
        if not name:
            continue
        
        # 提取字段
        humanName = param.get('humanName', '')
        documentation = param.get('documentation', '')
        user = param.get('user', '')
        
        # 范围
        range_elem = param.find('field[@name="Range"]')
        range_str = range_elem.text if range_elem is not None else None
        
        min_value = None
        max_value = None
        if range_str:
            parts = range_str.split()
            if len(parts) >= 2:
                try:
                    min_value = float(parts[0])
                    max_value = float(parts[1])
                except ValueError:
                    pass
        
        # 单位
        units_elem = param.find('field[@name="Units"]')
        unit = units_elem.text if units_elem is not None else ''
        
        # 默认值
        default_elem = param.find('field[@name="Default"]')
        default_value = None
        if default_elem is not None and default_elem.text:
            try:
                default_value = float(default_elem.text)
            except ValueError:
                pass
        
        # 步进
        increment_elem = param.find('field[@name="Increment"]')
        increment = None
        if increment_elem is not None and increment_elem.text:
            try:
                increment = float(increment_elem.text)
            except ValueError:
                pass
        
        # 位掩码
        bitmask_elem = param.find('field[@name="Bitmask"]')
        bitmask = {}
        if bitmask_elem is not None and bitmask_elem.text:
            bitmask = parse_bitmask(bitmask_elem.text)
        
        # 枚举值
        values_elem = param.find('field[@name="Values"]')
        values = {}
        if values_elem is not None and values_elem.text:
            values = parse_values(values_elem.text)
        
        # 重启需求
        reboot_elem = param.find('field[@name="RebootRequired"]')
        reboot_required = False
        if reboot_elem is not None and reboot_elem.text:
            reboot_required = reboot_elem.text.strip().lower() == 'true'
        
        # 只读
        readonly_elem = param.find('field[@name="ReadOnly"]')
        read_only = False
        if readonly_elem is not None and readonly_elem.text:
            read_only = readonly_elem.text.strip().lower() == 'true'
        
        # 构建元数据
        metadata[name] = {
            'display_name': humanName,
            'description': documentation,
            'user_description': user,
            'unit': unit,
            'min_value': min_value,
            'max_value': max_value,
            'default_value': default_value,
            'increment': increment,
            'values': values,
            'bitmask': bitmask,
            'reboot_required': reboot_required,
            'read_only': read_only
        }
    
    print(f"✅ 解析完成！共 {len(metadata)} 个参数")
    return metadata


def save_metadata(metadata: Dict[str, Dict[str, Any]], output_path: Path):
    """
    保存元数据为 JSON 文件
    
    Args:
        metadata: 参数元数据
        output_path: 输出文件路径
    """
    print(f"💾 保存到: {output_path}")
    
    # 确保目录存在
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # 保存为 JSON（带缩进，便于阅读）
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)
    
    print(f"✅ 保存成功！文件大小: {output_path.stat().st_size / 1024:.1f} KB")


def main():
    """主函数"""
    # 解析命令行参数
    vehicle_type = sys.argv[1] if len(sys.argv) > 1 else "Rover"
    
    print("=" * 70)
    print(f"ArduPilot 参数元数据下载器")
    print(f"飞行器类型: {vehicle_type}")
    print("=" * 70)
    
    try:
        # 1. 下载 XML
        xml_content = download_metadata(vehicle_type)
        
        # 2. 转换为 JSON
        metadata = convert_xml_to_json(xml_content)
        
        # 3. 保存文件
        # 获取脚本所在目录的父目录的 resource 目录
        script_dir = Path(__file__).parent
        resource_dir = script_dir.parent / 'resource'
        output_path = resource_dir / 'param_metadata.json'
        
        save_metadata(metadata, output_path)
        
        # 4. 显示统计
        print("\n" + "=" * 70)
        print("📊 统计信息：")
        print(f"   • 参数总数: {len(metadata)}")
        
        # 统计有单位的参数
        with_unit = sum(1 for m in metadata.values() if m['unit'])
        print(f"   • 有单位: {with_unit}")
        
        # 统计有描述的参数
        with_desc = sum(1 for m in metadata.values() if m['description'])
        print(f"   • 有描述: {with_desc}")
        
        # 统计有范围的参数
        with_range = sum(1 for m in metadata.values() if m['min_value'] is not None)
        print(f"   • 有范围: {with_range}")
        
        # 统计有枚举值的参数
        with_values = sum(1 for m in metadata.values() if m['values'])
        print(f"   • 有枚举值: {with_values}")
        
        # 统计有位掩码的参数
        with_bitmask = sum(1 for m in metadata.values() if m['bitmask'])
        print(f"   • 有位掩码: {with_bitmask}")
        
        print("=" * 70)
        print("\n✅ 完成！元数据已保存到:")
        print(f"   {output_path}")
        print("\n💡 重启 GUI 后，所有参数都将显示完整的单位和描述信息。")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
