"""
参数导入/导出模块

支持多种格式的参数文件导入和导出：
- .param 格式（QGroundControl 兼容）
- .json 格式（包含完整元数据）

提供参数批量导入/导出、验证、冲突处理等功能
"""

import json
import logging
import os
from datetime import datetime
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from .param_manager import ParamInfo
from .param_metadata import get_param_metadata

_logger = logging.getLogger("gs_gui.param_io")
from .param_validator import ParamValidator


@dataclass
class ImportResult:
    """导入结果"""
    success: bool
    imported_count: int
    skipped_count: int
    error_count: int
    messages: List[str]
    conflicts: List[Tuple[str, float, float]]  # (param_name, file_value, current_value)


class ParamImportExport:
    """
    参数导入/导出工具
    
    支持的格式：
    - .param: QGroundControl 格式（纯文本，每行一个参数）
    - .json: JSON 格式（包含元数据）
    """
    
    # QGC .param 文件格式
    PARAM_FORMAT_VERSION = 1
    
    @staticmethod
    def export_to_param_file(params: Dict[str, ParamInfo], 
                            filepath: str,
                            vehicle_type: str = "Rover",
                            firmware_version: str = "Unknown") -> bool:
        """
        导出参数到 .param 文件（QGroundControl 格式）
        
        格式示例：
        # Param file for: Rover
        # Firmware version: ArduPilot 4.5.0
        # Exported: 2025-11-05 14:30:00
        GPS_TYPE,1
        BATT_CAPACITY,5000
        ARMING_CHECK,1
        
        Args:
            params: 参数字典
            filepath: 文件路径
            vehicle_type: 机体类型
            firmware_version: 固件版本
            
        Returns:
            是否成功
        """
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                # 写入文件头
                f.write(f"# Param file for: {vehicle_type}\n")
                f.write(f"# Firmware version: {firmware_version}\n")
                f.write(f"# Exported: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# Total parameters: {len(params)}\n")
                f.write("#\n")
                
                # 写入参数（按名称排序）
                for name in sorted(params.keys()):
                    param = params[name]
                    # QGC 格式：参数名,值
                    f.write(f"{name},{param.value:.6g}\n")
            
            return True
            
        except Exception as e:
            _logger.error(f"导出 .param 文件失败: {e}")
            return False
    
    @staticmethod
    def import_from_param_file(filepath: str,
                               current_params: Dict[str, ParamInfo],
                               validate: bool = True) -> ImportResult:
        """
        从 .param 文件导入参数
        
        Args:
            filepath: 文件路径
            current_params: 当前参数字典
            validate: 是否验证参数值
            
        Returns:
            导入结果
        """
        result = ImportResult(
            success=False,
            imported_count=0,
            skipped_count=0,
            error_count=0,
            messages=[],
            conflicts=[]
        )
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                for line_num, line in enumerate(f, 1):
                    # 跳过注释和空行
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    # 解析参数行：参数名,值
                    parts = line.split(',')
                    if len(parts) != 2:
                        result.messages.append(
                            f"行 {line_num}: 格式错误，跳过：{line}"
                        )
                        result.skipped_count += 1
                        continue
                    
                    param_name = parts[0].strip()
                    try:
                        param_value = float(parts[1].strip())
                    except ValueError:
                        result.messages.append(
                            f"行 {line_num}: 无法解析值，跳过：{line}"
                        )
                        result.error_count += 1
                        continue
                    
                    # 检查参数是否存在
                    if param_name not in current_params:
                        result.messages.append(
                            f"参数 {param_name} 不存在，跳过"
                        )
                        result.skipped_count += 1
                        continue
                    
                    param = current_params[param_name]
                    
                    # 验证参数值
                    if validate:
                        valid, error_msg = ParamValidator.validate(param, param_value)
                        if not valid:
                            result.messages.append(
                                f"参数 {param_name} 验证失败：{error_msg}，跳过"
                            )
                            result.error_count += 1
                            continue
                    
                    # 检查冲突（值不同）
                    if abs(param.value - param_value) > 1e-9:
                        result.conflicts.append(
                            (param_name, param_value, param.value)
                        )
                    
                    # 更新参数值
                    param.value = param_value
                    result.imported_count += 1
            
            result.success = True
            result.messages.insert(0, f"成功导入 {result.imported_count} 个参数")
            
        except Exception as e:
            result.success = False
            result.messages.append(f"导入失败: {e}")
        
        return result
    
    @staticmethod
    def export_to_json_file(params: Dict[str, ParamInfo],
                           filepath: str,
                           include_metadata: bool = True,
                           vehicle_type: str = "Rover",
                           firmware_version: str = "Unknown") -> bool:
        """
        导出参数到 JSON 文件（包含完整元数据）
        
        格式示例：
        {
            "header": {
                "vehicle_type": "Rover",
                "firmware_version": "ArduPilot 4.5.0",
                "exported_at": "2025-11-05 14:30:00",
                "total_params": 100
            },
            "parameters": {
                "GPS_TYPE": {
                    "value": 1,
                    "type": "INTEGER",
                    "description": "GPS接收器类型",
                    "unit": null,
                    "min": 0,
                    "max": 19,
                    "default": 1,
                    "enum_values": {"0": "None", "1": "AUTO", ...}
                },
                ...
            }
        }
        
        Args:
            params: 参数字典
            filepath: 文件路径
            include_metadata: 是否包含元数据
            vehicle_type: 机体类型
            firmware_version: 固件版本
            
        Returns:
            是否成功
        """
        try:
            data = {
                "header": {
                    "vehicle_type": vehicle_type,
                    "firmware_version": firmware_version,
                    "exported_at": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    "total_params": len(params),
                    "format_version": "1.0"
                },
                "parameters": {}
            }
            
            # 导出参数
            for name, param in params.items():
                param_data = {
                    "value": param.value,
                    "type": param.param_type.name,
                }
                
                # 包含元数据
                if include_metadata:
                    meta = get_param_metadata(name)
                    if meta:
                        param_data.update({
                            "description": meta.description,
                            "user_description": meta.user_description,
                            "unit": meta.unit,
                            "min_value": meta.min_value,
                            "max_value": meta.max_value,
                            "default_value": meta.default_value,
                            "increment": meta.increment,
                            "reboot_required": meta.reboot_required,
                            "read_only": meta.read_only,
                        })
                        
                        # 枚举值
                        if meta.values:
                            param_data["enum_values"] = meta.values
                        
                        # 位掩码
                        if meta.bitmask:
                            param_data["bitmask"] = meta.bitmask
                    else:
                        param_data.update({
                            "description": param.description,
                            "unit": param.unit,
                            "min_value": param.min_value,
                            "max_value": param.max_value,
                        })
                
                data["parameters"][name] = param_data
            
            # 写入文件（格式化输出）
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            return True
            
        except Exception as e:
            _logger.error(f"导出 JSON 文件失败: {e}")
            return False
    
    @staticmethod
    def import_from_json_file(filepath: str,
                             current_params: Dict[str, ParamInfo],
                             validate: bool = True) -> ImportResult:
        """
        从 JSON 文件导入参数
        
        Args:
            filepath: 文件路径
            current_params: 当前参数字典
            validate: 是否验证参数值
            
        Returns:
            导入结果
        """
        result = ImportResult(
            success=False,
            imported_count=0,
            skipped_count=0,
            error_count=0,
            messages=[],
            conflicts=[]
        )
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 检查格式
            if "parameters" not in data:
                result.messages.append("无效的 JSON 格式：缺少 'parameters' 字段")
                return result
            
            # 读取文件头信息
            if "header" in data:
                header = data["header"]
                result.messages.append(
                    f"文件信息：{header.get('vehicle_type', 'Unknown')} - "
                    f"{header.get('firmware_version', 'Unknown')}"
                )
            
            # 导入参数
            params_data = data["parameters"]
            for param_name, param_data in params_data.items():
                # 检查参数是否存在
                if param_name not in current_params:
                    result.messages.append(f"参数 {param_name} 不存在，跳过")
                    result.skipped_count += 1
                    continue
                
                param = current_params[param_name]
                
                # 获取参数值
                if "value" not in param_data:
                    result.messages.append(f"参数 {param_name} 缺少值，跳过")
                    result.error_count += 1
                    continue
                
                param_value = float(param_data["value"])
                
                # 验证参数值
                if validate:
                    valid, error_msg = ParamValidator.validate(param, param_value)
                    if not valid:
                        result.messages.append(
                            f"参数 {param_name} 验证失败：{error_msg}，跳过"
                        )
                        result.error_count += 1
                        continue
                
                # 检查冲突
                if abs(param.value - param_value) > 1e-9:
                    result.conflicts.append(
                        (param_name, param_value, param.value)
                    )
                
                # 更新参数值
                param.value = param_value
                result.imported_count += 1
            
            result.success = True
            result.messages.insert(0, f"成功导入 {result.imported_count} 个参数")
            
        except json.JSONDecodeError as e:
            result.success = False
            result.messages.append(f"JSON 解析错误: {e}")
        except Exception as e:
            result.success = False
            result.messages.append(f"导入失败: {e}")
        
        return result
    
    @staticmethod
    def get_file_info(filepath: str) -> Optional[Dict]:
        """
        获取参数文件信息
        
        Args:
            filepath: 文件路径
            
        Returns:
            文件信息字典，包含格式、参数数量等
        """
        ext = os.path.splitext(filepath)[1].lower()
        
        try:
            if ext == '.param':
                # 读取 .param 文件头
                with open(filepath, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                
                info = {
                    "format": "QGroundControl .param",
                    "vehicle_type": "Unknown",
                    "firmware_version": "Unknown",
                    "exported_at": "Unknown",
                    "param_count": 0
                }
                
                # 解析文件头
                for line in lines:
                    line = line.strip()
                    if line.startswith("# Param file for:"):
                        info["vehicle_type"] = line.split(":", 1)[1].strip()
                    elif line.startswith("# Firmware version:"):
                        info["firmware_version"] = line.split(":", 1)[1].strip()
                    elif line.startswith("# Exported:"):
                        info["exported_at"] = line.split(":", 1)[1].strip()
                    elif line.startswith("# Total parameters:"):
                        try:
                            info["param_count"] = int(line.split(":", 1)[1].strip())
                        except (ValueError, IndexError):
                            pass
                    elif not line.startswith('#'):
                        # 数据行，计数
                        if ',' in line:
                            info["param_count"] += 1
                
                return info
                
            elif ext == '.json':
                # 读取 JSON 文件头
                with open(filepath, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                header = data.get("header", {})
                info = {
                    "format": "JSON with metadata",
                    "vehicle_type": header.get("vehicle_type", "Unknown"),
                    "firmware_version": header.get("firmware_version", "Unknown"),
                    "exported_at": header.get("exported_at", "Unknown"),
                    "param_count": len(data.get("parameters", {}))
                }
                
                return info
            
            else:
                return None
                
        except Exception as e:
            _logger.error(f"读取文件信息失败: {e}")
            return None
