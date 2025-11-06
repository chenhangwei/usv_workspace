"""
参数值验证器

提供参数输入验证功能，类似 QGroundControl
"""

from typing import Tuple, Optional
from .param_manager import ParamInfo, ParamType
from .param_metadata import get_param_metadata


class ParamValidator:
    """参数验证器"""
    
    @staticmethod
    def validate(param: ParamInfo, new_value: float) -> Tuple[bool, str]:
        """
        验证参数值是否有效
        
        Args:
            param: 参数信息
            new_value: 新值
        
        Returns:
            Tuple[bool, str]: (是否有效, 错误消息)
        """
        # 1. 类型检查
        if param.param_type == ParamType.INTEGER:
            if not float(new_value).is_integer():
                return False, f"参数 {param.name} 必须是整数"
        
        # 2. 范围检查
        if param.min_value is not None and new_value < param.min_value:
            return False, f"值 {new_value} 小于最小值 {param.min_value}"
        
        if param.max_value is not None and new_value > param.max_value:
            return False, f"值 {new_value} 大于最大值 {param.max_value}"
        
        # 3. 步进值检查（如果定义了步进值）
        if param.increment is not None and param.increment > 0:
            if param.min_value is not None:
                # 检查是否是步进值的整数倍（相对于最小值）
                offset = new_value - param.min_value
                if abs(offset % param.increment) > 1e-6:
                    return False, f"值必须是 {param.increment} 的倍数（从 {param.min_value} 开始）"
        
        return True, ""
    
    @staticmethod
    def suggest_valid_value(param: ParamInfo, input_value: float) -> float:
        """
        根据参数约束建议一个有效值
        
        Args:
            param: 参数信息
            input_value: 输入值
        
        Returns:
            float: 建议的有效值
        """
        value = input_value
        
        # 1. 限制到范围内
        if param.min_value is not None:
            value = max(value, param.min_value)
        
        if param.max_value is not None:
            value = min(value, param.max_value)
        
        # 2. 调整到最近的步进值
        if param.increment is not None and param.increment > 0:
            if param.min_value is not None:
                # 计算相对于最小值的偏移
                offset = value - param.min_value
                # 调整到最近的步进值
                steps = round(offset / param.increment)
                value = param.min_value + steps * param.increment
            else:
                # 无最小值，直接调整到步进值的整数倍
                steps = round(value / param.increment)
                value = steps * param.increment
        
        # 3. 整数参数取整
        if param.param_type == ParamType.INTEGER:
            value = float(int(round(value)))
        
        return value
    
    @staticmethod
    def get_value_description(param: ParamInfo, value: float) -> str:
        """
        获取参数值的描述（枚举值或位掩码说明）
        
        Args:
            param: 参数信息
            value: 参数值
        
        Returns:
            str: 值描述
        """
        metadata = get_param_metadata(param.name)
        if not metadata:
            return ""
        
        int_value = int(value)
        
        # 检查枚举值
        if metadata.values and int_value in metadata.values:
            return metadata.values[int_value]
        
        # 检查位掩码
        if metadata.bitmask:
            descriptions = []
            for bit, desc in metadata.bitmask.items():
                if int_value & (1 << bit):
                    descriptions.append(desc)
            
            if descriptions:
                return ", ".join(descriptions)
        
        return ""
    
    @staticmethod
    def is_default_value(param: ParamInfo) -> bool:
        """
        检查参数是否为默认值
        
        Args:
            param: 参数信息
        
        Returns:
            bool: 是否为默认值
        """
        metadata = get_param_metadata(param.name)
        if not metadata or metadata.default_value is None:
            return False
        
        return abs(param.value - metadata.default_value) < 1e-6
    
    @staticmethod
    def get_warning_level(param: ParamInfo, new_value: float) -> int:
        """
        获取参数修改的警告级别
        
        Args:
            param: 参数信息
            new_value: 新值
        
        Returns:
            int: 警告级别 (0=无警告, 1=提示, 2=警告, 3=严重警告)
        """
        metadata = get_param_metadata(param.name)
        
        # 需要重启的参数 = 警告
        if metadata and metadata.reboot_required:
            return 2
        
        # 某些关键参数的修改 = 严重警告
        critical_params = [
            'FRAME_TYPE',      # 机架类型
            'SYSID_THISMAV',   # MAVLink ID
            'ARMING_CHECK',    # 解锁检查
        ]
        
        if param.name in critical_params:
            # 检查是否禁用安全功能
            if param.name == 'ARMING_CHECK' and new_value == 0:
                return 3  # 禁用所有检查 = 严重警告
            return 2
        
        # 电池相关参数
        battery_params = ['BATT_CAPACITY', 'ARMING_VOLT_MIN', 'ARMING_VOLT2_MIN']
        if param.name in battery_params:
            return 1
        
        return 0
    
    @staticmethod
    def get_warning_message(param: ParamInfo, new_value: float) -> Optional[str]:
        """
        获取参数修改的警告消息
        
        Args:
            param: 参数信息
            new_value: 新值
        
        Returns:
            Optional[str]: 警告消息
        """
        level = ParamValidator.get_warning_level(param, new_value)
        
        if level == 0:
            return None
        
        metadata = get_param_metadata(param.name)
        
        messages = []
        
        # 重启提示
        if metadata and metadata.reboot_required:
            messages.append("[!] 此参数需要重启飞控后生效")
        
        # 特定参数警告
        if param.name == 'ARMING_CHECK' and new_value == 0:
            messages.append("🚨 警告：禁用所有解锁检查非常危险！仅用于测试环境。")
        
        elif param.name == 'FRAME_TYPE':
            messages.append("🚨 警告：修改机架类型会影响控制逻辑，错误设置会导致失控！")
        
        elif param.name == 'SYSID_THISMAV':
            messages.append("[!] 注意：修改 MAVLink ID 后需要重新配置地面站连接。")
        
        elif param.name.startswith('ARMING_VOLT'):
            messages.append("[*] 提示：确保电压阈值与电池规格匹配。")
        
        return "\n".join(messages) if messages else None
