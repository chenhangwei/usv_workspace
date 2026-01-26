#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of param metadata.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
ArduPilot 参数元数据加载器

提供参数描述、单位、范围等元数据，类似 QGroundControl
"""

import json
from pathlib import Path
from typing import Dict, Optional, Any
from dataclasses import dataclass


@dataclass
class ParamMetadata:
    """参数元数据"""
    name: str                           # 参数名称
    display_name: str = ""              # 显示名称
    description: str = ""               # 参数描述
    user_description: str = ""          # 用户友好的描述
    unit: str = ""                      # 单位（如 m, deg, %）
    min_value: Optional[float] = None   # 最小值
    max_value: Optional[float] = None   # 最大值
    default_value: Optional[float] = None  # 默认值
    increment: Optional[float] = None   # 步进值
    values: Dict[int, str] = None       # 枚举值（如 GPS_TYPE: {0: "None", 1: "AUTO"}）
    bitmask: Dict[int, str] = None      # 位掩码（如 ARMING_CHECK）
    reboot_required: bool = False       # 是否需要重启
    read_only: bool = False             # 是否只读
    
    def __post_init__(self):
        if self.values is None:
            self.values = {}
        if self.bitmask is None:
            self.bitmask = {}


class ParamMetadataLoader:
    """参数元数据加载器"""
    
    def __init__(self):
        self._metadata: Dict[str, ParamMetadata] = {}
        self._loaded = False
        
        # 元数据文件路径
        self._metadata_file = Path(__file__).parent.parent / 'resource' / 'param_metadata.json'
    
    def load(self) -> bool:
        """
        加载参数元数据
        
        Returns:
            bool: 是否成功加载
        """
        if self._loaded:
            return True
        
        try:
            # 如果文件不存在，使用内置的基础元数据
            if not self._metadata_file.exists():
                self._load_built_in_metadata()
                self._loaded = True
                return True
            
            # 从 JSON 加载
            with open(self._metadata_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 解析元数据
            for name, meta_dict in data.items():
                self._metadata[name] = ParamMetadata(
                    name=name,
                    display_name=meta_dict.get('display_name', ''),
                    description=meta_dict.get('description', ''),
                    user_description=meta_dict.get('user_description', ''),
                    unit=meta_dict.get('unit', ''),
                    min_value=meta_dict.get('min_value'),
                    max_value=meta_dict.get('max_value'),
                    default_value=meta_dict.get('default_value'),
                    increment=meta_dict.get('increment'),
                    values=meta_dict.get('values', {}),
                    bitmask=meta_dict.get('bitmask', {}),
                    reboot_required=meta_dict.get('reboot_required', False),
                    read_only=meta_dict.get('read_only', False)
                )
            
            self._loaded = True
            return True
            
        except Exception as e:
            print(f"加载参数元数据失败: {e}")
            # 降级到内置元数据
            self._load_built_in_metadata()
            self._loaded = True
            return False
    
    def _load_built_in_metadata(self):
        """加载内置的基础参数元数据（常用参数）"""
        # ArduPilot Rover 常用参数的元数据
        built_in_params = {
            # ==================== 解锁检查（ARMING） ====================
            "ARMING_CHECK": ParamMetadata(
                name="ARMING_CHECK",
                display_name="Arming Check",
                description="解锁前检查项（位掩码）",
                user_description="控制解锁前要执行哪些安全检查。每个位代表一项检查。",
                unit="",
                min_value=0,
                max_value=255,
                default_value=1,
                bitmask={
                    0: "All (所有检查)",
                    1: "Barometer (气压计)",
                    2: "Compass (指南针)",
                    3: "GPS (GPS定位)",
                    4: "INS (惯性导航)",
                    5: "Parameters (参数)",
                    6: "RC (遥控器)",
                    7: "Voltage (电压)",
                    8: "Battery (电池)",
                    9: "Airspeed (空速)",
                    10: "Logging (日志)",
                    11: "Switch (开关)",
                    12: "GPS Config (GPS配置)"
                },
                reboot_required=False
            ),
            "ARMING_VOLT_MIN": ParamMetadata(
                name="ARMING_VOLT_MIN",
                display_name="Minimum Arming Voltage",
                description="允许解锁的最低电池电压",
                user_description="低于此电压将无法解锁。设为0禁用此检查。",
                unit="V",
                min_value=0.0,
                max_value=30.0,
                default_value=10.5,
                increment=0.1,
                reboot_required=False
            ),
            "ARMING_VOLT2_MIN": ParamMetadata(
                name="ARMING_VOLT2_MIN",
                display_name="Minimum Arming Voltage (Battery 2)",
                description="第二块电池允许解锁的最低电压",
                user_description="适用于双电池配置",
                unit="V",
                min_value=0.0,
                max_value=30.0,
                default_value=0.0,
                increment=0.1
            ),
            
            # ==================== GPS ====================
            "GPS_TYPE": ParamMetadata(
                name="GPS_TYPE",
                display_name="GPS Type",
                description="GPS接收器类型",
                user_description="选择连接的GPS模块型号",
                unit="",
                min_value=0,
                max_value=19,
                default_value=1,
                values={
                    0: "None (无GPS)",
                    1: "AUTO (自动检测)",
                    2: "uBlox",
                    5: "NMEA",
                    6: "SiRF",
                    7: "HIL",
                    8: "SwiftNav",
                    9: "UAVCAN",
                    10: "SBF",
                    11: "GSOF",
                    13: "ERB",
                    14: "MAV",
                    15: "NOVA",
                    16: "HemisphereNMEA",
                    17: "uBlox-MovingBaseline-Base",
                    18: "uBlox-MovingBaseline-Rover",
                    19: "MSP"
                },
                reboot_required=True
            ),
            "GPS_AUTO_SWITCH": ParamMetadata(
                name="GPS_AUTO_SWITCH",
                display_name="GPS Auto Switch",
                description="自动切换GPS",
                user_description="启用后自动切换到最优GPS（双GPS配置）",
                unit="",
                min_value=0,
                max_value=2,
                default_value=1,
                values={
                    0: "Disabled (禁用)",
                    1: "Use Best (使用最佳)",
                    2: "Blend (混合)"
                }
            ),
            
            # ==================== 指南针（COMPASS） ====================
            "COMPASS_USE": ParamMetadata(
                name="COMPASS_USE",
                display_name="Use Compass",
                description="启用指南针",
                user_description="启用或禁用指南针。建议保持启用。",
                unit="",
                min_value=0,
                max_value=1,
                default_value=1,
                values={
                    0: "Disabled (禁用)",
                    1: "Enabled (启用)"
                },
                reboot_required=False
            ),
            "COMPASS_AUTODEC": ParamMetadata(
                name="COMPASS_AUTODEC",
                display_name="Auto Declination",
                description="自动磁偏角",
                user_description="自动使用GPS位置计算磁偏角",
                unit="",
                min_value=0,
                max_value=1,
                default_value=1,
                values={
                    0: "Disabled (使用COMPASS_DEC)",
                    1: "Enabled (自动计算)"
                }
            ),
            
            # ==================== 电池（BATT） ====================
            "BATT_CAPACITY": ParamMetadata(
                name="BATT_CAPACITY",
                display_name="Battery Capacity",
                description="电池容量",
                user_description="电池总容量，用于计算剩余电量百分比",
                unit="mAh",
                min_value=0,
                max_value=1000000,
                default_value=3300,
                increment=50,
                reboot_required=False
            ),
            "BATT_MONITOR": ParamMetadata(
                name="BATT_MONITOR",
                display_name="Battery Monitor",
                description="电池监视器类型",
                user_description="选择电池电压/电流监测方式",
                unit="",
                min_value=0,
                max_value=30,
                default_value=0,
                values={
                    0: "Disabled (禁用)",
                    3: "Analog Voltage Only (仅模拟电压)",
                    4: "Analog Voltage and Current (电压和电流)",
                    5: "Solo (Solo电池)",
                    6: "Bebop (Bebop电池)",
                    7: "SMBus-Generic (SMBus通用)",
                    8: "UAVCAN-BatteryInfo (UAVCAN)",
                    9: "BLHeli ESC (BLHeli电调)",
                    10: "SumOfFollowing (后续总和)",
                    11: "FuelFlow (燃油流量)",
                    12: "FuelLevel-PWM (燃油液位PWM)",
                    13: "SMBUS-SUI3 (SMBUS SUI3)",
                    14: "SMBUS-SUI6 (SMBUS SUI6)",
                    15: "NeoDesign (NeoDesign)",
                    16: "SMBus-Maxell (SMBus Maxell)",
                    17: "Generator-Elec (Generator电)",
                    18: "Generator-Fuel (Generator燃油)",
                    19: "Rotoye (Rotoye)",
                    20: "MPPT (MPPT)",
                    21: "INA2XX (INA2XX)",
                    22: "LTC2946 (LTC2946)",
                    23: "Torqeedo (Torqeedo)",
                    24: "FuelLevel-Analog (燃油液位模拟)"
                },
                reboot_required=True
            ),
            
            # ==================== 系统（SYS） ====================
            "SYSID_THISMAV": ParamMetadata(
                name="SYSID_THISMAV",
                display_name="MAVLink System ID",
                description="MAVLink系统ID",
                user_description="此飞控的唯一ID，用于多机通信。1-255之间。",
                unit="",
                min_value=1,
                max_value=255,
                default_value=1,
                increment=1,
                reboot_required=True
            ),
            "SYSID_MYGCS": ParamMetadata(
                name="SYSID_MYGCS",
                display_name="Ground Station ID",
                description="地面站MAVLink ID",
                user_description="地面站的MAVLink ID",
                unit="",
                min_value=1,
                max_value=255,
                default_value=255,
                increment=1
            ),
            
            # ==================== 机架类型（FRAME） ====================
            "FRAME_TYPE": ParamMetadata(
                name="FRAME_TYPE",
                display_name="Frame Type",
                description="机架类型",
                user_description="选择机体类型。错误设置会导致控制异常！",
                unit="",
                min_value=0,
                max_value=3,
                default_value=0,
                values={
                    0: "Rover (普通车型)",
                    1: "Skid-Steer (履带车型)",
                    2: "Boat (船型)",
                    3: "Balancebot (平衡车)"
                },
                reboot_required=True
            ),
            
            # ==================== 串口（SERIAL） ====================
            "SERIAL0_BAUD": ParamMetadata(
                name="SERIAL0_BAUD",
                display_name="Serial0 Baud Rate",
                description="USB串口波特率",
                user_description="USB连接的波特率。57=57600, 115=115200, 921=921600",
                unit="",
                min_value=1,
                max_value=2000,
                default_value=115,
                values={
                    1: "1200",
                    2: "2400",
                    4: "4800",
                    9: "9600",
                    19: "19200",
                    38: "38400",
                    57: "57600",
                    111: "111100",
                    115: "115200",
                    230: "230400",
                    256: "256000",
                    460: "460800",
                    500: "500000",
                    921: "921600",
                    1500: "1500000",
                    2000: "2000000"
                },
                reboot_required=True
            ),
            
            # ==================== 日志（LOG） ====================
            "LOG_BACKEND_TYPE": ParamMetadata(
                name="LOG_BACKEND_TYPE",
                display_name="Log Backend Type",
                description="日志后端类型",
                user_description="选择日志存储方式",
                unit="",
                min_value=0,
                max_value=3,
                default_value=1,
                values={
                    0: "None (不记录)",
                    1: "File (文件)",
                    2: "MAVLink (MAVLink)",
                    3: "Block (块设备)"
                },
                reboot_required=True
            ),
            "LOG_DISARMED": ParamMetadata(
                name="LOG_DISARMED",
                display_name="Log Disarmed",
                description="锁定时记录日志",
                user_description="启用后在未解锁时也记录日志",
                unit="",
                min_value=0,
                max_value=1,
                default_value=0,
                values={
                    0: "Disabled (禁用)",
                    1: "Enabled (启用)"
                }
            ),
        }
        
        self._metadata = built_in_params
    
    def get(self, param_name: str) -> Optional[ParamMetadata]:
        """
        获取参数元数据
        
        Args:
            param_name: 参数名称
        
        Returns:
            ParamMetadata 或 None
        """
        if not self._loaded:
            self.load()
        
        return self._metadata.get(param_name)
    
    def get_all(self) -> Dict[str, ParamMetadata]:
        """
        获取所有参数元数据
        
        Returns:
            Dict[str, ParamMetadata]: 参数名称 -> 元数据映射
        """
        if not self._loaded:
            self.load()
        
        return self._metadata.copy()
    
    def has_metadata(self, param_name: str) -> bool:
        """
        检查参数是否有元数据
        
        Args:
            param_name: 参数名称
        
        Returns:
            bool: 是否有元数据
        """
        if not self._loaded:
            self.load()
        
        return param_name in self._metadata


# 全局实例
_metadata_loader = ParamMetadataLoader()


def get_param_metadata(param_name: str) -> Optional[ParamMetadata]:
    """获取参数元数据（全局函数）"""
    return _metadata_loader.get(param_name)


def load_all_metadata() -> bool:
    """加载所有元数据（全局函数）"""
    return _metadata_loader.load()
