#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of param loader.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
统一参数加载工具类

提供带验证、日志记录和默认值的参数加载功能,
解决项目中参数加载不一致和异常处理缺失的问题。
"""

from typing import TypeVar, Optional, Callable, Any, Union
from rclpy.node import Node
from rclpy.parameter import Parameter
import logging

T = TypeVar('T')


class ParamLoader:
    """统一参数加载器,提供验证、日志和异常处理"""
    
    def __init__(self, node: Node):
        """
        初始化参数加载器
        
        Args:
            node: ROS 2 节点实例
        """
        self.node = node
        self.logger = node.get_logger()
    
    def load_param(
        self,
        name: str,
        default: T,
        validator: Optional[Callable[[T], bool]] = None,
        description: str = ""
    ) -> T:
        """
        加载单个参数,带验证和日志
        
        Args:
            name: 参数名称
            default: 默认值
            validator: 验证函数,返回 True 表示有效
            description: 参数描述(用于日志)
            
        Returns:
            参数值或默认值
            
        Example:
            loader = ParamLoader(self)
            rate = loader.load_param(
                'publish_rate', 
                20.0,
                validator=lambda x: 0 < x <= 100,
                description="发布频率(Hz)"
            )
        """
        try:
            # 声明参数
            self.node.declare_parameter(name, default)
            
            # 获取参数值
            param = self.node.get_parameter(name)
            value = param.value
            
            # 检查是否为 None
            if value is None:
                self.logger.warn(
                    f"参数 '{name}' 未设置,使用默认值: {default}"
                    + (f" ({description})" if description else "")
                )
                return default
            
            # 类型转换
            value = type(default)(value) if not isinstance(value, type(default)) else value
            
            # 验证
            if validator and not validator(value):
                self.logger.error(
                    f"参数 '{name}' 验证失败: {value}, 使用默认值: {default}"
                )
                return default
            
            # 成功加载
            self.logger.info(
                f"✓ 加载参数 '{name}' = {value}"
                + (f" ({description})" if description else "")
            )
            return value
            
        except (TypeError, ValueError) as e:
            self.logger.error(
                f"参数 '{name}' 类型转换失败: {e}, 使用默认值: {default}"
            )
            return default
        except Exception as e:
            self.logger.error(
                f"加载参数 '{name}' 失败: {e}, 使用默认值: {default}"
            )
            return default
        
        # 不应到达这里,但为了类型检查器
        return default
    
    def load_params(self, param_config: dict) -> dict:
        """
        批量加载参数
        
        Args:
            param_config: 参数配置字典,格式:
                {
                    'param_name': {
                        'default': value,
                        'validator': function,  # 可选
                        'description': 'desc'   # 可选
                    }
                }
        
        Returns:
            加载后的参数字典
            
        Example:
            config = {
                'publish_rate': {
                    'default': 20.0,
                    'validator': lambda x: 0 < x <= 100,
                    'description': '发布频率'
                },
                'timeout': {
                    'default': 5.0,
                    'description': '超时时间'
                }
            }
            params = loader.load_params(config)
        """
        result = {}
        for name, cfg in param_config.items():
            result[name] = self.load_param(
                name,
                cfg['default'],
                cfg.get('validator'),
                cfg.get('description', '')
            )
        return result
    
    def load_gps_origin(
        self,
        default_lat: float = 0.0,
        default_lon: float = 0.0,
        default_alt: float = 0.0
    ) -> dict:
        """
        加载 GPS 原点参数(专用方法,解决项目中 GPS 原点配置重复的问题)
        
        Args:
            default_lat: 默认纬度
            default_lon: 默认经度
            default_alt: 默认海拔
            
        Returns:
            {'lat': float, 'lon': float, 'alt': float}
        """
        def validate_lat(x): return -90 <= x <= 90
        def validate_lon(x): return -180 <= x <= 180
        def validate_alt(x): return -1000 <= x <= 10000
        
        lat = self.load_param(
            'gps_origin_lat',
            default_lat,
            validate_lat,
            'GPS原点纬度'
        )
        lon = self.load_param(
            'gps_origin_lon',
            default_lon,
            validate_lon,
            'GPS原点经度'
        )
        alt = self.load_param(
            'gps_origin_alt',
            default_alt,
            validate_alt,
            'GPS原点海拔'
        )
        
        self.logger.info(
            f"GPS 原点: ({lat:.7f}°, {lon:.7f}°, {alt:.2f}m)"
        )
        
        return {'lat': lat, 'lon': lon, 'alt': alt}


class ParamValidator:
    """常用参数验证器集合"""
    
    @staticmethod
    def positive(x: Union[int, float]) -> bool:
        """验证正数"""
        return x > 0
    
    @staticmethod
    def non_negative(x: Union[int, float]) -> bool:
        """验证非负数"""
        return x >= 0
    
    @staticmethod
    def in_range(min_val: float, max_val: float) -> Callable[[float], bool]:
        """验证范围"""
        return lambda x: min_val <= x <= max_val
    
    @staticmethod
    def port_number(x: int) -> bool:
        """验证端口号"""
        return 0 <= x <= 65535
    
    @staticmethod
    def non_empty_string(x: str) -> bool:
        """验证非空字符串"""
        return isinstance(x, str) and len(x) > 0
    
    @staticmethod
    def frequency(x: float) -> bool:
        """验证频率(Hz)"""
        return 0 < x <= 1000
    
    @staticmethod
    def timeout(x: float) -> bool:
        """验证超时时间"""
        return 0 < x <= 3600


# 使用示例
"""
from common_utils import ParamLoader, ParamValidator

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # 创建参数加载器
        loader = ParamLoader(self)
        
        # 方法1: 单个加载
        self.publish_rate = loader.load_param(
            'publish_rate',
            20.0,
            ParamValidator.frequency,
            '发布频率(Hz)'
        )
        
        # 方法2: 批量加载
        config = {
            'timeout': {
                'default': 5.0,
                'validator': ParamValidator.timeout,
                'description': '超时时间(秒)'
            },
            'port': {
                'default': 8080,
                'validator': ParamValidator.port_number,
                'description': '监听端口'
            }
        }
        params = loader.load_params(config)
        self.timeout = params['timeout']
        self.port = params['port']
        
        # 方法3: GPS 原点专用加载
        gps_origin = loader.load_gps_origin(
            default_lat=22.5180977,
            default_lon=113.9007239,
            default_alt=-5.17
        )
        self.origin_lat = gps_origin['lat']
        self.origin_lon = gps_origin['lon']
        self.origin_alt = gps_origin['alt']
"""
