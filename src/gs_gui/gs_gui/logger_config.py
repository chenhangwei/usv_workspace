#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2026 chenhangwei
# 
# This file is part of the USV Workspace project.
# 
# Implementation of logger config.
#
# Author: chenhangwei
# Date: 2026-01-26
"""
统一日志配置模块
提供标准化的日志格式、轮转和性能监控
"""

import logging
import logging.handlers
import os
import sys
from datetime import datetime
from typing import Optional
import json


class ColoredFormatter(logging.Formatter):
    """彩色日志格式化器（用于终端输出）"""
    
    # ANSI颜色代码
    COLORS = {
        'DEBUG': '\033[36m',      # 青色
        'INFO': '\033[32m',       # 绿色
        'WARNING': '\033[33m',    # 黄色
        'ERROR': '\033[31m',      # 红色
        'CRITICAL': '\033[35m',   # 紫色
        'RESET': '\033[0m'        # 重置
    }
    
    def format(self, record):
        # 添加颜色
        levelname = record.levelname
        if levelname in self.COLORS:
            record.levelname = f"{self.COLORS[levelname]}{levelname}{self.COLORS['RESET']}"
        
        return super().format(record)


class PerformanceFilter(logging.Filter):
    """性能监控过滤器，记录慢操作"""
    
    def __init__(self, slow_threshold_ms: float = 100.0):
        """
        Args:
            slow_threshold_ms: 慢操作阈值（毫秒）
        """
        super().__init__()
        self.slow_threshold_ms = slow_threshold_ms
    
    def filter(self, record):
        # 检查记录是否包含执行时间信息
        if hasattr(record, 'duration_ms'):
            if record.duration_ms > self.slow_threshold_ms:
                record.msg = f"[SLOW] {record.msg} (耗时: {record.duration_ms:.2f}ms)"
        return True


class LoggerConfig:
    """日志配置管理器"""
    
    DEFAULT_FORMAT = (
        '%(asctime)s [%(levelname)8s] [%(name)s] '
        '[%(filename)s:%(lineno)d] - %(message)s'
    )
    
    SIMPLE_FORMAT = '%(asctime)s [%(levelname)s] %(name)s - %(message)s'
    
    JSON_FORMAT = True  # 是否使用JSON格式（用于日志分析）
    
    @staticmethod
    def setup_logger(
        name: str,
        log_dir: Optional[str] = None,
        level: int = logging.INFO,
        console_output: bool = True,
        file_output: bool = True,
        max_bytes: int = 10 * 1024 * 1024,  # 10MB
        backup_count: int = 5,
        json_format: bool = False
    ) -> logging.Logger:
        """
        配置日志记录器
        
        Args:
            name: 日志记录器名称
            log_dir: 日志目录
            level: 日志级别
            console_output: 是否输出到控制台
            file_output: 是否输出到文件
            max_bytes: 单个日志文件最大字节数
            backup_count: 保留的日志文件数量
            json_format: 是否使用JSON格式
            
        Returns:
            配置好的日志记录器
        """
        logger = logging.getLogger(name)
        logger.setLevel(level)
        logger.handlers.clear()  # 清除现有处理器，避免重复
        
        # 控制台处理器
        if console_output:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(level)
            
            if sys.stdout.isatty():  # 如果是终端，使用彩色输出
                console_formatter = ColoredFormatter(
                    LoggerConfig.SIMPLE_FORMAT,
                    datefmt='%H:%M:%S'
                )
            else:
                console_formatter = logging.Formatter(
                    LoggerConfig.SIMPLE_FORMAT,
                    datefmt='%Y-%m-%d %H:%M:%S'
                )
            
            console_handler.setFormatter(console_formatter)
            logger.addHandler(console_handler)
        
        # 文件处理器
        if file_output:
            if log_dir is None:
                log_dir = os.path.abspath(os.path.join(os.getcwd(), '.logs'))
            
            os.makedirs(log_dir, exist_ok=True)
            
            # 普通日志文件（带轮转）
            log_file = os.path.join(log_dir, f'{name}.log')
            file_handler = logging.handlers.RotatingFileHandler(
                log_file,
                maxBytes=max_bytes,
                backupCount=backup_count,
                encoding='utf-8'
            )
            file_handler.setLevel(level)
            
            if json_format:
                file_formatter = JsonFormatter()
            else:
                file_formatter = logging.Formatter(
                    LoggerConfig.DEFAULT_FORMAT,
                    datefmt='%Y-%m-%d %H:%M:%S'
                )
            
            file_handler.setFormatter(file_formatter)
            logger.addHandler(file_handler)
            
            # 错误日志文件（只记录ERROR及以上）
            error_log_file = os.path.join(log_dir, f'{name}_error.log')
            error_handler = logging.handlers.RotatingFileHandler(
                error_log_file,
                maxBytes=max_bytes,
                backupCount=backup_count,
                encoding='utf-8'
            )
            error_handler.setLevel(logging.ERROR)
            error_handler.setFormatter(file_formatter)
            logger.addHandler(error_handler)
        
        # 添加性能监控过滤器
        perf_filter = PerformanceFilter(slow_threshold_ms=100.0)
        logger.addFilter(perf_filter)
        
        return logger


class JsonFormatter(logging.Formatter):
    """JSON格式化器，便于日志分析"""
    
    def format(self, record):
        log_data = {
            'timestamp': datetime.utcnow().isoformat(),
            'level': record.levelname,
            'logger': record.name,
            'module': record.module,
            'function': record.funcName,
            'line': record.lineno,
            'message': record.getMessage(),
        }
        
        # 添加异常信息
        if record.exc_info:
            log_data['exception'] = self.formatException(record.exc_info)
        
        # 添加额外字段
        if hasattr(record, 'duration_ms'):
            log_data['duration_ms'] = record.duration_ms
        
        if hasattr(record, 'usv_id'):
            log_data['usv_id'] = record.usv_id
        
        return json.dumps(log_data, ensure_ascii=False)


class LogContext:
    """日志上下文管理器，自动记录执行时间"""
    
    def __init__(self, logger: logging.Logger, operation: str, 
                 level: int = logging.DEBUG, include_result: bool = False):
        """
        Args:
            logger: 日志记录器
            operation: 操作描述
            level: 日志级别
            include_result: 是否包含返回结果
        """
        self.logger = logger
        self.operation = operation
        self.level = level
        self.include_result = include_result
        self.start_time = None
    
    def __enter__(self):
        import time
        self.start_time = time.time()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        import time
        duration_ms = (time.time() - self.start_time) * 1000
        
        if exc_type is None:
            # 成功执行
            extra = {'duration_ms': duration_ms}
            self.logger.log(
                self.level,
                f"{self.operation} 完成",
                extra=extra
            )
        else:
            # 发生异常
            self.logger.error(
                f"{self.operation} 失败: {exc_val}",
                exc_info=(exc_type, exc_val, exc_tb)
            )
        
        return False  # 不抑制异常


# 使用示例
"""
# 在包级别 __init__.py 中初始化
from .logger_config import LoggerConfig

logger = LoggerConfig.setup_logger(
    name='gs_gui',
    log_dir='/home/user/usv_workspace/.logs',
    level=logging.INFO,
    console_output=True,
    file_output=True,
    json_format=False
)

# 在各个模块中使用
import logging
logger = logging.getLogger('gs_gui')

# 方法1: 直接使用
logger.info("启动地面站节点")
logger.error("连接 usv_01 失败", extra={'usv_id': 'usv_01'})

# 方法2: 使用上下文管理器记录执行时间
from .logger_config import LogContext

with LogContext(logger, "发送导航目标", level=logging.INFO):
    send_navigation_goal()  # 自动记录耗时
"""
